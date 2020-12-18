#include "Combined.hpp"
#include <algorithm>
#include <iomanip>

namespace impl{


void Combined::setHumanInput(InterventionInfo humanInput){
		LOGDEBUGLV2(DEBUG_INFO_TELE, "[CTRL] Get human input ({}, {})", humanInput.v, humanInput.w);
		
		pt2D uH;
#ifdef __QNXNTO__
		uH.x = -humanInput.v;
		uH.y = humanInput.w;
#else
		uH.x = -humanInput.v;
		uH.y = humanInput.w;
#endif

		uH *= humanScale;
		uH = rot(uH, -pi / 2);

		HumanInput h;
		h.t = 0;			// This intervention info has just been received
		h.x = (real_t)uH.x;
		h.y = (real_t)uH.y;

		hiLast = h;
		ivLast = humanInput;

		
		if (base_function() == FUNC_TELE_BASIC) {
			teleDirect.v = humanInput.v;
			teleDirect.w = humanInput.w;
			teleDirect.loopCounter = lastTime.loopCounter;
			teleDirect.setValid(true);
		}
		else if (base_function() == FUNC_TELE_DXY) {
			teleDirectVxy.vx = humanInput.v;
			teleDirectVxy.vy = humanInput.w;
			teleDirectVxy.loopCounter = lastTime.loopCounter;
			teleDirectVxy.setValid(true);
		}
	}
	void Combined::setHumanPoisition(pt2D pos){
		human_pos = pos; 
		human_pos.setValid(true);
	}

	enum{
		IAM_FAULTY = (1<<0),
		USE_HFIELD = (1<<1),
		CHG_SHAPES = (1<<2)
	};


	static const std::size_t currBufferLimit = 80;

	void dump_mismatch(int currLoop, int srcID, void* dat, size_t size, unsigned char csum_get, unsigned char csum_compute, const char* fout = "dump_errmsg.txt") {
		std::ofstream out(fout, std::ios_base::app);
		out << "Loop " << currLoop
			<< ", mismatched buffer received from " << srcID << ", get " << (int)(csum_get) << ", computed " << (int)(csum_compute) << '\n';

		unsigned char* beg = (unsigned char*)dat;
		out.setf(std::ios_base::hex, std::ios_base::basefield);
		//out.setf(std::ios_base::showbase);
		for (size_t i = 0; i < size; ++i){
			out << std::setw(2) << (int)beg[i] << ' ' << ' ';
			if ((i + 1) % 10 == 0) out << '\n';
		}
		if (size % 10 != 0) out << '\n';
		out << "======================================\n";
		out.close();
	}
	void dump_errwrite(int currLoop, const std::string& err_msg, const char* fname) {
		std::ofstream out(fname, std::ios_base::app);
		out << "Loop " << currLoop
			<< ", write_buffer failed: " << err_msg << '\n';
		out.close();
	}

	conf::CtrlPack& Combined::updateMyCtrlPack() {
		static int last_loop_send = -1;
		compack.reset();
		if (DebugMode == 1) {
			return compack;
		}

		std::string log_msg;
		if (enableFDetect()) {
			compack.withIsFaulty(iamfaulty);
			if (iamfaulty)
				log_msg += " iam_fault";
		}
		if (enableHFieldMulti()) {
			log_msg += " hfield";
			compack.setHField(hfMe.ratio, hfMe.x, hfMe.y);
		}
		if (enableLatencyTest()) {
			if (latency_test.onSendMessage(nullptr) <= 4) {
				short latTest[4];
				latency_test.onSendMessage(latTest);
				compack.setLatency((std::uint16_t*)latTest);
			}
		}
		if (use_autoselect && needShapeSelection()) {
			char shp = getMyShapeIndex();
			compack.setShape(4, shp, (uint8_t)getFormationGroup(), 100);
			log_msg += fmt::format(" shape-{}, gp-{}", (int)shp, (int)getFormationGroup());
		}

		if (last_loop_send != (int)lastTime.loopCounter) {
			if (log_msg.empty())
				log_msg = " (None)";

			LOGDEBUGLV2(DEBUG_INFO_COM_SEND, "[Ctrl] Sending{}", log_msg);
			last_loop_send = (int)lastTime.loopCounter;
		}
		return compack;
	}
	size_t Combined::getCtrlMsg(void* buf) {
		updateMyCtrlPack();

		if (compack.write_buffer_all(buf, currBufferLimit, (char)getMyID()) == 0) {
			std::string dump_file_name = fmt::format("{}dump_errwrite_{}.txt", logDir, getMyID());
			std::string err_msg;
			if (compack.lastError() == conf::CtrlPack::ERR_GOOD) err_msg = "All is good";
			else if (compack.lastError() == conf::CtrlPack::ERR_BUF_SMALL) err_msg = "Buf too small";
			else if (compack.lastError() == conf::CtrlPack::ERR_OTHER) err_msg = "Buf other error";
			else if (compack.lastError() == conf::CtrlPack::ERR_CSUM) err_msg = "Check sum failed";
			else err_msg = "Unknown error";
			dump_errwrite(lastTime.loopCounter, err_msg, dump_file_name.c_str());
			LOGWARN("Warn, getCtrlMsg failed.");
		}
		return currBufferLimit;
	}

	void Combined::handleCtrlPack(int srcID, const conf::CtrlPack& comrec, std::string& log_msg) {
		if (DebugMode == 1) return;

		GoodMsgCounter++;
		if (comrec.withIsFaulty() && comrec.validIsFaulty()) {
			if (enableFDetect()) {
				errList.insert(srcID);
				log_msg += " iam_fault";
			}
			else {
				log_msg += " iam_fault(OFF)";
			}
		}
		if (comrec.validLatency() && enableLatencyTest()) {
			unsigned short lat[4];
			comrec.getLatency(lat);
			latency_test.onRecvMessage(srcID, (short*)lat);
		}
		if (comrec.validHField() && enableHFieldMulti()) {
			log_msg += " hfield";
			comrec.getHField(hfNet[srcID].ratio, hfNet[srcID].x, hfNet[srcID].y);
		}
		if (comrec.validShape()) {
			// 如果有人想要变换队形, 那么我也应该变换队形
			std::uint8_t n, shp, fmgroup;
			std::uint16_t prior;
			comrec.getShape(n, shp, fmgroup, prior);
			log_msg += fmt::format(" gp-{}", (int)fmgroup);
			if ((int)fmgroup == getFormationGroup()) {
				// 只考虑本formation_group里人发来的信息
				log_msg += fmt::format(" shape-{}", (int)shp);
				shapeReceive[srcID] = shp;
			}
		}
		if (log_msg.empty()) {
			log_msg = " (None)";
		}
	}

	void Combined::handleMsg(int srcID, void* dat, size_t size) {
		if (DebugMode == 1) {
			return ;
		}

		std::string log_msg = fmt::format("#{}:", srcID);
		conf::CtrlPack comrec;
		if (comrec.read_buffer_all(dat, size) > 0) {
			handleCtrlPack(srcID, comrec, log_msg);
		}
		else {
			if (comrec.lastError() == conf::CtrlPack::ERR_CSUM) {
				log_msg += fmt::format(" CSUM_FAILED");
				LOGWARN("[Ctrl] Recv from {}, checksum failed! ", srcID);
				std::string dump_file_name = fmt::format("{}dump_mismatch_{}.txt", logDir, getMyID());
				dump_mismatch(lastTime.loopCounter, srcID, dat, size, comrec._last_csum_get, comrec._last_csum_compute, dump_file_name.c_str());
				BadMsgCounter++;
			}
			else if (comrec.lastError() == conf::CtrlPack::ERR_BUF_SMALL) {
				log_msg += fmt::format(" INVALID_SIZE: {}", size);
				LOGWARN("[Ctrl] Recv from {}, buf size too small! ", srcID);
				MisSizeCounter++;
			}
		}
		LOGDEBUGLV2(DEBUG_INFO_COM_RECV, "[Ctrl] Recv: {}", log_msg);
		return;
	}

	size_t Combined::getCtrlMsgBackup(void* buf){
		if (DebugMode == 1) {
			return 0;
		}

		static int last_loop_send = -1;
		assert(sizeof(char) == 1 && "size char != 1");
		assert(sizeof(short) == 2 && "size short != 2");
		assert(sizeof(int) == 4 && "size int != 4");
		if (sizeof(short) != 2) {
			LOGWARN("[Ctrl] Error: sizeof(short) is {}, not 2 !!", sizeof(short));
		}

		char* dest = (char*)buf;
		char& fst = *dest;                  // 0
		char  sgn = 0;                      // 0: low four
		char  shp = 0;                      // 0: upper four
		char& hfr = *(dest + 1);			// 1
		short& hfx = *(short*)(dest + 2);	// 2, 3
		short& hfy = *(short*)(dest + 4);	// 4, 5
		short* latTest = (short*)(dest + 6);   // 67, 89, 1011, 1112
		unsigned char& check = *(unsigned char*)(dest + 20);
		
		size_t byte_num = 80;
		std::fill(dest, dest + byte_num, getMyID());
		
		std::string log_msg;
		if (enableFDetect() && iamfaulty){
			sgn |= IAM_FAULTY;
			log_msg += " iam_fault";
		}
		
		if (enableHFieldMulti()) {
			log_msg += " hfield";
			sgn |= USE_HFIELD;
			float val;
			val = saturate(hfMe.ratio, 1.0f);
			hfr = (uint8_t)(hfMe.ratio * 100);
			val = saturate(hfMe.x, 200.0f);
			hfx = (uint16_t)(val * 100);
			val = saturate(hfMe.y, 200.0f);
			hfy = (uint16_t)(val * 100);
		}
		else{
			uint8_t my_id = (uint8_t)getMyID();
			for (size_t i = 0; i < 5; ++i){
				(&hfr)[i] = my_id;
			}
		}
		if (enableLatencyTest()) {
			if (latency_test.onSendMessage(nullptr) <= 4) {
				latency_test.onSendMessage(latTest);
			}
		}
		if (use_autoselect && needShapeSelection()) {
			// Even if shape_selected = true for me,
			// since others may miss some messages,
			// I have to keep broadcasting my decisions.
			sgn |= CHG_SHAPES;
			shp = getMyShapeIndex();
			log_msg += fmt::format(" shape-{}", (int)shp);
		}
		fst = (sgn & 0xf) | ((shp & 0xf) << 4);     // This is the first byte
		
		check = 0;
		for (int i = 0; i < 20; ++i){
			check += dest[i];
		}

		if (last_loop_send != (int)lastTime.loopCounter) {
			if (log_msg.empty()) 
				log_msg = " (None)";
			
			LOGDEBUGLV2(DEBUG_INFO_COM_SEND, "[Ctrl] Sending{}", log_msg);
			last_loop_send = (int)lastTime.loopCounter;
		}
		return byte_num;
	}

	void Combined::handleMsgBackup(int srcID, void* dat, size_t size){
		if (DebugMode == 1) {
			return ;
		}
		std::string log_msg = fmt::format("#{}:", srcID);
		size_t byte_num = 80;
		if (size >= byte_num){
			// Valid
			char* dest = (char*)dat;
			char& fst = *dest;                  // 0
			char  sgn = *dest & 0xf;	        // 0: low four
			char  shp = ((*dest) >> 4) & 0xf;   // 1: upper four
			char&  hfr = *(dest + 1);			// 1
			short& hfx = *(short*)(dest + 2);	// 2, 3
			short& hfy = *(short*)(dest + 4);	// 4, 5
			short* latTest = (short*)(dest + 6);   // 67, 89, 1011, 1112
			unsigned char& check = *(unsigned char*)(dest + 20);

			unsigned char check_compute = 0;
			for (int i = 0; i < 20; ++i){
				check_compute += dest[i];
			}
			if (check != check_compute) {
				//LOGWARN("[Ctrl] Error: Check Failed for Message from {}: {:X} <> {:X}(get)", srcID, check_compute, check);
				BadMsgCounter++;
				return;
			}
			GoodMsgCounter++;

			if (sgn & IAM_FAULTY){
				if (enableFDetect()){
					errList.insert(srcID);
					log_msg += " iam_fault";
				}
				else{
					log_msg += " iam_fault(OFF)";
				}
			}
			if (enableLatencyTest()) {
				latency_test.onRecvMessage(srcID, latTest);
			}
			if (sgn & USE_HFIELD){
				if (enableHFieldMulti()){
					log_msg += " hfield";
					hfNet[srcID].ratio = hfr / 100.0f;
					hfNet[srcID].x = hfx / 100.0f;
					hfNet[srcID].y = hfy / 100.0f;
				}
			}
			if (sgn & CHG_SHAPES){
				// 如果有人想要变换队形, 那么我也应该变换队形
				log_msg += fmt::format(" shape-{}", (int)shp);
				shapeReceive[srcID] = shp;
			}
			if (log_msg.empty()){
				log_msg = " (None)";
			}
		}
		else{
			log_msg += fmt::format(" INVALID_SIZE: {}", size);
			MisSizeCounter++;
		}

		LOGDEBUGLV2(DEBUG_INFO_COM_RECV, "[Ctrl] Recv: {}", log_msg);
		return;
	}
	
	void Combined::print_com_stat() {
		int total = BadMsgCounter + GoodMsgCounter + MisSizeCounter;
		if (total == GoodMsgCounter && total > 0)
			return;

		std::string msg = fmt::format("[Ctrl] Message Get {}", total, GoodMsgCounter);
		if (GoodMsgCounter != total)
			msg += fmt::format(": Good {}", GoodMsgCounter);
		if (BadMsgCounter > 0) {
			msg += fmt::format(", Bad {}({:.1f}%)", BadMsgCounter, BadMsgCounter*(100.0/total));
		}
		if (MisSizeCounter > 0) {
			msg += fmt::format(", Mis {}({:.1f}%)", MisSizeCounter, MisSizeCounter*(100.0 / total));
		}
		LOGDEBUGLV2(DEBUG_INFO_STATES, msg);
	}

	
	void Combined::getDebugItem(uint item_id, void* item_ptr, void* args){
		if (item_id == 1){
			float* dest = (float*)item_ptr;
			if (enableHField()){
				dest[0] = hfMe.x;
				dest[1] = hfMe.y;
				dest[2] = hfMe.ratio;
			}
			else{
				dest[0] = dest[1] = dest[2] = 0;
			}
		}
		if (item_id == 2){
			/*
			int low = functionState & 0xff;
			if (low == FUNC_TRACE_HUMAN){
				float* dest = (float*)item_ptr;
				if (tracer.leader.valid()){
					dest[0] = (float)tracer.leader.x;
					dest[1] = (float)tracer.leader.y;
					dest[2] = (float)tracer.leader_vxy.x;
					dest[3] = (float)tracer.leader_vxy.y;
				}
				else{
					dest[0] = dest[1] = dest[2] = dest[3] = 0;
				}
			}
			*/
		}
		if (item_id == 3){
			/*
			size_t buf_size = 1024;
			if (args){
				buf_size = *((int*)args);
			}
			if (buf_size < sizeof(int) + sizeof(float) * 3 * obXY.size()){
				LOGWARN("[Ctrl] Failed to send obstacle info to debug, buf_size is too_small");
				return;
			}

			int* pnum = (int*)item_ptr;
			*pnum = (int)obXY.size();
			if (obXY.size() > 0){
				float *px, *py, *pr;
				px = (float*)(pnum + 1);
				for (int i = 0; i < *pnum; ++i){
					py = px + 1;
					pr = px + 2;
					*px = impl::cm2m((float)obXY[i].x);
					*py = impl::cm2m((float)obXY[i].y);
					*pr = impl::cm2m((float)obR[i]);
					px = px + 3;
				}
			}
			*/
		}
	}

}   // namespace impl
