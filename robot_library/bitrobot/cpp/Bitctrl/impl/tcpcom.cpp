#if defined(ENABLE_SSNETWORK)
#include "tcpcom.hpp"
#include "Combined.hpp"
#include "BIT.h"

namespace impl {

	void CommandHandler::onRobotStateLog(const PackageInfo& info, const RobotStateLog& data, SendBuffer& reply) {
		// As a server, I dont handle these messages
	}
	
	void CommandHandler::onRobotCommand(const PackageInfo& info, const RobotCommand& data, SendBuffer& reply) {
		if (combined)
			combined->loginfo_if(DEBUG_INFO_FUNCTION, "[Combined] TCPCom Get Command {}-{}", (int)data.command, (int)data.command_ex);

		if (data.command == RobotCommand::Pause) {
			ControllerPause(1);
		}
		else if (data.command == RobotCommand::Resume) {
			ControllerPause(0);
		}
		else if (data.command == RobotCommand::StopAll) {
			ControllerSetFunction(FUNC_ALL, 0);
		}
		else if (data.command == RobotCommand::StopAuxTask) {
			ControllerSetFunction(0xff00, 0);
		}
		else if (data.command == RobotCommand::StopMotorTask) {
			ControllerSetFunction(0x00ff, 0);
		}
		else if (data.command == RobotCommand::TaskStart) {
			if (data.command == FUNC_TURN_TO) {
				PathPoint tar;
				tar.x = data.dataX;
				tar.y = data.dataY;
				ControllerSetTargetPoint(tar);
			}
			ControllerSetFunction((int)data.command_ex, 1);
		}
		else if (data.command == RobotCommand::TaskStop) {
			ControllerSetFunction((int)data.command_ex, 0);
		}
	}
	
	void CommandHandler::onRobotTeleMsg(const PackageInfo& info, const RobotTeleMsg& data, SendBuffer& reply) {
		InterventionInfo uH;
		uH.v = data.v;
		uH.w = data.w;
		ControllerSetHumanInput(uH);
	}
	
	void CommandHandler::onRobotConfigState(const PackageInfo& info, const RobotConfigState& data, SendBuffer& reply) {
		// TBD
	}
	
	void CommandHandler::onRobotConfigPath(const PackageInfo& info, const RobotConfigPath& data, SendBuffer& reply) {
		if (data.qx.size() != data.qy.size()) return;

		std::vector<PathPoint> path(data.qx.size());
		for (size_t i = 0; i < data.qx.size(); ++i) {
			path[i].x = data.qx[i];
			path[i].y = data.qy[i];
			path[i].v = 0;
			if (data.uniform_velocity != 0) path[i].v = data.uniform_velocity;
			else if (data.vd.size() == data.qx.size()) {
				path[i].v = data.vd[i];
			}
		}
		ControllerSetPath(path.size(), path.data());
	}
	
	void CommandHandler::onRobotConfigMap(const PackageInfo& info, const RobotConfigMap& data, SendBuffer& reply) {
		// TBD, currently, the obstacles are valid only for one time
		if (!combined) return;
		if (data.command == RobotConfigMap::MapReset) {
			combined->nsavoid.ob_env().delete_all();
		}
		NSAvoid::ObstacleMan<real_t>& ob_env = combined->nsavoid.ob_env();
		try {
			for (size_t i = 0; i < data.obPoint.size(); ++i) {
				const RobotConfigMap::Circle& one = data.obPoint[i];
				ob_env.add_point(to2X(one.q), one.r);
			}
			for (size_t i = 0; i < data.obLine.size(); ++i) {
				const RobotConfigMap::Line& one = data.obLine[i];
				ob_env.add_line(to2X(one.p), to2X(one.q));
			}
			for (size_t i = 0; i < data.obCPoly.size(); ++i) {
				const RobotConfigMap::CPoly& one = data.obCPoly[i];
				ob_env.add_poly(toMat2X(one));
			}
		}
		catch (...) {
			// add_XXX can throw exceptions... TBD: no exception throwling
			combined->loginfo("[Ctrl] [TCPCom] Error: Failed to add mapinfo. Invalid Data.");
		}
	}

	ExtTCPCom::ExtTCPCom()
		: tobuf(&get_seralizer_binary()), send_buffer(500) 
	{
		combined = nullptr;
		server.set_proto_binary();
		server.set_handler(&handler);
		server.setsocketopt_sndbuf(1000);  // TODO: add to config
		server.setsocketopt_rcvbuf(1000);  // TODO: add to config
	}

	bool ExtTCPCom::is_binded() {
		return addr_server.m_ip != 0 && addr_server.m_port != 0;
	}

	void ExtTCPCom::bind_and_listen() {
		// So, bind and listen works only for the first time
		if (!is_binded()) {
			NetworkAddress new_addr(server_ip.c_str(), server_port);
			if (server.bind_and_listen(new_addr) == 0) {
				// bind success
				addr_server = new_addr;
			}
		}
	}

	void ExtTCPCom::check_status() {
		server.remove_unreachable();
	}

	void ExtTCPCom::broadcast_state() {
		if (!combined) return;
		Combined* ctrl = combined;

		PackageInfo info;
		info.package_type = PackageInfo::PackRobotStateLog;
		info.senderID = ctrl->getMyID();
		info.targetID = -1;
		info.data_size = 0;

		RobotStateLog data;
		data.loop_counter = ctrl->lastTime.loopCounter;
		data.heading = (float)ctrl->lastData.th[0];
		data.task_motor = (std::uint8_t)ctrl->base_function();
		data.task_aux = 0;	// TODO
		data.x = (float)ctrl->lastData.xy[0].x;
		data.y = (float)ctrl->lastData.xy[0].y;
		data.heading = (float)rad2deg(ctrl->lastData.th[0]);
		data.uv = (float)ctrl->lastControl.v;
		data.uw = (float)ctrl->lastControl.w;
		data.xd = 0;
		data.yd = 0;
		int base_fun = ctrl->base_function();
		if (base_fun == FUNC_TRACKING_SINGLE) {
			AlgoTracking::Info stat = ctrl->tracking_single.compute_info();
			data.xd = (float)stat.tar.x;
			data.yd = (float)stat.tar.y;
		}
		else if (base_fun == FUNC_TRACKING_GROUP) {
			AlgoTracking::Info stat = ctrl->tracking.compute_info();
			data.xd = (float)stat.tar.x;
			data.yd = (float)stat.tar.y;
		}
		data.task_progess = ctrl->task_progress();

		WriteBuffer buf(send_buffer.end(), send_buffer.length_free());
		int nwrite = tobuf->write(buf, info, data);
		if (nwrite > 0) {
			send_buffer.push(nwrite);
			server.sendtoall(send_buffer.begin(), (size_t)send_buffer.length());
		}
		send_buffer.clear();
	}

	void ExtTCPCom::accept_and_recv() {
		server.accept_and_recv();
	}

	/*
	 * Str Communication Protocol
	 */
	ExtStrCom::ExtStrCom() {
		combined = nullptr;
		server_ip = "";
		server_port = 0;
		server.onData = &handler;
		server.setsocketopt_sndbuf(500);   // TODO: add to config
		server.setsocketopt_rcvbuf(2000);  // TODO: add to config
		std::memset((char*)send_buffer, 0, 200);
	}
	bool ExtStrCom::is_binded() {
		if (addr_server.m_ip != 0 && addr_server.m_port != 0) {
			return server.is_binded();
		}
		return false;
	}
	void ExtStrCom::bind_and_listen() {
		// So, bind and listen works only for the first time
		if (!is_binded()) {
			NetworkAddress new_addr(server_ip.c_str(), server_port);
			if (server.bind_and_listen(new_addr) == 0) {
				// bind success
				addr_server = new_addr;
			}
		}
	}
	int ExtStrCom::onMessage(const char* data, size_t size) {
		static SendBuffer zero_buf(0);
		size_t nparser = server.parser.pack.size();
		int num = 0;
		for (size_t i = 0; i < nparser; ++i) {
			PackBase* one = server.parser.pack[i];
			if (one) {
				num = one->read(data, size);
				if (num > 0) {
					handler.onPack(one, zero_buf);
					break;
				}
			}
		}
		if (num == size) return num;
		else if (num > 0) return num + onMessage(data + num, size - num);
		else return 0;
	}

	void ExtStrCom::check_status() {
		server.remove_unreachable();
	}
	void ExtStrCom::accept_and_recv() {
		server.accept_and_recv();
	}
	void ExtStrCom::broadcast_state() {
		if (!combined) return;
		Combined* ctrl = combined;

		state_info.reset();
		state_info.id() = ctrl->getMyID();
		state_info.x()  = int(ctrl->lastData.xy[0].x + 0.5);
		state_info.y()  = int(ctrl->lastData.xy[0].y + 0.5);
		state_info.th() = int(rad2deg(ctrl->lastData.th[0]) + 0.5);
		state_info.v()  = int(ctrl->lastControl.v + 0.5);
		state_info.w()  = int(ctrl->lastControl.w + 0.5);
		state_info.status() = ctrl->state();
		state_info.type() = ctrl->base_function();
		state_info.progress() = std::min(int(ctrl->task_progress() * 100 + 0.5), 100);
	
		int nwrite = state_info.write(send_buffer, 200);
		if (nwrite > 0) {
			server.sendtoall(send_buffer, nwrite);
		}

		
		conf::CtrlPack& com = ctrl->updateMyCtrlPack();
		if (com.validShape()) {
			std::uint8_t num_agent, shape_index, fmgroup; std::uint16_t priority;
			com.getShape(num_agent, shape_index, fmgroup, priority);
			algo_info.reset();
			algo_info.id() = ctrl->getMyID();
			algo_info.subType() = protobit::ALGO::shp;
			algo_info.nrobot() = (int)num_agent;
			algo_info.gpID() = (int)fmgroup;
			algo_info.shpIndex() = (int)shape_index;
			algo_info.priority() = (int)priority;

			nwrite = algo_info.write(send_buffer, 200);
			if (nwrite > 0) {
				server.sendtoall(send_buffer, nwrite);
			}
		}
		if (com.validTaskState()) {
			std::uint8_t task, state;
			com.getTaskState(task, state);
			algo_info.reset();
			algo_info.id() = ctrl->getMyID();
			algo_info.subType() = protobit::ALGO::info;
			algo_info.task() = (int)task;
			algo_info.state() = (int)state;

			nwrite = algo_info.write(send_buffer, 200);
			if (nwrite > 0) {
				server.sendtoall(send_buffer, nwrite);
			}
		}
		if (com.validIsFaulty()) {
			algo_info.reset();
			algo_info.id() = ctrl->getMyID();
			algo_info.subType() = com.withIsFaulty() ? protobit::ALGO::iamfault : protobit::ALGO::iamgood;

			nwrite = algo_info.write(send_buffer, 200);
			if (nwrite > 0) {
				server.sendtoall(send_buffer, nwrite);
			}
		}
	}
	void StrHandler::onPack(PackBase* pack, SendBuffer& reply) {
		if (!pack) return;
		DataHandler::onPack(pack, reply);
		if (false && combined) {
			combined->loginfo("Get package {}", pack->pack_name());
		}
	}

	void StrHandler::onSTAT(protobit::STAT& data, SendBuffer&) {
		// Do nothing, 
		if (!combined) return;
		if (data.id() == combined->getMyID()) return;

		StateInfo one;
		one.ID = data.id();
		one.x = (real_t)data.x();
		one.y = (real_t)data.y();
		one.v = (real_t)data.v();
		one.w = (real_t)data.w();
		one.heading = (real_t)data.th();
		combined->ext_stateinfo[one.ID] = one;
	}
	void StrHandler::onALGO(protobit::ALGO& data, SendBuffer&) {
		// Handle Algorithm Data
		if (!combined) return;

		conf::CtrlPack comrec;
		if (data.subType() == protobit::ALGO::shp) {
			comrec.setShape((uint8_t)data.nrobot(), (uint8_t)data.shpIndex(), (uint8_t)data.gpID, (uint16_t)data.priority());
		}
		if (data.subType() == protobit::ALGO::info) {
			//uint8_t state = data.status() == protobit::STAT::Running ? STATE_GOOD;
			comrec.setTaskState((uint8_t)data.task(), (uint8_t)data.state());
		}
		if (data.subType() == protobit::ALGO::iamfault) {
			comrec.withIsFaulty(true);
		}
		if (data.subType() == protobit::ALGO::iamgood) {
			comrec.withIsFaulty(false);
		}

		std::string log_msg;
		combined->handleCtrlPack(data.id(), comrec, log_msg);
	}
	void StrHandler::onCOMD(protobit::COMD& data, SendBuffer&) {
		if (!data.valid() || !combined) return;
		if (data.id() == combined->getMyID()) {
			int cmd = data.cmd();
			if (cmd == protobit::COMD::Stop) {
				combined->setFunction(FUNC_ALL, false);
				combined->loginfo("[Ctrl] [StrCom] COMD: Stop All.");
			}
			else if (cmd == protobit::COMD::Pause) {
				combined->setPause(1);
				combined->loginfo("[Ctrl] [StrCom] COMD: Task Paused.");
			}
			else if (cmd == protobit::COMD::Resume) {
				combined->setPause(0);
				combined->loginfo("[Ctrl] [StrCom] COMD: Task Resumed.");
			}
			else if (cmd == protobit::COMD::ConfigReload) {
				combined->setFunction(FUNC_ALL, false);
				((Controller*)combined)->loadconfig();
				combined->loginfo("[Ctrl] [StrCom] COMD: Stop All Functions and ConfigReload.");
			}
			else if (cmd == protobit::COMD::LogTraceBegin) {
				combined->enableLogging(LOG_FILE_TRACE, "TraceLog", false);
				combined->loginfo("[Ctrl] [StrCom] COMD: LogTraceBegin.");
			}
			else if (cmd == protobit::COMD::LogTraceStop) {
				combined->disableLogging(LOG_FILE_TRACE);
				combined->loginfo("[Ctrl] [StrCom] COMD: LogTraceStop.");
			}
			else if (cmd == protobit::COMD::DetectFault) {
				combined->setFunction(FUNC_FAULT_DETECT, 1);
				combined->loginfo("[Ctrl] [StrCom] COMD: DetectFault.");
			}
			else if (cmd == protobit::COMD::ActFaulty) {
				combined->setFunction(FUNC_ACT_FAULTY, 1);
				combined->loginfo("[Ctrl] [StrCom] COMD: ActFaulty.");
			}
		}
	}
	void StrHandler::onPATH(protobit::PATH& data, SendBuffer&) {
		if (!data.valid() || !combined) return;
		if (data.id() == combined->getMyID()) {
			combined->setFunction(FUNC_ALL, false);
			std::vector<PathPoint> path(data.n());
			for (int i = 0; i < data.n(); ++i) {
				path[i].x = (real_t)data.x[i]();
				path[i].y = (real_t)data.y[i]();
				path[i].v = (real_t)data.v[i]();
			}
			combined->setPath(data.n(), &path[0]);
			combined->setFunction(FUNC_TRACKING_SINGLE, true);
			combined->loginfo("[Ctrl] [StrCom] PATH: Starting PATH Tracking Task.");
		}
	}
	void StrHandler::onFORM(protobit::FORM& data, SendBuffer&) {
		static const std::vector<float> empty;

		if (!data.valid() || !combined) return;
		bool notme = true;
		int myID = combined->getMyID();
		for (int i = 0; i < data.nrobot(); ++i) {
			if (data.robot[i]() == myID) {
				notme = false; break;
			}
		}
		if (notme) return;

		std::vector<int> ids(data.nrobot());
		for (int i = 0; i < data.nrobot(); ++i) ids[i] = data.robot[i]();
		if (data.type() == protobit::FORM::TestEight) {
			combined->setFunction(FUNC_ALL, false);
			combined->setFormationShape(ids, empty, empty, cm2m(data.elen() * 1.0f));
			combined->setFunction(FUNC_TEST_EIGHT, true);
			combined->loginfo("[Ctrl] [StrCom] FORM: Starting FORM task (TestEight).");
			return;
		}
		if (data.type() == protobit::FORM::Basic) {
			int fun = combined->base_function();
			combined->setFunction(FUNC_ALL, false);
			combined->setFormationShape(ids, empty, empty, cm2m(data.elen() * 1.0f));
			combined->setFormationGroup(data.fmgroup());
			std::vector<PathPoint> path(data.npoint());
			for (int i = 0; i < data.npoint(); ++i) {
				path[i].x = (real_t)data.x[i]();
				path[i].y = (real_t)data.y[i]();
				path[i].v = (real_t)data.v[i]();
			}
			combined->setPath(data.npoint(), &path[0]);
			combined->setFunction(FUNC_TRACKING_GROUP, true);
			combined->loginfo("[Ctrl] [StrCom] FORM: Starting FORM task, {} point, {} length", data.npoint(), data.elen());
			return;
		}
	}
	void StrHandler::onEIGT(protobit::EIGT& data, SendBuffer&) {
		static const std::vector<float> empty;
		if (!data.valid() || !combined) return;
		protobit::ID me = combined->getMyID();
		if (std::find(data.ids.begin(), data.ids.end(), me) == data.ids.end()) {
			return;
		}

		std::vector<int> ids(data.n());
		for (int i = 0; i < data.n(); ++i) ids[i] = data.ids[i]();

		combined->setFunction(FUNC_ALL, false);
		combined->setFormationShape(ids, empty, empty, cm2m(data.len() * 1.0f));
		combined->testEight.vel = cm2m(data.v() * 1.0f);
		combined->testEight.theta = (real_t)data.dir();
		combined->testEight.R = cm2m(data.r() * 1.0f);
		combined->setFunction(FUNC_TEST_EIGHT, true);
		combined->loginfo("[Ctrl] [StrCom] EIGT: Starting EIGT task (TestEight).");
	}

	void StrHandler::onTURN(protobit::TURN& data, SendBuffer&) {
		if (!data.valid() || !combined) return;
		if (data.id() == combined->getMyID()) {
			int curr = combined->base_function();
			if (curr != FUNC_TURN_TO)
				combined->setFunction(FUNC_ALL, false);
			PathPoint qd;
			qd.x = (real_t)data.xd();
			qd.y = (real_t)data.yd();
			combined->setTargetPoint(qd);
			combined->setFunction(FUNC_TURN_TO, true);
			combined->loginfo("[Ctrl] [StrCom] TURN: Starting TURNTO task.");
		}
	}
	void StrHandler::onTDEG(protobit::TDEG& data, SendBuffer&) {
		if (!data.valid() || !combined) return;
		if (data.id() == combined->getMyID()) {
			int curr = combined->base_function();
			if (curr != 0 && curr != FUNC_TURN_TO)
				combined->setFunction(FUNC_ALL, false);
			combined->setTargetHeading((real_t)data.theta(), data.type() == protobit::TDEG::Relative);
			combined->setFunction(FUNC_TURN_TO, true);
			combined->loginfo("[Ctrl] [StrCom] TDEG: Starting TurnDEG task.");
		}
	}
	void StrHandler::onTELE(protobit::TELE& data, SendBuffer&) {
		if (!data.valid() || !combined) return;
		if (data.id() == combined->getMyID()) {
			int fun = combined->base_function();
			int tele_fun = FUNC_TELE_BASIC;
			if (data.type() == protobit::TELE::ByAlgoVxVy) {
				tele_fun = FUNC_TELE_DXY;
			}
			if (fun != tele_fun) {
				if (fun != 0) {
					combined->setFunction(FUNC_ALL, false);
				}
				combined->setFunction(tele_fun, true);
				combined->loginfo("[Ctrl] [StrCom] TELE: Starting TELE Task.");
			}
			InterventionInfo uH;
			uH.v = (real_t)data.v;
			uH.w = (real_t)data.w;
			combined->setHumanInput(uH);
		}
	}
	void StrHandler::onTALG(protobit::TALG& data, SendBuffer&) {
		if (!data.valid() || !combined) return;
		if (data.id() == combined->getMyID()) {
			InterventionInfo uH;
			uH.v = uH.w = 0;

			int fun = combined->base_function();
			int tele_fun = FUNC_TELE_BASIC;
			if (data.type() == protobit::TALG::ByVxy) {
				tele_fun = FUNC_TELE_DXY;
				data.get_vxy(uH.v, uH.w);
			}
			else {
				data.get_vw(uH.v, uH.w);
			}

			if (fun != tele_fun) {
				if (fun != 0) {
					combined->setFunction(FUNC_ALL, false);
				}
				combined->setFunction(tele_fun, true);
				combined->loginfo("[Ctrl] [StrCom] TALG: Starting TALG Task.");
			}
			combined->setHumanInput(uH);
		}
	}

	void StrHandler::onOBST(protobit::OBST& data, SendBuffer&) {
		if (!data.valid() || !combined) return;
		if (data.id() != combined->getMyID()) return;

		NSAvoid::ObstacleMan<real_t>& ob_env = combined->nsavoid.ob_env();
		if (data.n() == 0) {
			ob_env.delete_all();
			combined->loginfo("[Ctrl] [StrCom] OBST: obstatic cleared.");
			return;
		}
		try {
			const char* obtype = "Unknown";
			real_t unit = 100.0f;	// Note: NSAvoid operates on meters....
			if (data.type() == protobit::OBST::Circle) {
				obtype = "Circle";
				for (int i = 0; i < data.n(); ++i) {
					pt2D q(data.x[i]() / unit, data.y[i]() / unit);
					ob_env.add_point(to2X(pt2D(q)), (real_t)data.r[i]() / unit);
				}
			}
			if (data.type() == protobit::OBST::Rect) {
				obtype = "Rect";
				std::vector<pt2D> pts;
				for (int i = 0; i < data.n(); ++i) {
					pts.resize(5);
					for (int j = 0; j < 4; ++j) {
						int k = 4 * i + j;
						pts[j].x = data.x[k]() / unit;
						pts[j].y = data.y[k]() / unit;
					}
					pts[4] = pts[0];
					ob_env.add_poly(toMat2X(pts));
				}
			}
			combined->loginfo("[Ctrl] [StrCom] OBST: added {} {} obs", data.n(), obtype);
		}
		catch (...) {
			combined->loginfo("[Ctrl] [StrCom] Error: Failed to add mapinfo. Invalid Data.");
		}
	}

	void pathToPATH(const std::vector<PathPoint>& q, protobit::PATH& p) {
		int n = (int)q.size();
		p.n() = n;
		p.x.resize(n);
		p.y.resize(n);
		p.v.resize(n);
		for (int i = 0; i < n; ++i) {
			p.x[i]() = (int)(q[i].x+0.5f);
			p.y[i]() = (int)(q[i].y+0.5f);
			p.v[i]() = (int)(q[i].v+0.5f);
		}
	}

	int planRequestRoutine(StrHandler* strcom, int robotID, const std::vector<PathPoint>& path, SendBuffer& reply, bool autoRun) {
		protobit::PATH rep;
		rep.id() = robotID;
		rep.n() = 0;
		if (!path.empty()) {
			// Plan Result is Good
			pathToPATH(path, rep);
			if (autoRun && strcom) {
				static SendBuffer unused(0);
				strcom->onPATH(rep, unused);
			}
		}
		
		int nwrite = rep.write(reply.begin(), reply.length_free());
		if (nwrite > 0) reply.push(nwrite);
		return nwrite;
	}

	void StrHandler::onPLAN(protobit::PLAN& data, SendBuffer& reply) {
		if (!data.valid() || !combined) return;
		if (data.id() != combined->getMyID()) return;

		pt2D p0;
		pt2D pd(data.xd(), data.yd());

		if (data.type() == protobit::PLAN::Basic) p0 = pt2D(data.x0(), data.y0());
		else if (combined->lastData.empty()) {
			combined->loginfo("[Ctrl] [StrCom] Last Data is Missing in handling PLAN");
			return;
		}
		else {
			p0 = combined->lastData.xy[0];
		}
		
		std::vector<PathPoint> path;
		if (!combined->computePath(path, p0, pd, (real_t)data.v(), (real_t)data.d())) {
			path.clear();
		}
		bool autoRun = data.type() == protobit::PLAN::PlanSelfAndGO;
		int nwrite = planRequestRoutine(this, combined->getMyID(), path, reply, autoRun);
		if (nwrite <= 0) {
			combined->loginfo("[Ctrl] [StrCom] [PLAN] Reply Buffer Too Small For Sending PATH");
		}
	}

	void StrHandler::onMPLN(protobit::MPLN& data, SendBuffer& reply) {
		if (!data.valid() || !combined) return;
		if (data.id() != combined->getMyID()) return;

		std::vector<PathPoint> path, target;
		target.resize(data.n());
		for (int i = 0; i < data.n(); ++i) {
			target[i].x = (real_t)data.x[i]();
			target[i].y = (real_t)data.y[i]();
			target[i].v = (real_t)data.v[i]();
		}

		bool startAtMe = data.type() != protobit::MPLN::Basic;
		if (!combined->computePath(path, target, (real_t)data.d(), startAtMe)) {
			path.clear();
		}

		bool autoRun = data.type() == protobit::MPLN::PlanSelfAndGO;
		int nwrite = planRequestRoutine(this, combined->getMyID(), path, reply, autoRun);
		if (nwrite <= 0) {
			combined->loginfo("[Ctrl] [StrCom] [MPLN] Reply Buffer Too Small For Sending PATH");
		}
	}

} // namespace impl 
#endif // #if defined(ENABLE_SSNETWORK)
