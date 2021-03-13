#pragma once
#if defined(ENABLE_SSNETWORK)
#include "ssnet/sstcp_proto.hpp"
#include "ssnet/sstcp_protobit.hpp"

namespace impl {
	class Controller;
	class Combined;

	class CommandHandler : public DataHandler {
	public:
		CommandHandler(Combined* ctrl = nullptr) :combined(ctrl) {}
		void onRobotStateLog(const PackageInfo& info, const RobotStateLog& data, SendBuffer& reply);
		void onRobotCommand(const PackageInfo& info, const RobotCommand& data, SendBuffer& reply);
		void onRobotTeleMsg(const PackageInfo& info, const RobotTeleMsg& data, SendBuffer& reply);
		void onRobotConfigState(const PackageInfo& info, const RobotConfigState& data, SendBuffer& reply);
		void onRobotConfigPath(const PackageInfo& info, const RobotConfigPath& data, SendBuffer& reply);
		void onRobotConfigMap(const PackageInfo& info, const RobotConfigMap& data, SendBuffer& reply);

		Combined* combined;
	};

	class ExtTCPCom {
	public:
		ExtTCPCom();
		void setController(Controller* controller, const std::string& name) {
			if (name == "Combined") {
				combined = (Combined*)controller;
				handler.combined = combined;
			}
		}

		void check_status();
		void broadcast_state();

		bool is_binded();
		void bind_and_listen();
		void accept_and_recv();
		ProtoServer& get_server() { return server; }

		std::string server_ip;
		int server_port;
	private:
		Combined* combined;
		SerializerBase* tobuf;
		CommandHandler handler;
		ProtoServer server;
		NetworkAddress addr_server;
		SendBuffer send_buffer;
	};


	class StrHandler : public protobit::DataHandler {
	public:
		StrHandler(Combined* ctrl = nullptr) :combined(ctrl) {}
		
		void onPack(PackBase* pack, SendBuffer& reply);
		void onSTAT(protobit::STAT& data, SendBuffer&);
		void onCOMD(protobit::COMD& data, SendBuffer&);
		void onPATH(protobit::PATH& data, SendBuffer&);
		void onTURN(protobit::TURN& data, SendBuffer&);
		void onOBST(protobit::OBST& data, SendBuffer&);
		void onTELE(protobit::TELE& data, SendBuffer&);
		void onTALG(protobit::TALG& data, SendBuffer&);
		void onTDEG(protobit::TDEG& data, SendBuffer&);
		void onFORM(protobit::FORM& data, SendBuffer&);
		void onEIGT(protobit::EIGT& data, SendBuffer&);
		void onALGO(protobit::ALGO& data, SendBuffer&);
		void onPLAN(protobit::PLAN& data, SendBuffer&);
		void onMPLN(protobit::MPLN& data, SendBuffer&);
		Combined* combined;
	};

	class ExtStrCom {
	public:
		ExtStrCom();
		void setController(Controller* controller, const std::string& name) {
			if (name == "Combined") {
				combined = (Combined*)controller;
				handler.combined = combined;
			}
		}

		void check_status();
		void broadcast_state();

		bool is_binded();
		void bind_and_listen();
		void accept_and_recv();
		protobit::BITServer& get_server() { return server; }

		int onMessage(const char* data, size_t size);

		std::string server_ip;
		int server_port;
	private:
		Combined* combined;
		protobit::STAT state_info;
		protobit::ALGO algo_info;
		StrHandler handler;
		protobit::BITServer server;
		NetworkAddress addr_server;
		char send_buffer[200];
	};


} // namespace impl 

#endif