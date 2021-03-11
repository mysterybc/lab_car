#pragma once
#include "sstcp.hpp"
#include "protocol/robotmsg.hpp"
#include "protocol/msgparser.hpp"

struct PacketDataAll {
	PackageInfo info;
	RobotStateLog state;
	RobotCommand  cmd;
	RobotTeleMsg  tele;
	RobotConfigState config_state;
	RobotConfigPath  config_path;
	RobotConfigMap   config_map;
};

struct DataHandler {
	virtual void onRobotStateLog(const PackageInfo& info, const RobotStateLog& data, SendBuffer& reply) {}
	virtual void onRobotCommand(const PackageInfo& info, const RobotCommand& data, SendBuffer& reply) {}
	virtual void onRobotTeleMsg(const PackageInfo& info, const RobotTeleMsg& data, SendBuffer& reply) {}
	virtual void onRobotConfigState(const PackageInfo& info, const RobotConfigState& data, SendBuffer& reply) {}
	virtual void onRobotConfigPath(const PackageInfo& info, const RobotConfigPath& data, SendBuffer& reply) {}
	virtual void onRobotConfigMap(const PackageInfo& info, const RobotConfigMap& data, SendBuffer& reply) {}
};

struct DataProcesser {
	DataProcesser() : onData(nullptr) {}
	void parse_package(ParserBase* parser, const PackageInfo& info, ReadBuffer buffer, SendBuffer& reply);

	DataHandler* onData;
	PacketDataAll m_data;
};


class ProtoServer: public TCPServer {
public:
	ProtoServer()
		: parser(nullptr), serializer(nullptr) {}
	
	void set_proto_binary() {
		parser = &get_parser_binary();
		serializer = &get_seralizer_binary();
	}
	void set_handler(DataHandler* handler) { process.onData = handler; }

protected:
	// Inherited virtual functions
	void handle_accept_with_reply(const NetworkAddress& addr, SendBuffer& reply) {}
	bool handle_data_with_reply(RecvBuffer& data, SendBuffer& reply);

	// Impl Data
	DataProcesser process;
	ParserBase* parser;
	SerializerBase* serializer;
};


class ProtoClient : public TCPClient {
public:
	ProtoClient()
		: parser(nullptr), serializer(nullptr) {}

	void set_proto_binary() {
		parser = &get_parser_binary();
		serializer = &get_seralizer_binary();
	}
	void set_handler(DataHandler* handler) { process.onData = handler; }

protected:
	// Inherited virtual functions
	bool handle_data(RecvBuffer& data);

	// Impl Data
	DataProcesser process;
	ParserBase* parser;
	SerializerBase* serializer;
};

namespace sstcp_proto_test {
	int test_local();
	int test_separate();
}