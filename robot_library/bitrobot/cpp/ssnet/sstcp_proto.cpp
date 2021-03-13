#include "sstcp_proto.hpp"
#include <cstdio>

bool ProtoServer::handle_data_with_reply(RecvBuffer& data, SendBuffer& reply) {
	PackageInfo info;
	ReadBuffer tmp(data.begin(), (std::size_t)data.length());
	int begin = parser->find_packet(tmp, info);
	if (begin != -1) {
		data.eat(begin);	// Data before the first header are ignored
		if (data.length() >= (int)info.data_size) {
			// The package is complete
			process.parse_package(parser, info, ReadBuffer(data.begin(), (int)info.data_size), reply);
			data.eat((int)info.data_size);
			return data.length() > 0;
		}
	}
	return false;
}

bool ProtoClient::handle_data(RecvBuffer& data) {
	static SendBuffer reply(0);
	PackageInfo info;
	ReadBuffer tmp(data.begin(), (std::size_t)data.length());
	int begin = parser->find_packet(tmp, info);
	if (begin != -1) {
		data.eat(begin);	// Data before the first header are ignored
		if (data.length() >= (int)info.data_size) {
			// The package is complete
			process.parse_package(parser, info, ReadBuffer(data.begin(), (int)info.data_size), reply);
			data.eat((int)info.data_size);
			if (reply.length() != 0) {
				puts("Warning: ProtoClient do not rely. reply data discarded");
				reply.clear();
			}
			return data.length() > 0;
		}
	}
	return false;
}


void DataProcesser::parse_package(ParserBase* parser, const PackageInfo& info, ReadBuffer buffer, SendBuffer& reply) {
	int pack_type = (int)info.package_type;
	switch (pack_type) {
	case PackageInfo::PackRobotStateLog: 
		if (parser->read(buffer, m_data.info, m_data.state) == (int)info.data_size) {
			if (onData) onData->onRobotStateLog(info, m_data.state, reply);
			return;
		}
		break;
	case PackageInfo::PackRobotCommand: 
		if (parser->read(buffer, m_data.info, m_data.cmd) == (int)info.data_size) {
			if (onData) onData->onRobotCommand(info, m_data.cmd, reply);
			return;
		}
		break;
	case PackageInfo::PackRobotTeleMsg: 
		if (parser->read(buffer, m_data.info, m_data.tele) == (int)info.data_size) {
			if (onData) onData->onRobotTeleMsg(info, m_data.tele, reply);
			return;
		}
		break;
	case PackageInfo::PackRobotConfigState: 
		if (parser->read(buffer, m_data.info, m_data.config_state) == (int)info.data_size) {
			if (onData) onData->onRobotConfigState(info, m_data.config_state, reply);
			return;
		}
		break;
	case PackageInfo::PackRobotConfigPath: 
		if (parser->read(buffer, m_data.info, m_data.config_path) == (int)info.data_size) {
			if (onData) onData->onRobotConfigPath(info, m_data.config_path, reply);
			return;
		}
		break;
	case PackageInfo::PackRobotConfigMap: 
		if (parser->read(buffer, m_data.info, m_data.config_map) == (int)info.data_size) {
			if (onData) onData->onRobotConfigMap(info, m_data.config_map, reply);
			return;
		}
		break;
	default: break;
	}
	puts("Parse Package Error");
}
