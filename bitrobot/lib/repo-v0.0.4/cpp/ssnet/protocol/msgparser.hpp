#pragma once
#include "robotmsg.hpp"
#ifdef __QNX__
#include <stdint.h>
#else
#include <cstdint>
#endif


struct PackageInfo {
	enum {
		PackUnknown = 0,
		PackRobotStateLog,
		PackRobotCommand,
		PackRobotTeleMsg,
		PackRobotConfigState,
		PackRobotConfigPath,
		PackRobotConfigMap
	};
	
	PackageInfo(std::int8_t sender = -1, std::int8_t target = -1)
		: senderID(0), targetID(0), package_type(PackUnknown), data_size(0) {}
	std::int8_t senderID;
	std::int8_t targetID;
	std::int8_t package_type;
	std::uint32_t data_size;
};

template<class PointerT>
struct BufferBase {
	BufferBase(PointerT buffer, std::size_t size) 
		: buffer(buffer), size(size) {}
	BufferBase(PointerT begin, PointerT end) 
		: buffer(begin), size(end >= begin ? end - begin: 0) {}
	bool eat(std::size_t k) {
		if (k > size) return false;
		buffer += k; size -= k;
		return true;
	}
	PointerT buffer;
	std::size_t size;
};
typedef BufferBase<char*> WriteBuffer;
typedef BufferBase<const char*> ReadBuffer;

/*
struct UnionMessage {
	// Well, we cannot actually use union
	// since there are non-POD types
	RobotStateLog    state;
	RobotCommand     command;
	RobotTeleMsg     tele;
	RobotConfigState config_state;
	RobotConfigPath  config_path;
	RobotConfigMap   config_map;
};
*/

struct SerializerBase {
	virtual ~SerializerBase(){}
	virtual int write(WriteBuffer buffer, PackageInfo& info, const RobotStateLog& data)    { return -1; }
	virtual int write(WriteBuffer buffer, PackageInfo& info, const RobotCommand& data)     { return -1; }
	virtual int write(WriteBuffer buffer, PackageInfo& info, const RobotTeleMsg& data)     { return -1; }
	virtual int write(WriteBuffer buffer, PackageInfo& info, const RobotConfigState& data) { return -1; }
	virtual int write(WriteBuffer buffer, PackageInfo& info, const RobotConfigPath& data)  { return -1; }
	virtual int write(WriteBuffer buffer, PackageInfo& info, const RobotConfigMap& data)   { return -1; }
};

struct ParserBase {
	virtual ~ParserBase(){}
	
	// Return -1 : means that the current buffer contains no header data
	//        >=0: the index of the first recogonized header
	virtual int find_packet(ReadBuffer buffer, PackageInfo& info) { return -1; }
	
	virtual int read(ReadBuffer buffer, PackageInfo& info, RobotStateLog& data)    { return -1; }
	virtual int read(ReadBuffer buffer, PackageInfo& info, RobotCommand& data)     { return -1; }
	virtual int read(ReadBuffer buffer, PackageInfo& info, RobotTeleMsg& data)     { return -1; }
	virtual int read(ReadBuffer buffer, PackageInfo& info, RobotConfigState& data) { return -1; }
	virtual int read(ReadBuffer buffer, PackageInfo& info, RobotConfigPath& data)  { return -1; }
	virtual int read(ReadBuffer buffer, PackageInfo& info, RobotConfigMap& data)   { return -1; }
};

SerializerBase& get_seralizer_json();
ParserBase& get_parser_json();

SerializerBase& get_seralizer_binary();
ParserBase& get_parser_binary();



namespace msgparser_test {
	// Implemented in the robotmsg_test.cpp
	int test_binary_method();
	int test_json_method();
	
	// Implemented in the robotmsg_test_data.cpp
	void set_random_seed(std::uint32_t seed);
	std::uint32_t rand_u32();
	void random_data(RobotStateLog& data);
	void random_data(RobotCommand& data);
	void random_data(RobotTeleMsg& data);
	void random_data(RobotConfigState& data);
	void random_data(RobotConfigPath& data);
	void random_data(RobotConfigMap& data);
	void random_data(PackageInfo& data);
}



//
// Defining the comparsion functions for the message...
//
#define __fast_test(name) ( a.name == b.name ) 
#define __fast_test2(n1, n2) ( __fast_test(n1) && __fast_test(n2) )
#define __fast_test3(n1, n2, n3) ( __fast_test(n1) && __fast_test2(n2, n3) )
inline bool operator == (const RobotStateLog& a, const RobotStateLog& b) {
	return __fast_test(loop_counter) && __fast_test(task_motor)
		&& __fast_test(task_aux) && __fast_test3(x, y, heading)
		&& __fast_test2(uv, uw) && __fast_test2(xd, yd)
		&& __fast_test(task_progess);
}
inline bool operator != (const RobotStateLog& a, const RobotStateLog& b) {
	return !(a == b);
}
inline bool operator == (const RobotCommand& a, const RobotCommand& b) {
	return __fast_test2(command, command_ex) && __fast_test2(dataX, dataY);
}
inline bool operator != (const RobotCommand& a, const RobotCommand& b) {
	return !(a == b);
}
inline bool operator == (const RobotTeleMsg& a, const RobotTeleMsg& b) {
	return __fast_test2(v, w);
}
inline bool operator != (const RobotTeleMsg& a, const RobotTeleMsg& b) {
	return !(a == b);
}
inline bool operator == (const RobotConfigState& a, const RobotConfigState& b) {
	return __fast_test2(x, y) && __fast_test2(vx, vy) && __fast_test(heading);
}
inline bool operator != (const RobotConfigState& a, const RobotConfigState& b) {
	return !(a == b);
}
inline bool operator == (const RobotConfigPath& a, const RobotConfigPath& b) {
	return __fast_test2(pathType, uniform_velocity)
		&& __fast_test2(qx, qy) && __fast_test2(vd, wd);
}
inline bool operator != (const RobotConfigPath& a, const RobotConfigPath& b) {
	return !(a == b);
}
inline bool operator == (const RobotConfigMap::Point & a, const RobotConfigMap::Point & b) {
	return __fast_test2(x, y);
}
inline bool operator == (const RobotConfigMap::Circle& a, const RobotConfigMap::Circle& b) {
	return __fast_test2(q, r);
}
inline bool operator == (const RobotConfigMap::Line& a, const RobotConfigMap::Line& b) {
	return __fast_test2(p, q);
}
inline bool operator == (const RobotConfigMap& a, const RobotConfigMap& b) {
	return __fast_test(command)
		&& __fast_test3(obPoint, obLine, obCPoly);
}
inline bool operator != (const RobotConfigMap& a, const RobotConfigMap& b) {
	return !(a == b);
}
inline bool operator == (const PackageInfo& a, const PackageInfo& b) {
	return __fast_test2(senderID, targetID)
		&& __fast_test2(package_type, data_size);
}
inline bool operator != (const PackageInfo& a, const PackageInfo& b) {
	return !(a == b);
}
#undef __fast_test
#undef __fast_test2
#undef __fast_test3
