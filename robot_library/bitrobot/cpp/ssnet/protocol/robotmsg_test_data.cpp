#include "msgparser.hpp"
#include <cstdlib>
#include <climits>
#include "../mersenne/mersenne.h"

static MersenneTwister rand_mt;

namespace msgparser_test {	
	void set_random_seed(std::uint32_t seed) {
		rand_mt = MersenneTwister(seed);
	}
	std::uint32_t rand_u32() {
		return rand_mt();
	}

	template<class T>
	void randval(T& value);
	
	template<> 
	void randval(int& value) { 
		value = (int)rand_u32();
	}
	template<> 
	void randval(std::uint32_t& value) { 
		value = (std::uint32_t)rand_u32();
	}
	template<> 
	void randval(std::uint8_t& value) { 
		value = (std::uint8_t)(rand_u32() % 0xff);
	}
	template<> 
	void randval(std::int8_t& value) { 
		value = (std::int8_t)(rand_u32() % 0xff);
	}
	template<> 
	void randval(std::uint16_t& value) { 
		value = (std::uint16_t)(rand_u32() % 300);
	}
	template<> 
	void randval(float& value) { 
		value = (float)((double)rand_u32() / (UINT_MAX / 2) - 1);
	}
	template<>
	void randval(RobotConfigMap::Point& value) {
		randval(value.x); randval(value.y);
	}
	template<>
	void randval(RobotConfigMap::Circle& value) {
		randval(value.q); randval(value.r);
	}
	template<>
	void randval(RobotConfigMap::Line& value) {
		randval(value.p); randval(value.q);
	}
	template<class T>
	void randval(std::vector<T>& value) {
		int nValue = rand_u32() % 200;
		value.resize(nValue);
		for (int i = 0; i < nValue; ++i) {
			randval(value[i]);
		}
	}
	
	void random_data(RobotStateLog& data) {
		randval(data.loop_counter);
		randval(data.task_motor);
		randval(data.task_aux);
		randval(data.x);
		randval(data.y);
		randval(data.heading);
		randval(data.uv);
		randval(data.uw);
		randval(data.xd);
		randval(data.yd);
		randval(data.task_progess);
	}
	
	void random_data(RobotCommand& data) {
		randval(data.command);
		randval(data.command_ex);
		randval(data.dataX);
		randval(data.dataY);
	}
	
	void random_data(RobotTeleMsg& data) {
		randval(data.v);
		randval(data.w);
	}
	
	void random_data(RobotConfigState& data) {
		randval(data.x);
		randval(data.y);
		randval(data.vx);
		randval(data.vy);
		randval(data.heading);
	}
	
	void random_data(RobotConfigPath& data) {
		randval(data.pathType);
		randval(data.uniform_velocity);
		randval(data.qx);
		randval(data.qy);
		randval(data.vd);
		randval(data.wd);
	}
	
	void random_data(RobotConfigMap& data) {
		randval(data.command);
		randval(data.obPoint);
		randval(data.obLine);
		randval(data.obCPoly);
	}
	
	void random_data(PackageInfo& data) {
		randval(data.senderID);
		randval(data.targetID);
		randval(data.package_type);
		randval(data.data_size);
	}
}