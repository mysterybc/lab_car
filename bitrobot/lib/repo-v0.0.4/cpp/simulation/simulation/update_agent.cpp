#include "simu_mas.hpp"
#include "BIT.h"
#include "utility.hpp"

#ifdef __QNX__
#ifndef nullptr
#define nullptr NULL
#endif
#endif

StateInfo build_stateinfo(int ID, const RobotData& data) {
	const RobotState&   q = data.q_sense;
	const RobotControl& u = data.u_compute;
		
	StateInfo state;
	state.ID = (unsigned int)ID;
	state.x = (real_t)(m2cm(q.x));
	state.y = (real_t)(m2cm(q.y));
	state.heading = (real_t)(rad2deg(q.theta));
	state.v = (real_t)(m2cm(u.v));
	state.w = (real_t)(rad2deg(u.w));
	return state;
}
TimeInfo build_timeinfo(const TimeData& current_time) {
	TimeInfo t;
	t.timeElapsed = current_time.timeElapsed;
	t.loopCounter = current_time.loopCounter;
	t.globalTime  = current_time.timeElapsed;
	return t;
}
RobotControl from_controlinfo(const ControlInfo& u) {
	RobotControl u2;
	u2.v = cm2m(u.v);
	u2.w = deg2rad(u.w);
	return u2;
}

void make_empty(std::string& s, unsigned int n) {
	s.resize(n);
	for (unsigned  i=0;i<n;++i) {
		s[i] = 0;
	}
}


RobotComputeResult update_bitagent(int index, SimulationData& simu) {
	RobotComputeResult result;
	
	const std::vector<RobotData>& data = simu.data_robot;
	const std::vector<RobotConfig>& config = simu.config_robot;
	
	// Tell Libbitctrl to use the data related to this controller
	ControllerSetPtr(data[index].controller_ptr);
	
	// First, handle message
	std::size_t nebs = config[index].nebIndex.size();
	for (std::size_t i=0; i<nebs; ++i) {
		int nebIndex = config[index].nebIndex[i];
		int nebID = config[nebIndex].robotID;
		const std::string& msg = data[nebIndex].msg_out;
		ControllerHandleMsg(nebID, (void*)msg.c_str(), msg.size());
	}
	
	// Then construct the needed vectors
	std::vector<StateInfo> others(nebs);
	for (std::size_t i=0; i<nebs; ++i) {
		int nebIndex = config[index].nebIndex[i];
		others[i] = build_stateinfo(config[nebIndex].robotID, data[nebIndex]);
	}
	StateInfo me = build_stateinfo(config[index].robotID, data[index]);
	TimeInfo tm = build_timeinfo(simu.current_time);
	
	// Compute control signal

	ControlInfo u = ControllerCompute(tm, me, nebs, &others.front());
	result.u_compute = from_controlinfo(u);
	
	// Compute Message
	std::string& msg_out = result.msg_out;
	make_empty(msg_out, 256);
	int num = ControllerGetMsg((void*)msg_out.data());
	msg_out.resize(num);
	
	// Reset Controller Pointer
	ControllerSetPtr(nullptr);
	
	return result;
}



void clean_up(SimulationData& simu) {
	int nRobot = simu.config_simu.nRobot();
	for (int i = 0; i < nRobot; ++i) {
		ControllerSetPtr(simu.data_robot[i].controller_ptr);
		ControllerStop();
		ControllerSetPtr(nullptr);
		simu.data_robot[i].controller_ptr = nullptr;
	}
}
