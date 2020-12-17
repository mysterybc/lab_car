#pragma once
#include <vector>
#include <string>
#include <map>

#ifdef __QNX__
#ifndef nullptr
#define nullptr NULL
#endif
#endif

/*
 * Simulation Data
*/
struct RobotState {
	RobotState(): x(0), y(0), theta(0) {}
	double x, y, theta;
};
struct RobotControl {
	RobotControl(): v(0), w(0) {}
	double v, w;
};
struct RobotData {
	RobotData(): controller_ptr(nullptr) {}
	RobotState   q_real, q_sense;
	RobotControl u_apply, u_compute;
	
	void* controller_ptr;
	std::string msg_out;
};
struct RobotComputeResult {
	RobotControl u_compute;
	std::string msg_out;
};
struct OneRecord {
	std::vector<RobotState>   q_real;
	std::vector<RobotControl> u_compute;
};
struct TimeData {
	TimeData(): timeElapsed(0), loopCounter(0){}
	unsigned int timeElapsed;
	unsigned int loopCounter;
};

/*
 * Configuartion Data
*/
struct PhysicSimuConfig {
	PhysicSimuConfig() : sensor_noise(0), actor_noise(0) {}
	double sensor_noise;
	double actor_noise;
};
struct RobotConfig {
	int robotID;
	std::vector<int> nebIndex;
	std::string config_path;
	std::string log_path;
	PhysicSimuConfig config;
	RobotState initial_state;
};
struct SimuConfig {
	int nRobot() const { return (int)robotID.size(); }
	std::vector<int> robotID;
	double simu_dt;
	double sleep_dt;
	PhysicSimuConfig physics_config;
	std::string simu_config_path;
};



/*
 * Components
*/

class PhysicSimulator {
public:
	RobotData operator() (const RobotData& curr, const PhysicSimuConfig& config, double dt);
	
private:
	double rand_value();
};



/*
 * Collection of All Data Components
*/
struct SimulationData {
	// Configuartion
	SimuConfig config_simu;
	
	// Data
	std::vector<RobotConfig> config_robot;
	std::vector<RobotData> data_robot;
	std::vector<RobotComputeResult> data_compute;
	TimeData current_time;
	
	// Components
	PhysicSimulator physical_update;	
};

// Main Functions
SimuConfig load_simuconfig(int argc, char* argv[]);
SimulationData initialization(const SimuConfig& config_simu);
void update_simulation(SimulationData& simu);
void clean_up(SimulationData& simu);


// Functions called by Main Functions
RobotConfig load_robotconfig(const SimuConfig& simu, int index);
RobotComputeResult update_bitagent(int index, SimulationData& simu);





