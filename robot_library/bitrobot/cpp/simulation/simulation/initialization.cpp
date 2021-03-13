#include "simu_mas.hpp"
#include "BIT.h"
#include "utility.hpp"
#include <cstdlib>

#ifdef __QNX__
#ifndef nullptr
#define nullptr NULL
#endif
#endif


SimulationData initialization(const SimuConfig& config_simu) {
/*
	double lat = 39.90920055555556;
	double lon = 116.26366478611111;
	float lat2dX_cm, lon2dY_cm, th;
	float tmpx, tmpy, tmpth;

	GPSSetAnchor(lat, lon, 0);
	GPS2Local(lat + 1e-6, lon, 0, &lat2dX_cm, &tmpy, &tmpth);
	GPS2Local(lat, lon + 1e-6, 0, &tmpx, &lon2dY_cm, &tmpth);

	printf("lat2dX_cm = %.4f, lon2dY_cm = %.4f\n", lat2dX_cm, lon2dY_cm);

	real_t xd = 12.6*100, yd = -20*100;
	double latd = lat + (xd / lat2dX_cm) * 1e-6;
	double lond = lon + (yd / lon2dY_cm) * 1e-6;
	printf("new anchor: lat = %.8f, lon = %.8f\n", latd, lond);

	GPSSetAnchor(latd, lond, 0);
	GPS2Local(lat, lon, 0, &tmpx, &tmpy, &tmpth);
	printf("in new anchr: (x, y) = %.4f, %.4f\n", tmpx, tmpy);

	system("pause");
	exit(1);
	*/
	SimulationData simu;
	
	// Get Simulation Configurations
	simu.config_simu = config_simu;
	int nRobot = simu.config_simu.nRobot();
	
	// Initialize SimulationData
	simu.config_robot.resize(nRobot);
	simu.data_robot.resize(nRobot);
	simu.data_compute.resize(nRobot);
	simu.current_time = TimeData();
	simu.physical_update = PhysicSimulator();
	
	// Get Robot Configurations
	for (int i=0; i<nRobot; ++i) {
		simu.config_robot[i] = load_robotconfig(simu.config_simu, i);
	}
	
	// Set Initial States of the Robots
	for (int i=0; i<nRobot; ++i) {
		RobotData one;
		one.q_real = simu.config_robot[i].initial_state;
		one.q_sense = one.q_real;
		one.msg_out.resize(80, '\0');
		
		simu.data_robot[i] = one;
	}
	
	// Initialize the Controller Component
	for (int i=0; i<nRobot; ++i) {
		const RobotConfig& config = simu.config_robot[i];
		
		ControllerSetConfigDir(config.config_path.c_str());
		ControllerSetLogDir(config.log_path.c_str());
		ControllerInit(EXP_GENERAL, simu.config_robot[i].robotID);
		ControllerSetDebugInfo(DEBUG_INFO_STATES | DEBUG_INFO_FUNCTION, 1);
		simu.data_robot[i].controller_ptr = ControllerGetPtr();
		ControllerSetPtr(nullptr);
	}
	
	return simu;
}

std::vector<PathPoint> random_path(int num, PathPoint p0, real_t step_length = 1.0f, real_t vel = 0.5f) {
	std::vector<PathPoint> path(num + 1);
	path[0] = p0;
	for (int i = 1; i <= num; ++i) {
		double dx = (((double)std::rand() / RAND_MAX) - 0.5) * step_length;
		double dy = (((double)std::rand() / RAND_MAX) - 0.5) * step_length;
		path[i].x = path[i - 1].x + (real_t)m2cm(dx);
		path[i].y = path[i - 1].y + (real_t)m2cm(dy);
		path[i].v = m2cm(vel);
	}
	return path;
}

void initialize_task(SimulationData& simu) {
	int nRobot = simu.config_simu.nRobot();
	for (int i = 0; i < nRobot; ++i) {
		ControllerSetPtr(simu.data_robot[i].controller_ptr);

		//ControllerSetPath();

		ControllerSetPtr(nullptr);
	}
}

