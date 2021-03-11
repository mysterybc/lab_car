#include "simu_mas.hpp"
#include "utility.hpp"
#include <queue>
//#include "ssnet/sstcp_protobit.hpp"


void update_simulation(SimulationData& simu) {
	SimuConfig& config_simu = simu.config_simu;
	double dt = config_simu.simu_dt;
	
	// Compute Actions
	int nRobot = (int)simu.data_robot.size();
	for (int i=0; i<nRobot; ++i) {
		simu.data_compute[i] = update_bitagent(i, simu);
	}
		
	// Apply Actions
	const PhysicSimuConfig& physics_config = config_simu.physics_config;
	for (int i=0; i<nRobot; ++i) {
		simu.data_robot[i].msg_out = simu.data_compute[i].msg_out;
		
		simu.data_robot[i].u_compute = simu.data_compute[i].u_compute;
		simu.data_robot[i] = 
			simu.physical_update(simu.data_robot[i], physics_config, dt);
	}
	
	// Update Time Info
	simu.current_time.timeElapsed += (int)(config_simu.simu_dt * 1000);
	simu.current_time.loopCounter += 1;
	
	// Sleep
	int sleep_ms = (int)(config_simu.sleep_dt * 1000);
	if (sleep_ms > 0) {
		my_sleepms(sleep_ms);
	}
}


