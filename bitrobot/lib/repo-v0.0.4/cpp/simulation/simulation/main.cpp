#include <cstdio>
#include <string>
#include "simu_mas.hpp"
#include "utility.hpp"

int main(int argc, char* argv[]) {
	std::puts("--- Initializing Simulation ---");
	SimulationData simu = initialization(load_simuconfig(argc, argv));
	
	std::puts("--- Simulation Running ---");
	while (ms2sec(simu.current_time.timeElapsed) < 36000) {
		update_simulation(simu);
	}
	clean_up(simu);
	std::puts("--- Simulation Ended ---");
	return 0;
}

