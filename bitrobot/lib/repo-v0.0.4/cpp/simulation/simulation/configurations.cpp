#include "simu_mas.hpp"
#include "sscmd.hpp"
#include "utility.hpp"
#include <cstdio>


std::string add_split(std::string s) {
	if (!s.empty()) {
		char c = s.back();
		if (c != '\\' && c != '/') s.append("\\");
	}
	return s;
}

SimuConfig load_simuconfig(int argc, char* argv[]) {
	SimuConfig config;
	config.simu_dt = 0.1;
	config.sleep_dt = 0.1;
	config.physics_config.sensor_noise = 0;
	config.physics_config.actor_noise = 0;
	config.simu_config_path = "";

	sscfg::ConfigFile cmdopt = sscfg::load_fromCMD(argc, argv);
	config.simu_config_path = add_split(sscfg::getAsOneString(cmdopt, "path"));
	if (!checkFileExist(config.simu_config_path + "simuConfig.txt")) {
		config.simu_config_path += "..\\";
	}
	cmdopt.get("rob", config.robotID);
	if (config.robotID.empty()) {
		for (int i = 0; i < 4; ++i) config.robotID.push_back(i + 1);
	}
	
	printf("config_path: %s\n", config.simu_config_path.c_str());
	printf("roblist: ");
	for (unsigned i = 0; i < config.robotID.size(); ++i) {
		printf(" %d", config.robotID[i]);
	}
	printf("\n");

	return config;
}




RobotConfig load_robotconfig(const SimuConfig& simu, int index) {
	// Default Configurations
	static const double pi = 3.141592653589793;
	std::string prefix = simu.simu_config_path;

	if (!checkFileExist(prefix + "simuConfig.txt")) {
		prefix += "..\\";
	}
	
	if (index < 0 || index >= 8) {
		std::puts("[Config] Invalid robot index.");
		return RobotConfig();
	}
	int gpIndex = index < 4 ? 0 : 1;

	RobotConfig config;
	config = RobotConfig();
	config.robotID = simu.robotID[index];
	for (int i = 0; i < (int)simu.robotID.size(); ++i) {
		if (i != index) config.nebIndex.push_back(i);
	}

	config.initial_state.x = 0;
	config.initial_state.y = 2;
	config.initial_state.theta = 0;	// Radian
	config.config_path = prefix;


	// Configurations By Config File
	std::ifstream conf_file(prefix+"simuConfig.txt");
	sscfg::ConfigFile con = sscfg::ConfigFile::load(conf_file);
	std::string name = "robot1";
	name[5] = '0' + config.robotID;
	if (con.exist(name)) {
		std::vector<float> data;
		con.get(name, data);
			
		config.initial_state.x = data[0] / 100.0;
		config.initial_state.y = data[1] / 100.0;
		config.initial_state.theta = data[2] / 180.0 * pi;
	}
	
	return config;
}
