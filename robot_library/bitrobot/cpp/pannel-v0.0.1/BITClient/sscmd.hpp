#pragma once

#include "ssconfig.hpp"
#include <sstream>
#include <string>

namespace sscfg {

inline ConfigFile load_fromCMD(int argc, char* argv[]) {
	std::stringstream ss;
	for (int k = 1; k < argc; ++k) {
		std::string s = argv[k];
		if (s[0] == '-') {
			s = s.substr(1);
			ss << "\n" << s;
		}
		else {
			ss << "  " << s;
		}
	}
	ss << "\n";
	return sscfg::ConfigFile::load(ss);
}

inline std::string getAsOneString(ConfigFile& conf, const std::string& option) {
	std::vector<std::string> slist;
	conf.get(option, slist);
	if (slist.empty()) return "";

	std::string s = slist[0];
	for (unsigned i = 1; i < slist.size(); ++i) {
		s += " " + slist[i];
	}
	return s;
}

}