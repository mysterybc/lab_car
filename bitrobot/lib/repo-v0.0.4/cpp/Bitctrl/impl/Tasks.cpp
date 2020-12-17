#include "Tasks.hpp"

namespace impl {

#define LOGDEBUG_TASK(...) do { if (util().logger) util().logger->loginfo_if(__VA_ARGS__); } while(0)

std::string getConfigPrefix(const std::string& prefix, const std::string& taskName) {
	if (prefix.empty() && taskName.empty()) return "unnamed_";
	if (prefix.empty()) return taskName + "_";
	if (taskName.empty()) return prefix + "_";
	return prefix + std::string("_") + taskName + "_";
}

CommonUtil TaskBase::m_default_util;

bool TaskTestMotion::load_config(const std::string& prefix, sscfg::ConfigFile& cfile) {
	std::string pre = getConfigPrefix(prefix, taskName());

	
	return true;
}
void TaskTestMotion::save_config(const std::string& prefix, std::ostream& out) {
	std::string pre = getConfigPrefix(prefix, taskName());


}

bool TaskTestMotion::onSetFunction(bool is_on) {
	if (is_on) {
		
	}
	return false;
}
TaskAction TaskTestMotion::compute(const TimeInfo& tm, const DataSet& dat) {
	return TaskAction();
}




}