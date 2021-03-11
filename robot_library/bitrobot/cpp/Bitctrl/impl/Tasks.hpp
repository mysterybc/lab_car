#pragma once
#include "Algo.hpp"
#include "BIT.h"
#include "Base.hpp"
#include "ssconfig.hpp"

namespace impl {
	struct TaskAction {
		TaskAction() : state(STATE_GOOD), progress(0) {}

		int    state;
		real_t progress;
		wValid<pt2D> u_xy;
		wValid<ControlInfo> u_vw;
	};

	struct CommonUtil {
		typedef Controller LoggerT;

		CommonUtil() {
			logger = nullptr;
			pos_check.set_range(m2cm(1000.0f));
			vel_check.set_range(m2cm(10.0f));
			rot_check.set_range(360.0f);
			id_check.set_range(0, 10);
		}

		LoggerT* logger;
		RangeCheck<real_t> pos_check;
		RangeCheck<real_t> vel_check;
		RangeCheck<real_t> rot_check;
		RangeCheck<uint>   id_check;
	};

	class TaskBase {
	public:
		virtual ~TaskBase() {}
		virtual int taskType() const = 0;
		virtual bool load_config(const std::string& prefix, sscfg::ConfigFile& cfile) = 0;
		virtual void save_config(const std::string& prefix, std::ostream& out) = 0;

		virtual bool onSetFunction(bool is_on) = 0;
		virtual TaskAction compute(const TimeInfo& tm, const DataSet& dat) = 0;

		TaskBase(const char* name) : m_name(name), m_util(nullptr) {}
		const std::string& taskName() const    { return m_name; }
		CommonUtil& util()              { return m_util ? *m_util : m_default_util; }
		void setUtil(CommonUtil& util)  { m_util = &util; }
	private:
		std::string m_name;
		CommonUtil* m_util;
		static CommonUtil m_default_util;
	};


	class TaskTestMotion : public TaskBase {
	public:
		TaskTestMotion(const char* name) : TaskBase(name) {
			
		}

		int taskType() const { return FUNC_TEST_MOTION; }

		bool load_config(const std::string& prefix, sscfg::ConfigFile& cfile);
		void save_config(const std::string& prefix, std::ostream& out);
		bool onSetFunction(bool is_on);
		TaskAction compute(const TimeInfo& tm, const DataSet& dat);

		// Paramters
		real_t vmax, wmax;
		real_t rmax, tmax;
	private:

	};

}
