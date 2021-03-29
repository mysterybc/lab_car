#include "Combined.hpp"
#include <algorithm>
#include <string>

bool print_debug = false;

namespace impl{
	double distance_pts(const pt2D& p, const pt2D& q){
		return norm2(p - q);
	}
	double distance_angle(const double& a, const double& b){
		return std::abs(anglediff(a, b));
	}

	inline void
	add_to_config(sscfg::ConfigSet& config, NSAvoid::ObstacleMan<real_t>& ob, const std::string& name) {
		config.addarg(ob.bound().min_avoid, name+"_dmin");
		config.addarg(ob.bound().K, name+"_vmax");
		config.addarg(ob.config().value_max, name+"_safe");
	}

	inline void
	add_to_config(sscfg::ConfigSet& config, AlgoTracking& tracking, const std::string& name){
		config.addarg(tracking.KNorml, name + "_kn");
		config.addarg(tracking.KTagent, name + "_kt");
		config.addarg(tracking.szDZoneENormal, name + "_ndzone");
		config.addarg(tracking.errThresh, name + "_ethresh");
		config.addarg(tracking.pathFollow, name + "_strict");
	}

	inline void
	add_to_config(sscfg::ConfigSet& config, TestEightConfig& eight, const std::string& name) {
		config.addarg(eight.enabled, name + "_enable");
		config.addarg(eight.R,      name + "_r");
		config.addarg(eight.vel,    name + "_v");
		config.addarg(eight.theta,  name + "_th");
		config.addarg(eight.tStay,  name + "_t");
		config.addarg(eight.nRound, name + "_n");
		config.addarg(eight.nPoint, name + "_np");
	}

	Combined::Combined(int robotID):Controller(robotID){
		//useAvoid = true;
		initState = 0;
		runtimeState = 0;
		functionState = FUNC_UNSET;
		current_progress = 0;
		targetTime = 0;
		fwcomp.v = fwcomp.w = 0;
		humanScale = 1.0f;
		thresh_tracking = thresh_consen = 10.0f;
		thresh_th = 1.0f;
		iamfaulty = false;
		dumpNSAvoid = false;
		enableLogging(LOG_FILE_INFO, "AlgoCombined", true);
		ob_size_robot = 0.6f;
		ob_size_human = 0.6f;
		ob_size_envob = -1.0f;
		DebugShowAllState = false;
		BadMsgCounter = 0;
		GoodMsgCounter = 0;
		MisSizeCounter = 0;
		human_virtual = false;
		use_slowdown = true;
		default_vpath = 80.0f;
		wFault = 60.0f;
		nsavoid_lamV = 10;
		nsavoid_2D = true;
		nsavoid_vw = false;
		teleDirectVxy.setValid(false);
		teleDirect.setValid(false);
		reach_position.type = ConditionChecker::Count_Good;
		reach_heading.type = ConditionChecker::Count_Good;

		ftd_fault_type = 0;
		ftd_tp1_v = 0;
		ftd_tp1_w = 30;
		ftd_tp2_v = 30;
		ftd_tp2_t = 5;

		formation_group = 0; // Default Formation Group, added 2020/10/16

		// About path velocity
		config.addarg(default_vpath,   "gen_vpath");

		// About Criterion on reaching the target
		config.addarg(thresh_consen,   "gen_thresh_consensus");
		config.addarg(thresh_tracking, "gen_thresh_tracking");
		config.addarg(thresh_th,       "gen_thresh_theta");

		// About Latency Test
		config.addarg(latency_test.dump_every, "latTestDump");
		config.addarg(latency_test.tick_max, "latTestMax");

		// About NSAvoid
		// add ob_xxx_dmin/vmax/safe
		add_to_config(config, nsavoid.ob_static(), "ob_static");
		add_to_config(config, nsavoid.ob_agent(), "ob_agent");
		add_to_config(config, nsavoid.ob_env(), "ob_dynamic");
		add_to_config(config, nsavoid.ob_sensor(), "ob_sensor");
		config.addarg(nsavoid_2D, "gen_nsavoid_2D");
		config.addarg(nsavoid_vw, "gen_nsavoid_vw");
		config.addarg(ob_size_robot, "ob_size_robot");
		config.addarg(ob_size_human, "ob_size_human");
		config.addarg(ob_size_envob, "ob_size_envob");
		config.addarg(nsavoid_lamV, "gen_nsavoid_lamV");

		// About Tracking Control
		add_to_config(config, tracking, "gen_track");
		add_to_config(config, tracking_single, "gen_tsingle");
		tracking.pline = &lineXY;
		tracking_single.pline = &lineXY;
		config.addarg(reach_position.limit, "gen_track_endreach_pos");
		config.addarg(reach_heading.limit, "gen_track_endreach_th");

		// About Consensus & HeadingAlignment
		config.addarg(consenRend.K,   "gen_consn_rnd");
		config.addarg(consenTrack.K,  "gen_consn_trk");
		config.addarg(alignRend.K,    "gen_align_rnd");
		config.addarg(alignLocal.K,   "gen_align_loc");
		config.addarg(alignTarget.K,  "gen_align_tar");
		// dq, wmax unset

		// About HumanField
		config.addarg(humanScale,     "gen_hscale");
		config.addarg(human_field.Kc, "gen_hfkc");
		config.addarg(human_field.Kt, "gen_hfkt");
		config.addarg(human_field.decRatioPS, "gen_hf_dec_ratio");
		config.addarg(human_field.decScalePS, "gen_hf_dec_scale");
		config.addarg(human_field.tHumanMax,  "gen_hf_tmax");
		config.addarg(human_field.normalize,  "gen_hf_normalize");
		config.addarg(human_field.vmax,       "gen_hf_vmax");
		human_field.pHfNet = &hfNet;

		// About FaultDetect
		config.addarg(wFault,                "gen_actfault_w");
		config.addarg(fault_xy.bound_type,   "gen_ftd_xy_type");
		config.addarg(fault_xy.detect_bound, "gen_ftd_xy_bound");
		config.addarg(fault_xy.min_time,     "gen_ftd_xy_mtime");
		config.addarg(fault_xy.err_decay,	 "gen_ftd_xy_decay");
		config.addarg(fault_th.bound_type,   "gen_ftd_th_type");
		config.addarg(fault_th.detect_bound, "gen_ftd_th_bound");
		config.addarg(fault_th.min_time,     "gen_ftd_th_mtime");
		config.addarg(fault_th.err_decay,	 "gen_ftd_th_decay");
		fault_xy.DistanceFunction = &distance_pts;      //[](const pt2D& p, const pt2D& q){ return norm2(p - q); };
		fault_th.DistanceFunction = &distance_angle;    //[](const double& a, const double& b){ return std::abs(anglediff(a, b)); };

		// About Act Faulty
		config.addarg(ftd_fault_type, "gen_ftd_ft_type");
		config.addarg(ftd_tp1_v, "gen_ftd_tp1_v");
		config.addarg(ftd_tp1_w, "gen_ftd_tp1_w");
		config.addarg(ftd_tp2_v, "gen_ftd_tp2_v");
		config.addarg(ftd_tp2_t, "gen_ftd_tp2_t");

		// About AlgoSlowDown
		config.addarg(use_slowdown,    "gen_use_slowdown");
		config.addarg(slow_down.errLevel, "gen_slow_down");
		config.addarg(slow_down.lamTan,   "gen_slow_lamT");
		config.addarg(slow_down.lamNor,   "gen_slow_lamN");

		// About Test Eight
		add_to_config(config, testEight, "gen_test8");

		// About Tracer (General)
		/*
		config.addarg(tracer.acc_max,   "gen_wchu_accmax");
		config.addarg(tracer.vel_dzone, "gen_wchu_vdzone");
		config.addarg(tracer.Khmn,		"gen_wchu_Khmn");
		config.addarg(tracer.Kneb,		"gen_wchu_Kneb");
		config.addarg(tracer.Kobs,		"gen_wchu_Kobs");

		// About Tracer's potential field for agents
		config.addarg(tracer.leader_update.K,    "gen_wchu_leader_K");
		config.addarg(tracer.leader_update.vmax, "gen_wchu_leader_vmax");

		config.addarg(tracer.pf_agent.react_range, "gen_wchu_pf_range");
		config.addarg(tracer.pf_ob.react_range,    "gen_wchu_pf_range");
		config.addarg(tracer.pf_agent.desired_dis, "gen_wchu_pfagent_d");
		config.addarg(tracer.pf_agent.k,		   "gen_wchu_pfagent_k");
		config.addarg(tracer.pf_agent.kv,		   "gen_wchu_pfagent_kv");
		config.addarg(tracer.pf_agent.c,		   "gen_wchu_pfagent_c");
		config.addarg(tracer.pf_agent.esp,         "gen_wchu_pfagent_e");
		config.addarg(tracer.pf_ob.k,			   "gen_wchu_pfob_k");
		config.addarg(tracer.pf_ob.kv,			   "gen_wchu_pfob_kv");
		config.addarg(tracer.pf_ob.c,              "gen_wchu_pfob_c");
		config.addarg(tracer.pf_ob.esp,            "gen_wchu_pfob_e");
		config.addarg(tracer.pf_human.k,           "gen_wchu_pfhmn_k");
		config.addarg(tracer.pf_human.k,           "gen_wchu_pfhmn_c");
		*/

		// About algorithm swithces
		iamfaulty = false;
		use_nsavoid = true;
		use_pfavoid = false;
		use_autofault = false;
		use_fakevel = false;
		config.addarg(use_nsavoid, "gen_use_nsavoid");
		config.addarg(use_pfavoid, "gen_use_pfavoid");
		config.addarg(use_autoselect, "gen_use_autoselect");
		config.addarg(use_autofault,  "gen_use_autofault");
		config.addarg(use_fakevel, "gen_use_fakevel");
		
		// About Shape Selection
		shape_selected    = false;
		selected_counter  = 0;

		// Last Control Message (Used to generate fake vel)
		lastControl.setValid(false);
		//setFunction(FUNC_LATENCY_TEST, true);
	}

	int Combined::loadconfig_debug(sscfg::ConfigFile& file){
		std::vector<int> id;
		file.get("IDshowAll", id);
		DebugShowAllState = (std::find(id.begin(), id.end(), getMyID()) != id.end());
		return 0;
	}

	template<class T>
	std::vector<T> filter_in(const std::vector<T>& tar, const std::vector<T>& filter) {
		std::vector<T> ret;
		for (size_t i = 0; i < tar.size(); ++i){
			if (std::find(filter.begin(), filter.end(), tar.at(i)) != filter.end()) {
				ret.push_back(tar.at(i));
			}
		}
		return ret;
	}

	template<class Scalar>
	void regular_polygon(unsigned int n, std::vector<Scalar>& dx, std::vector<Scalar>& dy) {
		dx.resize(n);
		dy.resize(n);
		if (n == 1) {
			dx[0] = (Scalar)0;
			dy[0] = (Scalar)0;
		}
		else if (n >= 2) {
			const Scalar pi = (Scalar)(3.141592654);
			Scalar theta = (Scalar)(2*pi / n);
			Scalar R = (Scalar)(0.5 / std::sin(theta/2));
			Scalar th = (n % 2 == 0) ? (pi / 2 - theta / 2) : (pi / 2);
			for (unsigned int i = 0; i < n; ++i) {
				dx[i] = R*std::cos(th);
				dy[i] = R*std::sin(th);
				th += theta;
			}
		}
	}

	int Combined::loadconfig_shape(sscfg::ConfigFile& file){
		// Config Formation
		std::vector<int>   id;
		std::vector<float> dx, dy;
		float dxy_scaling = 1.0f;
		std::string dxy_name;
		file.get("gen_dxy_id", id);
		file.get("gen_dxy_x", dx);
		file.get("gen_dxy_y", dy);
		file.get("gen_dxy_name", dxy_name);		// Temporarily not using
		file.get("gen_dxy_s", dxy_scaling);

		return setFormationShape(id, dx, dy, dxy_scaling) ? 0 : 1; // 0: no error, 1: error, failed to load
	}

	bool Combined::setFormationShape(const std::vector<int>& id, std::vector<float> dx, std::vector<float> dy, float dxy_scaling) {
		bool dxy_good = false;
		size_t num = id.size();
		if (dx.size() == num || dy.size() == num) {
			dxy_good = (dx.size() == dy.size());
		}
		else if (id.size() > 0) {
			// Default, edge_length is 1 meter
			regular_polygon(id.size(), dx, dy);
			dxy_good = true;
		}

		if (dxy_good) {
			//dxy_base.resize(num);
			dxy_base.clear();		// BUG FIXED: 2018/04/18
			dxyRemap.clear();
			for (size_t i = 0; i < num; ++i) {
				// dxy_x\y are in meters, but dxy is in cm
				pt2D dq = pt2D(m2cm(dx[i]), m2cm(dy[i])) * dxy_scaling;
				dxy_base[id[i]] = dq;
				dxyRemap[id[i]] = id[i];
				//dxy_base[i] = dq;
			}
			dxy_desire = dxy_base;
			onFormationChange();

			std::string idnames;
			for (size_t i=0;i<num;++i) {
				idnames += fmt::format(" {}", id[i]);
			}

			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] setFormationShape: gp={}, idsize={}, id={}", getFormationGroup(), id.size(), idnames);
			return true;
		}
		return false;
	}

	int Combined::loadconfig_planning(sscfg::ConfigFile& file) {
		bool withplan = false;
		file.get("withplan", withplan);
		if (withplan) {
			if (!file.get("obgrid_x0", obgrid.x0)) return 1;
			if (!file.get("obgrid_y0", obgrid.y0)) return 1;
			if (!file.get("obgrid_x1", obgrid.x1)) return 1;
			if (!file.get("obgrid_y1", obgrid.y1)) return 1;
			if (!file.get("obgrid_len", obgrid.edge_len)) return 1;
			
			obgrid.init_obgrid(mapinfo, "obstacle.bmp");
		}
		return 0;
	}
	int Combined::loadconfig(sscfg::ConfigFile& file){
		// This calls the parent's loadconfig on ConfigFile
		int non_exist = Controller::loadconfig(file);
		if (non_exist > 0)
			return non_exist;

		// Config Planner
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[CTRL] [Combined] load_config_planner");
		loadconfig_planning(file);

		// Config Debug Info
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[CTRL] [Combined] load_config_debug");
		loadconfig_debug(file);

		// Config shape
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[CTRL] [Combined] load_config_shape");
		loadconfig_shape(file);

		// Config Fault Detect
		std::vector<int> no_detect;
		file.get("gen_dont_detect", no_detect);
		dont_detect.clear();	// BUG FIXED: 2018/04/25
		for (size_t i = 0; i < no_detect.size(); ++i){
			dont_detect.insert(no_detect[i]);
		}

		// Config Latency Test
		std::vector<int> latID;
		int latStatWinSize = 600;
		bool latTestValid = latency_test.valid();
		file.get("latTestID", latID);
		file.get("latTestStatWindow", latStatWinSize);
		//latID = filter_in(latID, id);				// FIX ME: (or not?) That there should be one ID using for all...
		latency_test.setID(getMyID(), latID);
		latency_test.setStatWindow(latStatWinSize);
		latency_test.setDumpPath(Controller::logDir);
		if (!latTestValid)
			latency_test.reset();

		// If I should be faulty when fault detect is on
		if (use_autofault){
			std::vector<int> aut_fault;
			file.get("gen_auto_faulty", aut_fault);
			auto_faulty.clear();	// BUG FIXED: 2018/04/25
			for (size_t i = 0; i < aut_fault.size(); ++i){
				auto_faulty.insert(aut_fault[i]);
			}
		}

		// Config Tracer
		human_virtual = false;
		file.get("gen_human_virtual", human_virtual);
		float shift_human[2] = { 0, 0 };
		if (!human_virtual) {
			file.get("gen_shift_human", shift_human, 2);
		}
		//tracer.human_shift = pt2D{ shift_human[0], shift_human[1] };
		tracer_default_shift = pt2D{ shift_human[0], shift_human[1] };
		

		// Config & Initialize NSAvoid
		if (use_nsavoid){
			real_t len = 0.1f;
			file.get("gen_nsavoid_len", len);
			nsavoid.set_unicycle_point(len);
			LOGDEBUG("use nsavoid len = {}", len);

			// Add static map info to nsavoid
			if (mapinfo.size() > 0) {
				NSAvoid::ObstacleMan<real_t>& ob_static = nsavoid.ob_static();
				ob_static.delete_all();
				for (size_t i = 0; i < mapinfo.obPoint.size(); ++i) {
                    MapData::Circle& one = mapinfo.obPoint[i];
					ob_static.add_point(to2X(one.q), one.r);
				}
				for (size_t i = 0; i < mapinfo.obLine.size(); ++i) {
					MapData::Line& one = mapinfo.obLine[i];
					ob_static.add_line(to2X(one.p), to2X(one.q));
				}
				for (size_t i = 0; i < mapinfo.obCPoly.size(); ++i) {
					MapData::CPoly& one = mapinfo.obCPoly[i];
					ob_static.add_poly(toMat2X(one));
				}
				nsavoid.ob_static_changed();
				LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[CTRL] [Combined] {} static obstacles added to NSAvoid", ob_static.size());
			}
		}
		//char log_prefix[] = "0-";
		//log_prefix[0] += getMyID();
		//nsavoid.prob.dump_path = logDir+std::string(log_prefix);
		//LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[CTRL] [Combined] NSAvoid is dumped at {}", logDir);

		// Print out an example config
		std::string out_dir = logDir + "config_example.txt";
		std::ofstream out(out_dir.c_str(), std::ios_base::trunc);
		if (out.is_open()){
			config.write(out);
			out.close();
		}
		
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[CTRL] [Combined] Config file loaded");
		return non_exist;
	}
	

}   // namespace impl
