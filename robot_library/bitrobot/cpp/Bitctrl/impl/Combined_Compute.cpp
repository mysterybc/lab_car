#include "Combined.hpp"
#include <algorithm>
#include <cstring>

namespace impl{

	std::string func_name(int funcID);

	void Combined::refresh_obstacles(const DataSet& dat) {
		typedef fAlgoNSAvoid::Vec2X Vec2X;
		typedef fAlgoNSAvoid::VecX VecX;

		real_t robot_radius = ob_size_robot;
		real_t human_radius = ob_size_human;
		real_t unit = 100.0f;

		nsavoid.set_ulimit(maxVel()/unit, deg2rad(maxRotate()));
		nsavoid.ob_agent().delete_all();
		for (size_t i = 1; i < dat.size(); ++i) {
			//real_t radius = dat.ID[i] > dat.ID[0] ? robot_radius * 2 : robot_radius;
			real_t radius = robot_radius;
			nsavoid.ob_agent().add_point(to2X(dat.xy[i] / unit), radius);
			//nsavoid.ob_agent().set_velocity(i - 1, to2X(dat.vxy[i] / unit));
		}
		
		/*
		 * Commented out by Chengsi Shang 2020/09/11, 
		 * ob_env has been used to receive info from strcom
		 */
		/*
		 * Add a new interface of ob_sensor by Chengsi Shang 2020-10-31
		 */
		nsavoid.ob_sensor().delete_all();
		for (size_t i = 0; i < obXY.size(); ++i){
			if (ob_size_envob < 0) {
				nsavoid.ob_sensor().add_point(to2X(obXY[i] / unit), obR[i] / unit);
			}
			else{
				nsavoid.ob_sensor().add_point(to2X(obXY[i] / unit), ob_size_envob);
			}
		}
		
		if (base_function() == FUNC_TRACE_HUMAN && human_pos && !human_virtual) {
			nsavoid.ob_agent().add_point(Vec2X(human_pos.x/unit, human_pos.y/unit), human_radius);
		}
	}

	ControlInfo Combined::project_safe(const ControlInfo& u, const pt2D& q, double theta) {
		typedef fAlgoNSAvoid::Vec2X Vec2X;
		typedef fAlgoNSAvoid::VecX VecX;
		typedef fAlgoNSAvoid::Result Result;
		real_t unit = 100.0f;

		nsavoid.update_state(to2X(q/unit), (real_t)theta);
		nsavoid.set_metric(nsavoid_lamV, 1);
		Result ret = nsavoid.compute(Vec2X(u.v/unit, deg2rad(u.w)));
#ifdef _WIN32_xxx
		// These are meant for Debuging the NSAvoid Algorithm
		if (false) {
			printf("[%d] nActive = %d\n", ID, ret.n_active());
			if (ret.n_active() > 0) {
				for (int i = 0; 0 && i < ret.n_active(); ++i) {
					int index_qp = ret.ret_qp.active_set(i);
					int info_index = ret.info_index(index_qp);
					auto& info = ret.info[info_index];
					auto& grad = info.grad;
					auto& grad_dt = info.grad_dt;
					auto& value = info.value;
					auto& lb = info.vlim.vmin;
					auto& ub = info.vlim.vmax;

					printf("[%d] %dth - Active, qp_index = %d, index_from = %d, info_index = %d\n",
						ID, i, index_qp, info.index_from, info_index);
					printf("        value = %.2f, grad = (%.2f, %.2f), grad_dt = %.2f\n",
						value, grad(0), grad(1), grad_dt);
				}
				printf("     nTotal = %d\n", nsavoid.prob.g.size());
				for (int i = 0; i < ret.info.size(); ++i) {
					auto& info = ret.info[i];
					printf("[%d] %d-th cons, from %d-th cons, active = %s\n", ID, i, info.index_from, (info.valid ? "ON" : "OFF"));
					if (info.valid) {
						auto& grad = info.grad;
						auto& grad_dt = info.grad_dt;
						auto& value = info.value;
						auto& lb = info.vlim.vmin;
						auto& ub = info.vlim.vmax;
						printf("        value = %.2f, grad = (%.2f, %.2f), grad_dt = %.2f\n",
							value, grad(0), grad(1), grad_dt);
					}
				}
			}
		}
#endif
		ControlInfo u2;
		u2.v = ret.best_x()(0) * unit;
		u2.w = rad2deg(ret.best_x()(1));
		if (!ret.good()) {
			u2.v = 0;
			u2.w = 0;
		}
		return u2;
	}

	pt2D Combined::project_safe(const pt2D& u, const pt2D& q) {
		typedef fAlgoNSAvoid::Vec2X Vec2X;
		typedef fAlgoNSAvoid::VecX VecX;
		typedef fAlgoNSAvoid::Result Result;
		real_t unit = 100.0f;

		nsavoid.update_state_as_point(to2X(q/unit));
		nsavoid.set_metric(1, 1);
		Result ret = nsavoid.compute(to2X(u/unit));
		pt2D u2;
		u2.x = ret.best_x()(0) * unit;
		u2.y = ret.best_x()(1) * unit;
		if (!ret.good()) {
			u2.x = 0;
			u2.y = 0;
		}
		return u2;
	}

	void Combined::log_nsavoid(const DataSet& dat, const ControlInfo& u, const ControlInfo& u2, const pt2D& u2D){
		// TODO: Add logging info to help debug the NSAvoid algorithm
	}

	std::string idset2str(const std::set<int>& id){
		std::string s;
		std::set<int>::const_iterator it = id.begin();
		for (; it != id.end(); ++it) {
			s += fmt::format(" {}", *it);
		}
		return s;
	}

	template<class StateT>
	std::string fdtect_loginfo(const AlgoFaultDetect<StateT>& fd) {
		static const char* name_acc = "acc";
		static const char* name_avr = "avr";
		char e = fd.bound_reached() ? 'y' : 'n';
		char t = fd.mintime_reached() ? 'y' : 'n';
		const char* name = fd.bound_type == AlgoFaultDetect<StateT>::bound_acc ? name_acc : name_avr;
		
		return fmt::format("({}) err={:.2f}({}), t={:.2f}({})", name, fd.acc_err, e, fd.acc_time, t);
	}

	ControlInfo Combined::innerCompute(TimeInfo tm, DataSet& dat){
		if (DebugMode != 0){
			LOGDEBUGLV2(DEBUG_INFO_STATES, "[Ctrl] IN Debug Mode {}", DebugMode);
		}
		
		// Print Comm Info
		print_com_stat();
		MisSizeCounter = 0;
		BadMsgCounter = 0;
		GoodMsgCounter = 0;

		// Print States
		LOGDEBUGLV2(DEBUG_INFO_STATES, "[Ctrl] State ID={}: ({:.3f}, {:.3f}), {:.2f}",
						dat.ID[0], dat.xy[0].x/100, dat.xy[0].y/100, impl::rad2deg(dat.th[0]));
		if (DebugShowAllState){
			for (size_t i = 1; i < dat.size(); ++i){
				LOGDEBUGLV2(DEBUG_INFO_STATES, "       State {}: ({:.3f}, {:.3f}), {:.2f}",
					dat.ID[i], dat.xy[i].x / 100, dat.xy[i].y / 100, impl::rad2deg(dat.th[i]));
			}
		}

		if (base_function() == FUNC_TRACE_HUMAN){
			if (human_pos){
				LOGDEBUGLV2(DEBUG_INFO_STATES, "       Human : ({:.3f}, {:.3f})", human_pos.x/100, human_pos.y/100);
			}
			else{
				LOGDEBUGLV2(DEBUG_INFO_STATES, "       Human : (Invalid)");
			}
		}
		if (enableLatencyTest()) {
			latency_test.onNextStep();
			std::string lat_info = fmt::format("       LatTick {}", latency_test.currTick());
			if (latency_test.is_master()) {
				lat_info += fmt::format("/{}, ", latency_test.tick_max);
				const std::vector<int>& lat_id = latency_test.IDList();
				for (size_t i = 0; i < lat_id.size(); ++i) {
					if (lat_id[i] != getMyID()) {
						lat_info += fmt::format(" {}:({},{:.1f})",
							lat_id[i], latency_test.stat[i].latest, latency_test.stat[i].average());
					}
				}
			}
			LOGDEBUGLV2(DEBUG_INFO_STATES, lat_info);
		}

		if (use_fakevel){
			dat.vw[0].x = lastControl.v;
			dat.vw[0].y = deg2rad(lastControl.w);
		}

		// Add current obstacles to nsavoid
		refresh_obstacles(dat);
		if (!errList.empty()){
			LOGDEBUGLV2(DEBUG_INFO_STATES, "[Ctrl] Filtered out:{}", idset2str(errList));
			dat.filterOut(errList);
		}

		pt2D u2D = innerCompute2D(tm, dat);
		if (use_nsavoid && nsavoid_2D) {
			u2D = project_safe(u2D, dat.xy[0]);
		}

		ControlInfo u = fromVxy2VW(dat.th[0], u2D);
		u.v += fwcomp.v;
		u.w += fwcomp.w;

		// Apply NSAvoid, which is based on vw // && base_function() != FUNC_TRACE_HUMAN
		bool is_direct_tele = base_function() == FUNC_TELE_BASIC;
		if (use_nsavoid && (nsavoid_vw || is_direct_tele)) {
			u = project_safe(u, dat.xy[0], dat.th[0]);
		}

		// Saturation
		u = saturate(u, maxVel(), maxRotate());

		// Fault Detect
		if (enableFDetect() && dont_detect.find(ID) == dont_detect.end()){
			// I sould detect if myself is faulty
			real_t dt = (real_t)((tm.timeElapsed - lastTime.timeElapsed) / 1000.0);
			if (last_predict.valid()){
				pt2D   xy(last_predict.x, last_predict.y );
				double th = deg2rad(last_predict.heading);
				bool err_xy = fault_xy(dt, dat.xy[0], xy);
				bool err_th = fault_th(dt, dat.th[0], th);
				
				LOGDEBUGLV2(DEBUG_INFO_FDETECT, "[Ctrl] Fd-xy: {}", fdtect_loginfo(fault_xy));
				LOGDEBUGLV2(DEBUG_INFO_FDETECT, "       Fd-th: {}", fdtect_loginfo(fault_th));

				iamfaulty = err_xy || err_th;
				if (iamfaulty){
					runtimeState = STATE_FAULT_DETECTED;
				}
			}

			// Generate new predictions
			StateInfo now;
			now.x = (real_t) dat.xy[0].x;
			now.y = (real_t) dat.xy[0].y;
			now.heading = (real_t) rad2deg(dat.th[0]);
			now.v = (real_t)u.v;	now.w = (real_t) u.w;
			last_predict = predict_next(dt, now, u);
			last_predict.setValid();
		}

		if (isfaulty()){
			ControlInfo faulty = { 0, 0 };
			real_t dt = (tm.timeElapsed - tFautlStart.timeElapsed) / 1000.0f;
			if (dt <= 20) {
				if (ftd_fault_type == 0) {
					faulty.v = 0;
					faulty.w = 0;
				}
				if (ftd_fault_type == 1) {
					faulty.v = ftd_tp1_v;
					faulty.w = ftd_tp1_w;
				}
				if (ftd_fault_type == 2) {
					int k = int(dt / ftd_tp2_t);
					if (k % 2 == 0) {
						faulty.v = -ftd_tp2_v;
						faulty.w = 0;
					}
					else {
						faulty.v = ftd_tp2_v;
						faulty.w = 0;
					}
				}
			}
			LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Faulty] Computed ({}, {})", faulty.v, faulty.w);
			lastControl.setValid(false);
			return faulty;
		}
		else{
			LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [{}] Computed ({:.2f}, {:.2f}), u2D = ({:.2f}, {:.2f}), fw=({:.2f}, {:.2f}) ", 
				func_name(base_function()), u.v, u.w, u2D.x, u2D.y, fwcomp.v, fwcomp.w);
			LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl]      Current me = ({:.2f}, {:.2f}, {:.2f})", dat.xy[0].x, dat.xy[0].y, rad2deg(dat.th[0]));
			lastControl = u;
			lastControl.setValid(true);
			return u;
		}
	}

	
	pt2D Combined::innerCompute2D(TimeInfo tm, const DataSet& dat){
		pt2D ulocal, u;
		ulocal.x = ulocal.y = 0; u = ulocal;
		fwcomp.v = fwcomp.w = 0;

		int base_type = base_function();
		if (base_type == FUNC_TRACE_HUMAN) {
			if (onSelectFormation(dat)){
				onFormationChange();
			}
			if (shape_selected) {
				if (human_pos) {
					info = FormationInfo(dxy_desire, dat);
					pt2D dir = impl::normalize(human_pos - info.centerPhy);
					pt2D center = human_pos - impl::norm2(tracer_default_shift)*100.0f*dir;
					//pt2D center = human_pos - tracer_default_shift*100.0f;

					ulocal = consenRend(dat.xy[0], center);

					pt2D   tar = center + dxy_desire[ID];
					double err = norm2(tar - dat.xy[0]);
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Tracer] et, u_rend, tar, hmn: {}, {}, ({}, {}), ({}, {})",
						err, norm2(ulocal), tar.x, tar.y, human_pos.x, human_pos.y);

					pt2D rec_local = ulocal;
					ulocal = slow_down(err, ulocal, impl::normalize(tar - dat.xy[0]));
					if (slow_down.lastLamNor < 0.9 || slow_down.lastLamTan < 0.9) {
						LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Tracer] Slowing down, lamTan/Nor = {:.2f}, {:.2f}", slow_down.lastLamTan, slow_down.lastLamNor);
					}

					current_progress = 0.99f * std::exp((float)(-err));
					if (err < thresh_consen) {
						ulocal = pt2D();
						pt2D center_diff = human_pos - dat.xy[0] + dxy_desire[ID];
						fwcomp.w = alignRend((real_t)dat.th[0], center_diff);
						fwcomp.w = rad2deg(fwcomp.w);
						fwcomp.v = 0;
						current_progress = 0.99f;
					}
				}
				else {
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Tracer] No valid human position");

				}
			}
			else{
				// Shape Not Selected
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Tracer] Shape unselected");
			}
		} // if (base_type == FUNC_TRACE_HUMAN)
		else if (base_type == FUNC_RENDEZVOUS) {
			if (onSelectFormation(dat)){
				onFormationChange();
			}
			if (shape_selected){
				info = FormationInfo(dxy_desire, dat);
				pt2D center = info.center;
				ulocal = consenRend(dat.xy[0], center);

				pt2D   tar = center + dxy_desire[ID];
				double err = norm2(tar - dat.xy[0]);
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Rendezvous] et, u_rend, tar: {}, {}, ({}, {})", err, norm2(ulocal), tar.x, tar.y);

				pt2D rec_local = ulocal;
				ulocal = slow_down(err, ulocal, impl::normalize(tar - dat.xy[0]));
				if (slow_down.lastLamNor < 0.9 || slow_down.lastLamTan < 0.9) {
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Rendezvous] Slowing down, lamTan/Nor = {:.2f}, {:.2f}", slow_down.lastLamTan, slow_down.lastLamNor);
				}

				current_progress = 0.99f * std::exp((float)(-err));
				if (info.posError.max < thresh_consen) {
					ulocal = pt2D();
					fwcomp.w = alignRend((real_t)dat.th[0], (real_t)info.centerTh);
					fwcomp.w = rad2deg(fwcomp.w);
					fwcomp.v = 0;
					current_progress = 0.99f;

					if (info.thError.max < deg2rad(thresh_th)){
						fwcomp.v = 0;
						fwcomp.w = 0;
						current_progress = 1.0f;
						runtimeState = STATE_TARGET_REACHED;
						LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Rendezvous] Target reached");
					}
				}
			}
			else{
				// Shape Not Selected
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Rendezvous] Shape unselected");
			}
		} // else if (base_type == FUNC_RENDEZVOUS) 
		else if (base_type == FUNC_TRACKING_SINGLE) {
			//tracking_single.pathFollow = true;
			//tracking_single.errThresh = maxVel() * 1.0f;
			if (!lineXY.empty()){
				//real_t t = tracking_t; //(real_t)(tm.timeElapsed / 1000.0);
				real_t t = (real_t)(tm.timeElapsed / 1000.0);
				ulocal = tracking_single(t, dat.xy[0]);
				AlgoTracking::Info res = tracking_single.compute_info();
				const pt2D& tar = res.tar;
				double err = res.err;
				if (res.delayed) {
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TSingle] [Waiting] t = {:.2f}s, tDelayed = {:.2f}s, errThresh = {:.2f}",
								tracking_single.t_tracking, t - tracking_single.t_tracking, tracking_single.errThresh);
				}
				else {
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TSingle] [Normal] t = {:.2f}s, tDelayed = {:.2f}s",
						tracking_single.t_tracking, t - tracking_single.t_tracking);
				}
				current_progress = (real_t)(lineXY.percentage(t) * 0.99);

				std::string taskName = target_point.valid() ? "[TSingle] [Ex]" : "[TSingle]";
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] {} err: {:.2f}, tar: ({:.2f}, {:.2f}), u: ({:.2f}, {:.2f})",
					taskName, err, tar.x, tar.y, ulocal.x, ulocal.y);
				
				if (lineXY.reachEnd()){
					fwcomp.v = 0;
					fwcomp.w = 0;
					double dis = norm2(dat.xy[0] - 100 * lineXY(t));

					pt2D rec_local = ulocal;
					ulocal = slow_down(err, ulocal, impl::normalize(tar - dat.xy[0]));
					if (slow_down.lastLamNor < 0.9 || slow_down.lastLamTan < 0.9) {
						LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TSingle] Slowing down, lamTan/Nor = {:.2f}, {:.2f}", slow_down.lastLamTan, slow_down.lastLamNor);
					}

					if (dis < thresh_tracking) {
						if (target_point.valid()) {
							// Look at the target
							ulocal = pt2D();
							pt2D err = target_point - dat.xy[0];
							real_t th = (real_t)std::atan2(err.y, err.x);
							fwcomp.w = alignRend((real_t)dat.th[0], th);
							fwcomp.w = rad2deg(fwcomp.w);
							fwcomp.v = 0;
							LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TSingle] [TurnTo] TargetAt: ({:.2f}, {:.2f}), thTar: {:.1f}, w: {:.1f}", 
								target_point.x, target_point.y, rad2deg(th), fwcomp.w);
							if (std::abs(anglediff(th, (real_t)dat.th[0])) < deg2rad(thresh_th)) {
								fwcomp.w = 0;
								current_progress = 1.0f;
								runtimeState = STATE_TARGET_REACHED;
								LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TSingle] [TurnTo] Target Reached");
							}
						}
						else {
							current_progress = 1.0f;
							runtimeState = STATE_TARGET_REACHED;
							ulocal = pt2D();
							LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TSingle] Target Reached");
						}
					}
				}
			}
			else {
				// lineXY empty
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TSingle] LineXY empty");
			}
		} // else if (base_type == FUNC_TRACKING_SINGLE)
		else if (base_type == FUNC_TRACKING_GROUP) {
			if (onSelectFormation(dat)) {
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] Selecting formation");
				onFormationChange();
			}
			if (shape_selected) {
				info = FormationInfo(dxy_desire, dat);
				
				if (testEight.enabled && !testEight.all_prepared) {
					generateTestEightPath(info, tm);
				}
                
				if (!lineXY.empty()){
					real_t t = (real_t)(tm.timeElapsed / 1000.0);
					pt2D u_track  = tracking(t, dat.xy[0]);
					pt2D u_consen = consenTrack(dat.xy[0], info.center);
					ulocal = u_track + u_consen;

					AlgoTracking::Info res = tracking.compute_info();
					const pt2D& tar = res.tar;
					double err = res.err;
					if (res.delayed) {
						LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] [Waiting] t = {:.2f}s, tDelayed = {:.2f}s, errThresh = {:.2f}",
							tracking.t_tracking, t - tracking.t_tracking, tracking.errThresh);
					}
					else {
						LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] [Normal] t = {:.2f}s, tDelayed = {:.2f}s",
							tracking.t_tracking, t - tracking.t_tracking);
					}

					current_progress = (real_t)(lineXY.percentage(tracking.t_tracking) * 0.99);
					fwcomp.v = 0;
					fwcomp.w = 0;
					
					/*
					pt2D curve_tan = impl::normalize(lineXY.deriv(1, tracking.t_tracking));
					pt2D curve_nor = impl::rot90(curve_tan);
					pt2D curve_va  = lineXY.deriv(2, tracking.t_tracking);
					double curve_v = impl::norm2(lineXY.deriv(1, tracking.t_tracking));
					double curve_a = impl::norm2(lineXY.deriv(2, tracking.t_tracking));
					if (impl::dot(curve_va, curve_nor) < 0)
						curve_a = -curve_a;

					double curve_r = -1, curve_w = 0;
					if (curve_v > 0.05 && std::abs(curve_a) > 0.05) {
						curve_r = (curve_v*curve_v) / curve_a;
						curve_w = impl::rad2deg(curve_v / curve_r);
					}
					//fwcomp.v = curve_v;
					//fwcomp.w = -curve_w;

					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] curve_v/a: {:.2f}/{:.2f}, r/w: {:.2f}/{:.2f}", curve_v, curve_a, curve_r, curve_w);
					*/
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] [{}%] err: {:.2f}, tar: ({:.2f}, {:.2f}), u: ({:.2f}, {:.2f})",
						int(current_progress*100), err, tar.x, tar.y, ulocal.x, ulocal.y);
					if (!lineXY.reachEnd()) {
						fwcomp.w += alignLocal((real_t)dat.th[0], (real_t)info.centerTh);
						fwcomp.w += alignTarget((real_t)dat.th[0], lineXY.deriv(1, tracking.t_tracking));	// Note: t is not t_tracking in PathFollow mode
						fwcomp.w = rad2deg(fwcomp.w);
					}
					else {
						pt2D rec_local = ulocal;
						ulocal = slow_down(err, ulocal, impl::normalize(tar + dxy_desire[ID] - dat.xy[0]));
						if (slow_down.lastLamNor < 0.9 || slow_down.lastLamTan < 0.9) {
							LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] Slowing down, lamTan/Nor = {:.2f}, {:.2f}", slow_down.lastLamTan, slow_down.lastLamNor);
						}

						if (reach_position(err < thresh_tracking && info.posError.max < thresh_consen)) {
							ulocal = pt2D(0, 0);
							double ethMax = info.thError.max;
							double ethMe = info.centerTh - dat.th[0];
							if (true || reach_heading(info.thError.max < deg2rad(thresh_th) /*&& ethMe < deg2rad(thresh_th / 2.0)*/)){
								current_progress = 1.0f;
								runtimeState = STATE_TARGET_REACHED;
								fwcomp.w = 0;
								LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] Target Reached");
							}
							else{
								fwcomp.w = rad2deg(alignRend((real_t)dat.th[0], (real_t)info.centerTh));
								fwcomp.v = 0;
								LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] thErrMax {:.2f}, thErrMe {:.2f}, thThresh {:.2f}",
									rad2deg(ethMax), rad2deg(ethMe), thresh_th);
							}
						}
						else {
							LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] End postion err {:.2f}/{:.2f}, err_rela {:.2f}/{:.2f}",
								err, thresh_tracking, info.posError.max, thresh_consen);
						}
						LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] Endreach Counter {}/{}, {}/{}", 
							reach_position.counter, reach_position.limit, reach_heading.counter, reach_heading.limit);
					}
				}
				else {
					// lineXY empty
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] LineXY empty");
				}
			}
			else{
				// Shape Not Selected
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup] Shape unselected");
			}
		} // else if (base_type == FUNC_TRACKING_GROUP)
		/*
		else if (0 && base_type == FUNC_TRACE_HUMAN){
			if (onSelectFormation(dat)){
				onFormationChange();
			}
			if (shape_selected) {
				if (human_pos){
					tracer.print_debug = (currLogOption() & DEBUG_INFO_COMPUTE) != 0;

					real_t dt = (real_t)((tm.timeElapsed - lastTime.timeElapsed) / 1000.0f);
					ulocal = tracer(dt, human_pos, 0, dat.xy, dat.vxy, obXY);

					pt2D led = tracer.leader;	// in cm
					pt2D vled = tracer.leader_vxy; // in cm/s
					double err_me = impl::norm2(led - dat.xy[0]);    // in cm
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] Tracer: dt={:.3f}, hmn ({:.2f}, {:.2f}), led=({:.2f}, {:.2f}), shift=({:.2f}, {:.2f}), vLed=({:.2f}, {:.2f}), eLed={:.2f}, eMe={:.2f}",
						dt, human_pos.x, human_pos.y, led.x, led.y,
						tracer.human_shift.x, tracer.human_shift.y,
						vled.x, vled.y,
						impl::norm2(human_pos + tracer.human_shift * 100 - led),
						impl::norm2(led - dat.xy[0])
					);

					pt2D tmp = (ulocal - dat.vxy[0]);
					if (dt >= 0.001) tmp /= dt;
					else tmp = pt2D(0, 0);
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] Tracer: vMe=({:.2f}, {:.2f})/{:.2f}, vDesire=({:.2f}, {:.2f}), change/dt=({:.2f}, {:.2f})",
						dat.vxy[0].x, dat.vxy[0].y, dat.vw[0].x, ulocal.x, ulocal.y, tmp.x, tmp.y);

					if (err_me < thresh_consen) {
						pt2D center_diff = human_pos - dat.xy[0] + dxy[ID];
						fwcomp.w = alignRend((real_t)dat.th[0], center_diff);
						fwcomp.w = rad2deg(fwcomp.w);
						LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "       Tracer: Align with w = {:.2f}", fwcomp.w);
					}
					current_progress = 0.99f*std::exp((float)(-err_me));
				}
				else{
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] Tracer: no human pos");
					current_progress = 0.0f;
				}
			}
			else{
				// Shape Not Selected
			}
		}
		*/
		else if (base_type == FUNC_TURN_TO) {
			if (target_point || target_angle){
				ulocal = pt2D();
				real_t th;
				if (target_point) {
					pt2D err = target_point - dat.xy[0];
					th = (real_t)std::atan2(err.y, err.x);
				}
				else {
					th = deg2rad((real_t)target_angle);
				}
				fwcomp.w = alignRend((real_t)dat.th[0], th);
				fwcomp.w = rad2deg(fwcomp.w);
				fwcomp.v = 0;
				real_t errTh = anglediff(th, (real_t)dat.th[0]);
				current_progress = 0.99f * (1 - rad2deg(std::abs(errTh))/180.0f);
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TurnTo] tar ({:.2f}, {:.2f}), tarTh {:.2f}, myTh {:.2f}, errTh/Thresh {:.2f}/{:.2f}",
					target_point.x, target_point.y, rad2deg(th), rad2deg(dat.th[0]),
					rad2deg(errTh), thresh_th);
				if (std::abs(anglediff(th, (real_t)dat.th[0])) < deg2rad(thresh_th)){
					fwcomp.w = 0;
					current_progress = 1.0f;
					runtimeState = STATE_TARGET_REACHED;
					LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TurnTo] Target Reached");
				}
			}
			else{
				// Target Point Missing
				LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TurnTo] Target Point and Target Angle are Missing !!");
			}
		} // else if (base_type == FUNC_TURN_TO)
		else if (base_type == FUNC_TELE_BASIC) {
			uint maxValid = teleDirect.loopCounter + 2;
			if (tm.loopCounter >= maxValid) 
				teleDirect.setValid(false);

			if (teleDirect.valid()) {
				fwcomp.v = teleDirect.v;
				fwcomp.w = teleDirect.w;
			}
		} 
		else if (base_type == FUNC_TELE_DXY) {
			uint maxValid = teleDirectVxy.loopCounter + 5;
			if (tm.loopCounter >= maxValid)
				teleDirectVxy.setValid(false);

			if (teleDirectVxy.valid()) {
				ulocal = pt2D(teleDirectVxy.vx, teleDirectVxy.vy);
			}
		}
		
		// else if (base_type == FUNC_TELE_BASIC)

		// Share Control
		u = ulocal;
		if (enableHField() && base_type != FUNC_TELE_BASIC) {
			real_t dt = (real_t)((tm.timeElapsed - lastTime.timeElapsed) / 1000.0f);
			hfMe = human_field.update(dt, hiLast);

			//u = human_field(impl::saturate(ulocal, maxVel()));
			u = human_field(ulocal);
			fwcomp.v *= (1 - hfMe.ratio);
			fwcomp.w *= (1 - hfMe.ratio);
			LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [Hfield] hfMe ({:.2f}, {:.2f}) - {:.2f}, ulocal.ratio {:.2f}", 
				hfMe.x, hfMe.y, hfMe.ratio, impl::norm2(ulocal));
		} // if (enableHField())

		// Obstacle avoidance
		pt2D uavo(0, 0);
		/*
		if (base_type != FUNC_TRACE_HUMAN && use_pfavoid){
			for (size_t i = 0; i < dat.size(); ++i){
				uavo += power_avoid(dat.xy[0], dat.xy[i], robotRadius());
			}
			for (size_t i = 0; i < obXY.size(); ++i){
				uavo += power_avoid(dat.xy[0], obXY[i], obR[i]);
			}
			real_t szAvo = (real_t)norm2(uavo);
			real_t szLoc = (real_t)norm2(u);
			real_t rLoc = (szLoc + 0.001f) / (szAvo + szLoc + 0.001f);
			fwcomp.v *= rLoc;
			fwcomp.w *= rLoc;
		}
		*/
		//LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl]\tuavo=({:.2f}, {:.2f}), nob={}", uavo.x, uavo.y, obXY.size());
		return u + uavo;
	}
	
	void Combined::generateTestEightPath(const FormationInfo& info, const TimeInfo& tm) {
		if (testEight.enabled && !testEight.all_prepared) {
			LOGDEBUGLV2(DEBUG_INFO_COMPUTE, "[Ctrl] [TGroup-Test-8] Preparing shape");
			std::vector<double> tt, xx, yy;
			double R = std::max(testEight.R, 1.0f);		// in m
			double vel = testEight.vel > 0 ?
				testEight.vel : cm2m(default_vpath);	// in m/s
			if (vel > 0.8 * cm2m(maxVel())) {
				vel = 0.8 * cm2m(maxVel());					// Limit vel by maxVel
			}
			if (vel > deg2rad(maxRotate())* R * 0.8f) {
				vel = deg2rad(maxRotate()) * R * 0.8f;		// Limit wVel by maxRot
			}

			int nRound = testEight.nRound;
			int nPoint = testEight.nPoint;
			double rotTheta = deg2rad(testEight.theta);
			double tStay = testEight.tStay;

			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Appending terminal circles for Formation Test");
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] - R: {:.1f}, v: {:.1f}, nR: {}, nP: {}, rot: {:.1f}, t0: {:.1f}",
				R, vel, nRound, nPoint, rad2deg(rotTheta), tStay);

			pt2D center = info.centerPhy / 100;	  // cm -> m
			append_eight(&center, tt, xx, yy, R, vel, nRound, nPoint, rotTheta, tStay);
			lineXY.clear();
			lineXY.setpoints(tt, xx, yy);

			lineXY.setAutoLoop(false);
			lineXY.setAutoStop(true);
			
			lineXY.setT0(tm.timeElapsed / 1000.0);

			tracking.reset(tm.timeElapsed / 1000.0f);
			reach_heading.reset();
			reach_position.reset();

			testEight.all_prepared = true;
		}
	}

}   // namespace impl
