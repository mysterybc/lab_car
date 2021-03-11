#include "Algo.hpp"


namespace impl{

	/*
	 *  AlgoTracking
	 */
	void AlgoTracking::reset(real_t t0) {
		if (pline)
			pline->setT0(t0);
		t_tracking = t0;
		t_lasttime = t0;
		last_info.valid = false;
	}

	real_t AlgoTracking::compute_target(real_t t_now, pt2D me, real_t t_scaling) {
		if (pline) {
			real_t dt = (t_now - t_lasttime) * t_scaling;
			real_t t  = t_tracking + dt;
			spline2D& line = *pline;
			
			// The tracking control has three mode
			// 1. Tracking,     t_path++ as time++
			// 2. PathFollow,   t_path++ as time++ if |tar_pos-me_pos| <= errThresh
			// 3. PathFollowEx, t_path is selected to minimize |tar_pos-me_pos|
			//                  (in this way, the forward speed is gained from path_vel)
			last_info.delayed = false;
			if (pathFollow) {
				pt2D target = line(t) * target_uint + lineShift;
				if (impl::norm2(target - me) > errThresh) {
					// well, don't proceed
					t = t_tracking;
					last_info.delayed = true;
				}
			}
			else if (pathFollowEx) {
				if (dt > 0) {
					pt2D target = line(t) * target_uint + lineShift;
					real_t t_slice = dt / 2;
					real_t best_t = t;
					double best_err = impl::norm2(target - me);

					int n_test = 5;
					real_t t_test;
					double e_test;
					for (int i = 1; i <= n_test; ++i) {
						t_test = t + i * t_slice;
						e_test = impl::norm2(line(t_test) * target_uint + lineShift - me);
						if (e_test < best_err) {
							best_err = e_test;
							best_t = t_test;
						}

						t_test = t - i * t_slice;
						e_test = impl::norm2(line(t_test) * target_uint + lineShift - me);
						if (e_test < best_err) {
							best_err = e_test;
							best_t = t_test;
						}
					}

					if (best_t < t) {
						last_info.delayed = true;
					}
					t = best_t;
				}
			}
			return t;
		}
		else {
			return 0;
		}
	}

	pt2D AlgoTracking::operator()(real_t t_now, pt2D me, real_t t_scaling){
		if (!pline) {
			last_info.valid = false;
			return pt2D();
		}
		last_info.valid = true;

		pt2D target, targetV;
		if (pline) {
			real_t t_next = compute_target(t_now, me, t_scaling);
			target = (*pline)(t_next) * target_uint + lineShift;
			targetV = (*pline).deriv(1, t_next) * t_scaling * target_uint;
			if (t_next <= t_tracking) {
				last_info.delayed = true;
			}
			t_tracking = t_next;
		}
		else if (target_point) {
			target = *target_point;
			targetV = pt2D();
		}
		else {
			target = me;
			targetV = pt2D();
			last_info.valid = false;
			printf("Warning: Target Unset\n");
		}
		t_lasttime = t_now;

		/*
		pt2D target, targetV;
		real_t dt = t_now - t_lasttime;
		t_lasttime = t_now;
		if (pline) {
			double t = t_tracking;
			spline2D& line = *pline;
			target = line(t) * target_uint;
			targetV = line.deriv(1, t) * t_scaling * target_uint;
		}
		else if (target_point) {
			target  = *target_point;
			targetV = pt2D();
		}
		else {
			target  = me;
			targetV = pt2D();
			last_info.valid = false;
			printf("Warning: Target Unset\n");
		}
		

		pt2D errTracking = target + lineShift - me;
		last_info.tar = target + lineShift;
		last_info.err = impl::norm2(errTracking);
		if (pline) {
			if (pathFollow && last_info.err >= errThresh) {
				last_info.delayed = true;
			}
			else {
				last_info.delayed = false;
				t_tracking += dt;
			}
		}
		*/
		double targetHeading = atan2(targetV.y, targetV.x);
		pt2D errTracking = target - me;
		pt2D tarDir = pt2D(std::cos(targetHeading), std::sin(targetHeading)); // {  };
		pt2D eTarTagent = tarDir*dot(errTracking, tarDir);
		pt2D eTarNormal = errTracking - eTarTagent;

		if (szDZoneENormal > 0.01) {
			real_t szNormal = (real_t)norm2(eTarNormal);
			szNormal = dead_zone(szNormal, szDZoneENormal);
			if (szNormal < 0.001) {
				eTarNormal = pt2D();// { 0, 0 };
			}
			else {
				// Avoid the case that eTarNormal is approximately {0, 0}
				eTarNormal = normalize(eTarNormal) * szNormal;
			}
		}
		last_info.tar = target;
		last_info.err = impl::norm2(errTracking);
		last_info.tarV = targetV;

		//double dis2line = norm2(eTarNormal);
		return KTagent * eTarTagent + KNorml * eTarNormal +targetV;
	}

	/*
	 *  AlgoHumanField
	 */
	HumanField AlgoHumanField::update(real_t dt, wHumanInput& hiLast){
		if (!pHfNet) return hfMe;

		// Compute average field diff
		std::map<int, HumanField>& hfNet = *pHfNet;
		HumanField hfAvrDiff = HumanField(); // { 0, 0, 0 };
		if (!hfNet.empty()){
			std::map<int, HumanField>::iterator iter = hfNet.begin();
			for (; iter != hfNet.end(); iter++){
				HumanField& h = iter->second;
				hfAvrDiff = hfAvrDiff + h - hfMe;
			}
			hfAvrDiff = hfAvrDiff / (real_t)hfNet.size();
		}

		// Check if hiLast is still valid
		if (hiLast.valid()){
			hiLast.t += dt;
			if (hiLast.t > tHumanMax){
				hiLast.setValid(false);
			}
		}

		// Generate hfExt from hiLast
		// Check if received direct intervetion
		real_t decRatio; decRatio = std::pow(decRatioPS, dt);
		real_t decScale; decScale = std::pow(decScalePS, dt);
		HumanField hfExt = HumanField();// { 0, 0, 0 };
		if (hiLast){
			hfExt.x = hiLast.x;
			hfExt.y = hiLast.y;
			hfExt.ratio = std::pow(decRatioPS, hiLast.t);
		}

		// Update human field
		HumanField hfNew;
		if (hiLast.valid()){
			// The environment now is hfExt so we don't decay
			hfNew = hfMe + Kc * hfAvrDiff + Kt * (hfExt - hfMe);
		}
		else{
			hfNew = decScale * hfMe + Kc * hfAvrDiff;
			// Note we should compensate the difference between decScale and decRatio
			hfNew.ratio += (decRatio - decScale) * hfMe.ratio;
		}

		// Update hfMe
		hfMe = hfNew;

		// Clear the received net info
		hfNet.clear();

		// Return 
		return hfMe;
	}

}
