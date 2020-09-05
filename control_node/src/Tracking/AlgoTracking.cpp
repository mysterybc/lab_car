#include "Tracking/AlgoTracking.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

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

			// New Feature: add a hysteresis for the delay threshhold. 2019/06/03, by Shang C
			// However, this is not fully tested.
			bool last_delayed = last_info.delayed;
			last_info.delayed = false;
			if (pathFollow) {
				double thresh = errThresh;
				pt2D target = line(t) * target_uint + lineShift;
				if (last_delayed)
					thresh /= 2;
				if (impl::norm2(target - me) > thresh) {
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
        
		// 计算 目标位置, 目标速度
		// 1. 有pline, 用compute_target计算
		// 2. 只是一个点, 则targetV = 0
		// 3. 都没有设置(异常)
		pt2D target, targetV;
		if (pline) {
			real_t t_next = compute_target(t_now, me, t_scaling);
			target = (*pline)(t_next) * target_uint + lineShift;
			targetV = (*pline).deriv(1, t_next) * t_scaling * target_uint;
			//ROS_INFO("target pose is %lf %lf",(*pline)(t_next).x,(*pline)(t_next).y);
			//ROS_INFO("target speed is %lf %lf",targetV.x,targetV.y);
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
		// 计算当前轨线的运动方向, 
		// 计算当前轨线的切向量、法向量、以及相应误差
		double targetHeading = atan2(targetV.y, targetV.x);
		pt2D errTracking = target - me;	 // 误差
		pt2D tarDir = pt2D(std::cos(targetHeading), std::sin(targetHeading)); // 切向量
		pt2D eTarTagent = tarDir*dot(errTracking, tarDir);  // 切向误差
		pt2D eTarNormal = errTracking - eTarTagent;         // 法向误差
		// 如果法向误差上加设置了死区 (szDZoneENormal > 0.01)
		// 则减小法向误差
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
		// BUG Fix: 2019/06/03, by Shang C
		// Reason: if the target point is not moving, then the targetV should not be contained in the output
		//         otherwise it may cancelled out the converging terms
		if (last_info.delayed)
			return KTagent * eTarTagent + KNorml * eTarNormal;
		
		// 控制算法: 比例控制+前馈(目标轨线速度)
		return KTagent * eTarTagent + KNorml * eTarNormal + targetV;
	}


}
