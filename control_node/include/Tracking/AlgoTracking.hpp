#pragma once
#include <algorithm>
#include <functional>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include "Tracking/spline2D.hpp"
#include "Tracking/helper.hpp"
#include "Tracking/vxy2vw.hpp"

#define real_t double

#ifdef __GNUC__
#if __GNUC__ < 4
    #ifndef nullptr
    #define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
    #endif
#elif __GNUC__ == 4 && __GNUC_MINOR__ < 6
    #ifndef nullptr
    #define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
    #endif
#endif
#endif

namespace impl{
	/*
	 * AlgoTracking
	 * 功能：实现一个针对点模型小车，
	 *       基于 比例控制+前馈的 Tracking/Path Following算法
	 * Tracking: 车位置(t) - 轨线位置(t) -> 0
	 * Path Following: 车到轨线的距离(t) -> 0，车速度(t) -> 期望速度
	 * 算法：u = u_Tagent + u_Normal + 当前轨线速度 是
	 *       u_Tagent = KTagent * 误差e在轨线 切向方向 的分量大小
	 *       u_Normal = KNorml * 误差e在轨线 法向方向 的分量大小
	 *       误差e = 当前目标位置 - 当前小车位置
	 * Tracking 和 Path Following的区别在于：
	 *       当前目标位置 如何计算, 见cpp中compute_target部分的说明
	 * 使用说明:
	 * 1. 目标轨线指定: spline2D* pline
	 * 2. 或 指定目标点: pt2D*  target_point;
	 * 3. 设置参数、Tracking模式
	 * 4. 用operator()计算控制量
	 */
	
	class AlgoTracking{
	public:
		struct Info {
			Info() {
				valid = true;
			}
			bool   valid;   // 本信息是否有效
			pt2D   tar;     // 上次计算时的目标位置
			pt2D   tarV;	// 上次计算时的目标速度
			double err;     // 上次计算时到目标点的距离
			bool   delayed; // 上次计算时目标点是否静止(只在PathFollow时有用)
		};
		AlgoTracking(){
			pline        = nullptr;
			target_point = nullptr;
			target_uint = 5.0f;
			target_speed = 0.5f;
			szDZoneENormal = 0.0f;
			KTagent = 1.0f;
			KNorml  = 1.0f;
			lineShift = pt2D();
			pathFollow = false;
			pathFollowEx = true;
			t_tracking = 0.0f;
			t_lasttime = 0.0f;
			errThresh = 100.0f;
		}
		void reset(real_t t0);
		real_t compute_target(real_t t, pt2D me, real_t t_sacling);
		pt2D operator()(real_t t, pt2D me, real_t t_scaling = 1.0f);
		Info compute_info() const { return last_info; }
        

		// Parameters
		spline2D* pline;		// = nullptr;
		pt2D*  target_point;	// = nullptr;

		real_t target_uint;// = 100.0f;
		real_t target_speed;// = 1.0f;
		real_t szDZoneENormal;// = 0.0f;
		real_t KTagent;// = 1.0f;
		real_t KNorml;//  = 1.0f;
		pt2D   lineShift;// = pt2D{ 0, 0 };
		bool   pathFollow;
		bool   pathFollowEx;
		real_t t_tracking;
		real_t t_lasttime;
		real_t errThresh;
	private:
		Info last_info;
	};

} // namespace impl
