#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "Tracking/helper.hpp"
#include "Tracking/spline2D.hpp"
// (vx, vy) 转换成 (v, w)
// th是小车的车头朝向(i.e. 跟坐标轴x的夹角, 弧度)

namespace impl{

pt2D fromVxy2VW(double th, pt2D vxy){
	pt2D u;
    double limitV = 1.0; //上限线速度
	
	double thNow = rad2deg(th);
	double thDesire = rad2deg(std::atan2(vxy.y, vxy.x));

	if (norm2(vxy) > limitV)
		vxy = normalize(vxy) * limitV;

	pt2D dir = pt2D(std::cos(th), std::sin(th));
	pt2D nor = pt2D(-std::sin(th), std::cos(th));
	u.x = (double)dot(dir, vxy);
	u.y = (double)(dot(nor, vxy));    // 1/100 is for cm->m

	return u;
}
}
