#pragma once
#include "vector"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "robot_msgs/Separate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "string"


class RobotInfo{
public:
    RobotInfo(){}
    void PoseCallback(const nav_msgs::OdometryConstPtr &msg);
    ros::Subscriber pose_sub;
    geometry_msgs::Pose robot_pose;

};




class SeparateGoal{
public:
    SeparateGoal();
    ~SeparateGoal() = default;
    bool CalGoal(robot_msgs::Separate::Request &req,
                 robot_msgs::Separate::Response &res);
    void MapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
    bool IsOccupied(geometry_msgs::Point p);

private:
    std::vector<RobotInfo> robots_info;
    ros::ServiceServer separate_service; 
    ros::Subscriber map_sub;
    ros::NodeHandle nh;
    geometry_msgs::Pose goal_point;
    nav_msgs::OccupancyGrid map;
    int robot_id;

};