#pragma once
#include "vector"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "robot_msgs/Separate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "robot_msgs/RobotStates.h"
#include "robot_msgs/DebugInfo.h"
#include "string"


class RobotInfo{
public:
    RobotInfo():car_id(0){};
    void SetState(const robot_msgs::RobotState state);
    geometry_msgs::Pose robot_pose;
    int car_id;
};




class SeparateGoal{
public:
    SeparateGoal();
    ~SeparateGoal() = default;
    bool CalGoal(robot_msgs::Separate::Request &req,
                 robot_msgs::Separate::Response &res);
    void MapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
    bool IsOccupied(geometry_msgs::Point p);
    void RobotStateCallback(const robot_msgs::RobotStatesConstPtr &msg);
    geometry_msgs::Pose GetMyPose();

private:
    std::vector<RobotInfo> robots_info;
    ros::Subscriber robots_state_sub;
    ros::ServiceServer separate_service; 
    ros::Subscriber map_sub;
    ros::NodeHandle nh;
    geometry_msgs::Pose my_pose;
    geometry_msgs::Pose goal_point;
    nav_msgs::OccupancyGrid map;
    std::string tf_ns;
    int car_id;
    int total_car_number;

};