#pragma once
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "robot_msgs/HostCmd.h"
#include <geometry_msgs/PoseStamped.h>



enum class MissionType{
    SYSTEM_STANDBY = 0,
    BUILD_UP_TASK
};



class Blackboard {
public:
    typedef std::shared_ptr<Blackboard> Ptr;
    Blackboard(){
        ros::NodeHandle nh;
        pose_sub = nh.subscribe("odom",10,&Blackboard::PoseCallback,this);
        //测试使用
        //goal_sub = nh.subscribe("/move_base_simple/goal",10,&Blackboard::GoalCallback,this);
        cmd_sub = nh.subscribe("/host_cmd",10,&Blackboard::CmdCallback,this);
        map_sub = nh.subscribe("/map",10,&Blackboard::MapCallback,this);
        //目前
        car_number = 1;

        mission = MissionType::SYSTEM_STANDBY;

    }

    ~Blackboard() = default;
    // Goal
    void GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg){
        goal = msg->pose;
    }

    void PoseCallback(const nav_msgs::OdometryConstPtr &msg){
        robot_pose = msg->pose.pose;
    }

    void CmdCallback(const robot_msgs::HostCmdConstPtr &msg){
        //首先判断我是否需要执行该指令；
        bool my_cmd = false;
        for(int i = 0; i < msg->car_id.size(); i++){
            if(msg->car_id.at(i) == car_number){
                my_cmd = true;
            }
        }
        if(!my_cmd){
            return ;
        }
        mission = (MissionType)msg->mission.mission;
        goal = msg->goal.pose;
    }

    void MapCallback(const nav_msgs::OccupancyGridConstPtr &msg){
        map_height = msg->info.height * msg->info.resolution;
        map_width = msg->info.width * msg->info.resolution;
    }

    //getter
    geometry_msgs::Pose GetGoal(){
        return goal;
    }

    geometry_msgs::Pose GetRobotMapPose() {
        return robot_pose;
    }

    int GetCarNumber(){
        return car_number;
    }

    MissionType GetMission(){
        return mission;
    }

    void SetMission(MissionType mission_type){
        mission = mission_type;
    }

    //以下是判断条件
    bool IsSystemStart(){
        if(mission != MissionType::SYSTEM_STANDBY){
            return true;
        }
        return false;
    }



private:
    MissionType mission;
    //! Goal info
    geometry_msgs::Pose goal;
    //! robot map pose
    geometry_msgs::Pose robot_pose;
    // car id 
    int car_number;
    // map info ,单位m
    double map_height,map_width;

    ros::Subscriber pose_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber cmd_sub;
    ros::Subscriber map_sub;

};
