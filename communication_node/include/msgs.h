#pragma once
//c++ standard
#include "map"
//third party lib
#include "jsoncpp/json/json.h"
//ros
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
//my lib
#include "zmq_lib.h"


//发送的
//机器人状态消息
struct StateMsg{
    StateMsg(const int car_id){
        message_type = "state";
        id = car_id;
        state = "stand_by";
        x = 0;
        y = 0;
        yaw = 0;
    }
    std::string fmtMsg(){
        Json::Value msg;
        msg["message_type"] = message_type;
        msg["id"] = id;
        msg["state"] = state;
        msg["pose"].append(x);
        msg["pose"].append(y);
        msg["pose"].append(yaw);
        return msg.toStyledString();
    }
    std::string message_type;
    int id;
    std::string state;
    double x,y,yaw; //in m m deg
};

//上报path消息
struct PathMsg{
    PathMsg(const int car_id){
        message_type = "state";
        id = car_id;
    }
    std::string fmtMsg(const nav_msgs::Path path){
        Json::Value msg;
        msg["message_type"] = "path";
        msg["id"] = id;
        for(const auto &pose:path.poses){
            msg["path_x"].append(pose.pose.position.x);
            msg["path_y"].append(pose.pose.position.y);
        }
        return msg.toStyledString();
    }
    std::string message_type;
    int id;
};


//接收的
struct RobotState{
    RobotState(){
        have_config = false;
    }
    int id;
    bool have_config;
    geometry_msgs::Pose robot_pose;
    std::string robot_state;
};