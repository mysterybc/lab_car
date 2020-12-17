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
        state = 0;
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
    uint8_t state;
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


//接收的机器人消息
struct RobotState{
    RobotState(){
        have_config = false;
    }
    int json2Robostate(std::string msg){
        RobotState state;
        Json::Value json;
        Json::Reader reader;
        reader.parse(msg.c_str(),json);
        if(json["message_type"].asString() != "state"){
            return -1;
        }
        have_config = true;
        id = json["id"].asInt();
        robot_state = json["state"].asUInt();
        robot_pose.position.x = json["pose"][0].asDouble();
        robot_pose.position.y = json["pose"][1].asDouble();
        robot_pose.position.z = 0;
        double yaw = json["pose"][2].asDouble();
        yaw = yaw / 180.0 * 3.1415926;
        robot_pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        return 1;
    }
    int id;
    bool have_config;
    geometry_msgs::Pose robot_pose;
    uint8_t robot_state;
};

enum HostMessageType{
    SingleMission = 0,
    MultiMission = 1,
    Cmd = 2
};

struct Mission{
    std::vector<int> id;
    
};