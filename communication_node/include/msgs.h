#pragma once
//c++ standard
#include "map"
#include "algorithm"
//third party lib
#include "jsoncpp/json/json.h"
//ros
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "robot_msgs/HostCmdArray.h"
//my lib
#include "zmq_lib.h"


//发送的，给上位机发也给机器人发
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


//接收的host指令
struct HostCmd{
    //message类型
    enum HostMessageType{
        SingleMission = 0,
        MultiMission = 1,
        Cmd = 2
    };

    //cmd
    enum MissionCmd{
        OK = 0,
        NO = 1,
        NotRecv = 2
    };

    //任务类型 
    enum MissionType{
        NonTask = 0,
        March_gps = 1,
        March_laser = 2,
        Assemble = 3,
        STOP = 4,
        Pause = 5,
        Resume = 6
    };

    //具体任务消息
    struct Mission{
        std::vector<int> id;
        geometry_msgs::Pose goal;
        MissionType mission_type;
        int index;
    };

    void json2Mission(Json::Value json){
        host_cmd_array.host_cmd_array.clear();
        for(int i = 0; i < json.size(); i++){
            robot_msgs::HostCmd cmd;
            cmd.car_id.clear();
            if(json["id"].isArray()){
                Json::Value json_id = json["id"];
                for(int i = 0; i < json_id.size(); i++){
                    cmd.car_id.push_back(json_id[i].asInt());
                }
            }
            cmd.mission.mission = json["type"].asUInt();
            cmd.goal.header.stamp = ros::Time().now();
            cmd.goal.header.frame_id = "map";
            cmd.goal.pose.position.x = json["instruction"][0].asDouble();
            cmd.goal.pose.position.y = json["instruction"][1].asDouble();
            cmd.goal.pose.position.z = 0;
            double yaw = json["instruction"][2].asDouble();
            //目前输入角度，手动坐标-1
            yaw = yaw / 180.0 * 3.1415926;
            cmd.goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            host_cmd_array.host_cmd_array.clear();
            host_cmd_array.host_cmd_array.push_back(cmd);
        }
    }

    void getCmd(Json::Value json){
        for(int i = 0; i < json.size(); i++){
            mission_cmd = (MissionCmd)json[i]["type"].asUInt();
        }
    }

    //TODO 如果尚未及发送过来的消息不是按顺序的，则需要排序
    void sortMission(){

    }

    HostMessageType json2Msg(std::string msg){
        Json::Value json;
        Json::Reader reader;
        reader.parse(msg.c_str(),json);
        message_type = (HostMessageType)json["message_type"].asInt();
        switch(message_type){
            case HostMessageType::SingleMission : json2Mission(json["mission_array"]);break;
            case HostMessageType::MultiMission : json2Mission(json["mission_array"]);break;
            case HostMessageType::Cmd : getCmd(json["mission_array"]); break;
        }
    }

    HostMessageType message_type;
    robot_msgs::HostCmdArray host_cmd_array;
    MissionCmd mission_cmd;
};