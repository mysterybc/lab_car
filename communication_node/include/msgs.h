#pragma once
//c++ standard
#include "map"
#include "string"
#include "algorithm"
//third party lib
#include "jsoncpp/json/json.h"
#include "iostream"
//ros
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "robot_msgs/HostCmdArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/Joy.h"
//my lib
#include "zmq_lib.h"



Json::Value string2json(std::string &msg){
    Json::Value json;
    Json::Reader reader;
    reader.parse(msg.c_str(),json);
    return json;
}

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
        for(auto number:group){
            msg["group"].append(number);
        }
        return msg.toStyledString();
    }
    std::string message_type;
    int id;
    uint8_t state;
    std::vector<int> group;
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

//商哥的控制消息
std::string multiArray2json(const std_msgs::UInt8MultiArrayConstPtr &msg){
    Json::Value json;
    json["message_type"] = "control_msg";
    for(int i = 0; i < msg->data.size() ; i++){
        json["algomsg"].append(msg->data[i]);
    }
    return json.toStyledString();
}

void json2multiarray(std::string& msg, std_msgs::UInt8MultiArray& data){
    Json::Value json = string2json(msg);
    int data_size = json["algomsg"].size();
    data.layout.dim.resize(1);
	data.layout.data_offset = 0;
	data.layout.dim[0].size = data_size;
	data.layout.dim[0].stride = 1;
    data.data.resize(data_size);
    for(int i = 0 ; i < data_size; i++){
        data.data[i] = json["algomsg"][i].asUInt();
    }    
}



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
        for(int i = 0 ; i < json["group"].size(); i++){
            group.push_back(json["group"][i].asInt());
        }
        double yaw = json["pose"][2].asDouble();
        yaw = yaw / 180.0 * 3.1415926;
        robot_pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        return 1;
    }
    int id;
    bool have_config;
    std::vector<int> group;
    geometry_msgs::Pose robot_pose;
    uint8_t robot_state;
};


//接收的host指令
struct HostCmd{
    HostCmd(zmq_lib::Receiver* receiver_,std::string ip){
        ros::NodeHandle nh;
        joy_pub = nh.advertise<sensor_msgs::Joy>("joy",10);
        receiver = receiver_;
        host_ip = ip;
    }
    //message类型
    enum HostMessageType{
        SingleMission = 0,
        MultiMission = 1,
        Cmd = 2,
        Service = 3,
        RemoteCtrl = 4
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

    //任务类型 
    enum ServiceType{
        ChangeHost = 1
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
            if(json[i]["id"].isArray()){
                Json::Value json_id = json[i]["id"];
                for(int i = 0; i < json_id.size(); i++){
                    cmd.car_id.push_back(json_id[i].asInt());
                }
            }
            cmd.mission.mission = json[i]["type"].asUInt();
            cmd.goal.header.stamp = ros::Time().now();
            cmd.goal.header.frame_id = "map";
            double yaw = 0;
            if(json[i]["instruction"].size() != 0){
                cmd.goal.pose.position.x = json[i]["instruction"][0].asDouble();
                cmd.goal.pose.position.y = json[i]["instruction"][1].asDouble();
                cmd.goal.pose.position.z = 0;
                yaw = json[i]["instruction"][2].asDouble();
            }
            //目前输入角度，手动坐标-1
            yaw = yaw / 180.0 * 3.1415926;
            cmd.goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            host_cmd_array.host_cmd_array.push_back(cmd);
        }
    }

    void getCmd(Json::Value json){
        for(int i = 0; i < json.size(); i++){
            mission_cmd = (MissionCmd)json[i]["type"].asUInt();
        }
    }

    void getService(Json::Value json){
        for(int i = 0 ; i <  json.size(); i++){
            if((ServiceType)json[i]["type"].asUInt() == ServiceType::ChangeHost){
                receiver->removeIp(host_ip);
                host_ip = json[i]["instruction"].asString();
                receiver->addIp(host_ip);
            }
        }
    }

    void getRemoteCtrl(Json::Value json){
        for(int i = 0 ; i <  json.size(); i++){
            //首先判断id有没有我
            // for(int j = 0 ; j > json[i]["id"].size(); j++){
            //     if(json[i]["id"])
            // }
        }
    }

    //TODO 如果尚未及发送过来的消息不是按顺序的，则需要排序
    void sortMission(){

    }

    HostMessageType json2Msg(std::string msg){
        std::cout << msg << std::endl;
        Json::Value json;
        Json::Reader reader;
        reader.parse(msg.c_str(),json);
        message_type = (HostMessageType)json["message_type"].asInt();
        switch(message_type){
            case HostMessageType::SingleMission : json2Mission(json["mission_array"]);break;
            case HostMessageType::MultiMission : json2Mission(json["mission_array"]);break;
            case HostMessageType::Cmd : getCmd(json["mission_array"]); break;
            case HostMessageType::Service : getService(json["mission_array"]);break;
            case HostMessageType::RemoteCtrl : getRemoteCtrl(json["mission_array"]);break;
        }
    }

    HostMessageType message_type;
    robot_msgs::HostCmdArray host_cmd_array;
    MissionCmd mission_cmd;
    std::string host_ip;

    private:
    zmq_lib::Receiver* receiver;
    ros::Publisher joy_pub;
};