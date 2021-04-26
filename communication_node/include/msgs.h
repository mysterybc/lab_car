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
#include "robot_msgs/CurrentTask.h"
//my lib
#include "zmq_lib.h"



Json::Value string2json(std::string &msg){
    Json::Value json;
    Json::Reader reader;
    reader.parse(msg.c_str(),json);
    return json;
}

/********************************   jiacheng_ver*************************************/
#define ROBOT_STATE 1
#define RRBOT_TASK 2
#define ROBOT_PERCEPTION 3
//机器人状态消息
struct RobotStateMsg{
    RobotStateMsg(const int car_id){
        message_type = ROBOT_STATE;
        id = car_id;
    }
    std::string fmtMsg(){
        Json::Value msg;
        msg["message_id"] = message_type;
        msg["id"] = id;
        msg["state"] = "running";
        msg["pose"].append(x);
        msg["pose"].append(y);
        msg["pose"].append(yaw);
        msg["speed"].append(vx);
        msg["speed"].append(vy);
        msg["speed"].append(w);
        msg["acc"].append(acc_x);
        msg["acc"].append(acc_y);
        msg["acc"].append(acc_yaw);
        return msg.toStyledString();
    }
    void GetDataFromMsg(Json::Value msg){
        id = msg["id"].asInt();
        robot_state = msg["state"].asString();
        x = msg["pose"][0].asDouble();
        y = msg["pose"][1].asDouble();
        yaw = msg["pose"][2].asDouble();
        vx = msg["speed"][0].asDouble();
        vy = msg["speed"][1].asDouble();
        w = msg["speed"][2].asDouble();
        acc_x = msg["acc"][0].asDouble();
        acc_y = msg["acc"][1].asDouble();
        acc_yaw = msg["acc"][2].asDouble();
    }
    int message_type;
    int id;
    double x,y,yaw; //in m m deg
    double vx,vy,w; // in m/s m/s deg/s
    double acc_x,acc_y,acc_yaw;
    std::string robot_state;
};

//机器人任务信息
struct RobotTaskMsg{
    RobotTaskMsg(const int car_id){
        message_type = RRBOT_TASK;
        id = car_id;
        current_task_int = 0;
    }
    std::string fmtMsg(){
        Json::Value msg;
        msg["message_id"] = message_type;
        msg["id"] = id;
        switch(current_task_int){
            case 0: msg["current_task_str"] = "NONE TASK!!!";break;
            case 1: msg["current_task_str"] = "GPS March!!!";break;
            case 2: msg["current_task_str"] = "Laser March!!!";break;
            case 3: msg["current_task_str"] = "Search Task!!!";break;
            case 4:	msg["current_task_str"] = "Remote Control TASK!!!";break;
            case 5: msg["current_task_str"] = "ASSEMBLE TASK!!!";break;
            case 6:	msg["current_task_str"] = "STOP TASK!!!";break;
            case 7:	msg["current_task_str"] = "Pause TASK!!!";break;
            case 8:	msg["current_task_str"] = "Resume TASK!!!";break;
        }
        switch(task_state_int){
            case 0: msg["task_state_str"] = "Running";break;
            case 1: msg["task_state_str"] = "Success";break;
            case 2: msg["task_state_str"] = "Failure";break;
            case 3: msg["task_state_str"] = "Idle";break;
        }
        msg["current_task_int"] = current_task_int;
        return msg.toStyledString();
    }
    void GetDataFromMsg(Json::Value msg){
        id = msg["id"].asInt();
        current_task_str = msg["current_task_str"].asString();
        task_state_str = msg["task_state_str"].asString();
        current_task_int = msg["current_task_int"].asInt();
    }
    int message_type;
    int id;
    int task_state_int;
    std::string task_state_str;
    uint8_t current_task_int;
    std::string current_task_str;

};

struct Point{
    Point(){
        x = 0; 
        y = 0;
    }
    double x;
    double y;
};

//机器人检测信息
struct RobotPerceptionMsg{
    RobotPerceptionMsg(const int car_id){
        message_type = ROBOT_PERCEPTION;
        id = car_id;
        has_obstracle = false;
        has_target = false;
    }
    std::string fmtMsg(){
        Json::Value msg;
        msg["message_id"] = message_type;
        msg["id"] = id;
        if(points.size() >= 5){
            msg["obstracle_number"] = 5;
        }
        else{
            msg["obstracle_number"] = (int)points.size();
        }
        if(points.size() > 0){
            msg["has_obstracle"] = true;
        }
        else{
            msg["has_obstracle"] = false;
        }
        int count = 0;
        for(auto point:points){
            if(count > 4){
                break;
            }
            Json::Value json_point;
            json_point["x"] = point.x;
            json_point["y"] = point.y;
            msg["points"].append(json_point);
            count++;
        }
        msg["has_target"] = has_target;
        if(has_target){
            Json::Value json_point;
            json_point["x"] = target_coord.x;
            json_point["y"] = target_coord.y;
            msg["target_coord"] = json_point;
        }
        return msg.toStyledString();
    }
    void GetDataFromMsg(Json::Value json){
        id = json["id"].asInt();
        obstracle_number = json["obstracle_number"].size();
        for(int i = 0 ; i < json["points"].size(); i++){
            Point point;
            point.x = json["points"][i]["x"].asDouble();
            point.y = json["points"][i]["y"].asDouble();
            points.push_back(point);
        }
        obstracle_number = json["obstracle_number"].asInt();
        has_obstracle = json["has_obstracle"].asBool();
        has_target = json["has_target"].asBool();
        if(has_target){
            Point point;
            point.x = json["target_coord"]["x"].asDouble();
            point.y = json["target_coord"]["y"].asDouble();
        }
    }
    int message_type;
    int id;
    bool has_target;
    Point target_coord;
    bool has_obstracle;
    int obstracle_number;
    std::vector<Point> points;
};


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
        message_type = "path";
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
    HostCmd(zmq_lib::Receiver* receiver_,std::string ip,int car_id){
        ros::NodeHandle nh;
        joy_pub = nh.advertise<sensor_msgs::Joy>("joy",10);
        receiver = receiver_;
        host_ip = ip;
        robot_id = car_id;
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
        Search_Task = 3,
        Remote_Control = 4,
        Assemble = 5,
        STOP = 6,
        Pause = 7,
        Resume = 8
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
            geometry_msgs::PoseStamped goal;
            goal.header.stamp = ros::Time().now();
            goal.header.frame_id = "map";
            double yaw = 0;
            if(cmd.mission.mission == 3){
                for(int j = 0 ; j < json[i]["instruction"].size(); j++){
                    if(json[i]["instruction"][i].isArray()){
                        goal.pose.position.x = json[i]["instruction"][j][0].asDouble();
                        goal.pose.position.y = json[i]["instruction"][j][1].asDouble();
                        goal.pose.position.z = 0;
                    }
                    goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                    cmd.goal.push_back(goal);
                }
            }
            else{
                if(json[i]["instruction"].isArray()){
                    goal.pose.position.x = json[i]["instruction"][0].asDouble();
                    goal.pose.position.y = json[i]["instruction"][1].asDouble();
                    goal.pose.position.z = 0;
                }
                yaw = json[i]["instruction"][2].asDouble();
                //目前输入角度，手动坐标-1
                yaw = yaw / 180.0 * 3.1415926;
                goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                cmd.goal.push_back(goal);
            }
            
            host_cmd_array.host_cmd_array.push_back(cmd);
        }
    }

    void getCmd(Json::Value json){
        for(int i = 0; i < json.size(); i++){
            mission_cmd = (MissionCmd)json[i]["type"].asUInt();
        }
    }
    //服务类消息，目前只有切换hostip
    void getService(Json::Value json){
        for(int i = 0 ; i <  json.size(); i++){
            if((ServiceType)json[i]["type"].asUInt() == ServiceType::ChangeHost){
                receiver->removeIp(host_ip);
                host_ip = json[i]["instruction"].asString();
                receiver->addIp(host_ip);
            }
        }
    }

    //接受上位机遥控指令
    void getRemoteCtrl(Json::Value json){
        for(int i = 0 ; i <  json.size(); i++){
            sensor_msgs::Joy joy;
            bool has_me{false};
            if(json[i]["id"].isArray()){
                Json::Value json_id = json[i]["id"];
                for(int i = 0; i < json_id.size(); i++){
                    if(json_id[i].asInt() == robot_id){
                        has_me = true;
                    }
                }
            }
            if(!has_me){
                return ;
            }
            if(json[i]["instruction"].size()!=0){
                double axes1,axes2;
                axes1 = json[i]["instruction"][0].asDouble();
                axes2 = json[i]["instruction"][1].asDouble();
                joy.axes.push_back(axes1);
                joy.axes.push_back(axes2);
                for(int j = 0 ; j < 5; j++){
                    joy.buttons.push_back(0);
                }
                joy.buttons.push_back(json[i]["instruction"][2].asInt());
            }
            joy_pub.publish(joy);
        }
    }

    //TODO 如果尚未及发送过来的消息不是按顺序的，则需要排序
    void sortMission(){

    }

    bool Has_Me(std::string msg){
        Json::Value json;
        Json::Reader reader;
        reader.parse(msg.c_str(),json);
        if(json["mission_array"][0]["id"].isArray()){
            Json::Value json_id = json["mission_array"][0]["id"];
            for(int i = 0; i < json_id.size(); i++){
                if(json_id[i].asInt() == robot_id){
                    return true;
                }
            }
        }
        return false;
    }

    HostMessageType json2Msg(std::string msg){
        Json::Value json;
        Json::Reader reader;
        reader.parse(msg.c_str(),json);
        message_type = (HostMessageType)json["mission_type"].asInt();
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
    int robot_id;
};