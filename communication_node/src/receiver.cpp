//c++ standard
#include "algorithm"
//third party lib
#include "jsoncpp/json/json.h"
//ros
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "robot_msgs/RobotStates.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/UInt8MultiArray.h"
//my lib
#include "zmq_lib.h"
#include "msgs.h"


//将其他车的id和ip建立联系
std::vector<std::string> fmtInputIp(std::vector<std::string> &ips, std::string my_ip){
    auto it = std::find(ips.begin(),ips.end(),my_ip);
    ips.erase(it);
    return ips;
}

//解码机器人状态消息
void decodeRobotMsgs(std::string msg,std::map<int,RobotState> &id2states)
{
    RobotState state;
    state.json2Robostate(msg);
    id2states[state.id] = state;
}

//解码上位机消息
void decodeHostMsg(std::string msg, HostCmd &host_cmd){
    host_cmd.json2Msg(msg);
}


//发布机器人状态
void pubStatus(std::map<int,RobotState> id2states,ros::Publisher &pub){
    robot_msgs::RobotStates robot_states_msg;
    robot_states_msg.online_robot_number = 0;
    for(const auto & state : id2states ){
        if(!state.second.have_config){
            continue;
        }
        robot_states_msg.online_robot_number ++;
        robot_msgs::RobotState state_msg;
        state_msg.car_id = state.second.id;
        state_msg.robot_pose = state.second.robot_pose;
        state_msg.robot_state.robot_states_enum = state.second.robot_state;
        robot_states_msg.robot_states.push_back(state_msg);     
    }
    pub.publish(robot_states_msg);
}

//发布host cmd
void pubHostCmd(HostCmd &host_cmd, ros::Publisher &cmd_pub){
    cmd_pub.publish(host_cmd.host_cmd_array);
}

//pub algomsg
void pubAlgomsg(std::string &msg,ros::Publisher& algomsg_pub){
    std_msgs::UInt8MultiArray algomsg;
    json2multiarray(msg,algomsg);
    algomsg_pub.publish(algomsg);
}



int main(int argc, char** argv){
    ros::init(argc,argv,"receiver");
    ros::NodeHandle nh;

    //config id & ip
    int car_id;
    std::string host_ip;
    std::string my_ip;
    std::vector<std::string> total_ip;
    //获取group空间名
    std::string namespace_;
    namespace_ = nh.getNamespace();
    //获取group下的参数
    if(!nh.getParam(namespace_+"/my_ip_address",my_ip)){
        my_ip = "127.0.0.1:6661";
        ROS_WARN("RECEIVER FAILED TO GET ip_address");
        ROS_WARN("RECEIVER RESET CAR ID TO 127.0.0.1");
    }
    if(!nh.getParam(namespace_+"/car_id",car_id)){
        car_id = 1;
        ROS_WARN("RECEIVER FAILED TO GET CAR ID");
        ROS_WARN("RECEIVER RESET CAR ID TO 1");
    }
    if(!nh.getParam("/total_robot_ip",total_ip)){
        ROS_WARN("RECEIVER FAILED TO GET TOTAL IP");
    }
    if(!nh.getParam("/host_ip_address",host_ip)){
        ROS_WARN("RECEIVER FAILED TO GET HOST IP");
    }

    //zmq_init
    zmq::context_t ctx(1);
    zmq_lib::Receiver state_receive(fmtInputIp(total_ip,my_ip),ctx);
    zmq_lib::Receiver host_receive(std::vector<std::string>{host_ip},ctx);


    //message init
    std::map<int,RobotState> id2states;
    std::vector<std::string> robot_msgs;
    std::string host_msg;
    HostCmd host_cmd;

    //ros pub
    ros::Publisher robot_state_pub = nh.advertise<robot_msgs::RobotStates>("robot_states",10);
    ros::Publisher algomsg_pub = nh.advertise<std_msgs::UInt8MultiArray>("algomsg_others",10);
    ros::Publisher cmd_pub = nh.advertise<robot_msgs::HostCmdArray>("host_cmd",10);

    

    ros::Rate loop(20);
    while(ros::ok()){
        robot_msgs.clear();
        host_msg.clear();
        //deal with msgs from host
        state_receive.receiveMsg(robot_msgs);
        host_receive.receiveMsg(host_msg);
        if(!host_msg.empty()){
            decodeHostMsg(host_msg,host_cmd);
            if(host_cmd.message_type != HostCmd::HostMessageType::Cmd){
                pubHostCmd(host_cmd,cmd_pub);
            }
        }
        //deal with msgs from robots
        if(!robot_msgs.empty()){
            for(auto msg:robot_msgs){
                Json::Value json = string2json(msg);
                if(json["message_type"] == "control_msg"){
                    pubAlgomsg(msg,algomsg_pub);
                }
                else{
                    decodeRobotMsgs(msg,id2states);
                }
            }
        }
        pubStatus(id2states,robot_state_pub);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

