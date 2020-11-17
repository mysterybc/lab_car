//c++ standard
#include "algorithm"
//third party lib
#include "jsoncpp/json/json.h"
//ros
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "robot_msgs/RobotStates.h"
#include "tf/transform_datatypes.h"
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
void decodeRobotMsgs(std::vector<std::string> msgs,std::map<int,RobotState> &id2states){
    for(auto msg:msgs){
        if(msg.size() <= 5){
            continue;
        }
        RobotState state;
        Json::Value json;
        Json::Reader reader;
        reader.parse(msg.c_str(),json);
        if(json["message_type"].asString() != "state")
            continue ;
        state.have_config = true;
        state.id = json["id"].asInt();
        state.robot_state = json["state"].asString();
        state.robot_pose.position.x = json["pose"][0].asDouble();
        state.robot_pose.position.y = json["pose"][1].asDouble();
        state.robot_pose.position.z = 0;
        double yaw = json["pose"][2].asDouble();
        yaw = yaw / 180.0 * 3.1415926;
        state.robot_pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        id2states[state.id] = state;
    }
}

//解码上位机消息
void decodeHostMsgs(std::vector<std::string> msgs){
    
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
        state_msg.robot_state.data = state.second.robot_state;
        robot_states_msg.robot_states.push_back(state_msg);     
    }
    pub.publish(robot_states_msg);
}

//发布host cmd
void pubHostCmd(){

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
    std::vector<std::string> host_msgs;

    //ros pub
    ros::Publisher robot_state_pub = nh.advertise<robot_msgs::RobotStates>("robot_states",10);

    

    ros::Rate loop(20);
    while(ros::ok()){
        robot_msgs.clear();
        // host_msgs.clear();
        state_receive.receiveMsg(robot_msgs);
        // host_receive.receiveMsg(host_msgs);
        // if(!host_msgs.empty()){
        //     decodeHostMsgs(host_msgs);
        //     pubHostCmd();
        // }
        if(!robot_msgs.empty()){
            decodeRobotMsgs(robot_msgs,id2states);
            pubStatus(id2states,robot_state_pub);
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

