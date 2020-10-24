#include "zmq_test.h"
#include "ros/ros.h"
#include "vector"
#include "geometry_msgs/Pose.h"
#include "robot_msgs/RobotStates.h"
#include "tf/transform_broadcaster.h"
#include "iostream"
#include "thread"
#include "memory"

//单个机器人状态
class RobotState{
public:
    RobotState(std::string ip);
    void RecvMsg();
    void DecodeMsg(Json::Value json);
    int GetId() const {return car_id;}
    std::string GetState() const {return robot_state;}
    geometry_msgs::Pose GetPose() const { return robot_pose;}
    bool HaveConfig() const { return have_config;}
private:
    bool have_config;
    int car_id;
    geometry_msgs::Pose robot_pose;
    std::string robot_state;
    std::shared_ptr<ZMQ_TEST> zmq_test;
    //TODO 需要添加每个接受消息的单独的线程
    //目前在线程中接受socket会产生问题
    // std::shared_ptr<std::thread> receive_robot_msg_thread;
};

RobotState::RobotState(std::string ip){
    have_config = false;
    robot_pose = geometry_msgs::Pose();
    car_id = 0;
    zmq_test = std::make_shared<ZMQ_TEST>(ZMQ_TEST());
    zmq_test->zmq_init(1,0,6666,ip);
    // receive_robot_msg_thread = std::make_shared<std::thread>(std::thread(&RobotState::RecvMsg,this));
};


// void RobotState::RecvMsg(){
//     Json::Value json;
//     Json::Reader jsonread;
//     char topic_name[256]={0}; //用于接收订阅的主题名
//     char payload[1024]={0};   //用于接收订阅主题的内容
//     while(ros::ok())
//     {
//         ros::spinOnce();
//         memset(topic_name,0,sizeof(topic_name));
//         memset(payload,0,sizeof(payload));

//         int size = zmq_recv (zmq_test->subscriber, topic_name, sizeof(topic_name), ZMQ_NOBLOCK); //接收订阅的主题名称
//         if (size == -1)
//         {
//             ROS_INFO("didn't get topic!!");
//             continue;
//         }
//         ROS_INFO(" get message!!");
//         size = zmq_recv (zmq_test->subscriber, payload, sizeof(payload), ZMQ_NOBLOCK); //接收订阅的消息
//         if (size == -1)
//         {
//             ROS_INFO("didn't get message!!");
//             ROS_INFO("recv payload error!!");
//             continue;
//         }
//         if(!jsonread.parse(payload,json))
//         {
//             ROS_INFO("json_error");
//         }
//         else
//         {
//             cout << "get command !!!" ;
//             DecodeMsg(json);
//         }
        
//     }
// }

void RobotState::RecvMsg(){
    Json::Value json;
    Json::Reader jsonread;
    char topic_name[256]={0}; //用于接收订阅的主题名
    char payload[1024]={0};   //用于接收订阅主题的内容
    ros::spinOnce();
    memset(topic_name,0,sizeof(topic_name));
    memset(payload,0,sizeof(payload));

    int size = zmq_recv (zmq_test->subscriber, topic_name, sizeof(topic_name), ZMQ_NOBLOCK); //接收订阅的主题名称
    if (size == -1)
    {
        return ;
    }
    size = zmq_recv (zmq_test->subscriber, payload, sizeof(payload), ZMQ_NOBLOCK); //接收订阅的消息
    if (size == -1)
    {
        return ;
    }
    if(!jsonread.parse(payload,json))
    {
        ROS_INFO("json_error");
    }
    else
    {
        DecodeMsg(json);
    }
        
}

void RobotState::DecodeMsg(Json::Value json){
    if(json["message_type"].asString() != "state")
        return ;
    have_config = true;
    car_id = json["id"].asInt();
    robot_state = json["state"].asString();
    robot_pose.position.x = json["pose"][0].asDouble();
    robot_pose.position.y = json["pose"][1].asDouble();
    robot_pose.position.z = 0;
    double yaw = json["pose"][2].asDouble();
    yaw = yaw / 180.0 * 3.1415926;
    robot_pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
}

//多机器人信息汇总
class RobotStates{
public:
    RobotStates();
    void PubStates();
    void ResvMsg();
    
private:
    std::vector<RobotState> robots_state;
    ros::Publisher robot_state_pub;
    std::vector<std::string> robot_ip;
};

void RobotStates::ResvMsg(){
    for(auto &robot : robots_state){
        robot.RecvMsg();
    }
}

//初始化ip群，topic和zmq
RobotStates::RobotStates(){
    ros::NodeHandle nh;
    std::string my_ip_address;
    std::vector<std::string> total_robot_ip;
    nh.getParam("my_ip_address",my_ip_address);
    nh.getParam("total_robot_ip",total_robot_ip);
    for(auto ip : total_robot_ip){
        if(ip == my_ip_address)
            continue;
        RobotState robot(ip);
        robots_state.push_back(robot);
    }

    robot_state_pub = nh.advertise<robot_msgs::RobotStates>("robot_states",10);
}

void RobotStates::PubStates(){
    robot_msgs::RobotStates robot_states_msg;
    robot_states_msg.online_robot_number = 0;
    for(const auto & robot : robots_state ){
        if(!robot.HaveConfig()){
            continue;
        }
        robot_states_msg.online_robot_number ++;
        robot_msgs::RobotState state_msg;
        state_msg.car_id = robot.GetId();
        state_msg.robot_pose = robot.GetPose();
        state_msg.robot_state.data = robot.GetState();
        robot_states_msg.robot_states.push_back(state_msg);     
    }
    robot_state_pub.publish(robot_states_msg);
}


int main(int argc, char** argv){
    ros::init(argc,argv,"receive_robot_state");
    ros::NodeHandle nh;
    RobotStates robot_states;
    

    ros::Rate loop(50);
    while(ros::ok()){
        robot_states.PubStates();
        robot_states.ResvMsg();
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}