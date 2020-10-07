#include "location_node.h"

LocalizationNode::LocalizationNode(){
    ros::NodeHandle nh;
    std::string path = "config file path";
    if(LoadConfig(path)){
        node_state = State::HAVE_CONFIG;
    }else{
        ROS_WARN("localization_node: Config failed");
    }
    state_pub = nh.advertise<robot_msgs::PerceptionNodeMsg>("localization_state",10);
    cmd_sub = nh.subscribe<robot_msgs::Cmd>("localization_cmd",10,&LocalizationNode::CmdCallback,this);
    fusion_thread = new std::thread(std::bind(&ImuGpsFusion::Fusion,&imu_gps_fusion));
}


bool LocalizationNode::LoadConfig(std::string file){
    ROS_INFO("localization_node: load config");
    node_name = "localization_node";
    update_frequence = 10;
}

void LocalizationNode::UpdateState(){
    ros::NodeHandle nh;
    ros::Rate loop(update_frequence);
    double start_time = ros::Time().now().toSec();
    ROS_INFO("localization_node: state thread start");
    while(nh.ok() && node_state!=State::EXIT){
        robot_msgs::PerceptionNodeMsg msg;
        //TODO 目前获取的是ros秒可能需要进一步处理
        msg.time = ros::Time().fromSec(ros::Time().now().toSec() - start_time);
        msg.node_name = "driver_node";
        msg.state.state = (uint8_t)node_state;
        imu_gps_fusion.GetPose(msg.pose);
        state_pub.publish(msg);
        loop.sleep();
        ros::spinOnce();
    }
}


State LocalizationNode::Start(){
    ROS_INFO("localization node running");
    return State::RUNNING;
}
//Stop需要reset参数
State LocalizationNode::Stop(){
    ROS_INFO("localization node stop");
    return State::STOP;
}
//退出需要清理线程
State LocalizationNode::Exit(){
    ROS_INFO("localization node exit");
    delete fusion_thread;
    state_pub.shutdown();
    cmd_sub.shutdown();
    return State::EXIT;
}
//pause和resume不需要对数据进行处理
State LocalizationNode::Pause(){
    if(node_state!=State::RUNNING)
        return node_state;
    ROS_INFO("localization node pause");
    return State::PAUSED;
}
State LocalizationNode::Resume(){
    if(node_state!=State::PAUSED)
        return node_state;
    ROS_INFO("localization node resume");
    return State::RUNNING;
}

void LocalizationNode::CmdCallback(const robot_msgs::CmdConstPtr &msg){
    ROS_INFO("localization node get command");
    switch(msg->cmd){
        case (int)Cmd::START : node_state = Start();  break;
        case (int)Cmd::PAUSE : node_state = Pause();  break;
        case (int)Cmd::STOP  : node_state = Stop();   break;
        case (int)Cmd::RESUME: node_state = Resume(); break;
        case (int)Cmd::EXIT  : node_state = Exit();   break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");
    LocalizationNode localization_node;
    localization_node.UpdateState();
    return 0;
}


