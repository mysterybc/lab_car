#include "navigation_source.h"
#include "robot_msgs/SourceNodeMsg.h"

NavigationSource::NavigationSource(){
    ros::NodeHandle nh;
    std::string path = "config file path";
    if(LoadConfig(path)){
        node_state = State::HAVE_CONFIG;
    }else{
        node_error = Error::NO_ERROR;
        ROS_WARN("navigation_node: Config failed");
    }
    state_pub = nh.advertise<robot_msgs::SourceNodeMsg>("navigation_state",10);
    cmd_sub = nh.subscribe<robot_msgs::Cmd>("navigation_cmd",10,&NavigationSource::CmdCallback,this);
}


void NavigationSource::UpdateState(){
    ros::NodeHandle nh;
    ros::Rate loop(update_frequence);
    double start_time = ros::Time().now().toSec();
    ROS_INFO("navigation_node: state thread start");
    while(nh.ok() && node_state!=State::EXIT){
        robot_msgs::SourceNodeMsg msg;
        //TODO 目前获取的是ros秒可能需要进一步处理
        msg.time = ros::Time().fromSec(ros::Time().now().toSec() - start_time);
        msg.node_name = "navigation_node";
        msg.error.error = (uint8_t)node_error;
        msg.state.state = (uint8_t)node_state;
        state_pub.publish(msg);
        loop.sleep();
        ros::spinOnce();
    }
}



bool NavigationSource::LoadConfig(std::string file){
    ROS_INFO("navigation_node: load config");
    //TODO 应该在这里读取配置文件，目前只初始化更新频率和节点名称；
    update_frequence = 10;
    node_name = "navigation_node";
}

State NavigationSource::Start(){
    ROS_INFO("navigation source running");
    //gps_.Start();
    imu_.Start();
    return State::RUNNING;
}
//Stop需要reset参数
State NavigationSource::Stop(){
    ROS_INFO("navigation source stop");
    //gps_.Stop();
    imu_.Stop();
    return State::STOP;
}
//退出需要清理线程
State NavigationSource::Exit(){
    ROS_INFO("navigation source exit");
    //gps_.Exit();
    imu_.Exit();
    state_pub.shutdown();
    cmd_sub.shutdown();
    return State::EXIT;
}
//pause和resume不需要对数据进行处理
State NavigationSource::Pause(){
    if(node_state!=State::RUNNING)
        return node_state;
    ROS_INFO("navigation source pause");
    //gps_.Pause();
    imu_.Pause();
    return State::PAUSED;
}
State NavigationSource::Resume(){
    if(node_state!=State::PAUSED)
        return node_state;
    ROS_INFO("navigation source resume");
    //gps_.Resume();
    imu_.Resume();
    return State::RUNNING;
}

void NavigationSource::CmdCallback(const robot_msgs::CmdConstPtr &msg){
    ROS_INFO("navigation source get command");
    switch(msg->cmd){
        case (int)Cmd::START : node_state = Start(); break;
        case (int)Cmd::PAUSE : node_state = Pause(); break;
        case (int)Cmd::STOP  : node_state = Stop();  break;
        case (int)Cmd::RESUME: node_state = Resume();break;
        case (int)Cmd::EXIT  : node_state = Exit();  break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_source");
    NavigationSource navigation_source;
    navigation_source.UpdateState();
    return 0;
}




