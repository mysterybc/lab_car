#include "driver_source.h"

DriverSource::DriverSource(){
    ros::NodeHandle nh;
    std::string path = "config file path";
    if(LoadConfig(path)){
        node_state = State::HAVE_CONFIG;
    }else{
        node_error = Error::NO_ERROR;
        ROS_WARN("driver_node: Config failed");
    }
    state_pub = nh.advertise<robot_msgs::SourceNodeMsg>("driver_state",10);
    cmd_sub = nh.subscribe<robot_msgs::Cmd>("driver_cmd",10,&DriverSource::CmdCallback,this);
}


void DriverSource::UpdateState(){
    ros::NodeHandle nh;
    ros::Rate loop(update_frequence);
    double start_time = ros::Time().now().toSec();
    ROS_INFO("driver_node: state thread start");
    while(nh.ok() && node_state!=State::EXIT){
        robot_msgs::SourceNodeMsg msg;
        //TODO 目前获取的是ros秒可能需要进一步处理
        msg.time = ros::Time().fromSec(ros::Time().now().toSec() - start_time);
        msg.node_name = "driver_node";
        msg.error.error = (uint8_t)node_error;
        msg.state.state = (uint8_t)node_state;
        state_pub.publish(msg);
        loop.sleep();
        ros::spinOnce();
    }
}



bool DriverSource::LoadConfig(std::string file){
    ROS_INFO("driver_node: load config");
    //TODO 应该在这里读取配置文件，目前只初始化更新频率和节点名称；
    update_frequence = 10;
    node_name = "driver_node";
}

State DriverSource::Start(){
    ROS_INFO("driver source running");
    return State::RUNNING;
}
//Stop需要reset参数
State DriverSource::Stop(){
    ROS_INFO("driver source stop");
    return State::STOP;
}
//退出需要清理线程
State DriverSource::Exit(){
    ROS_INFO("driver source exit");
    state_pub.shutdown();
    cmd_sub.shutdown();
    return State::EXIT;
}
//pause和resume不需要对数据进行处理
State DriverSource::Pause(){
    if(node_state!=State::RUNNING)
        return node_state;
    ROS_INFO("driver source pause");
    return State::PAUSED;
}
State DriverSource::Resume(){
    if(node_state!=State::PAUSED)
        return node_state;
    ROS_INFO("driver source resume");
    return State::RUNNING;
}

void DriverSource::CmdCallback(const robot_msgs::CmdConstPtr &msg){
    ROS_INFO("driver source get command");
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
    ros::init(argc, argv, "driver_source");
    DriverSource driver_source;
    driver_source.UpdateState();
    return 0;
}


