#include "driver_source.h"
#include "robot_msgs/SourceNodeMsg.h"

EncoderSource::EncoderSource(){
    ros::NodeHandle nh;
    std::string path = "config file path";
    if(LoadConfig(path)){
        node_state = State::HAVE_CONFIG;
    }else{
        node_error = Error::NO_ERROR;
        ROS_WARN("encoder_node: Config failed");
    }
    state_pub = nh.advertise<robot_msgs::SourceNodeMsg>("encoder_state",10);
    cmd_sub = nh.subscribe("encoder_cmd",10,&EncoderSource::CmdCallback,this);
    encoder_thread_ = new std::thread(std::bind(&Encoder::UpdateOdom,&encoder));
}


bool EncoderSource::LoadConfig(std::string file){
    ROS_INFO("encoder_node: load config");
    //TODO 应该在这里读取配置文件，目前只初始化更新频率和节点名称；
    update_frequence = 10;
    node_name = "encoder_node";
}

void EncoderSource::UpdateState(){
    ros::NodeHandle nh;
    ros::Rate loop(update_frequence);
    double start_time = ros::Time().now().toSec();
    ROS_INFO("encoder_node: state thread start");
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


State EncoderSource::Start(){
    ROS_INFO("encoder source running");
    return State::RUNNING;
}
//Stop需要reset参数
State EncoderSource::Stop(){
    ROS_INFO("encoder source stop");
    return State::STOP;
}
//退出需要清理线程
State EncoderSource::Exit(){
    ROS_INFO("encoder source exit");
    delete encoder_thread_;
    state_pub.shutdown();
    cmd_sub.shutdown();
    return State::EXIT;
}
//pause和resume不需要对数据进行处理
State EncoderSource::Pause(){
    if(node_state!=State::RUNNING)
        return node_state;
    ROS_INFO("encoder source pause");
    return State::PAUSED;
}
State EncoderSource::Resume(){
    if(node_state!=State::PAUSED)
        return node_state;
    ROS_INFO("encoder source resume");
    return State::RUNNING;
}

void EncoderSource::CmdCallback(const robot_msgs::CmdConstPtr &msg){
    ROS_INFO("encoder source get command");
    switch(msg->cmd){
        case (int)Cmd::START : node_state = Start();  encoder.SetFlag(true);  break;
        case (int)Cmd::PAUSE : node_state = Pause();  encoder.SetFlag(false); break;
        case (int)Cmd::STOP  : node_state = Stop();   encoder.SetFlag(false); break;
        case (int)Cmd::RESUME: node_state = Resume(); encoder.SetFlag(true);  break;
        case (int)Cmd::EXIT  : node_state = Exit();   encoder.SetFlag(false); break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_source");
    EncoderSource encoder_source;
    encoder_source.UpdateState();
    return 0;
}


