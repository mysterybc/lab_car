#include "control_node/remote_control_node.h"
#include "my_debug_info.h"
#include "my_param_server.h"
Debug::DebugLogger logger;

RemoteControlNode::RemoteControlNode(){
    ros::NodeHandle nh;
    std::string path = "config file path";
    my_lib::GetParam("publish_robot_state",&car_id);
    logger.init_logger(car_id);
    if(LoadConfig(path)){
        node_state = State::HAVE_CONFIG;
    }else{
        logger.WARNINFO(car_id,"remote control _node: Config failed");
    }
    state_pub = nh.advertise<robot_msgs::PerceptionNodeMsg>("remote_control_state",10);
    cmd_sub = nh.subscribe<robot_msgs::Cmd>("remote_control_cmd",10,&RemoteControlNode::CmdCallback,this);
}


bool RemoteControlNode::LoadConfig(std::string file){
    logger.DEBUGINFO(car_id,"remote control _node: load config");
    //TODO 应该在这里读取配置文件，目前只初始化更新频率和节点名称；
    update_frequence = 10;
    node_name = "remote control node";
}

void RemoteControlNode::UpdateState(){
    ros::NodeHandle nh;
    ros::Rate loop(update_frequence);
    double start_time = ros::Time().now().toSec();
    logger.DEBUGINFO(car_id,"remote control _node: state thread start");
    while(nh.ok() && node_state!=State::EXIT){
        robot_msgs::PerceptionNodeMsg msg;
        //TODO 目前获取的是ros秒可能需要进一步处理
        msg.time = ros::Time().fromSec(ros::Time().now().toSec() - start_time);
        msg.node_name = "driver_node";
        msg.state.state = (uint8_t)node_state;
        //msg.state.pose = 
        state_pub.publish(msg);
        loop.sleep();
        ros::spinOnce();
    }
}


State RemoteControlNode::Start(){
    logger.DEBUGINFO(car_id,"remote control source running");
    return State::RUNNING;
}
//Stop需要reset参数
State RemoteControlNode::Stop(){
    logger.DEBUGINFO(car_id,"remote control source stop");
    return State::STOP;
}
//退出需要清理线程
State RemoteControlNode::Exit(){
    logger.DEBUGINFO(car_id,"remote control source exit");
    state_pub.shutdown();
    cmd_sub.shutdown();
    return State::EXIT;
}
//pause和resume不需要对数据进行处理
State RemoteControlNode::Pause(){
    if(node_state!=State::RUNNING)
        return node_state;
    logger.DEBUGINFO(car_id,"remote control source pause");
    return State::PAUSED;
}
State RemoteControlNode::Resume(){
    if(node_state!=State::PAUSED)
        return node_state;
    logger.DEBUGINFO(car_id,"remote control source resume");
    return State::RUNNING;
}

void RemoteControlNode::CmdCallback(const robot_msgs::CmdConstPtr &msg){
    logger.DEBUGINFO(car_id,"remote control  source get command");
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
    ros::init(argc, argv, "remote_control_source");
    RemoteControlNode remote_control_node;
    remote_control_node.UpdateState();
    return 0;
}




//以下是功能
RemoteControl::RemoteControl()
{
    nh_.param("/is_simulation",is_simulation,true);
    joystrick_drive_ = true;
    subJoy_   = nh_.subscribe("joy", 1000, &RemoteControl::joyCallback, this);
    if(is_simulation){
        // subTwist_ = nh_.subscribe("cmd_vel", 1000, &RemoteControl::twistCallback, this);
        pubTwist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    }
    else{
        subTwist_ = nh_.subscribe("cmd_vel", 1000, &RemoteControl::twistCallback, this);
        pubTwist_ = nh_.advertise<geometry_msgs::Twist>("komodo/cmd_vel", 1000);
    }
    
}
//如果在手动模式下，就发布手柄控制命令
void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->buttons[4] || msg->buttons[5]){
        joystrick_drive_ = false;
    }
    else{
        joystrick_drive_ = true;
    }
    
    geometry_msgs::Twist out_cmd_vel;
    if (! joystrick_drive_)
        out_cmd_vel = in_cmd_vel_;
    else {
        out_cmd_vel.angular.z = -MAX_ANGULAR_VEL*msg->axes[0];
        if(is_simulation){
            out_cmd_vel.linear.x = MAX_VEL*msg->axes[1];
        }
        else{
            out_cmd_vel.linear.x = -MAX_VEL*msg->axes[1];
        }
        
    }
    pubTwist_.publish(out_cmd_vel);
}
void RemoteControl::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // memorize the command
    in_cmd_vel_.angular = msg->angular;
    in_cmd_vel_.linear  = msg->linear;
    // publish the command if joystick button pressed
    if (! joystrick_drive_) {
        geometry_msgs::Twist out_cmd_vel;
        out_cmd_vel = in_cmd_vel_;
        pubTwist_.publish(out_cmd_vel);
    }
    
}


