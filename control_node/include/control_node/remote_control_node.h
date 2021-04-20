#pragma once
#include "control_node/control.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#define MAX_VEL 1.2f
#define MAX_ANGULAR_VEL 0.8f

class RemoteControl{
public:
    ~RemoteControl()=default;
    RemoteControl();
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    bool is_simulation;
private:
    ros::NodeHandle nh_;
    ros::Publisher  pubTwist_;
    ros::Subscriber subJoy_;
    ros::Subscriber subTwist_;

    geometry_msgs::Twist in_cmd_vel_; // input cmd_vel
    bool joystrick_drive_;
};

class RemoteControlNode:public ControlNode{
public:
    RemoteControlNode();
    ~RemoteControlNode()=default;
    bool  LoadConfig(std::string file);
    //这些函数应该在cmd的callback中被调用
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  UpdateState();
    void CmdCallback(const robot_msgs::CmdConstPtr &msg);

    RemoteControl remote_control;
    int car_id;
};

