/** brief:头文件声明了remote_control_node
 *  note:
 *  
 */

#pragma once
#include "control_node/control.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#define MAX_VEL 1.2f
#define MAX_ANGULAR_VEL 0.8f

class RemoteControl{
public:
    ~RemoteControl()=default;
    RemoteControl()
    {
        joystrick_drive_ = true;
        subJoy_   = nh_.subscribe("joy", 1000, &RemoteControl::joyCallback, this);
        subTwist_ = nh_.subscribe("cmd_vel", 1000, &RemoteControl::twistCallback, this);
        pubTwist_ = nh_.advertise<geometry_msgs::Twist>("komodo/cmd_vel", 1000);
    }
    //如果在手动模式下，就发布手柄控制命令
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        if (msg->buttons[4] || msg->buttons[5])
            joystrick_drive_ = false;
        else
            joystrick_drive_ = true;

        geometry_msgs::Twist out_cmd_vel;
        if (! joystrick_drive_)
            out_cmd_vel = in_cmd_vel_;
        else {
            out_cmd_vel.angular.z = MAX_ANGULAR_VEL*msg->axes[2];
            out_cmd_vel.linear.x = MAX_VEL*msg->axes[3];
        }
        pubTwist_.publish(out_cmd_vel);
    }
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
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
};

