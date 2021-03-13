#pragma once

#include "source_node.h"
#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_broadcaster.h"
#include "robot_msgs/DebugInfo.h"
#include "robot_msgs/Cmd.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "thread"
#include "string"
#include "my_param_server.h"




class Encoder{
public:
    #define VEL_TO_RPM  826.0f//单位转换 m/s到rpm   v/(pi*d)*减速比   826.0f
    Encoder();
    ~Encoder()=default;
    int UpdateOdom();
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
private:    
    serial::Serial sp;
    ros::Publisher vel_pub;
    ros::Publisher odom_pub_;
    int car_id;
    //与车id相关的参数
    double turn_radius;        //小车0.26 大车0.29
    double twist_flag;         //小车-1 大车1
    //反馈参数
    double angular_fb_factor; // 
    double linear_fb_factor;  //小车3.7
    //控制参数
    double angular_cmd_factor; //小车1.2 大车0.7
    double linear_cmd_factor;  //小车大车均为0.7
};

class EncoderSource{
public:
    EncoderSource();
    ~EncoderSource()=default;
    void  CmdCallback(const robot_msgs::CmdConstPtr &msg);
    bool  LoadConfig(std::string file);
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  UpdateState();

    ros::Publisher  state_pub;
    ros::Subscriber cmd_sub;

    int car_id;
    State node_state;
    Error node_error;
    uint8_t update_frequence;
    std::string node_name;
    std::thread* encoder_thread_;
    Encoder encoder;
};

