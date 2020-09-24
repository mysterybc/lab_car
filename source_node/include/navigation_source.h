#pragma once

#include "source_node.h"
#include "gps.h"
#include "imu.h"
#include "ros/ros.h"
#include "robot_msgs/Cmd.h"

class NavigationSource{
public:
    NavigationSource();
    ~NavigationSource(){};
    bool  LoadConfig(std::string file);
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  UpdateState();
    void CmdCallback(const robot_msgs::CmdConstPtr &msg);
    //GPS gps_;
    IMU imu_;

    ros::Publisher  state_pub;
    ros::Subscriber cmd_sub;

    State node_state;
    Error node_error;
    uint8_t update_frequence;
    std::string node_name;
};



