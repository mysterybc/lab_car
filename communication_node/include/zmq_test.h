#pragma once

#include "zmq.hpp"
#include <iostream>
#include <string>
#include <jsoncpp/json/json.h>
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"
#include "robot_msgs/HostCmd.h"
#include "robot_msgs/GetConfigCmd.h"
#include "ros/ros.h"

using namespace std;


//该类负责基础的信息接受、发送和链接
class ZMQ_TEST
{
public:
    ZMQ_TEST();
    ~ZMQ_TEST();
    void run();
    void zmq_init(int server_or_client, int sub_or_pub, string ip);
    void send_data(string payload);

    void *context;
    void *subscriber; //订阅
    void *publisher; //发布

};


