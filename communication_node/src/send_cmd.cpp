#include <ros/ros.h>
#include "zmq_test.h"
#include <jsoncpp/json/json.h> 
#include <string>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_cmd", ros::init_options::AnonymousName);

    ros::Time::init();
    ros::Rate loop_rate(10);

    ZMQ_TEST *zmq_test;
    zmq_test = new ZMQ_TEST();
    string ip_local = "tcp://192.168.1.110:";
    zmq_test->zmq_init(0,1,6666,ip_local);      //bind,pub,port,local_ip
    int count = 0;
    Json::Value json1;
    json1["type"] = 1;
    json1["ID"] = 1;
    double test_a[3] = {6,12,0};
    for (int i=0;i<3;i++)
    {
        json1["instruction"].append(test_a[i]);
    }
    cout<<json1.toStyledString()<<endl;
    while(ros::ok())
    {
        std::string cmd;
        std::cin >> cmd;
        if(cmd == "send")
            zmq_test->send_data(json1.toStyledString());
        loop_rate.sleep();
    }

    return 0;
}