#include "zmq_test.h"
#include "iostream"
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "jsoncpp/json/json.h"

std::string state{"stand by"};

void DecisionSub(const std_msgs::StringConstPtr &msg){
    state = msg->data;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"send_robot_state");
    ros::NodeHandle nh;
    std::string ip_address;
    int car_number;
    nh.getParam("car_id",car_number);
    car_number ++;
    nh.getParam("ip_address",ip_address);
    ros::Subscriber state_subs;
    state_subs = nh.subscribe("decision_state",10,&DecisionSub);
    ZMQ_TEST *zmq_test;
    zmq_test = new ZMQ_TEST();
    zmq_test->zmq_init(0,1,6666,ip_address);
    ros::Rate loop(1);
    Json::Value json1;
    json1["state"] = state;
    cout << json1.toStyledString();
    while(ros::ok())
    {
        zmq_test->send_data(json1.toStyledString());
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}