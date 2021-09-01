#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "iostream"
#include "thread"


void OnNewMsg(const std_msgs::Float64ConstPtr &msg){
    std::cout << " time between sub and pub is : " << ros::WallTime().now().toSec() - msg->data << std::endl;
}

void SubThread(){
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("test_sub_delay",10,&OnNewMsg);
    ros::Rate loop(100);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
}

int main(int argc,char** argv){
    ros::init(argc,argv,"test_sub_delay");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("test_sub_delay",10);
    // ros::Subscriber sub = nh.subscribe("test_sub_delay",1,&OnNewMsg);
    std::thread sub_thread(SubThread);
    ros::Rate loop(1);
    while(ros::ok()){
        std_msgs::Float64 data;
        data.data = ros::WallTime().now().toSec();
        pub.publish(data);
        ros::spinOnce();
        loop.sleep();
        
    }
    return 0;
}