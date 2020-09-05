#include "ros/ros.h"
#include "iostream"
#include "robot_msgs/Planning.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "AStar.hpp"
#include <thread>
using namespace std;

bool pub_path_flag = false;
geometry_msgs::Pose goal;
bool get_goal = false;

void pub_path(nav_msgs::Path path,ros::Publisher pub){
    ros::NodeHandle nh;
    ros::Rate loop(10);
    while(ros::ok()){  
        pub.publish(path);
        if(pub_path_flag)
            return ;
        loop.sleep();
        ros::spinOnce();
    }
}

void GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    goal = msg->pose;
    get_goal = true;
}


int main(int argc, char **argv){
    ros::init(argc, argv,"test_planning");
    ros::NodeHandle nh;
    robot_msgs::Planning planner;
    ros::ServiceClient client = nh.serviceClient<robot_msgs::Planning>("global_planning");
    ros::Publisher path_pub_ = nh.advertise<nav_msgs::Path>("path", 10);
    ros::Subscriber goal_sub = nh.subscribe("move_base_simple/goal",10,GoalCallback);
    std::thread *pub_path_thread;
    ROS_INFO("please set goal");
    ros::Rate loop(100);
    while(ros::ok()){
        loop.sleep();
        ros::spinOnce();
        if(!get_goal){
            continue;
        }
        get_goal = false;
        planner.request.goal = goal;
        if (client.call(planner))
        {
            ROS_INFO("call global_planning succeed");
            pub_path_thread = new std::thread(pub_path,planner.response.path,path_pub_);
            pub_path_flag = true;
        }
        else
        {
            ROS_ERROR("Failed to call golbal_planning");
            pub_path_flag = false;
        }
        ROS_INFO("please set goal");
    }
    return 0;
}

