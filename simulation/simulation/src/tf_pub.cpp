#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <ros/ros.h>
#include <iostream>

ros::Subscriber odom_sub;
ros::Publisher  tf_pub;
geometry_msgs::Point Start_point;
bool first_flag{true};


void OdomCallback(const nav_msgs::OdometryConstPtr &msg){
    geometry_msgs::TransformStamped odom_tf_;
    static tf::TransformBroadcaster tf_broadcaster_;
    if(first_flag){
        first_flag = false;
        //record start position
        Start_point.x = msg->pose.pose.position.x;
        Start_point.y = msg->pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch;
        tf::Matrix3x3(quat).getRPY(roll, pitch, Start_point.z);//进行转换
        //publish start position
        odom_tf_.header.stamp = msg->header.stamp;
        odom_tf_.header.frame_id = "map";
        odom_tf_.child_frame_id = msg->header.frame_id;
        odom_tf_.transform.translation.x = 0;
        odom_tf_.transform.translation.y = 0;
        odom_tf_.transform.translation.z = 0;
        geometry_msgs::Quaternion q;
        odom_tf_.transform.rotation = q;
        tf_broadcaster_.sendTransform(odom_tf_);
    }else{
        odom_tf_.header.stamp = msg->header.stamp;
        odom_tf_.header.frame_id = "map";
        odom_tf_.child_frame_id = msg->header.frame_id;
        odom_tf_.transform.translation.x = msg->pose.pose.position.x - Start_point.x;
        odom_tf_.transform.translation.y = msg->pose.pose.position.y - Start_point.y;
        odom_tf_.transform.translation.z = msg->pose.pose.position.z;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
        odom_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(yaw - Start_point.z);
        tf_broadcaster_.sendTransform(odom_tf_);

    }
    
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_pub");

    
    ros::NodeHandle nh;
    odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1000, &OdomCallback);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}