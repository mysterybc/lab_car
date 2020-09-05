#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <ros/ros.h>
#include <iostream>

ros::Subscriber odom_sub;
ros::Publisher  tf_pub;

void OdomCallback(const nav_msgs::OdometryConstPtr &msg){
    geometry_msgs::TransformStamped odom_tf_;
    static tf::TransformBroadcaster tf_broadcaster_;
    odom_tf_.header.stamp = msg->header.stamp;
    odom_tf_.header.frame_id = "map";
    odom_tf_.child_frame_id = msg->header.frame_id;
    odom_tf_.transform.translation.x = msg->pose.pose.position.x;
    odom_tf_.transform.translation.y = msg->pose.pose.position.y;
    odom_tf_.transform.translation.z = msg->pose.pose.position.z;
    odom_tf_.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_.sendTransform(odom_tf_);
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