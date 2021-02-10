#include "ros/ros.h"
#include "robot_msgs/PathCoverage.h"

geometry_msgs::PoseStamped set_pose(double x,double y){
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    return pose;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"test_pathcoverage");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<robot_msgs::PathCoverage>("path_coverage");
    robot_msgs::PathCoverage path_coverage;
    path_coverage.request.select_point.poses.push_back(set_pose(6,6));
    path_coverage.request.select_point.poses.push_back(set_pose(6.1,20.1));
    path_coverage.request.select_point.poses.push_back(set_pose(20.1,6.1));
    path_coverage.request.select_point.poses.push_back(set_pose(20,20));
    client.call(path_coverage);
    return 0;
}