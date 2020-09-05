#pragma once

#include "control_node/control.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "robot_msgs/Cmd.h"
#include "Tracking/AlgoTracking.hpp"
#include "std_msgs/builtin_string.h"
#include "robot_msgs/State.h"
#include "tf/tf.h"
#include <thread>


class PathTrackingNode:public ControlNode{
public:
    PathTrackingNode();
    ~PathTrackingNode()=default;
    bool  LoadConfig(std::string file);
    //这些函数应该在cmd的callback中被调用
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  Reset();
    void  UpdateState();
    void  CmdCallback(const robot_msgs::CmdConstPtr &msg);
    void  PathCallback(const nav_msgs::PathConstPtr &msg);
    void  RobotPoseCallback(const nav_msgs::OdometryConstPtr &msg);
    void  PathTrackingThread();
    void  StatePub(std::string state);
    impl::pt2D  PoseTospt2D(const geometry_msgs::Pose pose);
    double QuaternionToYaw(const geometry_msgs::Quaternion quaterion);

    //tracking
    impl::AlgoTracking tracking;     //算法
    impl::spline2D  path;            //接收路径，tracking.pline = &path
    geometry_msgs::Pose robot_pose;  //机器人位姿
    geometry_msgs::Pose target_pose; //目标机器人位姿
    geometry_msgs::Pose last_target_pose; //上次规划目标
    bool get_path;
    double max_v;
    
    //sub&pub
    ros::Publisher twist_pub;
    ros::Subscriber path_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber cmd_sub;
    std::thread *state_pub_thread;
    std::thread *path_tracking;


};