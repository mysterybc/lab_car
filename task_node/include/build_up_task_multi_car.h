#pragma once
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "robot_msgs/Planning.h"
#include "actionlib/server/simple_action_server.h"
#include "robot_msgs/BuildUpAction.h"
#include "robot_msgs/Cmd.h"
#include "std_msgs/builtin_string.h"
#include "robot_msgs/Separate.h"
#include <thread>
#include <string>

class BuildUpTask{
public:
    BuildUpTask();
    ~BuildUpTask() = default;

    void pub_path(nav_msgs::Path path,ros::Publisher pub);
    void PreemptCallback();
    void ExcuteCallback(const robot_msgs::BuildUpGoalConstPtr &goal);
    void PathTrackingCallback(const std_msgs::StringConstPtr &msg);
    void PubTrackingCmd(robot_msgs::Cmd cmd);

    ros::NodeHandle nh;
    ros::Publisher path_pub_;
    ros::Publisher cmd_pub;
    ros::Subscriber path_tracking_sub;
    ros::ServiceClient planning_client;
    ros::ServiceClient separate_client;
    actionlib::SimpleActionServer<robot_msgs::BuildUpAction> build_up_action;
    std::thread *pub_path_thread;
    int car_id;
    std::string path_tracking_state;
    bool pub_path_flag;
    geometry_msgs::Pose goal_point;
    robot_msgs::BuildUpResult result;


};