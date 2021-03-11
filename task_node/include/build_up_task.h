#pragma once
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib/server/simple_action_server.h"
#include "robot_msgs/BuildUpAction.h"
#include "robot_msgs/Cmd.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/builtin_string.h"
#include "robot_msgs/Separate.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "robot_msgs/GetConfigCmd.h"
#include "robot_msgs/ReportPath.h"
#include "nav_msgs/GetPlan.h"
#include "robot_msgs/DebugInfo.h"
#include <thread>
#include <string>

class BuildUpTask{
public:
    #define CANCEL 2
    BuildUpTask();
    ~BuildUpTask() = default;

    void BuildUpPreemptCB();
    void BuildUpExcuteCB(const robot_msgs::BuildUpGoalConstPtr &goal);
    void PubGoal(geometry_msgs::Pose pose);
    void MoveBaseStatusCB(const actionlib_msgs::GoalStatusArrayConstPtr &msg);
    bool MakePlanWithoutExcute(geometry_msgs::Pose goal);
    void ResetTask();


    ros::NodeHandle nh;
    ros::Publisher goal_pub_;
    ros::Subscriber statue_sub_;
    ros::ServiceClient report_path_client;
    ros::ServiceClient get_host_config_client;
    ros::ServiceClient make_plan_client;
    ros::ServiceClient separate_goal_client;
    actionlib::SimpleActionServer<robot_msgs::BuildUpAction> build_up_action;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> plan_action;
    int car_id;
    std::string tf_ns;
    geometry_msgs::Pose goal_point;
    robot_msgs::BuildUpResult result;
    actionlib_msgs::GoalStatus task_state;


};