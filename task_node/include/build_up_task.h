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
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include <thread>
#include <string>

enum ActionState{
    PENDING = 0,
    ACTIVE = 1,
    SUCCEEDED = 3,
    ABORTED = 4
};

class BuildUpTask{
public:
    BuildUpTask();
    ~BuildUpTask() = default;

    void BuildUpPreemptCB();
    void BuildUpExcuteCB(const robot_msgs::BuildUpGoalConstPtr &goal);
    void PubGoal(geometry_msgs::Pose pose);
    void MoveBaseStatusCB(const actionlib_msgs::GoalStatusArrayConstPtr &msg);


    ros::NodeHandle nh;
    ros::Publisher goal_pub_;
    ros::Subscriber statue_sub_;
    actionlib::SimpleActionServer<robot_msgs::BuildUpAction> build_up_action;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> plan_action;
    int car_id;
    geometry_msgs::Pose goal_point;
    robot_msgs::BuildUpResult result;
    ActionState task_state;


};