#pragma once
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "robot_msgs/Planning.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "robot_msgs/BuildUpAction.h"
#include "robot_msgs/Cmd.h"
#include "std_msgs/builtin_string.h"
#include "robot_msgs/Separate.h"
#include "robot_msgs/LocalPlannerAction.h"
#include <thread>
#include <string>

class BuildUpTask{
public:
    BuildUpTask();
    ~BuildUpTask() = default;

    void pub_path(nav_msgs::Path path,ros::Publisher pub);
    void PreemptCallback();
    void ExcuteCallback(const robot_msgs::BuildUpGoalConstPtr &goal);
    void DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::LocalPlannerResultConstPtr &result);
    void ActiveCallback();
    void FeedbackCallback(const robot_msgs::LocalPlannerFeedbackConstPtr &feedback);


    ros::NodeHandle nh;
    ros::Publisher path_pub_;
    ros::ServiceClient client;
    actionlib::SimpleActionClient<robot_msgs::LocalPlannerAction> local_planner_action;
    actionlib::SimpleActionServer<robot_msgs::BuildUpAction> build_up_action;
    std::thread *pub_path_thread;
    int car_id;
    bool local_planner_finish;
    bool pub_path_flag;
    geometry_msgs::Pose goal_point;
    robot_msgs::BuildUpResult result;


};