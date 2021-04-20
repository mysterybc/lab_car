#include "build_up_task.h"
#include "my_debug_info.h"

Debug::DebugLogger logger;
BuildUpTask::BuildUpTask()
    :build_up_action(nh,"build_up_action",boost::bind(&BuildUpTask::BuildUpExcuteCB,this,_1),false),
    plan_action(nh,"move_base",true)
{
    param_server.GetParam("build_up_task_node");
    build_up_action.registerPreemptCallback(std::bind(&BuildUpTask::BuildUpPreemptCB,this));
    statue_sub_ = nh.subscribe("move_base/status",10,&BuildUpTask::MoveBaseStatusCB,this);
    report_path_client = nh.serviceClient<robot_msgs::ReportPath>("report_path");
    get_host_config_client = nh.serviceClient<robot_msgs::GetConfigCmd>("get_config_cmd");
    make_plan_client = nh.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
    separate_goal_client = nh.serviceClient<robot_msgs::Separate>("separate_goal");
    task_state.status = actionlib_msgs::GoalStatus::PENDING;
    build_up_action.start();

    //Debug info
    logger.init_logger(param_server.car_id);

}


void BuildUpTask::ResetTask(){
    if(build_up_action.isActive()){
        if(task_state.status == actionlib_msgs::GoalStatus::ACTIVE){
            plan_action.cancelGoal();
        }
        result.succeed = false;
        build_up_action.setAborted(result,"task failed");
    }
}

void BuildUpTask::BuildUpPreemptCB(){
    
    if(build_up_action.isPreemptRequested()){
        logger.DEBUGINFO(param_server.car_id,"action_preempt");
        if(task_state.status == actionlib_msgs::GoalStatus::ACTIVE)
            plan_action.cancelGoal();
        result.succeed = CANCEL;
        build_up_action.setPreempted(result,"goal cancel");
    }
}

bool BuildUpTask::MakePlanWithoutExcute(geometry_msgs::Pose goal){
    //call make plan 
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    ros::Rate rate(100.0);
    nav_msgs::GetPlan get_plan;
    get_plan.request.start.header.frame_id = "map";
    get_plan.request.start.header.stamp = ros::Time().now();
    get_plan.request.start.pose = start_point;
    get_plan.request.goal.header =  get_plan.request.start.header;
    get_plan.request.goal.pose = goal_point;
    get_plan.request.tolerance = 0.3;
    while(!ros::service::waitForService("move_base/make_plan",ros::Duration(2.0))){
        logger.DEBUGINFO(param_server.car_id,"waiting for service move_base/make_plan");
        ros::spinOnce();
    }
    if(make_plan_client.call(get_plan)){
        logger.DEBUGINFO(param_server.car_id,"call path");
    }
    else{
        logger.DEBUGINFO(param_server.car_id,"fail to call path");
        return false;
    }
    //report path
    robot_msgs::ReportPath report_path;
    report_path.request.Path = get_plan.response.plan;
    if(report_path_client.call(report_path)){
        logger.DEBUGINFO(param_server.car_id,"report  path");
    }
    else{
        logger.DEBUGINFO(param_server.car_id,"fail to report path");
        return false;
    }
    return true;
}

void BuildUpTask::BuildUpExcuteCB(const robot_msgs::BuildUpGoalConstPtr &goal){
    //set goal
    logger.DEBUGINFO(param_server.car_id,"get goal");
    goal_point = goal.get()->goal;
    //separate_goal
    robot_msgs::Separate separate_goal;
    separate_goal_client.waitForExistence(ros::Duration(1));
    separate_goal.request.goal = goal_point;
    separate_goal.request.idList = goal->idList;
    if(separate_goal_client.call(separate_goal)){
        logger.DEBUGINFO(param_server.car_id,"separate goal success!");
        goal_point = separate_goal.response.goal;
    }
    else{
        logger.DEBUGINFO(param_server.car_id,"separate goal failed!");
        ResetTask();
        return ;
    }
    logger.DEBUGINFO(param_server.car_id," car number is %d, build up target position is [%f,%f]",param_server.car_id,goal_point.position.x,goal_point.position.y);
    //make plan and report path
    if(!MakePlanWithoutExcute(goal_point)){
        ResetTask();
        return ;
    }
    //get host cmd
    robot_msgs::GetConfigCmd config_cmd;
    ros::Rate loop1(1);
    while(ros::ok()){
        if(!build_up_action.isActive()){
            return ;
        }
        if(get_host_config_client.call(config_cmd)){
            logger.DEBUGINFO(param_server.car_id,"get host cmd");
            if(config_cmd.response.cmd.data == "ok"){
                logger.DEBUGINFO(param_server.car_id,"path accepted!!");
                break;
            }
            else if(config_cmd.response.cmd.data == "no"){
                logger.DEBUGINFO(param_server.car_id,"replan!!");
                MakePlanWithoutExcute(goal_point);
            }
            else{
                
            }
        }
        else{
            logger.DEBUGINFO(param_server.car_id,"host cmd not accseeable!! retrying!!");
        }
        ros::spinOnce();
        loop1.sleep();
    }
    move_base_msgs::MoveBaseGoal planning_goal;
    planning_goal.target_pose.header.frame_id = "map";
    planning_goal.target_pose.header.stamp = ros::Time().now();
    planning_goal.target_pose.pose = goal_point;
    plan_action.sendGoal(planning_goal);
    ros::Rate loop(10);
    loop.sleep();
    while(ros::ok() && build_up_action.isActive()){
        ros::spinOnce();
        if(task_state.status == actionlib_msgs::GoalStatus::SUCCEEDED){
            result.succeed = true;
            logger.DEBUGINFO(param_server.car_id,"build up task finished");
            build_up_action.setSucceeded(result,"goal_reached");
            break;
        }
        else if(task_state.status == actionlib_msgs::GoalStatus::ABORTED){
            result.succeed = false;
            logger.DEBUGINFO(param_server.car_id,"build up task error");
            build_up_action.setAborted(result,"task failed");
            break;
        }
        else{
            robot_msgs::BuildUpFeedback feedback;
            feedback.error_occured = false;
            build_up_action.publishFeedback(feedback);
        }
        loop.sleep();
    }
    task_state.status = actionlib_msgs::GoalStatus::PENDING;
    
}

/* 以下是planning move_base的callback */
void BuildUpTask::MoveBaseStatusCB(const actionlib_msgs::GoalStatusArrayConstPtr &msg){
    if(msg->status_list.empty()){
        task_state.status = actionlib_msgs::GoalStatus::PENDING;
        return ;
    }
    task_state.status = (--msg->status_list.end())->status;    
}

void BuildUpTask::OnNewPose(const nav_msgs::OdometryConstPtr& msg){
    start_point = msg->pose.pose;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "build_up_task");
    BuildUpTask build_up_task;
    ros::Rate loop(10);
    while(ros::ok()){
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}
