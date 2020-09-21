#include "build_up_task.h"

BuildUpTask::BuildUpTask()
    :build_up_action(nh,"build_up_action",boost::bind(&BuildUpTask::ExcuteCallback,this,_1),false),
    local_planner_action(nh,"local_planner_node_action",true)
{
    car_id = 1;
    pub_path_flag = false;
    local_planner_finish = false;
    build_up_action.registerPreemptCallback(std::bind(&BuildUpTask::PreemptCallback,this));
    client = nh.serviceClient<robot_msgs::Planning>("global_planning");
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 10);
    build_up_action.start();
    ROS_INFO("build up task initialized");
}

void BuildUpTask::PreemptCallback(){
    if(build_up_action.isPreemptRequested()){
        result.succeed = false;
        build_up_action.setAborted(result,"goal cancel");
    }
}

void BuildUpTask::ExcuteCallback(const robot_msgs::BuildUpGoalConstPtr &goal){
    //set goal
    ROS_INFO("get goal");
    geometry_msgs::Pose origin_goal;
    goal_point = goal.get()->goal;
    origin_goal = goal_point;
    float temp[4][2] = {{-1,-1},{-1,1},{1,1},{1,-1}};
    goal_point.position.x += temp[car_id-1][0] * 0.5;
    goal_point.position.y += temp[car_id-1][1] * 0.5;
    ROS_INFO("target position is [%f,%f],the car number is %d, build up target position is [%f,%f]",origin_goal.position.x,origin_goal.position.y,car_id,goal_point.position.x,goal_point.position.y);
    //1、call global planner
    //2、call local planner
    ROS_INFO("set planning");
    robot_msgs::Planning planner;
    planner.request.goal = goal_point;
    if (client.call(planner))
    {
        if(planner.response.path.poses.size() == 0){
            ROS_ERROR("wrong target! please select right target position");
            build_up_action.setAborted(result,"planning error");
        }
        ROS_INFO("call global_planning succeed");
        pub_path_thread = new std::thread(&BuildUpTask::pub_path,this,planner.response.path,path_pub_);
        robot_msgs::LocalPlannerGoal path;
        path.route = planner.response.path;
        local_planner_action.sendGoal(  path ,
                                        boost::bind(&BuildUpTask::DoneCallback,this,_1,_2),
                                        boost::bind(&BuildUpTask::ActiveCallback,this),
                                        boost::bind(&BuildUpTask::FeedbackCallback,this,_1));
        pub_path_flag = true;
    }
    else
    {
        ROS_ERROR("wrong target! please select right target position");
        pub_path_flag = false;
        result.succeed = false;
        build_up_action.setAborted(result,"planning server error");
    }
    //路径发送有人端
    //可能需要有人端确认
    //判断是否到达目标点
    ros::Rate loop(5);
    ROS_INFO("local planner run");
    while(!local_planner_finish && build_up_action.isActive()){
        robot_msgs::BuildUpFeedback feedback;
        feedback.error_occured = false;
        build_up_action.publishFeedback(feedback);
        loop.sleep();
        ros::spinOnce();
    }
    if(!build_up_action.isActive()){
        return ;
    }
    if(local_planner_action.getResult()->error_code == 1){
        result.succeed = true;
        ROS_INFO("build up task finished");
        build_up_action.setSucceeded(result,"goal_reached");
    }
    else{
        result.succeed = false;
        ROS_INFO("build up task failed");
        build_up_action.setAborted(result,"error occured");
    }
}

void BuildUpTask::pub_path(nav_msgs::Path path,ros::Publisher pub){
    ros::Rate loop(10);
    while(ros::ok()){
        pub.publish(path);
        if(pub_path_flag)
            return ;
        loop.sleep();
        ros::spinOnce();
    }
}


void BuildUpTask::DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::LocalPlannerResultConstPtr &result){
    local_planner_finish = true;
}
void BuildUpTask::ActiveCallback(){
    local_planner_finish = false;
}
void BuildUpTask::FeedbackCallback(const robot_msgs::LocalPlannerFeedbackConstPtr &feedback){

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
