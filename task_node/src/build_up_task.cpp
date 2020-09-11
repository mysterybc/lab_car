#include "build_up_task.h"


BuildUpTask::BuildUpTask()
    :build_up_action(nh,"build_up_action",boost::bind(&BuildUpTask::ExcuteCallback,this,_1),false)
{
    car_id = 1;
    pub_path_flag = false;
    build_up_action.registerPreemptCallback(std::bind(&BuildUpTask::PreemptCallback,this));
    client = nh.serviceClient<robot_msgs::Planning>("global_planning");
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 10);
    cmd_pub = nh.advertise<robot_msgs::Cmd>("path_tracking_cmd",10);
    path_tracking_sub = nh.subscribe<std_msgs::String>("path_tracking_state",10,&BuildUpTask::PathTrackingCallback,this);
    build_up_action.start();
    ROS_INFO("build up task initialized");
}

void BuildUpTask::PreemptCallback(){
    if(build_up_action.isPreemptRequested()){
        result.succeed = false;
        robot_msgs::Cmd cmd;
        cmd.cmd = cmd.PAUSE;
        PubTrackingCmd(cmd);
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
    //1、call路径规划服务
    //2、发布path
    //3、开启path_tracking任务
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
        robot_msgs::Cmd cmd;
        cmd.cmd = cmd.START;
        PubTrackingCmd(cmd);
        path_tracking_state = "running";
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
    ROS_INFO("path tracking running");
    while(path_tracking_state == "running" && build_up_action.isActive()){
        robot_msgs::BuildUpFeedback feedback;
        feedback.error_occured = false;
        build_up_action.publishFeedback(feedback);
        loop.sleep();
        ros::spinOnce();
    }
    if(!build_up_action.isActive()){
        return ;
    }
    if(path_tracking_state == "success"){
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

void BuildUpTask::PathTrackingCallback(const std_msgs::StringConstPtr &msg){
    path_tracking_state = msg->data;
}

void BuildUpTask::PubTrackingCmd(robot_msgs::Cmd cmd){
    ros::Rate loop(10);
    for(int i = 0; i < 2 ; i++){
        cmd_pub.publish(cmd);
        loop.sleep();
    }
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
