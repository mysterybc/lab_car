﻿// -------------------------------------------------------------------------------
// THIS FILE IS ORIGINALLY GENERATED BY THE DESIGNER.
// YOU ARE ONLY ALLOWED TO MODIFY CODE BETWEEN '///<<< BEGIN' AND '///<<< END'.
// PLEASE MODIFY AND REGENERETE IT IN THE DESIGNER FOR CLASS/MEMBERS/METHODS, ETC.
// -------------------------------------------------------------------------------

#include "BlackBoard.h"

///<<< BEGIN WRITING YOUR CODE FILE_INIT
///<<< END WRITING YOUR CODE

BlackBoard::BlackBoard()
{
    ros::NodeHandle nh;
	car_number = 0;
///<<< BEGIN WRITING YOUR CODE CONSTRUCTOR
    cmd_sub = nh.subscribe<robot_msgs::HostCmd>("host_cmd",10,&BlackBoard::CmdCallback,this);   
    ROS_INFO("init blackborad");                                              
    decision_state_pub = nh.advertise<std_msgs::String>("decision_state",10);
    nh.getParam("car_id",car_number);
    car_number++;
    g_BasicLogicAgent->InputTask = TaskIndividual::NonTask;
///<<< END WRITING YOUR CODE
}

BlackBoard::~BlackBoard()
{
///<<< BEGIN WRITING YOUR CODE DESTRUCTOR

///<<< END WRITING YOUR CODE
}


///<<< BEGIN WRITING YOUR CODE FILE_UNINIT
void BlackBoard::CmdCallback(const robot_msgs::HostCmdConstPtr &msg){
    //首先判断我是否需要执行该指令；
    bool my_cmd = false;
    for(int i = 0; i < msg->car_id.size(); i++){
        if(msg->car_id.at(i) == car_number){
            my_cmd = true;
        }
    }
    if(!my_cmd){
        return ;
    }

    g_BasicLogicAgent->InputTask  = (TaskIndividual)msg->mission.mission;
    goal = msg->goal.pose;
}

void BlackBoard::PubDecisionState(std::string state){
    std_msgs::String msg;
    msg.data = state;
    decision_state_pub.publish(msg);
}
	
geometry_msgs::Pose BlackBoard::GetGoal(){
    return goal;
}
///<<< END WRITING YOUR CODE