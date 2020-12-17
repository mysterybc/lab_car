﻿// -------------------------------------------------------------------------------
// THIS FILE IS ORIGINALLY GENERATED BY THE DESIGNER.
// YOU ARE ONLY ALLOWED TO MODIFY CODE BETWEEN '///<<< BEGIN' AND '///<<< END'.
// PLEASE MODIFY AND REGENERETE IT IN THE DESIGNER FOR CLASS/MEMBERS/METHODS, ETC.
// -------------------------------------------------------------------------------
#include "BlackBoard.h"
#include "algorithm"
#include "vector"
///<<< BEGIN WRITING YOUR CODE FILE_INIT

///<<< END WRITING YOUR CODE





BlackBoard::BlackBoard()
{
		ros::NodeHandle nh;
    	car_number = 0;
///<<< BEGIN WRITING YOUR CODE CONSTRUCTOR
        cmd_sub = nh.subscribe("/host_cmd",10,&BlackBoard::CmdCallback,this);                                                            //1、2
		decision_state_pub = nh.advertise<robot_msgs::robot_states_enum>("decision_state",10);
        group_state_sub=nh.subscribe("robot_states",10,&BlackBoard::GroupStateCallback,this);
        nh.getParam("car_id",car_number);
        g_BasicLogicAgent->InputTask = TaskIndividual::NonTask;
///<<< END WRITING YOUR CODE
}

BlackBoard::~BlackBoard()
{
///<<< BEGIN WRITING YOUR CODE DESTRUCTOR

///<<< END WRITING YOUR CODE
}
void BlackBoard::GroupStateCallback(const robot_msgs::RobotStatesConstPtr &msg)
{

auto iter1=g_GroupLogicAgent->GroupMember.begin();
auto iter2=msg->robot_states.begin();

behaviac::vector<ForeFuncState>().swap(g_GroupLogicAgent->GroupState);
for(int i=0;i<g_GroupLogicAgent->GroupMember.size();i++)
{
    if(*(iter1+i)==car_number)//本车状态不在msg中，但要加入groupstates
    {
        g_GroupLogicAgent->GroupState.push_back(g_ForegrdFuncAgent->fore_func_state);
    }
    
    for(int j=0;j<msg->robot_states.size();j++)//在robot_states中找GroupMember的状态，加入group_states
    {
        if(*(iter1+i)==(iter2+j)->car_id)//carid==GroupMember[i]
        {
            g_GroupLogicAgent->GroupState.push_back((ForeFuncState)(iter2+j)->robot_state.robot_states_enum);

        }
    }

}

// if(car_number==2)
// {
// ROS_INFO("GroupMember_size:%d",g_GroupLogicAgent->GroupMember.size());
// 			for(int i=0;i<g_GroupLogicAgent->GroupMember.size();i++)
// 			{
// 				ROS_INFO("Member:%d",g_GroupLogicAgent->GroupMember[i]);
// 			}
// ROS_INFO("GroupState_size:%d",g_GroupLogicAgent->GroupState.size());
// }

}


///<<< BEGIN WRITING YOUR CODE FILE_UNINIT
void BlackBoard::CmdCallback(const robot_msgs::HostCmdArrayConstPtr &msg){

        //1.清空list
        //2.反向初始化，复制局部变量vector到类内变量
        //3.本车任务+数据类型转换
        //4.第一个InputTask立即给出，来立即打断执行中的任务。(单步行为树支持state=pause、running、IDLE下的新任务)
        //5.如果还有任务，交给任务列表
        //6.list到InputTask的转换由行为树中的BasicLogic::UpperCope()依时序实现。
        
        std::vector<robot_msgs::HostCmd>().swap(msgs);//1

        msgs.assign(msg->host_cmd_array.rbegin(),msg->host_cmd_array.rend());//2反向初始化：rbegin->rend.
        std::vector<robot_msgs::HostCmd>::iterator lr1=msgs.begin();
        std::vector<robot_msgs::HostCmd>::iterator lr2=msgs.begin();
        do
        {
            for(int i=0;i<lr1->car_id.size();i++){
                if(lr1->car_id.at(i)==car_number){
                    (*lr2++)=(*lr1);
                    break;
                }
            }
            if(lr1>=lr2){
            msgs.erase(lr1);
            }
            else
                lr1++;
        }while(lr1!=msgs.end());//3


        if(!msgs.empty())//有本车命令
    {
        g_BasicLogicAgent->InputTask  = (TaskIndividual)msgs.back().mission.mission;
        ROS_INFO("%d",g_BasicLogicAgent->InputTask);
        //if(g_BasicLogicAgent->InputTask==Assemble)
        goal = msgs.back().goal.pose;
        behaviac::vector<int>().swap(g_GroupLogicAgent->GroupMember);
        g_GroupLogicAgent->GroupMember.assign(msgs.back().car_id.begin(),msgs.back().car_id.end());
        msgs.pop_back();//4
        
       if(!msgs.empty())//本车命令总数>2
       {
        std::vector<robot_msgs::HostCmd>().swap(TaskList);//1
        TaskList.assign(msgs.begin(),msgs.end());//5
       }
    }

}
	void BlackBoard::PubDecisionState(ForeFuncState state){
        robot_msgs::robot_states_enum state_;
        state_.robot_states_enum = state;
        decision_state_pub.publish(state_);
    }
	
	geometry_msgs::Pose BlackBoard::GetGoal(){
        return goal;
    }
    ///<<< END WRITING YOUR CODE
