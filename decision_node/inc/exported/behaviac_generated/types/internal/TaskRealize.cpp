#include "TaskRealize.h"
extern Debug::DebugLogger logger;

#define _cancel  2

TaskRealize::TaskRealize():
fore_func_state (Idle)
{
    build_up_action  = new actionlib::SimpleActionClient<robot_msgs::BuildUpAction>(nh,"build_up_action",true);
    gps_march_action  = new actionlib::SimpleActionClient<robot_msgs::MarchAction>(nh,"march_action_gps",true);
    laser_march_action  = new actionlib::SimpleActionClient<robot_msgs::MarchAction>(nh,"march_action_laser",true);
}

TaskRealize::~TaskRealize()
{
    delete build_up_action;
    delete gps_march_action;
    delete laser_march_action;
}

void TaskRealize::Assemble()
{
    if(g_BlackBoardAgent->car_id==1)
    ROS_INFO("Assemble_Start");
    build_up_action->waitForServer();
    assemble_goal.goal =g_BlackBoardAgent->GetGoal();
    assemble_goal.idList.clear();
    for(int i = 0 ; i < g_GroupAsBasicLogicAgent->GroupMember.size(); i++)
        assemble_goal.idList.push_back(g_GroupAsBasicLogicAgent->GroupMember[i]);
    build_up_action->sendGoal(   assemble_goal,
                                boost::bind(&TaskRealize::Assemble_DoneCallback,this,_1,_2),
                                boost::bind(&TaskRealize::Assemble_ActiveCallback,this),
                                boost::bind(&TaskRealize::Assemble_FeedbackCallback,this,_1));
}

void TaskRealize::March_gps()
{
    ROS_INFO("March_gps");
    gps_march_action->waitForServer();
    march_goal.goal =g_BlackBoardAgent->GetGoal();
    march_goal.idList.clear();
    for(int i = 0 ; i < g_GroupAsBasicLogicAgent->GroupMember.size(); i++)
        march_goal.idList.push_back(g_GroupAsBasicLogicAgent->GroupMember[i]);
    gps_march_action->sendGoal(   march_goal,
                                boost::bind(&TaskRealize::March_DoneCallback,this,_1,_2),
                                boost::bind(&TaskRealize::March_ActiveCallback,this),
                                boost::bind(&TaskRealize::March_FeedbackCallback,this,_1));
}

void TaskRealize::March_laser()
{
    ROS_INFO("March_laser");
    laser_march_action->waitForServer();
    march_goal.goal =g_BlackBoardAgent->GetGoal();
    march_goal.idList.clear();
    for(int i = 0 ; i < g_GroupAsBasicLogicAgent->GroupMember.size(); i++)
        march_goal.idList.push_back(g_GroupAsBasicLogicAgent->GroupMember[i]);
    laser_march_action->sendGoal(   march_goal,
                                boost::bind(&TaskRealize::March_DoneCallback,this,_1,_2),
                                boost::bind(&TaskRealize::March_ActiveCallback,this),
                                boost::bind(&TaskRealize::March_FeedbackCallback,this,_1));
}

void TaskRealize::Assemble_FeedbackCallback(const robot_msgs::BuildUpFeedbackConstPtr &feedback){
    if(feedback->error_occured)
        fore_func_state= ForeFuncState::Failure;
    else
        fore_func_state = ForeFuncState::Running;
    return ;
}

void TaskRealize::Assemble_ActiveCallback(void){
    fore_func_state = ForeFuncState::Running;
    return ;
}

void TaskRealize::Assemble_DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::BuildUpResultConstPtr &result){
    if(result->succeed!=_cancel)
        g_GroupAsBasicLogicAgent->CurrentTask=TaskIndividual::NonTask;
    if(g_BlackBoardAgent->car_id==2)
        logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Assemble Done");
    if(g_BlackBoardAgent->car_id==2&&g_GroupAsBasicLogicAgent->CurrentTask==TaskIndividual::NonTask)
        logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Assemble really really really Done");
    return ;
}

void TaskRealize::March_FeedbackCallback(const robot_msgs::MarchFeedbackConstPtr &feedback){
    if(feedback->error_occured){
        fore_func_state= ForeFuncState::Failure;
    }
    else{
        fore_func_state = ForeFuncState::Running;
    }
    return ;
}

void TaskRealize::March_ActiveCallback(void){
    fore_func_state = ForeFuncState::Running;
    return ;
}

void TaskRealize::March_DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::MarchResultConstPtr &result){
    if(result->succeed!=_cancel)
        g_GroupAsBasicLogicAgent->CurrentTask=TaskIndividual::NonTask;
    return ;
}
