#include "GroupAsBasicLogic.h"

GroupAsBasicLogic::GroupAsBasicLogic()
{
	CurrentTask = NonTask;
	GroupMember.push_back(g_BlackBoardAgent->car_id);
	GroupState.push_back(Idle);
	wait_for_CB = false;
	test_count=0;
}

GroupAsBasicLogic::~GroupAsBasicLogic()
{

}

void GroupAsBasicLogic::ActionCancel()
{
	    if(g_BlackBoardAgent->car_id==2)
		ROS_INFO("ActionCancel");
	switch (CurrentTask)
	{
	case Assemble:
		g_TaskRealizeAgent->build_up_action->cancelGoal();
		break;

	case March_gps:
		g_TaskRealizeAgent->gps_march_action->cancelGoal();
		break;

	case March_laser:
		g_TaskRealizeAgent->laser_march_action->cancelGoal();
		break;

	default://其中含NonTask
		break;
	}
}

bool GroupAsBasicLogic::GroupIdle()
{
	test_count++;
	// 	if(g_BlackBoardAgent->car_id==2)
	// logger.DEBUGINFO(g_BlackBoardAgent->car_id,"GroupIdle debugging");
	if(g_BlackBoardAgent->car_id==2&&wait_for_CB==true)
	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"wait_for_CB Error");
bool group_idle=true;
auto iter=GroupState.begin();
	// if(g_BlackBoardAgent->car_id==1)
	// ROS_INFO("car1's GroupState.size()is %d",GroupState.size());
for(int j=0;j<GroupState.size();j++)//组内状态
{
	// 	if(g_BlackBoardAgent->car_id==1)
	// ROS_INFO("car1's Members'state: %d",*(iter+j));
	if(*(iter+j)==ForeFuncState::Running)//当前Success和Failure都作为Idle处理
		{
			group_idle=false;//有非空闲状态
		}
}

	// if(g_BlackBoardAgent->car_id==1)
	// ROS_INFO("car1's GROUPIdleis %d",group_idle);
//g_BlackBoardAgent->car_id==2&&
		if(group_idle==true)
	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"GroupIdle Passed！！！！！！！！！！！！！！！！！");
return group_idle;

}

bool GroupAsBasicLogic::MemberConsistent()
{
	bool member_consistent=true;
    for(auto i:MembersFromOtherMembers)
	{
// if(g_BlackBoardAgent->car_id==4)
// {
// 		for(auto aa:i)
// 		ROS_INFO("car4's members'smembers:%d",aa);
// }

        if(GroupMember!=i)
			member_consistent= false;
	}

	// if(g_BlackBoardAgent->car_id==4)
	// for(auto j:GroupMember)
	// 	ROS_INFO("car4's members:%d",j);
	// 		if(g_BlackBoardAgent->car_id==4)
	// 	ROS_INFO("car4's member_consistent:%d",member_consistent);
		// if(g_BlackBoardAgent->car_id==2)
		// 	ROS_INFO("car1's MemberConsistent is %d",member_consistent);
	if(g_BlackBoardAgent->car_id==2&&member_consistent==true)
		logger.DEBUGINFO(g_BlackBoardAgent->car_id,"MemberConsistent Passed");
	return member_consistent;
}

void GroupAsBasicLogic::SendGoal()//Resume用的
{
	switch (CurrentTask)
	{
	case Assemble:
		g_TaskRealizeAgent->Assemble();
		break;

	case March_gps:
		g_TaskRealizeAgent->March_gps();
		break;

	case March_laser:
		g_TaskRealizeAgent->March_laser();
		break;

	default:
		break;
	}
}

void GroupAsBasicLogic::SetMemberAndGoal()
{
behaviac::vector<int>().swap(GroupMember);
GroupMember.assign(g_BlackBoardAgent->TaskList.back().car_id.begin(),g_BlackBoardAgent->TaskList.back().car_id.end());
if(g_BlackBoardAgent->car_id==2)
for(auto i:GroupMember)
	ROS_INFO("GroupMember Update:%d",i);
g_BlackBoardAgent->	goal = g_BlackBoardAgent->TaskList.back().goal.pose;
if(g_BlackBoardAgent->car_id==2)
ROS_INFO("goal update");
}

bool GroupAsBasicLogic::TaskListEmpty()
{
		if(g_BlackBoardAgent->car_id==2)
	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"g_BlackBoardAgent->TaskList.empty()=%d",g_BlackBoardAgent->TaskList.empty());//running 是3 behaviac::BT_SUCCESS是1
	return g_BlackBoardAgent->TaskList.empty();
}

TaskIndividual GroupAsBasicLogic::TaskListPop()
{
	TaskIndividual PopTask;
	if(g_BlackBoardAgent->TaskList.empty()!=true)
		PopTask=(TaskIndividual)g_BlackBoardAgent->TaskList.back().mission.mission;
	g_BlackBoardAgent->TaskList.pop_back();
	return PopTask;
}


bool GroupAsBasicLogic::IsForegrdFunc(const TaskIndividual& Task)
{
if(Task<=Assemble&&Task>NonTask)
	return true;
else
	return false;
}

void GroupAsBasicLogic::RealTimeProcessing()
{
		g_BlackBoardAgent->PubDecisionState();
		g_BlackBoardAgent->PubMembers();//定频率Pub
}
