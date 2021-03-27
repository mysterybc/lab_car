#include "GroupAsBasicLogic.h"
extern Debug::DebugLogger logger;

GroupAsBasicLogic::GroupAsBasicLogic()
{
	CurrentTask = NonTask;
	GroupMember.push_back(g_BlackBoardAgent->car_id);
	GroupState.push_back(Idle);
	wait_for_CB = false;
}

GroupAsBasicLogic::~GroupAsBasicLogic()
{

}

void GroupAsBasicLogic::ActionCancel()
{
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
	bool group_idle=true;
	auto iter=GroupState.begin();
	for(int j=0;j<GroupState.size();j++)//组内状态
	{
		if(*(iter+j)==ForeFuncState::Running)//当前Success和Failure都作为Idle处理
			{
				group_idle=false;//有非空闲状态
			}
	}

	return group_idle;
}

bool GroupAsBasicLogic::MemberConsistent()
{
	bool member_consistent=true;
    for(auto i:MembersFromOtherMembers)
        if(GroupMember!=i)
			member_consistent= false;

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
	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"set a new goal");

	std::vector<geometry_msgs::Pose>().swap(g_BlackBoardAgent->goal);

	GroupMember.assign(g_BlackBoardAgent->TaskList.back().car_id.begin(),g_BlackBoardAgent->TaskList.back().car_id.end());
	for(auto i:g_BlackBoardAgent->TaskList.back().goal)
	{
		g_BlackBoardAgent->	goal.push_back(i.pose);
		logger.DEBUGINFO(g_BlackBoardAgent->car_id,"goal is x:%lf,y:%lf",i.pose.position.x,i.pose.position.y);
	}


	//g_BlackBoardAgent->	goal = g_BlackBoardAgent->TaskList.back().goal.pose;
}

bool GroupAsBasicLogic::TaskListEmpty()
{
		if(g_BlackBoardAgent->car_id==2)
	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"jobs finish");
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


bool GroupAsBasicLogic::IsForegrdFunc(const TaskIndividual& Task)//add ForegrdFunc only between nontask and assemble
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

	//read topic of apriltag,if recognition then cancel search task.
}
