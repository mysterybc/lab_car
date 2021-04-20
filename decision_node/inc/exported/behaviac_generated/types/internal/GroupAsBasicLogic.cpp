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
		logger.DEBUGINFO(g_BlackBoardAgent->car_id,"cancel assemble goal");
		g_TaskRealizeAgent->build_up_action->cancelGoal();
		break;

	case March_gps:
		logger.DEBUGINFO(g_BlackBoardAgent->car_id,"cancel march goal");
		g_TaskRealizeAgent->gps_march_action->cancelGoal();
		break;

	case March_laser:
		logger.DEBUGINFO(g_BlackBoardAgent->car_id,"cancel march goal");
		g_TaskRealizeAgent->laser_march_action->cancelGoal();
		break;
	case Search:
		logger.DEBUGINFO(g_BlackBoardAgent->car_id,"cancel Search goal");
		g_TaskRealizeAgent->search_action->cancelGoal();
		break;	
    case Remote_Control:
         g_GroupAsBasicLogicAgent->CurrentTask = TaskIndividual::NonTask;
         g_TaskRealizeAgent->fore_func_state = ForeFuncState::Idle;
         break;

	default://其中含NonTask
		break;
	}
}

bool GroupAsBasicLogic::GroupIdle()
{
	bool group_idle=true;
	auto iter=GroupState.begin();
	// logger.DEBUGINFO(g_BlackBoardAgent->car_id,"the size is:%d",	GroupState.size());
	//test
	// for(;iter!=GroupState.end();iter++)
	// {
	// 	 logger.DEBUGINFO(g_BlackBoardAgent->car_id,"GroupState:%d",	*iter);
	// }
	//test
	// for(int j=0;j<GroupState.size();j++)//组内状态
	// {
	// 	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"GroupState truly is:%d",	*(iter));
	// 	if(*(++iter)==ForeFuncState::Running)//当前Success和Failure都作为Idle处理
	// 		{
	// 			group_idle=false;//有非空闲状态
	// 			logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Not Pass");	
	// 		}
	// }
	iter=GroupState.begin();
	for(;iter!=GroupState.end();iter++)
	{
		// logger.DEBUGINFO(g_BlackBoardAgent->car_id,"GroupState truly is :%d",	*iter);
		if(*iter==ForeFuncState::Running)//当前Success和Failure都作为Idle处理
		{
			group_idle=false;//有非空闲状态
			// logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Not Pass");	
		}
	}
	
	// if(group_idle==true)
	// 	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"PASS!!!");
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
	case Search:
		g_TaskRealizeAgent->Search();
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
	// if(g_BlackBoardAgent->TaskList.empty())
	// logger.DEBUGINFO(g_BlackBoardAgent->car_id,"jobs finish");
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
	// logger.DEBUGINFO(g_BlackBoardAgent->car_id,"CurrentTask:%d",CurrentTask);
	g_BlackBoardAgent->PubDecisionState();
	g_BlackBoardAgent->PubMembers();//定频率Pub

	//read topic of apriltag,if recognition then cancel search task.
}
