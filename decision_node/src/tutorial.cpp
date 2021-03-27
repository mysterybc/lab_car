#include "behaviac_generated/types/behaviac_types.h"
#include <ros/ros.h>
#include <ros/package.h>

#include "BlackBoard.h"

#include "iostream"

#define LOGI printf

bool set_behavior_tree;
extern Debug::DebugLogger logger;

behaviac::Agent* g_MakeTreeAgent=NULL;
BlackBoard* g_BlackBoardAgent=NULL;
TaskRealize* g_TaskRealizeAgent=NULL;
GroupAsBasicLogic* g_GroupAsBasicLogicAgent=NULL;

//初始化加载
bool InitBehavic()
{
	LOGI("InitBehavic\n");
	std::string package_path = ros::package::getPath("decision_node");
	std::string file_path = package_path + "/inc/exported";
	behaviac::Workspace::GetInstance()->SetFilePath(file_path.c_str());//行为树所在的目录

	behaviac::Workspace::GetInstance()->SetFileFormat(behaviac::Workspace::EFF_xml);//加载的行为树格式（xml）

	return true;
}
//初始化实例
bool InitPlayer()
{
	LOGI("InitPlayer\n");
	
	g_MakeTreeAgent = behaviac::Agent::Create<behaviac::Agent>();
	g_BlackBoardAgent = behaviac::Agent::Create<BlackBoard>("BlackBoard");
	g_TaskRealizeAgent=behaviac::Agent::Create<TaskRealize>("TaskRealize");
	g_GroupAsBasicLogicAgent=behaviac::Agent::Create<GroupAsBasicLogic>("GroupAsBasicLogic");
	bool bRet = g_MakeTreeAgent->btload("MainTree");
	g_MakeTreeAgent->btsetcurrent("MainTree");
	LOGI("InitPlayer finish\n");

	return bRet;
}

//执行行为树
void UpdateLoop()
{
	behaviac::EBTStatus status = behaviac::BT_RUNNING;

	while (status == behaviac::BT_RUNNING)
	{
		 status = g_MakeTreeAgent->btexec();//循环节点在没有直到成功时会返回BT_RUNNING，此时继续执行此语句，让程序返回原先位置。
		ros::spinOnce();//进入回调函数
		ros::Duration(0.1).sleep();
	}

}


//销毁实例，释放工作区
static void CleanupPlayer()
{
	LOGI("CleanupPlayer\n");

	g_MakeTreeAgent = NULL;
}

static void CleanupBehaviac()
{
	LOGI("CleanupBehaviac\n");

		behaviac::Workspace::GetInstance()->Cleanup();
}






int main(int argc, char** argv)
{
	BEHAVIAC_UNUSED_VAR(argc);
	BEHAVIAC_UNUSED_VAR(argv);
	//init
	ros::init(argc, argv, "behavior_test_node");
    ros::NodeHandle nh;
	InitBehavic();
	 InitPlayer();

	//Loop
	ros::Rate loop(20);
	int count = 0;
	while(ros::ok())//init information
		{
		g_BlackBoardAgent->PubDecisionState();
		g_BlackBoardAgent->PubMembers();
		break;
		}
	while(ros::ok()){
	// 	switch(g_BasicLogicAgent->CurrentTask){                               //给上位机发送，20hz
	// 	case 0:g_BlackBoardAgent->PubDecisionState(ForeFuncState::IDLE);
	// 	case 1:g_BlackBoardAgent->PubDecisionState(ForeFuncState::Running);
	// 	}
		// count ++;
		// if(count == 20){										//ROS_Info，1hz
		// 	switch(g_BasicLogicAgent->InputTask){
		// 		case 0: logger.DEBUGINFO(g_BlackBoardAgent->car_id,"InputTask:NONE TASK!!!");break;
		// 		case 1: logger.DEBUGINFO(g_BlackBoardAgent->car_id,"InputTask:gps march!!!");break;
		// 		case 2: logger.DEBUGINFO(g_BlackBoardAgent->car_id,"InputTask:laser march!!!");break;
		// 		case 3: logger.DEBUGINFO(g_BlackBoardAgent->car_id,"InputTask:ASSEMBLE TASK!!!");break;
		// 		case 4:	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"InputTask:STOP TASK!!!");break;
		// 		case 5:	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"InputTask:Pause TASK!!!");break;
		// 		case 6:	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"InputTask:Resume TASK!!!");break;
		// 	}
		// 	switch(g_BasicLogicAgent->CurrentTask){
		// 		case 0: logger.DEBUGINFO(g_BlackBoardAgent->car_id,"CurrentTask:NONE TASK!!!");break;
		// 		case 1: logger.DEBUGINFO(g_BlackBoardAgent->car_id,"CurrentTask:gps march!!!");break;
		// 		case 2: logger.DEBUGINFO(g_BlackBoardAgent->car_id,"CurrentTask:laser march!!!");break;
		// 		case 3: logger.DEBUGINFO(g_BlackBoardAgent->car_id,"CurrentTask:ASSEMBLE TASK!!!");break;
		// 		case 4:	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"CurrentTask:STOP TASK!!!");break;
		// 		case 5:	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"CurrentTask:Pause TASK!!!");break;
		// 		case 6:	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"CurrentTask:Resume TASK!!!");break;
		// 	}
			    //test
			// for(int i=0;i<g_GroupLogicAgent->GroupMember.size();i++)
			// {
			// 	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Member:%d,state:%d",g_GroupLogicAgent->GroupMember[i],g_GroupLogicAgent->GroupState[i]);
			// }
    //test


		// 	  count = 0;
		// }
		
		// UpdateLoop();
		if(set_behavior_tree)
		{
			if(g_BlackBoardAgent->car_id==2)
			logger.DEBUGINFO(g_BlackBoardAgent->car_id,"ForegrdFunc Tree Start");
			UpdateLoop();
			if(g_BlackBoardAgent->car_id==2)
			logger.DEBUGINFO(g_BlackBoardAgent->car_id,"ForegrdFunc Tree End");
			set_behavior_tree=false;
		}
		
        ros::spinOnce();
		loop.sleep();
    }
	 CleanupPlayer();

	CleanupBehaviac();

	return 0;
}

