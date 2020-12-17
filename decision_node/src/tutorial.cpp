#include "behaviac_generated/types/behaviac_types.h"
#include <ros/ros.h>

#include "BlackBoard.h"

#include "iostream"
#include "ros/package.h"
#define LOGI printf
behaviac::Agent* g_MakeTreeAgent=NULL;
BasicLogic* g_BasicLogicAgent=NULL;
BlackBoard* g_BlackBoardAgent=NULL;
ForegrdFunc* g_ForegrdFuncAgent=NULL;
BackgrdFunc* g_BackgrdFuncAgent=NULL;
GroupLogic* g_GroupLogicAgent=NULL;

//初始化加载
bool InitBehavic()
{
	LOGI("InitBehavic\n");

	std::string folder_path = ros::package::getPath("decision_node");
	std::string file_path = folder_path + "/inc/exported";
	behaviac::Workspace::GetInstance()->SetFilePath(file_path.c_str());//行为树所在的目录

	behaviac::Workspace::GetInstance()->SetFileFormat(behaviac::Workspace::EFF_xml);//加载的行为树格式（xml）

	return true;
}
//初始化实例
bool InitPlayer()
{
	LOGI("InitPlayer\n");
	
	g_MakeTreeAgent = behaviac::Agent::Create<behaviac::Agent>();
	g_BasicLogicAgent = behaviac::Agent::Create<BasicLogic>("BasicLogic");
	g_BlackBoardAgent = behaviac::Agent::Create<BlackBoard>("BlackBoard");
	g_ForegrdFuncAgent=behaviac::Agent::Create<ForegrdFunc>("ForegrdFunc");
	g_BackgrdFuncAgent=behaviac::Agent::Create<BackgrdFunc>("BackgrdFunc");
	g_GroupLogicAgent=behaviac::Agent::Create<GroupLogic>("GroupLogic");
	bool bRet = g_MakeTreeAgent->btload("MainTree");
	g_MakeTreeAgent->btsetcurrent("MainTree");

	return bRet;
}

//执行行为树
void UpdateLoop()
{
	//LOGI("UpdateLoop\n");
	behaviac::EBTStatus status = behaviac::BT_RUNNING;
		status = g_MakeTreeAgent->btexec();
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
	// auto BasicLogic_ptr=std::make_shared<BasicLogic>();
    // auto BlackBoard_ptr=std::make_shared<BlackBoard>(BasicLogic_ptr);
	// auto ForegrdFunc_ptr=std::make_shared<ForegrdFunc>(BasicLogic_ptr);
	// auto BackgrdFunc_ptr=std::make_shared<BackgrdFunc>(BasicLogic_ptr);
    ros::NodeHandle nh;

	InitBehavic();
	InitPlayer();

	//Loop
	ros::Rate loop(20);
	int count = 0;
	while(ros::ok()){
	// 	switch(g_BasicLogicAgent->CurrentTask){                               //给上位机发送，20hz
	// 	case 0:g_BlackBoardAgent->PubDecisionState(ForeFuncState::IDLE);
	// 	case 1:g_BlackBoardAgent->PubDecisionState(ForeFuncState::Running);
	// 	}

		count ++;
		if(count == 20){										//ROS_Info，1hz
		if(g_BlackBoardAgent->car_number==1){//INFO 2车 state
			switch(g_BasicLogicAgent->InputTask){
				case 0: ROS_INFO("InputTask:NONE TASK!!!");break;
				case 1: ROS_INFO("InputTask:gps march!!!");break;
				case 2: ROS_INFO("InputTask:laser march!!!");break;
				case 3: ROS_INFO("InputTask:ASSEMBLE TASK!!!");break;
				case 4:ROS_INFO("InputTask:STOP TASK!!!");break;
				case 5:ROS_INFO("InputTask:Pause TASK!!!");break;
				case 6:ROS_INFO("InputTask:Resume TASK!!!");break;
			}
			switch(g_BasicLogicAgent->CurrentTask){
				case 0: ROS_INFO("CurrentTask:NONE TASK!!!");break;
				case 1: ROS_INFO("CurrentTask:gps march!!!");break;
				case 2: ROS_INFO("CurrentTask:laser march!!!");break;
				case 3: ROS_INFO("CurrentTask:ASSEMBLE TASK!!!");break;
				case 4:ROS_INFO("CurrentTask:STOP TASK!!!");break;
				case 5:ROS_INFO("CurrentTask:Pause TASK!!!");break;
				case 6:ROS_INFO("CurrentTask:Resume TASK!!!");break;
			}
			    //test
			for(int i=0;i<g_GroupLogicAgent->GroupMember.size();i++)
			{
				ROS_INFO("Member:%d,state:%d",g_GroupLogicAgent->GroupMember[i],g_GroupLogicAgent->GroupState[i]);
			}
    //test

		}

			  count = 0;
		}
		
		UpdateLoop();
        ros::spinOnce();
		loop.sleep();
    }
	CleanupPlayer();

	CleanupBehaviac();

	return 0;
}
 //behaviac::GetEnumClassValueNames