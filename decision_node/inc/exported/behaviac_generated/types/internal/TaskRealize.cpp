#include "TaskRealize.h"
extern Debug::DebugLogger logger;

#define _cancel  2

TaskRealize::TaskRealize():
fore_func_state (Idle)
{
    build_up_action  = new actionlib::SimpleActionClient<robot_msgs::BuildUpAction>(nh,"build_up_action",true);
    gps_march_action  = new actionlib::SimpleActionClient<robot_msgs::MarchAction>(nh,"march_action_gps",true);
    laser_march_action  = new actionlib::SimpleActionClient<robot_msgs::MarchAction>(nh,"march_action_laser",true);
	search_action = new actionlib::SimpleActionClient<robot_msgs::SearchAction>(nh,"search_action",true);
}

TaskRealize::~TaskRealize()
{
    delete build_up_action;
    delete gps_march_action;
    delete laser_march_action;
}

void TaskRealize::Assemble()
{
    logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Assemble_Start");
    build_up_action->waitForServer();
    assemble_goal.goal =g_BlackBoardAgent->GetGoal().front();
    assemble_goal.idList.clear();
    for(int i = 0 ; i < g_GroupAsBasicLogicAgent->GroupMember.size(); i++)
        assemble_goal.idList.push_back(g_GroupAsBasicLogicAgent->GroupMember[i]);
    build_up_action->sendGoal(   assemble_goal,
                                boost::bind(&TaskRealize::Assemble_DoneCallback,this,_1,_2),
                                boost::bind(&TaskRealize::Assemble_ActiveCallback,this),
                                boost::bind(&TaskRealize::Assemble_FeedbackCallback,this,_1));
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
    return ;
}

void TaskRealize::March_gps()
{
    logger.DEBUGINFO(g_BlackBoardAgent->car_id,"March_gps_Start");
    gps_march_action->waitForServer();
    march_goal.goal =g_BlackBoardAgent->GetGoal().front();
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
    logger.DEBUGINFO(g_BlackBoardAgent->car_id,"March_laser_Start");
    laser_march_action->waitForServer();
    march_goal.goal =g_BlackBoardAgent->GetGoal().front();
    march_goal.idList.clear();
    for(int i = 0 ; i < g_GroupAsBasicLogicAgent->GroupMember.size(); i++)
        march_goal.idList.push_back(g_GroupAsBasicLogicAgent->GroupMember[i]);
    laser_march_action->sendGoal(   march_goal,
                                boost::bind(&TaskRealize::March_DoneCallback,this,_1,_2),
                                boost::bind(&TaskRealize::March_ActiveCallback,this),
                                boost::bind(&TaskRealize::March_FeedbackCallback,this,_1));
}



void TaskRealize::March_FeedbackCallback(const robot_msgs::MarchFeedbackConstPtr &feedback){
    if(feedback->error_occured)
        fore_func_state= ForeFuncState::Failure;
    else
        fore_func_state = ForeFuncState::Running;

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



void TaskRealize::Search()
{
    logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Search_Start");
    // logger.DEBUGINFO(g_BlackBoardAgent->car_id,"waiting");
    search_action->waitForServer();
    // logger.DEBUGINFO(g_BlackBoardAgent->car_id,"waiting done");
    std::vector<geometry_msgs::Pose> temp_vector = g_BlackBoardAgent->GetGoal();
    std::vector<geometry_msgs::PoseStamped>().swap(search_goal.area);//1
    // logger.DEBUGINFO(g_BlackBoardAgent->car_id,"size:%d",temp_vector.size());
    search_goal.area.clear();
    for(auto pose_:temp_vector)
    {
        geometry_msgs::PoseStamped goal_;
        goal_.pose = pose_;
        search_goal.area.push_back(goal_);
        logger.DEBUGINFO(g_BlackBoardAgent->car_id,"point is : %f %f",pose_.position.x,pose_.position.y);
    }
    logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Search_Is——SETTLED111");
    search_goal.idList.clear();
    for(int i = 0 ; i < g_GroupAsBasicLogicAgent->GroupMember.size(); i++)
        search_goal.idList.push_back(g_GroupAsBasicLogicAgent->GroupMember[i]);
    search_action->sendGoal(   search_goal,
                                boost::bind(&TaskRealize::Search_DoneCallback,this,_1,_2),
                                boost::bind(&TaskRealize::Search_ActiveCallback,this),
                                boost::bind(&TaskRealize::Search_FeedbackCallback,this,_1));
    // logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Search_Is——SETTLED");
}

void TaskRealize::Search_FeedbackCallback(const robot_msgs::SearchFeedbackConstPtr &feedback){
    // logger.DEBUGINFO(g_BlackBoardAgent->car_id,"in search CB and size is %d",g_BlackBoardAgent->tag_id.size());
    if(g_BlackBoardAgent->tag_id.size()==0)//when have not dectected the tag.
    {
    if(feedback->error_occured)
        fore_func_state= ForeFuncState::Failure;
    else
        fore_func_state = ForeFuncState::Running;
    }

    else if(fore_func_state==ForeFuncState::Running)//tag detected
    {
        g_GroupAsBasicLogicAgent->CurrentTask=TaskIndividual::NonTask;
    	g_TaskRealizeAgent->search_action->cancelGoal();//finish search behavior
        g_BlackBoardAgent->PubTagPose();//Pub tag pose
        logger.DEBUGINFO(g_BlackBoardAgent->car_id,"Detect the tag,Search complete");
    }

    return ;
}

void TaskRealize::Search_ActiveCallback(void){
    fore_func_state = ForeFuncState::Running;
    return ;
}

void TaskRealize::Search_DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::SearchResultConstPtr &result){
    if(result->succeed!=_cancel)
        g_GroupAsBasicLogicAgent->CurrentTask=TaskIndividual::NonTask;
    return ;
}


