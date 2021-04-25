#ifndef _BEHAVIAC_TASKREALIZE_H_
#define _BEHAVIAC_TASKREALIZE_H_

#include "behaviac_headers.h"
#include "BlackBoard.h"
#include "GroupAsBasicLogic.h"
#include "robot_msgs/BuildUpAction.h"
#include "robot_msgs/MarchAction.h"
#include "robot_msgs/SearchAction.h"
#include "actionlib/client/simple_action_client.h"
#include <iostream>
#include <string>
#include "memory.h"

class GroupAsBasicLogic;
class BlackBoard;

extern GroupAsBasicLogic* g_GroupAsBasicLogicAgent;
extern BlackBoard* g_BlackBoardAgent;

class TaskRealize : public behaviac::Agent
{
public:
	TaskRealize();
	virtual ~TaskRealize();

	BEHAVIAC_DECLARE_AGENTTYPE(TaskRealize, behaviac::Agent)

	public: ForeFuncState fore_func_state;

	public: void Assemble();

	private: void Assemble_FeedbackCallback(const robot_msgs::BuildUpFeedbackConstPtr &feedback);
	private: void Assemble_ActiveCallback(void);
	private: void Assemble_DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::BuildUpResultConstPtr &result);

	public: void March_gps();
	public: void March_laser();

	private: void March_FeedbackCallback(const robot_msgs::MarchFeedbackConstPtr &feedback);
	private: void March_ActiveCallback(void);
	private: void March_DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::MarchResultConstPtr &result);

	public: void Search();

	private: void Search_FeedbackCallback(const robot_msgs::SearchFeedbackConstPtr &feedback);
	private: void Search_ActiveCallback(void);
	private: void Search_DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::SearchResultConstPtr &result);

	public: void Remote_Control();
	
	public:actionlib::SimpleActionClient<robot_msgs::BuildUpAction>* build_up_action;
	public:actionlib::SimpleActionClient<robot_msgs::MarchAction>* gps_march_action;
	public:actionlib::SimpleActionClient<robot_msgs::MarchAction>* laser_march_action;
	public:actionlib::SimpleActionClient<robot_msgs::SearchAction>* search_action;

	private: ros::NodeHandle nh;
    private: robot_msgs::BuildUpGoal assemble_goal;
	private: robot_msgs::MarchGoal march_goal;
	private: robot_msgs::SearchGoal search_goal;
};

#endif
