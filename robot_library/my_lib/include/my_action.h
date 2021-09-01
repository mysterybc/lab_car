#pragma once
//ros
#include "ros/ros.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
//c++
#include "string"
//my debug header
#include "my_debug_info.h"



namespace my_lib{
/**
 * @brief definition of action variable
 */
template <typename Action>
using FeedbackConstPtr = typename actionlib::SimpleActionServer<Action>::FeedbackConstPtr;
template <typename Action>
using GoalConstPtr = typename actionlib::SimpleActionServer<Action>::GoalConstPtr;
template <typename Action>
using Goal = typename actionlib::SimpleActionServer<Action>::Goal;
template <typename Action>
using ResultConstPtr = typename actionlib::SimpleActionServer<Action>::ResultConstPtr;
template <typename Action>
using Result = typename actionlib::SimpleActionServer<Action>::Result;


/**
 * @description: base class of action class，shouldn't be use directly!!
 *               inherit the base class to implement your action, and SetGoal() function should bt rewrite
 *               demo action in my_action_demo.cpp
 * @param Action : xxxxAction,xxxx represent the name of the action
 * @return: using action_state to judge the state of the action
 */
template <typename Action>
class ActionClientRealize{
public:
    /**
     * @description: constructor
     * @param action_name_ : action name used in ROS
     * @param car_id_      : the id of the agent
     */
    ActionClientRealize(std::string action_name_,int car_id_,ros::NodeHandle &nh):action_name(action_name_),car_id(car_id_),
        action_client(nh,action_name_,true)
    {
        logger.init_logger(car_id_);
    }
    /**
     * @description: call action server
     */
    void StartAction(){
        logger.DEBUGINFO(car_id,"[%s client] waiting for server !!!",action_name.c_str());
        action_client.waitForServer();
        action_client.sendGoal( action_goal,
                                        boost::bind(&ActionClientRealize::DoneCallback,this,_1,_2),
                                        boost::bind(&ActionClientRealize::ActiveCallback,this),
                                        boost::bind(&ActionClientRealize::FeedbackCallback,this,_1));
        action_state.status = actionlib_msgs::GoalStatus::ACTIVE;
        logger.DEBUGINFO(car_id,"[%s client] send goal!!",action_name.c_str());
    }
    /**
     * @description: set goal of your action,need rewrite
     */
    virtual void SetGoal(){}
    /**
     * @description: get action state
     */
    actionlib_msgs::GoalStatus GetActionState(){return action_state;}
    /**
     * @description: three callback，feedback the action state
     */
    void FeedbackCallback(const FeedbackConstPtr<Action> &feedback){
        if(feedback->action_state.status == actionlib_msgs::GoalStatus::ABORTED){
            action_state.status = actionlib_msgs::GoalStatus::ABORTED;
            logger.WARNINFO(car_id,"[%s client]: task error!!",action_name.c_str());
        }
    }
    void DoneCallback(const actionlib::SimpleClientGoalState &state,const ResultConstPtr<Action> &result){
        action_state.status = (uint8_t)state.state_;
        logger.DEBUGINFO(car_id,"[%s client]: task complete!!",action_name.c_str());
    }
    void ActiveCallback(void){}

    actionlib::SimpleActionClient<Action>  action_client;
    actionlib_msgs::GoalStatus action_state;
    Goal<Action> action_goal;
    std::string action_name;
    int car_id;
    Debug::DebugLogger logger;
};
//end of class ActionClientRealize



/**
 * @description: base class of action class，shouldn't be use directly!!
 *               inherit the base class to implement your action, and CustomExcuteAction() function should bt rewrite
 *               demo action in my_action_demo.cpp
 * @param action_name_ : action name used in ROS
 */
template <typename Action>
class ActionServerRealize{
public:
    ActionServerRealize(std::string action_name_,int car_id_,ros::NodeHandle& nh):action_name(action_name_),car_id(car_id_),
        action_server(nh,action_name_,boost::bind(&ActionServerRealize::OnNewAction,this,_1),false)
	{
        logger.init_logger(car_id_);
		action_server.registerPreemptCallback(boost::bind(&ActionServerRealize::CancelAction,this));
		action_server.start();
        logger.DEBUGINFO(car_id_,"[%s server] init success, server start!!",action_name.c_str());
	}
    /**
     * @description: cancel action
     */
    void CancelAction();
    /**
     * @description: new action callback
     */
    void OnNewAction(const GoalConstPtr<Action> & goal){
        action_state.status = actionlib_msgs::GoalStatus::ACTIVE;
        Result<Action> result;
        if(CustomExcuteAction(goal)){
            action_state.status = actionlib_msgs::GoalStatus::SUCCEEDED;
            result.action_state.status = actionlib_msgs::GoalStatus::SUCCEEDED;
            action_server.setSucceeded(result,"goal success");
            logger.DEBUGINFO(car_id,"[%s server]: task complete!!",action_name.c_str());
        }
        else{
            action_state.status = actionlib_msgs::GoalStatus::ABORTED;
            result.action_state.status = actionlib_msgs::GoalStatus::ABORTED;
            action_server.setAborted(result,"goal failed");
            logger.DEBUGINFO(car_id,"[%s server]: task error!! Abort!!",action_name.c_str());
        }
        
    }
    /**
     * @description: your action code
     * @return: true-success false-filed 
     */
    virtual bool CustomExcuteAction(const GoalConstPtr<Action>& goal){}

    actionlib::SimpleActionServer<Action> action_server;
    std::string action_name;
    int car_id;
    Debug::DebugLogger logger;
    actionlib_msgs::GoalStatus action_state;
};

template <typename Action>
void ActionServerRealize<Action>::CancelAction(){
    if(action_server.isPreemptRequested()){
        logger.DEBUGINFO(car_id,"[%s server] action_preempt",action_name.c_str());
        Result<Action> result;
        result.action_state.status = actionlib_msgs::GoalStatus::PREEMPTED;
        action_server.setPreempted(result,"goal cancel");
    }
}

//end of class ActionServerRealize


};
//end of namespace my_lib
