#pragma once

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_node.h"
#include "../behavior_tree/behavior_state.h"
#include "robot_msgs/BuildUpAction.h"
#include "actionlib/client/simple_action_client.h"
#include <iostream>
#include <string>


class BuildUpBehavior{
public:
    BuildUpBehavior(Blackboard* &blackboard,std::string behavior_name):  
        blackboard_(blackboard),
        build_up_action(nh,"build_up_action",true),
        state(BehaviorState::IDLE)
    {}

    //进入任务时初始化
    void OnInitialize(){
        build_up_action.waitForServer();
        goal.goal = blackboard_->GetGoal();
        ROS_INFO("goal x is : %f",goal.goal.position.x);
        ROS_INFO("goal y is : %f",goal.goal.position.y);
        ROS_INFO("goal z is : %f",goal.goal.position.z);
        build_up_action.sendGoal(   goal,
                                    boost::bind(&BuildUpBehavior::DoneCallback,this,_1,_2),
                                    boost::bind(&BuildUpBehavior::ActiveCallback,this),
                                    boost::bind(&BuildUpBehavior::FeedbackCallback,this,_1));
        state = BehaviorState::RUNNING;
        }

    void Run() {

    }

    void Cancel() {
        state = BehaviorState::IDLE;
        build_up_action.cancelGoal();

    }

    BehaviorState Update() {
        switch(state){
            case BehaviorState::FAILURE: ROS_INFO("build up state is FAILURE");break;
            case BehaviorState::IDLE:    ROS_INFO("build up state is IDLE");break;
            case BehaviorState::RUNNING: ROS_INFO("build up state is RUNNING");break;
            case BehaviorState::SUCCESS: ROS_INFO("build up state is SUCCESS");break;
        }
        
        return state;
    }

    void FeedbackCallback(const robot_msgs::BuildUpFeedbackConstPtr &feedback){
        if(feedback->error_occured){
            state = BehaviorState::FAILURE;
        }
        else{
            state = BehaviorState::RUNNING;
        }
        return ;
    }

    void ActiveCallback(){
        state = BehaviorState::RUNNING;
        return ;
    }

    void DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::BuildUpResultConstPtr &result){
        if(result->succeed){
            this->state = BehaviorState::SUCCESS;
            blackboard_->SetMission(MissionType::SYSTEM_STANDBY);
        }
        else{
            this->state = BehaviorState::FAILURE;
        }
        return ;
    }

    ~BuildUpBehavior() = default;

private:
    ros::NodeHandle nh;
    //! perception information
    Blackboard* const blackboard_;
    //build up action
    actionlib::SimpleActionClient<robot_msgs::BuildUpAction> build_up_action;
    robot_msgs::BuildUpGoal goal;
    BehaviorState state;

};

class BuildUpAction:public ActionNode{
public:
    BuildUpAction(const Blackboard::Ptr &blackboard,BuildUpBehavior *behavior):
        ActionNode::ActionNode("build_up_action",blackboard)
        {
            build_up_behavior = behavior;
        }
private:
    void OnInitialize(){
        ROS_INFO("build up behavior run!");
        build_up_behavior->OnInitialize();
    }
    BehaviorState Update(){
        build_up_behavior->Run();
        return build_up_behavior->Update();
    }
    void OnTerminate(BehaviorState state) {
        switch(state){
            case BehaviorState::IDLE : build_up_behavior->Cancel();break;
            default : break;
        }
        
    }

    BuildUpBehavior *build_up_behavior;

};

