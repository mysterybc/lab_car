#pragma once


#include "../blackboard/blackboard.h"
#include "behavior_tree/behavior_node.h"
#include "../behavior_tree/behavior_state.h"
#include "actionlib/client/simple_action_client.h"
#include <iostream>
#include <string>


class StandbyBehavior{
public:
    StandbyBehavior(Blackboard* &blackboard,std::string behavior_name):  
        blackboard_(blackboard),
        state(BehaviorState::IDLE)
    {}

    void Run() {
        blackboard_->PubDecisionState("stand by");
        state = BehaviorState::RUNNING;
    }

    void Cancel() {
        state = BehaviorState::IDLE;
    }

    BehaviorState Update() {
        switch(state){
            case BehaviorState::FAILURE: break;
            case BehaviorState::IDLE:    break;
            case BehaviorState::RUNNING: break;
            case BehaviorState::SUCCESS: break;
        }
        
        return state;
    }

    ~StandbyBehavior() = default;

private:
    ros::NodeHandle nh;
    //! perception information
    Blackboard* const blackboard_;
    BehaviorState state;

};

class StandbyActon:public ActionNode{
public:
    StandbyActon(const Blackboard::Ptr &blackboard,StandbyBehavior *behavior):
        ActionNode::ActionNode("build_up_action",blackboard)
        {
            standby_behavior = behavior;
        }
private:
    void OnInitialize(){
        ROS_INFO("standby behavior run!");
    }
    BehaviorState Update(){
        standby_behavior->Run();
        return standby_behavior->Update();
    }
    void OnTerminate(BehaviorState state) {
        switch(state){
            case BehaviorState::IDLE : standby_behavior->Cancel();break;
            default : break;
        }
        
    }

    StandbyBehavior *standby_behavior;

};