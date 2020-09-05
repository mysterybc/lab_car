#include <ros/ros.h>

#include "example_behavior/build_up_behavior.h"
#include "example_behavior/stand_by_behavior.h"
#include "behavior_tree/behavior_node.h"
#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "behavior_test_node");
    //blackboard
    auto blackboard = new Blackboard();
    std::shared_ptr<Blackboard> blackboard_ptr(blackboard);
    //bahevior
    BuildUpBehavior build_up_behavior(blackboard,"build_up_behavior");
    StandbyBehavior Standby_behavior(blackboard,"standby_behavior");
    //action
    auto build_up_action = std::make_shared<BuildUpAction>(blackboard_ptr,&build_up_behavior);
    auto standby_action = std::make_shared<StandbyActon>(blackboard_ptr,&Standby_behavior);
    
    //root_node
    std::shared_ptr<SelectorNode> root_node(new SelectorNode("root_selector",blackboard_ptr));
    //first level 决策树是否执行
    std::shared_ptr<PreconditionNode> system_stop_condition(new PreconditionNode("system_stop_condition",blackboard_ptr,
                                                          [&](){return (!blackboard->IsSystemStart());},
                                                          AbortType::BOTH));
    std::shared_ptr<SelectorNode> system_start_selector(new SelectorNode("system_start_selector",blackboard_ptr));
    root_node->AddChildren(system_stop_condition);
    root_node->AddChildren(system_start_selector);
    system_stop_condition->SetChild(standby_action);
    //second level
    std::shared_ptr<PreconditionNode> build_up_condition(new PreconditionNode("build_up_condition",blackboard_ptr,
                                                          [&](){ return (blackboard->GetMission() == MissionType::BUILD_UP_TASK?  true : false);},
                                                          AbortType::BOTH));
    system_start_selector->AddChildren(build_up_condition);
    build_up_condition->SetChild(build_up_action);

    //behavior tree
    BehaviorTree behavior_tree(root_node,1000);

    //main loop 
    while(ros::ok()){
        behavior_tree.Run();
        ros::spinOnce();
    }


  return 0;
}



