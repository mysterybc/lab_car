#include "actionlib/client/simple_action_client.h"
#include "robot_msgs/MarchAction.h"
#include "std_msgs/Int16.h"
#include "ros/ros.h"


void FeedbackCallback(const robot_msgs::MarchFeedbackConstPtr &feedback){}

void ActiveCallback(void){}

void DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::MarchResultConstPtr &result){
    std::cout << "result is " << result->succeed << std::endl;
}


geometry_msgs::Pose getGoal(){
    geometry_msgs::Pose pose;
    // std::cout << "please input x" << std::endl;
    // std::cin >> pose.position.x;
    // std::cout << "please input y" << std::endl;
    // std::cin >> pose.position.y;
    pose.position.x = 6;
    pose.position.y = 12;
    pose.position.z = 0;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    return pose;
}



void sendgoal(actionlib::SimpleActionClient<robot_msgs::MarchAction> &action){
    action.waitForServer();
    ROS_INFO("server connected!!");
    robot_msgs::MarchGoal goal;
    goal.goal = getGoal();
    ROS_INFO("goal x is : %f",goal.goal.position.x);
    ROS_INFO("goal y is : %f",goal.goal.position.y);
    ROS_INFO("goal z is : %f",goal.goal.position.z);
    action.sendGoal(    goal,
                        boost::bind(&DoneCallback,_1,_2),
                        boost::bind(&ActiveCallback),
                        boost::bind(&FeedbackCallback,_1));
    ROS_INFO("send goal to server!!");
}

void on_start(const std_msgs::Int16ConstPtr &msg, actionlib::SimpleActionClient<robot_msgs::MarchAction> &action){
    ROS_INFO("sender get goal!!");
    sendgoal(action);
}


int main(int argc, char** argv){
    ros::init(argc,argv,"send_mission");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<robot_msgs::MarchAction> action(nh,"march_action",true);
    ros::Subscriber start_signal = nh.subscribe<std_msgs::Int16>("/start_signal",10,boost::bind(&on_start,_1,std::ref(action)));

    ros::Rate loop(20);
    while(ros::ok()){
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}