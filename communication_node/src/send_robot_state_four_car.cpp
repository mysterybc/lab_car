#include "zmq_test.h"
#include "iostream"
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"

class Robot{
public:
    Robot(){}
    void DecisionSub(const std_msgs::StringConstPtr &msg);
    std::string state;
};

void Robot::DecisionSub(const std_msgs::StringConstPtr &msg){
    state = msg->data;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"send_robot_state");
    ros::NodeHandle nh;
    std::string ip_address;
    nh.getParam("my_ip_address",ip_address);
    std::vector<ros::Subscriber> state_subs(4);
    std::vector<Robot> robots(4);
    for(int i = 0 ; i < 4; i++){
        std::string temp = "/robot_" + std::to_string(i) + "/decision_state";
        state_subs[i] = nh.subscribe(temp,10,&Robot::DecisionSub,&robots[i]);
    }

    ZMQ_TEST *zmq_test;
    zmq_test = new ZMQ_TEST();
    zmq_test->zmq_init(0,1,6666,ip_address);
    string test;
    ros::Rate loop(1);
    std::vector<std::string> name = {"robot1 state is :","  robot2 state is :","  robot3 state is :","  robot4 state is :"};
    while(ros::ok())
    {
        for(int i = 0 ;i < 4; i++){
            test += name[i];
            test += robots[i].state;
        }
        zmq_test->send_data(test);
        ros::spinOnce();
        loop.sleep();
        test.clear();
    }

    return 0;
}