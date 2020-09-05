#include "ros/ros.h"
#include "robot_msgs/HostCmd.h"
#include <thread>

void Command();
void MissionPub();
robot_msgs::HostCmd host_cmd;
ros::Publisher host_cmd_pub;
char command = '0';
char last_command = '0';

int main(int argc, char **argv){
    ros::init(argc,argv,"host_cmd_pub_node");
    ros::NodeHandle nh;

    for(unsigned int i = 1; i <= 4; i++){
        host_cmd.car_id.push_back(i);
    }
    std::thread command_thread = std::thread(Command);
    host_cmd_pub = nh.advertise<robot_msgs::HostCmd>("host_cmd",10);

    ros::Rate loop(10);
    while(ros::ok()){
        ros::spinOnce();

        switch(command){
            case '1':
                if(last_command == command)
                    break;
                host_cmd.mission.mission = host_cmd.mission.system_standby;
                last_command = '1';
                MissionPub();
                break;
            case '2':
                if(last_command == command)
                    break;
                host_cmd.goal.pose.orientation.w = 1;
                host_cmd.goal.pose.orientation.x = 0;
                host_cmd.goal.pose.orientation.y = 0;
                host_cmd.goal.pose.orientation.z = 0;
                host_cmd.goal.pose.position.x = 6;
                host_cmd.goal.pose.position.y = 12;
                host_cmd.goal.pose.position.z = 0;
                host_cmd.mission.mission = host_cmd.mission.build_up_task;
                last_command = '2';
                MissionPub();
                break;
            default :
                break;
        }

        loop.sleep();
    }
    
    return 0;
}

void MissionPub(){
    ros::Rate loop(10);
    for(int i = 0; i < 2; i++){
        host_cmd_pub.publish(host_cmd);
        ros::spinOnce();
        loop.sleep();
    }
}

void Command(){
    while(command != 27){
        std::cout << "please input command" << std::endl
                  << "1: system stand by"   << std::endl
                  << "2: build up mission"  << std::endl;
        std::cin  >> command;
        if(command != '1' && command != '2'){
            std::cout << "please input again!" << std::endl;
            std::cin >> command;
        }
    }
}