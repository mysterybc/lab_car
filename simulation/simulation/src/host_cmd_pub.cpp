#include "ros/ros.h"
#include "robot_msgs/HostCmd.h"
#include "tf/tf.h"
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
    host_cmd_pub = nh.advertise<robot_msgs::HostCmd>("host_cmd",10);

    ros::Rate loop(10);
    while(ros::ok()){
        ros::spinOnce();
        Command();
        switch(command){
            case '1':{
                if(last_command == command)
                    break;
                double yaw = 3.14;
                std::cout << "please input x :" << std::endl;
                std::cin >> host_cmd.goal.pose.position.x;
                std::cout << "please input y :" << std::endl;
                std::cin >> host_cmd.goal.pose.position.y;
                std::cout << "please input yaw :" << std::endl;
                std::cin >> yaw;
                host_cmd.goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                host_cmd.goal.pose.position.z = 0;
                host_cmd.mission.mission = host_cmd.mission.build_up_task;
                last_command = '1';
                MissionPub();
                break;
            }
            case '2':{
                if(last_command == command)
                    break;
                host_cmd.mission.mission = host_cmd.mission.STOP;
                last_command = '2';
                MissionPub();
                break;
            }
            case '3':{
                if(last_command == command)
                    break;
                host_cmd.mission.mission = host_cmd.mission.pause;
                last_command = '3';
                MissionPub();
                break;
            }

            case '4':{
               if(last_command == command)
                    break;
                host_cmd.mission.mission = host_cmd.mission.resume;
                last_command = '4';
                MissionPub();
                break;
            }
            default:
                break;

        }

        loop.sleep();
    }
    
    return 0;
}

void MissionPub(){
    ros::Rate loop(10);
    for(int i = 0; i < 1; i++){
        host_cmd_pub.publish(host_cmd);
        ros::spinOnce();
        loop.sleep();
    }
}

void Command(){
    while(ros::ok()){
        std::cout << "please input command" << std::endl
                  << "1: build up mission"   << std::endl
                  << "2: system stand by"  << std::endl
                 << "3: mission pause"  << std::endl
                 << "4: mission resume"  << std::endl;
        std::cin  >> command;
        if(command != '1' && command != '2' && command != '3' && command != '4'){
            std::cout << "please input again!" << std::endl;
            std::cin >> command;
        }else{
            break;
        }
    }
}