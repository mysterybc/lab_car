#include "ros/ros.h"
#include "robot_msgs/HostCmdArray.h"
#include "tf/tf.h"
#include <thread>
#include <vector>

void Command();
void GetMission(robot_msgs::HostCmd &cmd);
void SetId(robot_msgs::HostCmd &cmd);
ros::Publisher host_cmd_pub;
robot_msgs::HostCmdArray host_cmd_array;
bool new_cmd{false};
char command = '0';


void SelectMission(){
    robot_msgs::HostCmd host_cmd;
    switch(command){
        case '1':{GetMission(host_cmd);break;}
        case '2':{ host_cmd.mission.mission = host_cmd.mission.STOP;break;}
        case '3':{ host_cmd.mission.mission = host_cmd.mission.pause;break;}
        case '4':{ host_cmd.mission.mission = host_cmd.mission.resume;break;}
        case '9':{
            int n;
            std::cout << "pleasr input task number" << std::endl;
            std::cin >> n;
            for(int i = 1; i <= n; i++){
                Command();
                SelectMission();
            }
            return ;
        }
        default:break;
    }
    SetId(host_cmd);
    host_cmd_array.host_cmd_array.push_back(host_cmd);

}

void SetId(robot_msgs::HostCmd &cmd){
    std::cout << "please input car id" << std::endl;
    std::string id;
    std::cin >> id;
    for(auto i:id){
        cmd.car_id.push_back(i-48);
    }
}

void GetMission(robot_msgs::HostCmd &cmd){
    double yaw = 3.14;
    std::cout << "please input x :" << std::endl;
    std::cin >> cmd.goal.pose.position.x;
    std::cout << "please input y :" << std::endl;
    std::cin >> cmd.goal.pose.position.y;
    std::cout << "please input yaw :" << std::endl;
    std::cin >> yaw;
    cmd.goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    cmd.goal.pose.position.z = 0;
    cmd.mission.mission = cmd.mission.build_up_task;
}


void MissionPub(){
    host_cmd_pub.publish(host_cmd_array);
}

void Command(){
    while(ros::ok()){
        std::cout   << "-----------------please input command-----------------" << std::endl
                    << "                1 :  build up mission                 "  << std::endl
                    << "                2 :  system stand by                  "  << std::endl
                    << "                3 :  mission pause                    "  << std::endl
                    << "                4 :  mission resume                   "  << std::endl
                    << "                9 :  multi mission                    "  << std::endl;
        std::cin  >> command;
        if(command != '1' && command != '2' && command != '3' && command != '4' && command != '9'){
            std::cout << "---input wrong!!  please input again!!!---" << std::endl;
            std::cout << std::endl;
        }else{
            new_cmd = true;
            break;
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"host_cmd_pub_node");
    ros::NodeHandle nh;
    host_cmd_pub = nh.advertise<robot_msgs::HostCmdArray>("host_cmd",10);

    ros::Rate loop(10);
    while(ros::ok()){
        ros::spinOnce();
        Command();
        if(new_cmd){
            SelectMission();
            MissionPub();
            new_cmd = false;
            std::cout << "                TASK SET!!!" << std::endl;
            std::cout << std::endl;
            host_cmd_array.host_cmd_array.clear();
        }
        loop.sleep();
    }
    
    return 0;
}