#include "communication_node/zmq_test.h"
#include "ros/ros.h"
#include "robot_msgs/HostCmd.h"
#include "thread"

bool new_data{false};
robot_msgs::HostCmd cmd;
ros::Publisher host_cmd_pub;
std::string data;


void host_pub(){
    ros::Rate loop(1);
    for(int i = 1; i < 5; i++){
        if(data.find(std::to_string(i))){
            cmd.car_id.push_back(i);
        }
    }
    cmd.mission.mission = robot_msgs::HostCmd::_mission_type::system_standby;
    while(ros::ok()){
        if(new_data){
            string temp = data.substr(data.find("ID"),10);
            for(int i = 1; i < 5; i++){
                if(temp.find(std::to_string(i)) != temp.npos){
                    cmd.car_id[i-1] = i;
                }
                else{
                    cmd.car_id[i-1] = 10;
                }
            }
            if(data.find("build_up") != data.npos ){
                cmd.mission.mission = robot_msgs::HostCmd::_mission_type::build_up_task;
                int index1,index2;
                index1 = data.find("x:");
                index2 = data.find("y:");
                cmd.goal.pose.position.x = stoi(data.substr(index1+2,1)) + ((double)stoi(data.substr(index1+4,1)))/10;
                cmd.goal.pose.position.y = stoi(data.substr(index2+2,1)) * 10 + stoi(data.substr(index2+3,1)) + ((double)stoi(data.substr(index2+5,1)))/10;
                cmd.goal.pose.position.z = 0;
                cmd.goal.pose.orientation.w = 1;
                cmd.goal.pose.orientation.x = 0;
                cmd.goal.pose.orientation.y = 0;
                cmd.goal.pose.orientation.z = 0;

            }
            else{
                cmd.mission.mission = robot_msgs::HostCmd::_mission_type::system_standby;
            }
            
            new_data = false;
        }
        host_cmd_pub.publish(cmd);
        ros::spinOnce();
        loop.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"zmq_recv");
    ros::NodeHandle nh;
    host_cmd_pub = nh.advertise<robot_msgs::HostCmd>("/host_cmd",10);
    std::thread host_pub_thread(host_pub);
    ZMQ_TEST *zmq_test;
    zmq_test = new ZMQ_TEST();
    zmq_test->zmq_init(0,0,6666);
    zmq_test->run(data,new_data);

    return 0;
}

