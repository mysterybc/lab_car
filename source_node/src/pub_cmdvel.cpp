#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "thread"
#include "iostream"

using std::cin;
using std::cout;
using std::endl;

struct VelInput{
    void GetInputVel(){
        while(ros::ok()){
            cout << "--------please input linear speed--------" << endl;
            cin >> linear_speed;
            linear_speed = linear_speed > 1.0 ? 1.0 : linear_speed;
            cout << "--------please input angular speed--------" << endl;
            cin >> angular_speed;
            angular_speed = angular_speed > 0.4 ? 0.4 : angular_speed;
            cmd_update = true; 
            cout << "--------get cmd speed success--------" << endl;

        }
        
    }
    bool cmd_update{false};
    double linear_speed;
    double angular_speed;
};


int main(int argc,char** argv){
    ros::init(argc,argv,"pub_cmdvel");
    ros::NodeHandle nh;

    ros::Rate delay(1);
    delay.sleep();

    ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    struct VelInput vel_input;
    std::thread get_input_vel(&VelInput::GetInputVel,&vel_input);

    ros::Rate loop(20);
    geometry_msgs::Twist vel;
    while(ros::ok()){
        if(vel_input.cmd_update){
            vel.linear.x = vel_input.linear_speed;
            vel.angular.z = vel_input.angular_speed;
            vel_input.cmd_update = false;
        }
        cmdvel_pub.publish(vel);
        ros::spinOnce();
        loop.sleep(); 
    }
    return 0;
}