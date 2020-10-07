#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "iostream"

double yaw{0};
bool state{false};
bool last_state{false};

void ImuCallback(const sensor_msgs::ImuConstPtr &msg){
    double pitch,roll;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation,quat);
    tf::Matrix3x3(quat).getEulerYPR(yaw,pitch,roll);
}

void JoyCallback(const sensor_msgs::JoyConstPtr &msg){
    static int count = 0;
    last_state = state;
    if (msg->buttons[4] || msg->buttons[5]){
        count ++;
        if(count > 3){
            state = true;
        }
    }else{
        count = 0;
        state = false;
    }
    
    
}
int main(int argc, char **argv){
    ros::init(argc,argv,"test_twist");
    ros::NodeHandle nh;
    ros::Subscriber joy_sub = nh.subscribe("joy",10,JoyCallback);
    ros::Subscriber odom_sub = nh.subscribe("imu_data",10,ImuCallback);
    double start_yaw{0};
    double start_stime{0};
    ros::Rate loop(100);
    while(ros::ok()){
        if(state){
            if(state != last_state){
                start_yaw = yaw;
                ROS_INFO("start angle is : %f",start_yaw);
                start_stime = ros::Time().now().toSec();
            }
            if(fabs(yaw-start_yaw) > 0.5 ){
                double cost_time = ros::Time().now().toSec() - start_stime;
                double speed = fabs(yaw-start_yaw) / cost_time;
                ROS_INFO("angular speed is : %f",speed);
            }
        }
    ros::spinOnce();
    loop.sleep();
    }
    
}