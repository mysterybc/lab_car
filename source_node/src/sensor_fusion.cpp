//ros
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
//my lib
#include "my_param_server.h"


struct SensorFusion{

    SensorFusion(){
        ros::NodeHandle nh;
        lidar_odom_sub = nh.subscribe("odom",1,&SensorFusion::OnNewLidarPose,this);
        gps_odom_sub = nh.subscribe("gps_odom",2,&SensorFusion::OnNewGpsPose,this);
        encoder_odom_sub = nh.subscribe("odom_raw",2,&SensorFusion::OnNewEncoderPose,this);
        imu_data_sub = nh.subscribe("IMU_data",2,&SensorFusion::OnNewImuData,this);
        lidar_imu_fusion_pub = nh.advertise<nav_msgs::Odometry>("lidar_imu_fusion_odom",5);
        robot_static_count = 0;
        is_robot_static = true;
        is_lidar_odom_recv = false;
        
    }


    void OnNewLidarPose(const nav_msgs::OdometryConstPtr& msg){
        is_lidar_odom_recv = true;
        lidar_odom = *msg;
    }

    void OnNewImuData(const sensor_msgs::ImuConstPtr& msg){
        imu_data = *msg;
        IsRobotStatic();
        if(is_lidar_odom_recv){
            Fusion();
        }
    }

    void OnNewGpsPose(const nav_msgs::OdometryConstPtr& msg){
        gps_odom = *msg;
    }

    void OnNewEncoderPose(const nav_msgs::OdometryConstPtr& msg){
        encoder_odom = *msg;
    }

    bool IsRobotStatic(){
        if( fabs(encoder_odom.twist.twist.linear.x) < 0.1 && 
            fabs(imu_data.angular_velocity.z) < 0.1 && 
            fabs(imu_data.angular_velocity.y) < 0.1)
        {
            robot_static_count++;
        }
        else{
            robot_static_count = 0;
        }
        if(robot_static_count >= 100){
            if(robot_static_count > 1e4){
                robot_static_count = 100;
            }
            return true;
        }
        return false;

    }
    void Fusion(){
        double roll,pitch;
        //车辆静止，直接使用lidar位姿，并更新yaw角
        if(is_robot_static){
            lidar_imu_fusion_odom = lidar_odom;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(lidar_odom.pose.pose.orientation,quat);
            tf::Matrix3x3(quat).getEulerYPR(yaw,pitch,roll);
        }
        else{
            lidar_imu_fusion_odom = lidar_odom;
            yaw += imu_data.angular_velocity.z * 0.01;
            lidar_imu_fusion_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        }
        
    }
    //sub & pub
    ros::Subscriber lidar_odom_sub;
    ros::Subscriber gps_odom_sub;
    ros::Subscriber encoder_odom_sub;
    ros::Subscriber imu_data_sub;
    ros::Publisher lidar_imu_fusion_pub;


    nav_msgs::Odometry lidar_odom;
    nav_msgs::Odometry gps_odom;
    nav_msgs::Odometry encoder_odom;
    nav_msgs::Odometry lidar_imu_fusion_odom;
    sensor_msgs::Imu imu_data;
    int robot_static_count;
    bool is_robot_static;
    bool is_lidar_odom_recv;
    double yaw;
};




int main(int argc,char**argv){
    ros::init(argc,argv,"sensor_fusion");
    ros::NodeHandle nh;
    SensorFusion sensor_fusion;
    //这个频率是按照imu的频率设置的
    ros::Rate loop(200);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}