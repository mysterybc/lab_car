//ros
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "robot_msgs/CurrentTask.h"
#include "tf/transform_broadcaster.h"
//my lib
#include "my_param_server.h"


struct SensorFusion{

    SensorFusion(){
        ros::NodeHandle nh;
        param_server.GetParam("sensor_fusion_node");
        lidar_odom_sub = nh.subscribe("lidar_odom",1,&SensorFusion::OnNewLidarPose,this);
        gps_odom_sub = nh.subscribe("gps_odom",2,&SensorFusion::OnNewGpsPose,this);
        encoder_odom_sub = nh.subscribe("odom_raw",2,&SensorFusion::OnNewEncoderPose,this);
        imu_data_sub = nh.subscribe("IMU_data",2,&SensorFusion::OnNewImuData,this);
        current_task_sub = nh.subscribe("current_task",2,&SensorFusion::OnNewTask,this);
        lidar_imu_fusion_pub = nh.advertise<nav_msgs::Odometry>("lidar_imu_fusion_odom",5);
        robot_static_count = 0;
        is_robot_static = true;
        is_lidar_odom_recv = false;
        trans.frame_id_ = "odom";
        trans.child_frame_id_ = param_server.tf_ns + "/base_link";
    }


    void OnNewLidarPose(const nav_msgs::OdometryConstPtr& msg){
        is_lidar_odom_recv = true;
        lidar_odom = *msg;
    }

    void OnNewImuData(const sensor_msgs::ImuConstPtr& msg){
        imu_data = *msg;
        //等待接收到lidar pose
        if(!is_lidar_odom_recv){
            return ;
        }
        IsRobotStatic();
        Fusion();
    }

    void OnNewGpsPose(const nav_msgs::OdometryConstPtr& msg){
        gps_odom = *msg;
    }

    void OnNewEncoderPose(const nav_msgs::OdometryConstPtr& msg){
        encoder_odom = *msg;
    }

    void OnNewTask(const robot_msgs::CurrentTaskConstPtr& msg){
        current_task = msg->current_task;
    }

    bool IsRobotStatic(){
        if( current_task == 0 &&
            fabs(encoder_odom.twist.twist.linear.x) < 0.05 && 
            fabs(imu_data.angular_velocity.z) < 0.05 && 
            fabs(imu_data.angular_velocity.y) < 0.05)
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
        lidar_imu_fusion_pub.publish(lidar_imu_fusion_odom);
        SetTF();
        tf_pub.sendTransform(trans);
        is_lidar_odom_recv = false;
    }

    void SetTF(){
        //xyz
        tf::Vector3 tf_origin;
        tf_origin.setX(lidar_imu_fusion_odom.pose.pose.position.x);
        tf_origin.setY(lidar_imu_fusion_odom.pose.pose.position.y);
        tf_origin.setZ(lidar_imu_fusion_odom.pose.pose.position.z);
        trans.setOrigin(tf_origin);
        //oriention
        tf::Quaternion tf_quat;
        tf::quaternionMsgToTF(lidar_imu_fusion_odom.pose.pose.orientation,tf_quat);
        trans.setRotation(tf_quat);
        //time
        trans.stamp_ = imu_data.header.stamp;
    }


    //sub & pub
    ros::Subscriber lidar_odom_sub;
    ros::Subscriber gps_odom_sub;
    ros::Subscriber encoder_odom_sub;
    ros::Subscriber imu_data_sub;
    ros::Subscriber current_task_sub;
    ros::Publisher lidar_imu_fusion_pub;
    tf::TransformBroadcaster tf_pub;


    nav_msgs::Odometry lidar_odom;
    nav_msgs::Odometry gps_odom;
    nav_msgs::Odometry encoder_odom;
    nav_msgs::Odometry lidar_imu_fusion_odom;
    sensor_msgs::Imu imu_data;
    tf::StampedTransform trans;
    int robot_static_count;
    bool is_robot_static;
    bool is_lidar_odom_recv;
    double yaw;
    my_lib::ParamServer param_server;
    uint8_t current_task;
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