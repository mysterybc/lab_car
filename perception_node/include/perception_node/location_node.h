/** brief:头文件声明了source节点的基类，SourceNode
 *  note:
 *  
 */

#ifndef LOCALIZATION_NODE_H
#define LOCALIZATION_NODE_H
#include "perception.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"


class ImuGpsFusion{
public:
    ImuGpsFusion(){}
    ~ImuGpsFusion(){}
    int Fusion(){
        ros::NodeHandle n;

        //! ros publisher for odometry information
        ros::Subscriber subTwist_ = n.subscribe("komodo_odom", 10, &ImuGpsFusion::komodoOdomCallback,this);
        ros::Subscriber subGps_ = n.subscribe("gps_odom", 10, &ImuGpsFusion::GpsCallback,this);
        ros::Subscriber subImu_ = n.subscribe("imu", 10, &ImuGpsFusion::ImuCallback,this);

        ros::Publisher odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 10);

        //! ros chassis odometry tf broadcaster
        tf::TransformBroadcaster tf_broadcaster_;

        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";

        odom_tf_.header.frame_id = "odom";
        odom_tf_.child_frame_id = "base_link";

        odom_.pose.pose.position.x = 0.0;
        odom_.pose.pose.position.y = 0.0;
        odom_.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(0);
        odom_.pose.pose.orientation = q;
        odom_.twist.twist.linear.x = 0.0;
        odom_.twist.twist.linear.y = 0.0;
        odom_.twist.twist.linear.z = 0.0;
        odom_.twist.twist.angular.x = 0.0;
        odom_.twist.twist.angular.y = 0.0;
        odom_.twist.twist.angular.z = 0.0;

        ros::Rate loop_rate(100);
        while(ros::ok())
        {
            if(odom_update && start_flag)
            {
                odom_pub_.publish(odom_);
                tf_broadcaster_.sendTransform(odom_tf_);
                odom_update = false;
            }
            ros::spinOnce();
            loop_rate.sleep();

        }
        return 0;
    }
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        odom_.twist.twist.angular.z = msg->angular_velocity.z;
        odom_.pose.pose.orientation = msg->orientation;
        odom_update = true;
    }
    void GpsCallback(const nav_msgs::OdometryConstPtr& msg){
        odom_.pose.pose.position.x = msg->pose.pose.position.x;
        odom_.pose.pose.position.y = msg->pose.pose.position.y;
        odom_.pose.pose.position.z = msg->pose.pose.position.z;
        odom_update = true;
    }
    void komodoOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
        odom_update = true;
    }
    void GetPose(geometry_msgs::Pose& pose){
        pose = odom_.pose.pose;
    }
private:
    nav_msgs::Odometry odom_;
    geometry_msgs::TransformStamped odom_tf_;
    bool odom_update{false};
    bool start_flag{false};

};

class LocalizationNode:public PerceptionNode{
public:
    LocalizationNode();
    ~LocalizationNode()=default;
    bool  LoadConfig(std::string file);
    //这些函数应该在cmd的callback中被调用
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  UpdateState();
    void CmdCallback(const robot_msgs::CmdConstPtr &msg);
    std::thread* fusion_thread;
    ImuGpsFusion imu_gps_fusion;
};


#endif