/** brief:头文件声明了编码器资源节点，继承source_node
 *  note:
 *  
 */

#ifndef ENCODER_SOURCE_H
#define ENCODER_SOURCE_H

#include "source_node.h"
#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"


#define VEL_TO_RPM  826.0f//单位转换 m/s到rpm   v/(pi*d)*减速比   826.0f

//TODO 车号的定义需要后期使用度config文件的形式确定
#define TURN_RADIUS_K2  0.25f //komodo 2 的转弯半径 单位m  (340+10+150)/2
#define TURN_RADIUS_K3  0.29f //komodo 2 的转弯半径 单位m  (420+10+150)/2
#define TURN_RADIUS  TURN_RADIUS_K3 //选择对应的车


class Encoder{
public:
    Encoder()=default;
    ~Encoder()=default;
    int UpdateOdom()
    {
        ros::NodeHandle n;

        std::string port;
        ros::NodeHandle nh_private("~");
        nh_private.param<std::string>("port", port, "/dev/ttyUSB0"); 

        //创建timeout
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        //设置要打开的串口名称
        // sp.setPort("/dev/ttyUSB1");
        sp.setPort(port);

        //设置串口通信的波特率
        sp.setBaudrate(115200);
        //串口设置timeout
        sp.setTimeout(to);
    
        try
        {
            //打开串口
            sp.open();
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            return -1;
        }
        
        //判断串口是否打开成功
        if(sp.isOpen())
        {
            // ROS_INFO(port);
            std::cout << "port:" << port <<std::endl;
            ROS_INFO_STREAM("port is opened.");
        }
        else
        {
            return -1;
        }

        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("komodo/vel", 20);
        //! ros publisher for odometry information
        ros::Subscriber subTwist_ = n.subscribe("komodo/cmd_vel", 1000, &Encoder::cmdVelCallback,this);
        ros::Publisher ros_odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 30);
        //! ros chassis odometry tf
        geometry_msgs::TransformStamped odom_tf_;
        //! ros chassis odometry tf broadcaster
        tf::TransformBroadcaster tf_broadcaster_;
        //! ros odometry message
        nav_msgs::Odometry odom_;

        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";

        odom_tf_.header.frame_id = "odom";
        odom_tf_.child_frame_id = "base_link";

        ros::Rate loop_rate(100);
        static int get_vel_cnt = 0;
        while(ros::ok())
        {
            size_t num = sp.available();
            if(num!=0)
            {
                uint8_t buffer[1024];
                //读出数据
                num = sp.read(buffer, num);
                if(!start_flag){
                    continue;
                }
                int vel_right = 0,vel_left = 0; 
                std::string str;
                for(int i=0; i<num; i++)
                {
                    str += buffer[i];
                } 
                if(str.find("BS=") != std::string::npos)
                {
                    int str_index = str.find("BS=");
                    int vel_right,vel_left;
                    float vel_linear,vel_angular;
                    std::stringstream aa(str.substr(str_index+3,5));
                    aa >> vel_left;
                    str_index = str.find(':');
                    std::stringstream bb(str.substr(str_index+1,5));
                    bb >> vel_right;
                    vel_linear = 0.5f * (vel_left + vel_right) * 0.00121052;
                    vel_angular = (vel_right - vel_left) / TURN_RADIUS*0.00121052 / 2;

                    vel_angular = vel_angular*0.83;  //根据实际情况修正

                    geometry_msgs::Twist out_vel;
                    out_vel.linear.x = vel_linear;
                    out_vel.angular.z = vel_angular;
                    vel_pub.publish(out_vel);

                    static float pos_x = 0,pos_y = 0,theta = 0;
                    float delta_s = 0,delta_theta = 0;

                    delta_s = vel_linear*0.01;
                    delta_theta = vel_angular*0.01;
                    pos_x = pos_x + delta_s*cos(theta + 0.5*delta_theta);
                    pos_y = pos_y + delta_s*sin(theta + 0.5*delta_theta);
                    theta = theta + delta_theta;

                    ros::Time current_time = ros::Time::now();
                    odom_.header.stamp = current_time;
                    odom_.pose.pose.position.x = pos_x;
                    odom_.pose.pose.position.y = pos_y;
                    odom_.pose.pose.position.z = 0.0;
                    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
                    odom_.pose.pose.orientation = q;
                    odom_.twist.twist.linear.x = vel_linear;
                    odom_.twist.twist.linear.y = 0.0;
                    odom_.twist.twist.angular.z = vel_angular;
                    ros_odom_pub_.publish(odom_);

                    odom_tf_.header.stamp = current_time;
                    odom_tf_.transform.translation.x = pos_x;
                    odom_tf_.transform.translation.y = pos_y;

                    odom_tf_.transform.translation.z = 0.0;
                    odom_tf_.transform.rotation = q;
                    tf_broadcaster_.sendTransform(odom_tf_);
                }
            }           
            get_vel_cnt++;
            if(get_vel_cnt >=1){
                std::string str_get_vel = "?BS\r";
                sp.write(str_get_vel);
                get_vel_cnt = 0;
            } 
            ros::spinOnce();
            loop_rate.sleep();
        }
        sp.close();
        return 0;
    }
    //TODO 暂时把驱动放在这里 不太清楚两个节点能否同时打开一个串口？？
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        int linear_vel = cmd_vel->linear.x * VEL_TO_RPM*0.67; //0-1500对应0-1000
        int angular_vel =cmd_vel->angular.z * VEL_TO_RPM*TURN_RADIUS*0.67; //  820*0.25 = 205  
        std::stringstream ss;
        ss << "!M " << -angular_vel << " " << linear_vel << "\r";
        sp.write(ss.str()); 

    }
    void SetFlag(bool state){
        start_flag = state;
    }
private:    
    bool start_flag{false};
    serial::Serial sp;
    ros::Publisher vel_pub;
    ros::Publisher odom_pub_;
};

class EncoderSource : public SourceNode{
public:
    EncoderSource();
    ~EncoderSource()=default;
    void  CmdCallback(const robot_msgs::CmdConstPtr &msg);
    bool  LoadConfig(std::string file);
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  UpdateState();
    std::thread* encoder_thread_;
    Encoder encoder;
};

#endif
