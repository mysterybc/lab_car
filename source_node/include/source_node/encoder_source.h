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
    int UpdateOdom();
    //TODO 暂时把驱动放在这里 不太清楚两个节点能否同时打开一个串口？？
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
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
