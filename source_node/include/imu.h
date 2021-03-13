#pragma once

#include <ros/ros.h> 
#include <serial/serial.h> 
#include <std_msgs/String.h>
#include <sstream> 
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h> 
#include <thread>
#include "my_debug_info.h"


class IMU{
public:
    IMU() = default;
    ~IMU() = default;
    int UpDateIMU();
    int32_t comb32(uint8_t first ,uint8_t second ,uint8_t third ,uint8_t fouth); //数据解析8->32
    void write_callback(const std_msgs::String::ConstPtr& msg); 
    int16_t comb16(uint8_t first ,uint8_t second);  //数据解析8->16 
    void RosRPY2Q(const double& roll,const double& pitch,const double& yaw,geometry_msgs::Quaternion& quaternion);   
    void Start(){
        start_flag = true;
        imu_thread_ = new std::thread(std::bind(&IMU::UpDateIMU,this));
    }
    void Stop(){start_flag = false;};
    void Pause(){start_flag = false;}
    void Resume(){start_flag = true;}
    void Exit(){
        start_flag = false;
        delete imu_thread_;
    }
    int car_id;
    double start_angle;
    bool imu_init{false};


private:
    bool start_flag{false};
    serial::Serial ser;             //声明串口对象
    uint8_t imu_dat[100]; //定义串口数据存放数组
    std::thread* imu_thread_;       //gps数据读取线程
    union var{  
        uint8_t data8[48];  
        float data32[12];  
    }imu_var;
};








