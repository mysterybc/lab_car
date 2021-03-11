#pragma once

#include <ros/ros.h> 
#include <serial/serial.h>  
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <conversions.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_broadcaster.h"
#include <sstream> 
#include <std_msgs/Empty.h> 
#include <thread>
#include "my_debug_info.h"



class GPS{
public:
    double GPS_OFFSET_X = 440600.68;
    double GPS_OFFSET_Y = 4423318.00;
    GPS() = default;
    ~GPS() = default;
    //显示收到的数据 debug用
    void InfoData();
    void TransData();
    double deg2rad(double angle);
    //如果gps状态改变，则info状态信息
    void InfoGpsState();
    int UpDateGPS();
    void Start();
    void Stop(){start_flag = false;};
    void Pause(){start_flag = false;}
    void Resume(){start_flag = true;}
    void Exit(){
        start_flag = false;
        delete gps_thread_;
    }
    int car_id;


private:
    bool start_flag{false};
    serial::Serial ser;             //声明串口对象
    std::thread* gps_thread_;       //gps数据读取线程
    uint8_t temp_dat[200];          //定义串口数据存放数组
    union u8todouble{  
        uint8_t data8[16];  
        double data64[2];  
    }var1; 
    union u8tofloat{  
        uint8_t data8[28];  
        float data32[7];  
    }var2;
    union u8toshort{  
        uint8_t data8[24];  
        short int data16[6];  
    }var3;
    struct temp_data{
      double latitude;
      double longitude;
      float  altitude;
      float  northSpeed;
      float  eastSpeed;
      float  upSpeed;
      float  roll;
      float  pitch;
      float  yaw;
      short  northAcc;
      short  eastAcc;
      short  upAcc;
      short  roll_gro;
      short  pitch_gro;
      short  yaw_gro;
      uint8_t  sulutionMode;
      uint8_t  loctionMode;

    }GI320_data;
};







