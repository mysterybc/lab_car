#ifndef IMU_H
#define IMU_H

#include <ros/ros.h> 
#include <serial/serial.h> 
#include <std_msgs/String.h>
#include <sstream> 
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h> 
#include <thread>


class IMU{
public:
    IMU() = default;
    ~IMU() = default;
    int UpDateIMU(){
        //声明节点句柄
        ros::NodeHandle nh;
        //订阅主题，并配置回调函数 
        ros::Subscriber write_sub = nh.subscribe("write", 1000, &IMU::write_callback,this); 
        ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 20); 
        try 
        { 
            //设置串口属性，并打开串口 
            ser.setPort("/dev/ttyUSB0"); 
            ser.setBaudrate(115200); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            ser.setTimeout(to); 
            ser.open(); 
        } 
        catch (serial::IOException& e) 
        { 
            ROS_ERROR_STREAM("Unable to open port "); 
            return -1; 
        } 
        //检测串口是否已经打开，并给出提示信息 
        if(ser.isOpen()) 
        { 
            ROS_INFO_STREAM("Serial Port initialized"); 
        } 
        else 
        { 
            return -1; 
        } 
        //指定循环的频率 
        ros::Rate loop_rate(200); 
        sensor_msgs::Imu imu_data;
        double robot_angle = 0.0;
        while(ros::ok())
        {
            while(ser.available() >= 57)
            {
                ser.read(imu_dat,1);
                if(imu_dat[0] == 0xaa)
                {
                    ser.read(imu_dat,1);
                    if(imu_dat[0] == 0x57)
                    {
                        ser.read(imu_dat,6);
                        ser.read(imu_dat,49);
                        memcpy(imu_var.data8, imu_dat, 48);

                        imu_data.header.stamp = ros::Time::now();
                        imu_data.header.frame_id = "imu_data";
                        //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
                        imu_data.orientation.x = 0;
                        imu_data.orientation.y = 0;
                        imu_data.orientation.z = 0;
                        imu_data.orientation.w = 1;
                        //线加速度
                        imu_data.angular_velocity.x = imu_var.data32[0]; 
                        imu_data.angular_velocity.y = imu_var.data32[1];
                        imu_data.angular_velocity.z = imu_var.data32[2];
                        //角速度
                        imu_data.linear_acceleration.x = imu_var.data32[3]; 
                        imu_data.linear_acceleration.y = imu_var.data32[4]; 
                        imu_data.linear_acceleration.z = imu_var.data32[5];

                        robot_angle += imu_var.data32[2] * 0.01;
                        ROS_INFO("imu:%f",robot_angle);
                        IMU_pub.publish(imu_data);                  
                    }                
                }
            }
            ros::spinOnce();  
            loop_rate.sleep();      
        }
        ser.close();
        return 0;
    }

    void write_callback(const std_msgs::String::ConstPtr& msg) 
    { 
        ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
        ser.write(msg->data);   //发送串口数据 
    } 
    int16_t comb16(uint8_t first ,uint8_t second)  //数据解析8->16
    {
        int16_t temp=0;
        temp = ((int16_t)first<<8);
        temp |=((int16_t)second);
        
        return temp;
    }
    int32_t comb32(uint8_t first ,uint8_t second ,uint8_t third ,uint8_t fouth) //数据解析8->32
    {
        int32_t temp=0;
        temp  = ((int32_t)first<<24);
        temp |= ((int32_t)second<<16);
        temp |= ((int32_t)third<<8);
        temp |= ((int32_t)fouth); 
        return temp;
    }      
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

#endif






