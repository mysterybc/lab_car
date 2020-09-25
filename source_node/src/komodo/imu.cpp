#include "imu.h"

double roll = 0,pitch = 0,yaw = 0;
const double GRAVITY = 9.80665;

void IMU::RosRPY2Q(const double& roll,const double& pitch,const double& yaw,geometry_msgs::Quaternion& quaternion){
    tf::Quaternion quat;
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

int IMU::UpDateIMU(){
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数 
    ros::Subscriber write_sub = nh.subscribe("write", 1000, &IMU::write_callback,this); 
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 20); 
    try 
    { 
        //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB9"); 
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

                    //角速度
                    imu_data.angular_velocity.x = (imu_var.data32[1])/180.0*3.14; 
                    imu_data.angular_velocity.y = (imu_var.data32[0])/180.0*3.14;
                    imu_data.angular_velocity.z = (imu_var.data32[2])/180.0*3.14;
                    //线加速度
                    imu_data.linear_acceleration.x = imu_var.data32[4]*GRAVITY;
                    imu_data.linear_acceleration.y = imu_var.data32[3]*GRAVITY;
                    imu_data.linear_acceleration.z = imu_var.data32[5]*GRAVITY;

                    double accx,accy,accz;
                    accx = imu_data.linear_acceleration.x;
                    accy = imu_data.linear_acceleration.y;
                    accz = imu_data.linear_acceleration.z;
                    
                    pitch = - std::atan2(accx,std::sqrt(accy*accy+accz*accz)); //俯仰
                    roll = std::atan2(accy,accz);  //横滚

                    //debug output
                    //std::cout<<"initialize pitch: "<< 180.0* pitch/3.14<<"roll: "<<180.0 *roll/3.14<<std::endl;



                    //roll += imu_data.angular_velocity.x * 0.01;
                    //pitch += imu_data.angular_velocity.y * 0.01;
                    yaw += imu_data.angular_velocity.z * 0.01;

                    //std::cout<<"roll:  "<<roll<<" pitch: "<<pitch<<"yaw: "<<yaw<<std::endl;

                    geometry_msgs::Quaternion q;

                    RosRPY2Q(roll,pitch,yaw,q);

                    imu_data.orientation.x = q.x;
                    imu_data.orientation.y = q.y;
                    imu_data.orientation.z = q.z;
                    imu_data.orientation.w = q.w;

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

void IMU::write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    ser.write(msg->data);   //发送串口数据 
} 
int16_t IMU::comb16(uint8_t first ,uint8_t second)  //数据解析8->16
{
    int16_t temp=0;
    temp = ((int16_t)first<<8);
    temp |=((int16_t)second);
    
    return temp;
}
int32_t IMU::comb32(uint8_t first ,uint8_t second ,uint8_t third ,uint8_t fouth) //数据解析8->32
{
    int32_t temp=0;
    temp  = ((int32_t)first<<24);
    temp |= ((int32_t)second<<16);
    temp |= ((int32_t)third<<8);
    temp |= ((int32_t)fouth); 
    return temp;
}  