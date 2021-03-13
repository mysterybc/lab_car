#include "gps.h"
#include "iostream"
int count = 0;
extern Debug::DebugLogger logger;
int GPS::UpDateGPS()
{
    //声明节点句柄
    ros::NodeHandle nh;

    //发布还不确定 
    //ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 20); 
    ros::Publisher gps_odom_pub_ = nh.advertise<nav_msgs::Odometry>("gps_odom", 30);
    ros::Publisher navsatfix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("navsatfix", 30);
    ros::Publisher gps_imu_pub = nh.advertise<sensor_msgs::Imu>("GPS_IMU_data",30);
    //! ros gps odometry tf
    geometry_msgs::TransformStamped gps_odom_tf;
    //! ros gps odometry tf broadcaster
    tf::TransformBroadcaster tf_broadcaster_;
    //! ros odometry message
    nav_msgs::Odometry odom_;
    sensor_msgs::NavSatFix nav_msg;

    odom_.header.frame_id = "gps_odom";
    odom_.child_frame_id = "base_link";

    gps_odom_tf.header.frame_id = "map";
    gps_odom_tf.child_frame_id = "gps_odom";

    try 
    { 
        //获取gps port
        std::string gps_port;
        nh.param<std::string>("/gps_port",gps_port,"/dev/ttyUSB1");
        //设置串口属性，并打开串口 
        ser.setPort(gps_port);
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
    bool data_right = false;
    double position_x,position_y;
    while(ros::ok())
    {
        int i = 0;
        while(ser.available()>=69)
        {
            ser.read(temp_dat,1);
            if(temp_dat[0] == 0xaa)
            {
                ser.read(temp_dat,1);
                if(temp_dat[0] == 0x55)
                {
                    uint8_t check_sum = 0;
                    ser.read(temp_dat,67);
                    for(i=0;i<66;i++){
                        check_sum += temp_dat[i];
                    }
                    if(check_sum == temp_dat[66]){
                        data_right = true;
                        break;
                    }
                }
            }
        }
        if(!start_flag){
            data_right = false;
            continue;
        }
        if(data_right == true)
        {
            data_right = false;
            TransData();
            //发布odom信息和tf信息
            double x,y;
            gps_common::UTM(GI320_data.latitude,GI320_data.longitude,&x,&y);
           
            //time stamp
            ros::Time current_time = ros::Time::now();
            odom_.header.stamp = current_time;
            //position
            odom_.pose.pose.position.x = y-GPS_OFFSET_Y;
            odom_.pose.pose.position.y = GPS_OFFSET_X-x;
            odom_.pose.pose.position.z = GI320_data.altitude-46.8;
            
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0,
                                                                                  0,
                                                                                  deg2rad(GI320_data.yaw-90)    );

            //liner speed 31b-39b
            odom_.twist.twist.linear.x = GI320_data.eastSpeed;
            odom_.twist.twist.linear.y = GI320_data.northSpeed;
            odom_.twist.twist.linear.z = GI320_data.upSpeed;
            odom_.twist.twist.angular.z = GI320_data.yaw_gro;
            

            gps_odom_tf.header.stamp = current_time;
            gps_odom_tf.transform.translation.x = y-GPS_OFFSET_Y;
            gps_odom_tf.transform.translation.y = GPS_OFFSET_X-x;

            gps_odom_tf.transform.translation.z = GI320_data.altitude-46.8;
            gps_odom_tf.transform.rotation = q;
            tf_broadcaster_.sendTransform(gps_odom_tf);

            odom_.pose.pose.orientation = q;

            sensor_msgs::Imu imu;
            imu.header.stamp = ros::Time().now();
            imu.header.frame_id = "imu_link";
            //左边是x，前是y，z向下
            imu.linear_acceleration.x = ((double)GI320_data.northAcc)/100.0;
            imu.linear_acceleration.y = ((double)GI320_data.eastAcc)/100.0;
            imu.linear_acceleration.z = -((double)GI320_data.upAcc)/100.0;
            imu.angular_velocity.x = deg2rad(((double)GI320_data.roll_gro)/100.0);
            imu.angular_velocity.y = deg2rad(((double)GI320_data.pitch_gro)/100.0);
            imu.angular_velocity.z = -deg2rad(((double)GI320_data.yaw_gro)/100.0);
            imu.orientation = q;
            if(count++%10==0){
                // gps_imu_pub.publish(imu);
                gps_odom_pub_.publish(odom_);
                // std::cout << "roll angle is" << GI320_data.roll << std::endl;
                // std::cout << "yaw angle is" << GI320_data.yaw << std::endl;
                // std::cout << "pitch angle is" << GI320_data.pitch << std::endl;
            }
            if(count >10000){
                count = 0;
            }
            //发布gps经纬度信息
            nav_msg.header.stamp = current_time;
            nav_msg.latitude = GI320_data.latitude;
            nav_msg.longitude = GI320_data.longitude;
            nav_msg.altitude = GI320_data.altitude;
            navsatfix_pub_.publish(nav_msg);      
        }
        ros::spinOnce();  
        loop_rate.sleep();      
    }
    ser.close();
    return 0;
}

double GPS::deg2rad(double angle){
    return angle * 3.1415926 / 180.0;
}


//如果gps状态改变，则info状态信息
void GPS::InfoGpsState()
{
    static uint8_t state = 0;
    static uint8_t count = 0;
    if( GI320_data.loctionMode != state){
        count++;
    }
    else{
        count= 0;
    } 
    if(count>=2)
    {
        state = GI320_data.loctionMode;
        count = 0;
        logger.DEBUGINFO(car_id,"GNSS state changed");
        switch(state)
        {
            case 0:logger.DEBUGINFO(car_id,"lacation mode:      SPP     ");break;
            case 1:logger.DEBUGINFO(car_id,"lacation mode:   RTK-FLOAT  ");break;
            case 2:logger.DEBUGINFO(car_id,"lacation mode:   RTK-FIXED  ");break;
            case 3:logger.DEBUGINFO(car_id,"lacation mode:    NO_GNSS   ");break;
            case 4:logger.DEBUGINFO(car_id,"lacation mode: OLD LOCATION ");break;
            default:logger.DEBUGINFO(car_id,"location mode :  data error ");
        }
    }
}

//显示收到的数据 debug用
void GPS::InfoData()
{
    logger.DEBUGINFO(car_id,"latitude:%lf",GI320_data.latitude);
    logger.DEBUGINFO(car_id,"longitude:%lf",GI320_data.longitude);
    logger.DEBUGINFO(car_id,"altitude:%f",GI320_data.altitude);
    logger.DEBUGINFO(car_id,"northSpeed:%f",GI320_data.northSpeed);
    logger.DEBUGINFO(car_id,"eastSpeed:%f",GI320_data.eastSpeed);
    logger.DEBUGINFO(car_id,"upSpeed:%f",GI320_data.upSpeed);
    logger.DEBUGINFO(car_id,"roll:%f",GI320_data.roll);
    logger.DEBUGINFO(car_id,"pitch:%f",GI320_data.pitch);
    logger.DEBUGINFO(car_id,"yaw:%f",GI320_data.yaw);
    logger.DEBUGINFO(car_id,"northAcc:%d",GI320_data.northAcc);
    logger.DEBUGINFO(car_id,"eastAcc:%d",GI320_data.eastAcc);
    logger.DEBUGINFO(car_id,"upAcc:%d",GI320_data.upAcc);
    logger.DEBUGINFO(car_id,"roll_gro:%d",GI320_data.roll_gro);
    logger.DEBUGINFO(car_id,"pitch_gro:%d",GI320_data.pitch_gro);
    logger.DEBUGINFO(car_id,"yaw_gro:%d",GI320_data.yaw_gro);
}
//将数据保存到GI320_data中
void GPS::TransData()
{
    //处理数据，暂存   InfoData     
    memcpy(var1.data8, &temp_dat[9], 16); 
    
    GI320_data.latitude = var1.data64[0];    //经度
    GI320_data.longitude = var1.data64[1];   //纬度

    memcpy(var2.data8, &temp_dat[25], 28);
    GI320_data.altitude = var2.data32[0];    //海拔
    GI320_data.northSpeed = var2.data32[1];  //北向速度
    GI320_data.eastSpeed = var2.data32[2];   //东向速度
    GI320_data.upSpeed = var2.data32[3];     //地向速度
    GI320_data.roll = var2.data32[4];        //横滚
    GI320_data.pitch = var2.data32[5];       //俯仰
    GI320_data.yaw = var2.data32[6];         //航向
    GI320_data.yaw  = -GI320_data.yaw + 90;  //坐标系转换

    memcpy(var3.data8,  &temp_dat[53], 12);
    GI320_data.northAcc = var3.data16[0];   //北向加速度
    GI320_data.eastAcc = var3.data16[1];    //东向加速度
    GI320_data.upAcc = var3.data16[2];      //向上的加速度
    GI320_data.roll_gro = var3.data16[3];
    GI320_data.pitch_gro = var3.data16[4];
    GI320_data.yaw_gro = var3.data16[5];

    GI320_data.loctionMode = temp_dat[65]&0x3f;
    GI320_data.sulutionMode = temp_dat[65]>>6;
    InfoGpsState();
    //InfoData();

}

void GPS::Start(){
    start_flag = true;
    gps_thread_ = new std::thread(std::bind(&GPS::UpDateGPS,this));
}