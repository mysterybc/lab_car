#include "driver_source.h"
extern Debug::DebugLogger logger;
Encoder::Encoder(){
    ros::NodeHandle nh;
    my_lib::GetParam("encoder",&car_id);
    if(car_id == 1){
        turn_radius = 0.26;
        twist_flag = -1;
        linear_fb_factor = 1.0;
        angular_fb_factor = 3.5;
        linear_cmd_factor = 0.74;
        angular_cmd_factor = 1.2;
    }
    else{
        turn_radius = 0.29;
        twist_flag = 1;
        linear_fb_factor = 1;
        angular_fb_factor = 2.71;
        linear_cmd_factor = 0.525;
        angular_cmd_factor = 0.7;
    }
    

}
int Encoder::UpdateOdom()
{
    ros::NodeHandle n;

    std::string port;
    n.param<std::string>("vehicle_port", port, "/dev/driver_port"); 

    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
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
        ROS_INFO("port is opened.");
    }
    else
    {
        return -1;
    }
    //! ros publisher for odometry information
    ros::Subscriber subTwist_ = n.subscribe("komodo/cmd_vel", 1000, &Encoder::cmdVelCallback,this);
    ros::Publisher ros_odom_pub_ = n.advertise<nav_msgs::Odometry>("odom_raw", 30);
    //! ros odometry message
    nav_msgs::Odometry odom_;

    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";

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
                //计算线速度和角速度
                vel_linear = 0.5f * (vel_left + vel_right) * 0.00121052 / linear_fb_factor;
                vel_angular = (vel_right - vel_left) / turn_radius * 0.00121052 / angular_fb_factor;


                static float pos_x = 0,pos_y = 0,theta = 0;
                float delta_s = 0,delta_theta = 0;

                //计算角度
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
                odom_.twist.twist.angular.z = twist_flag * vel_angular;
                ros_odom_pub_.publish(odom_);

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


void Encoder::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    int linear_vel = -cmd_vel->linear.x * VEL_TO_RPM * linear_cmd_factor; //0-1500对应0-1000
    int angular_vel = twist_flag * cmd_vel->angular.z * VEL_TO_RPM * turn_radius * angular_cmd_factor; //  820*0.25 = 205  
    std::stringstream ss;
    ss << "!M " << linear_vel << " " <<  angular_vel << "\r";
    sp.write(ss.str()); 

}