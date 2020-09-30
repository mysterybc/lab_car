#include "driver_source.h"


int Encoder::UpdateOdom()
{
    ros::NodeHandle n;

    std::string port;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("vehicle_port", port, "/dev/ttyUSB0"); 

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
        ROS_INFO_STREAM("port is opened.");
    }
    else
    {
        return -1;
    }

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("komodo/vel", 20);
    //! ros publisher for odometry information
    ros::Subscriber subTwist_ = n.subscribe("komodo/cmd_vel", 1000, &Encoder::cmdVelCallback,this);
    ros::Publisher ros_odom_pub_ = n.advertise<nav_msgs::Odometry>("odom_raw", 30);
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

                vel_angular = vel_angular*0.6;  //根据实际情况修正

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
                // tf_broadcaster_.sendTransform(odom_tf_);
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
    int linear_vel = -cmd_vel->linear.x * VEL_TO_RPM*0.67; //0-1500对应0-1000
    int angular_vel = -cmd_vel->angular.z * VEL_TO_RPM*TURN_RADIUS * 1.2; //  820*0.25 = 205  
    std::stringstream ss;
    ss << "!M " << linear_vel << " " <<  angular_vel << "\r";
    sp.write(ss.str()); 

}