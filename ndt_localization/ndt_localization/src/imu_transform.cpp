#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/common/common.h>

#include<queue>

ros::Publisher imu_pub,imu_odom_pub;
//z轴向下，x轴向左的右手坐标系 >> x轴向前，y轴向右，z轴向上的右手坐标系  >> z轴向前,x轴向左的右手坐标系
const int imuQueLength = 200;
ros::Time last_time;
//观测噪声：一般是最大偏差的3倍

std::queue<sensor_msgs::Imu> imu_queue;

struct IMU_SHIFT
{   
    float imuShiftX;
    float imuShiftY;
    float imuShiftZ;
    float imuVeloX;
    float imuVeloY;
    float imuVeloZ;
    float imuAngularRotationX;
    float imuAngularRotationY;
    float imuAngularRotationZ;
};
//IMU_SHIFT imu_shift[200];
bool newimu_receiver = false;


//imu回调线程
void imu_data_cb(const sensor_msgs::Imu::ConstPtr& imuIn)
{
    newimu_receiver = true;
    double roll,pitch,yaw;
    sensor_msgs::Imu imu_out;

    roll = imuIn->orientation.x;
    pitch = imuIn->orientation.y;
    yaw = imuIn->orientation.z;

    geometry_msgs::Quaternion quaternion;//定义四元数
    quaternion=tf::createQuaternionMsgFromRollPitchYaw(roll*M_PI/180.0,pitch*M_PI/180,(yaw+360.0)*M_PI/180); //欧拉角转为4元数
    
    imu_out.header = imuIn->header;
    imu_out.orientation = quaternion;
    imu_out.angular_velocity = imuIn->angular_velocity;
    imu_out.linear_acceleration = imuIn->linear_acceleration;

    //imu_queue.push(imu_out);

    imu_pub.publish(imu_out);

}

// void RosQ2RPY(const geometry_msgs::Quaternion& quaternion ,double roll,double pitch,double yaw)
// {
//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(quaternion, quat);
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
// }

// void sensorimu2memory(const sensor_msgs::Imu& imuin, float imumemory[])
// {

//     double roll,pitch,yaw;
//     RosQ2RPY(imuin.orientation,roll,pitch,yaw);
//     imumemory[0] = roll;
//     imumemory[1] = pitch;
//     imumemory[2] = yaw;
//     imumemory[3] = imuin.angular_velocity.x;
//     imumemory[4] = imuin.angular_velocity.y;
//     imumemory[5] = imuin.angular_velocity.z;
//     imumemory[6] = imuin.linear_acceleration.x;
//     imumemory[7] = imuin.linear_acceleration.y;
//     imumemory[8] = imuin.linear_acceleration.z;
//     imumemory[9] = imuin.header.stamp.toSec();

// }

// void AccumulateIMUShiftAndRotation(IMU_SHIFT& curimushift,const IMU_SHIFT& lastimushift,const float imumemoryfront[],const float imumemoryback[])
// {
//     // float roll = imuRoll[imuPointerLast];
//     // float pitch = imuPitch[imuPointerLast];
//     // float yaw = imuYaw[imuPointerLast];
//     // float accX = imuAccX[imuPointerLast];
//     // float accY = imuAccY[imuPointerLast];
//     // float accZ = imuAccZ[imuPointerLast];

//     // float x1 = cos(roll) * accX - sin(roll) * accY;
//     // float y1 = sin(roll) * accX + cos(roll) * accY;
//     // float z1 = accZ;

//     // float x2 = x1;
//     // float y2 = cos(pitch) * y1 - sin(pitch) * z1;
//     // float z2 = sin(pitch) * y1 + cos(pitch) * z1;

//     // accX = cos(yaw) * x2 + sin(yaw) * z2;
//     // accY = y2;
//     // accZ = -sin(yaw) * x2 + cos(yaw) * z2;

//     // int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
//     // double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
//     // if (timeDiff < scanPeriod) {

//     //     imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
//     //     imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
//     //     imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

//     //     imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
//     //     imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
//     //     imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

//     //     imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
//     //     imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
//     //     imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
//     // }

//     curimushift = lastimushift;
// }
// void RemoveGravity(const sensor_msgs::Imu& imuin,float imumemory[])//去除imu中的重力加速度
// { 
//         //imumemory[9];
//         //roll,pitch,yaw,Vx,Vy,Vz,accX,accY,accZ,time
//         //0   ,1    ,2  ,3 ,4 ,5 ,6   ,7   ,8   , 9
//         sensorimu2memory(imuin,imumemory);
//         float roll = imumemory[0];
//         float pitch = imumemory[1];
//         float yaw = imumemory[2];

//         imumemory[7] = float(imu_queue.front().linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);//y轴加速度
//         imumemory[8] = float(imu_queue.front().linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);//z轴加速度 
//         imumemory[6] = float(imu_queue.front().linear_acceleration.x + sin(pitch)             * 9.81);//x轴加速度

//         imumemory[9] = imu_queue.front().header.stamp.toSec();

// }
// IMU_SHIFT lastimushift;
// void Imu_Integration(const ros::Time& cur_time) //对imu进行积分
// {
//     std::vector<IMU_SHIFT> imu_shift_vector;
//     imu_shift_vector.resize(0);

//     while(!imu_queue.empty())
//     {
//         if(imu_queue.front().header.stamp.toSec() <= cur_time.toSec())
//         {
//             float imumemoryfront[10];

//             RemoveGravity(imu_queue.front(),imumemoryfront);//去除imu中的重力

//             if(imu_queue.empty())
//             {
//                 break;
//             }
//             else  imu_queue.pop();

//             //对imu进行积分
//             float imumemoryback[10];

//             RemoveGravity(imu_queue.front(),imumemoryback);//去除imu中的重力
            
//             IMU_SHIFT curimushift;
//             //AccumulateIMUShiftAndRotation(curimushift,lastimushift,imumemoryfront,imumemoryback);
//             imu_shift_vector.push_back(curimushift);

//             /////////////

        
//         }
//         else break;
//     }
    


    
//     nav_msgs::Odometry imu_odom;

// }


// //定时器线程
// void timerCallback(const ros::TimerEvent& event)
// {
//     //使用rosbag中的仿真时间作为当前定时器的时间
//     ros::Time  cur_time = ros::Time::now();

//     //如果imu还未接收到信息，则退出
//     if(newimu_receiver == false)
//     return;

//     while(imu_queue.empty() != true){
//         if (imu_queue.front().header.stamp.toSec() <= last_time.toSec())
//             imu_queue.pop();
//         else
//             break;
//     }

//     Imu_Integration(cur_time);

//     last_time = cur_time;

//     newimu_receiver = false;

// }


//主函数
int main (int argc, char** argv)
{
    ros::init (argc, argv, "imu_transform");

    ros::NodeHandle nh; 

    ros::Subscriber sub_Imu = nh.subscribe<sensor_msgs::Imu> ("/imu", 50, imu_data_cb);

    //ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);

    //imu_odom_pub = nh.advertise<sensor_msgs::Imu>("/imu_data", 50);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data", 50);

    //last_time = ros::Time::now();


    ros::spin();

    return 0;
}