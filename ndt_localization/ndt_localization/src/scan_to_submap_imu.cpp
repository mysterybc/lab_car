#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/ndt.h>
#include <nav_msgs/Odometry.h>
#include <pclomp/ndt_omp.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/filter.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <thread>
#include <mutex>
#include "ros/package.h"

#include <ndt_localization/ndt_match.h>

typedef pcl::PointXYZ PointT;

bool imu_enable;
bool get_rviz_pose = false;
double score_threshold;
double score;
double imu_stamp;
double initial_x;
double initial_y;
double initial_yaw;
std::string map_source;

pcl::Filter<PointT>::Ptr downsample_filter;
pcl::Filter<PointT>::Ptr outlier_removal_filter;

//

//局部地图的参数
int laserCloudCenX = 12;
int laserCloudCenY = 12;

const int laserCloudX = 25;
const int laserCloudY = 25;

double map_segment_size = 20.0;//meter

const int laserCloudNum = laserCloudX * laserCloudY;

pcl::PointCloud<PointT>::Ptr laserCloudMapArray[laserCloudNum];


ros::Publisher sub_map_pub,transform_before_points_pub,transformed_points_pub,pubPoseForKITTI,filtered_pointcloud_pub;
//点云指针
pcl::PointCloud<PointT>::ConstPtr  prev_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr  cur_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::PointXYZ>::ConstPtr  point_cloud_map_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr sub_map(new pcl::PointCloud<PointT>);

//下采样滤波器
pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
}

//野值点滤波器
double radius = 0.5;
int min_neighbors = 2;

pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
}


//距离滤波器
//距离滤波器参数
double distance_near_thresh = 1.0;
double distance_far_thresh = 20;

pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
      [&](const PointT& p) {
        double d = p.getVector3fMap().norm();
        return d > distance_near_thresh && d < distance_far_thresh;
      }
    );

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
}
//高度滤波器
// double height = 4.0;
pcl::PointCloud<PointT>::ConstPtr height_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud, double max_height,double min_height)
{
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
      [&](const PointT& p) {
        double d = p.z;
        return d > min_height && d < max_height;
      }
    );

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
}

double downsample_resolution = 0.2;

//点云滤波处理
pcl::PointCloud<PointT>::ConstPtr filter_point(const pcl::PointCloud<PointT>::Ptr& source_point)
{    
    //remove nan
    pcl::PointCloud<PointT>::Ptr nan_filtered(source_point);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*nan_filtered, *nan_filtered, indices);


    //distance filter
    pcl::PointCloud<PointT>::ConstPtr filtered= distance_filter(nan_filtered);
    // ROS_INFO("distance");
    

    filtered = height_filter(filtered,4.0,0);

    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;
    filtered = downsample(filtered);
    // ROS_INFO("downsample");
    

    pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
    rad->setRadiusSearch(radius);
    rad->setMinNeighborsInRadius(min_neighbors);
    outlier_removal_filter = rad;
    filtered = outlier_removal(filtered);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*filtered, output);
    output.header.frame_id = "velodyne";

    filtered_pointcloud_pub.publish(output);
    return filtered;
}

double roll,pitch,yaw;

Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
Eigen::Matrix4f cur_trans;                       // current estimated transform from keyframe

Eigen::Quaternionf cur_imu_quaternion;
Eigen::Quaternionf prev_imu_quaternion;
int not_first_frame;


//submap生成： 输入：1、预估的变换关系 2、完整的点云地图（或者分割好的点云cube） 输出：当前预估关系的map下的submap
void sub_map_generation(const Eigen::Matrix4f transform,const pcl::PointCloud<PointT>::Ptr laserCloudMapArray[] , pcl::PointCloud<PointT>::Ptr sub_map)
{
    double sub_map_start_time = ros::Time::now().toSec();
    int cur_I,cur_J;
    int trans_to_map_X,trans_to_map_Y;

    //lidar视域范围内(FOV)的点云集索引
    int laserCloudValidInd[125];
    //lidar周围的点云集索引
    int laserCloudSurroundInd[125];

    trans_to_map_X = transform(0,3);
    trans_to_map_Y = transform(1,3);

    //确定x,y坐标对应的cube索引
    cur_I = int((trans_to_map_X + map_segment_size/2.0) / map_segment_size) + laserCloudCenX;
    cur_J = int((trans_to_map_Y + map_segment_size/2.0) / map_segment_size) + laserCloudCenY;

    if (trans_to_map_X + map_segment_size/2.0 < 0) cur_I--;
    if (trans_to_map_Y + map_segment_size/2.0 < 0) cur_J--;

    int cubeInd = cur_I + laserCloudX * cur_J; //ind = I + laserCloudX * J


    int laserCloudValidNum = 0;

    for (int i = cur_I - 1; i <= cur_I + 1; i++) 
    {
        for (int j = cur_J - 1; j <= cur_J + 1; j++) 
        {
            if (i >= 0 && i < laserCloudX && 
                j >= 0 && j < laserCloudY ) 
            {
                //如果索引合法
                //记住视域范围内的cube索引，匹配用
                laserCloudValidInd[laserCloudValidNum] = i + laserCloudX * j;

                laserCloudValidNum++;
            }
        }
    }

    sub_map->clear();


    for (int i = 0; i < laserCloudValidNum; i++) 
    {
        *sub_map += *laserCloudMapArray[laserCloudValidInd[i]];
    }
    //显示地图的点云数量
//    std::cerr << "map_PointCloud : " << sub_map->width * sub_map->height
//    << " data points (" << pcl::getFieldsList(*sub_map) << ")."<<std::endl;
    // std::cout << "sub map generation cost time is " << ros::Time::now().toSec() - sub_map_start_time << std::endl;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*sub_map, output);
    output.header.frame_id = "world";

    sub_map_pub.publish(output);
    // std::cout << "sub map generation include publish pointcloud cost time is " << ros::Time::now().toSec() - sub_map_start_time << std::endl;
}

void RosQ2RPY(const geometry_msgs::Quaternion& quaternion ,double& roll,double& pitch,double& yaw)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

geometry_msgs::TransformStamped odom_trans;

void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) 
{
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    Eigen::Matrix3f rotation_matrix = pose.block(0,0,3,3);
    Eigen::Quaternionf tem_q = Eigen::Quaternionf(rotation_matrix);

    geometry_msgs::Quaternion q_geo;
    q_geo.x = tem_q.x();
    q_geo.y = tem_q.y();
    q_geo.z = tem_q.z();
    q_geo.w = tem_q.w();

    double tmp_roll,tmp_pitch,tmp_yaw;
    RosQ2RPY(q_geo,tmp_roll,tmp_pitch,tmp_yaw);

    odom.header.stamp = stamp;
    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);


    odom.pose.pose.orientation.x = tem_q.x();
    odom.pose.pose.orientation.y = tem_q.y();
    odom.pose.pose.orientation.z = tem_q.z();
    odom.pose.pose.orientation.w = tem_q.w();


    odom.child_frame_id = "lidar_link";
    //odom.twist.twist.linear.x = tmp_roll*180/M_PI;//;
    //odom.twist.twist.linear.y = tmp_pitch*180/M_PI;
    //odom.twist.twist.linear.z = tmp_yaw*180/M_PI;
    //odom.twist.twist.angular.z = tmp_yaw;

    pubPoseForKITTI.publish(odom);

    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "lidar_link";
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = q_geo;

    //odom_broadcaster.sendTransform(odom_trans);


}

Eigen::Matrix4f error_trans;
double timeScanCur;

//验证一下自己的想法，把旋转和平移分开
Eigen::Vector3f pre_translation;
Eigen::Vector3f cur_translation;

Eigen::Matrix4f matching_scan_to_map_fix(double& score,const pcl::PointCloud<PointT>::ConstPtr& cur_point,const pcl::PointCloud<PointT>::ConstPtr& map_point)
{
    NdtMatch ndtmatch;
    //估计当前相对于上一帧增加的变换矩阵
    if( not_first_frame != 1 )
    {
        prev_trans.setIdentity();   
        cur_trans.setIdentity();
        error_trans.setIdentity();
        
        cur_trans(0,3) = initial_x;
        cur_trans(1,3) = initial_y;

        geometry_msgs::Quaternion q_first;//定义四元数
	    q_first=tf::createQuaternionMsgFromRollPitchYaw(0,0,initial_yaw*M_PI/180); //欧拉角转为4元数
        Eigen::Quaternionf first_Eq(q_first.w,q_first.x, q_first.y, q_first.z);
        cur_trans.block(0,0,3,3) = first_Eq.matrix();
        sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

        //如果说不在数据集的起始位置
        cur_trans = ndtmatch.matching(score,cur_point,sub_map,cur_trans);

        pcl::PointCloud<PointT>::Ptr  transformed_cloud_2(new pcl::PointCloud<PointT>);

        pcl::transformPointCloud (*cur_point, *transformed_cloud_2, cur_trans);

        ros::Time msg_time(timeScanCur);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg (*transformed_cloud_2, output);
        output.header.frame_id = "world";
        output.header.stamp =  msg_time;

        transformed_points_pub.publish(output);

        geometry_msgs::Quaternion quaternion;//定义四元数
	    quaternion=tf::createQuaternionMsgFromRollPitchYaw(roll*M_PI/180.0,pitch*M_PI/180,(yaw+360.0)*M_PI/180); //欧拉角转为4元数 
        Eigen::Quaternionf q(quaternion.w,quaternion.x, quaternion.y, quaternion.z);
        prev_imu_quaternion = q;


        prev_trans = cur_trans;

        return cur_trans;
    } 
    //确定imu引入的旋转矩阵
    geometry_msgs::Quaternion cur_quaternion;//定义四元数
    cur_quaternion=tf::createQuaternionMsgFromRollPitchYaw(roll*M_PI/180.0,pitch*M_PI/180,(yaw+360.0)*M_PI/180); //欧拉角转为4元数
    Eigen::Quaternionf cur_q(cur_quaternion.w,cur_quaternion.x, cur_quaternion.y, cur_quaternion.z);
    cur_imu_quaternion = cur_q;



    Eigen::Matrix3f imu_prev_rotation_matrix;
    Eigen::Matrix3f imu_cur_rotation_matrix;
    Eigen::Matrix3f imu_err_rotation_matrix;
    imu_prev_rotation_matrix =  prev_imu_quaternion.matrix();
    imu_cur_rotation_matrix = cur_imu_quaternion.matrix();
    imu_err_rotation_matrix = imu_cur_rotation_matrix * imu_prev_rotation_matrix.transpose(); //确定旋转增量

    //确定当前的旋转量
    Eigen::Matrix3f prev_rotation;
    Eigen::Matrix3f cur_rotation;
    prev_rotation = prev_trans.block(0,0,3,3);
    cur_rotation  = imu_err_rotation_matrix * prev_rotation;

    cur_trans.block(0,0,3,3) = cur_rotation.matrix();

    //确定当前的平移量
    Eigen::Vector3f err_translation(error_trans(0,3),error_trans(1,3),error_trans(2,3)); //确定平移增量
    Eigen::Vector3f prev_translation(cur_trans(0,3),cur_trans(1,3),cur_trans(2,3)); //先前平移相对于平移增量
    Eigen::Vector3f cur_translation;
    cur_translation = err_translation+prev_translation;

    cur_trans(0,3) =  cur_translation(0);
    cur_trans(1,3) =  cur_translation(1);
    cur_trans(2,3) =  cur_translation(2);

    //当前相对于世界坐标系的位移和旋转

    sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

    sensor_msgs::PointCloud2 before_cloud;
    pcl::PointCloud<PointT>::Ptr  transformed_cloud_1(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud (*cur_point, *transformed_cloud_1, cur_trans);
    pcl::toROSMsg (*transformed_cloud_1, before_cloud);
    before_cloud.header.frame_id = "world";

    transform_before_points_pub.publish(before_cloud);

    //该函数可以读一帧，匹配一帧
        // double matching_start_time = ros::Time::now().toSec();
    cur_trans = ndtmatch.matching(score,cur_point,sub_map,cur_trans);
    // std::cout << "scan matching cost time is " << ros::Time::now().toSec() - matching_start_time << std::endl;

    error_trans.block(0,0,3,3) = cur_trans.block(0,0,3,3) * prev_trans.block(0,0,3,3).transpose();
    error_trans.block<3, 1>(0, 3) = cur_trans.block<3, 1>(0, 3) - prev_trans.block<3, 1>(0, 3);


    double dx = error_trans.block<3, 1>(0, 3).norm();

    double da = std::acos(Eigen::Quaternionf(error_trans.block<3, 3>(0, 0)).w());

    if(dx > 2 || da > M_PI_2||score >= score_threshold)
    {
        Eigen::Matrix3f R;
        R.setIdentity();
        R.matrix()= imu_err_rotation_matrix.matrix();
        Eigen::Matrix3f p_R;
        p_R.matrix() = prev_trans.block(0,0,3,3);
        R = R*p_R;
        cur_trans.block(0,0,3,3) = R;
        ROS_ERROR("score >= score_threshold");
    }

    prev_trans = cur_trans;

    prev_imu_quaternion = cur_imu_quaternion;

    pcl::PointCloud<PointT>::Ptr  transformed_cloud_2(new pcl::PointCloud<PointT>);

    pcl::transformPointCloud (*cur_point, *transformed_cloud_2, cur_trans);

    ros::Time msg_time(timeScanCur);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*transformed_cloud_2, output);
    output.header.frame_id = "world";
    output.header.stamp =  msg_time;

    transformed_points_pub.publish(output);

    publish_odometry(msg_time, cur_trans);

    return cur_trans;

}

Eigen::Matrix4f matching_scan_to_map_fix_without_imu(double& score,const pcl::PointCloud<PointT>::ConstPtr& cur_point,const pcl::PointCloud<PointT>::ConstPtr& map_point)
{
    NdtMatch ndtmatch;
    //估计当前相对于上一帧增加的变换矩阵
    if( not_first_frame != 1 ) {
        prev_trans.setIdentity();
        cur_trans.setIdentity();
        error_trans.setIdentity();

        //如果说不在数据集的起始位置
        cur_trans(0, 3) = initial_x;
        cur_trans(1, 3) = initial_y;
        
        geometry_msgs::Quaternion q_first;//定义四元数
        q_first = tf::createQuaternionMsgFromRollPitchYaw(0, 0, initial_yaw * M_PI / 180); //欧拉角转为4元数
        Eigen::Quaternionf first_Eq(q_first.w, q_first.x, q_first.y, q_first.z);
        cur_trans.block(0, 0, 3, 3) = first_Eq.matrix();

        sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

        cur_trans = ndtmatch.matching(score, cur_point, sub_map, cur_trans);

        pcl::PointCloud<PointT>::Ptr  transformed_cloud_2(new pcl::PointCloud<PointT>);

        pcl::transformPointCloud (*cur_point, *transformed_cloud_2, cur_trans);

        ros::Time msg_time(timeScanCur);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg (*transformed_cloud_2, output);
        output.header.frame_id = "world";
        output.header.stamp =  msg_time;

        transformed_points_pub.publish(output);

        prev_trans = cur_trans;

        return cur_trans;
    }

    //确定当前的旋转量
    Eigen::Matrix3f err_rotation;
    Eigen::Matrix3f prev_rotation;
    Eigen::Matrix3f cur_rotation;
    prev_rotation = prev_trans.block(0,0,3,3);
    err_rotation = error_trans.block(0,0,3,3);
    cur_rotation  = err_rotation*prev_rotation;

    cur_trans.block(0,0,3,3) = cur_rotation.matrix();

    //确定当前的平移量
    Eigen::Vector3f err_translation(error_trans(0,3),error_trans(1,3),error_trans(2,3)); //确定平移增量
    Eigen::Vector3f prev_translation(cur_trans(0,3),cur_trans(1,3),cur_trans(2,3)); //先前平移相对于平移增量
    Eigen::Vector3f cur_translation;
    cur_translation = err_translation+prev_translation;
    //当前相对于世界坐标系的位移和旋转
    cur_trans(0,3) =  cur_translation(0);
    cur_trans(1,3) =  cur_translation(1);
    cur_trans(2,3) =  cur_translation(2);

    sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

    sensor_msgs::PointCloud2 before_cloud;
    pcl::PointCloud<PointT>::Ptr  transformed_cloud_1(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud (*cur_point, *transformed_cloud_1, cur_trans);
    pcl::toROSMsg (*transformed_cloud_1, before_cloud);
    before_cloud.header.frame_id = "world";

    transform_before_points_pub.publish(before_cloud);

    cur_trans = ndtmatch.matching(score,cur_point,sub_map,cur_trans);

    error_trans.block(0,0,3,3) = cur_trans.block(0,0,3,3) * prev_trans.block(0,0,3,3).transpose();
    error_trans.block<3, 1>(0, 3) = cur_trans.block<3, 1>(0, 3) - prev_trans.block<3, 1>(0, 3);

    prev_trans = cur_trans;

    pcl::PointCloud<PointT>::Ptr  transformed_cloud_2(new pcl::PointCloud<PointT>);

    pcl::transformPointCloud (*cur_point, *transformed_cloud_2, cur_trans);

    ros::Time msg_time(timeScanCur);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*transformed_cloud_2, output);
    output.header.frame_id = "world";
    output.header.stamp =  msg_time;

    transformed_points_pub.publish(output);

    publish_odometry(msg_time, cur_trans);

    return cur_trans;

}


//读入点云,进行点云数据处理
void velodyne_points_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    ros::Time begin = ros::Time::now();

    timeScanCur = msg->header.stamp.toSec();

	// ROS_INFO("velodyne points is receiving ");
    pcl::fromROSMsg(*msg, *cur_cloud_Ptr);

    pcl::PointCloud<PointT>::ConstPtr fliter_cloud_Ptr = filter_point(cur_cloud_Ptr);

    if(imu_enable)
        matching_scan_to_map_fix(score,fliter_cloud_Ptr,point_cloud_map_Ptr);
    else
        matching_scan_to_map_fix_without_imu(score,fliter_cloud_Ptr,point_cloud_map_Ptr);

    ros::Time end = ros::Time::now();

    ros::Duration cost_time = end - begin;

    std::cout<<"cost_time: "<<cost_time.toSec()<<"   score: "<<score<<std::endl;

    not_first_frame = 1;

}
//对地图进行分割 输入：点云地图 输出：分块后的点云地图
void map_segmentation(const pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::PointCloud<PointT>::Ptr laserCloudMapArray[])
{
    int map_points_num = cloud->points.size();
    for (int i = 0; i < map_points_num; i++) {
        
        PointT map_point = cloud->points[i];
        //按50的比例尺缩小，四舍五入，偏移laserCloudCen*的量，计算索引
        int cubeI = int((map_point.x + map_segment_size/2.0) / map_segment_size) + laserCloudCenX;
        int cubeJ = int((map_point.y + map_segment_size/2.0) / map_segment_size) + laserCloudCenY;
        

        if (map_point.x + map_segment_size/2.0 < 0) cubeI--;
        if (map_point.y + map_segment_size/2.0 < 0) cubeJ--;

        if (cubeI >= 0 && cubeI < laserCloudX && 
            cubeJ >= 0 && cubeJ < laserCloudY ) {//只挑选-laserCloudCenWidth * 50.0 < point.x < laserCloudCenWidth * 50.0范围内的点，y和z同理
            //按照尺度放进不同的组，每个组的点数量各异
        int cubeInd = cubeI + laserCloudX * cubeJ; //ind = I + laserCloudX * J
        laserCloudMapArray[cubeInd]->push_back(map_point);
        }
    }
}

//输出读取的点云地图（并对其进行滤波） 依赖的全局参数：downsample_resolution
void map_input(pcl::PointCloud<PointT>::ConstPtr& filtered_map_ptr)
{
    pcl::PointCloud<PointT>::Ptr map_Ptr(new pcl::PointCloud<PointT>);
    //读地图pcd文件，储存到map
    //std::string file_name = "/home/robot/catkin_ws/src/ndt_localization/ndt_localization/lio_sam_filtered.pcd";
    pcl::io::loadPCDFile (map_source, *map_Ptr);
    ROS_INFO("point_cloud_map points is receiving ");

    //显示地图的点云数量
    std::cerr << "map_PointCloud : " << map_Ptr->width * map_Ptr->height
    << " data points (" << pcl::getFieldsList(*map_Ptr) << ")."<<std::endl;

    //对地图进行滤波

    filtered_map_ptr = height_filter(map_Ptr,4.0,-0.3);
    
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    ROS_INFO("height filter");

    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;
    filtered_map_ptr = downsample(filtered_map_ptr);
    ROS_INFO("downsample");

    pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
    rad->setRadiusSearch(radius);
    rad->setMinNeighborsInRadius(min_neighbors);
    outlier_removal_filter = rad;
    filtered_map_ptr = outlier_removal(filtered_map_ptr);
    ROS_INFO("outlier_removal");

    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ> (file_name+"_filter", *filtered_map_ptr, false);

    //显示滤波后地图的点云数量
    std::cerr << "downsample_map_PointCloud : " << filtered_map_ptr->width * filtered_map_ptr->height
    << " data points (" << pcl::getFieldsList(*filtered_map_ptr) << ")."<<std::endl;

}

//z轴向下，x轴向左的右手坐标系 >> x轴向前，y轴向右，z轴向上的右手坐标系  >> z轴向前,x轴向左的右手坐标系
void imu_data_cb(const sensor_msgs::Imu::ConstPtr& imuIn)
{
    if(imu_enable){
        imu_stamp = imuIn->header.stamp.toSec();
        roll = imuIn->orientation.x;
        pitch = imuIn->orientation.y;
        yaw = imuIn->orientation.z;
    }

}

void initialpose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial_pose_in)
{
    initial_x = initial_pose_in->pose.pose.position.x;
    initial_y = initial_pose_in->pose.pose.position.y;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(initial_pose_in->pose.pose.orientation, quat);
 
     
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    initial_yaw = yaw * 180 /M_PI;

    get_rviz_pose = true;
    ROS_INFO("get_initial_pose");

}

//主函数
int main (int argc, char** argv)
{
    ros::init (argc, argv, "pc_fliter");

    ros::NodeHandle nh; 
    ros::NodeHandle nh_p("~");

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time odom_time = ros::Time::now();
    odom_trans.header.stamp = odom_time;

    not_first_frame = 0;
    score = 0;

    nh_p.param<double>("score_threshold",score_threshold,2.00);
    nh_p.param<bool>("imu_enable",imu_enable,true);

    nh_p.param<double>("initial_x",initial_x,0.0);
    nh_p.param<double>("initial_y",initial_y,0.0);
    nh_p.param<double>("initial_yaw",initial_yaw,0.0);
    map_source = ros::package::getPath("ndt_localization") + "/lidar_imu_odom_0611_0509.bag_map.pcd";
    //nh_p.param<std::string>("map_source",map_source," ");

    map_input(point_cloud_map_Ptr);//读入地图并滤波

    //地图分块

    for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudMapArray[i].reset(new pcl::PointCloud<PointT>());
	}
    map_segmentation(point_cloud_map_Ptr, laserCloudMapArray);

    ROS_INFO("map segmentation finish");

    ros::Subscriber sub_initial_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> 
                                    ("/initialpose", 50, initialpose_cb);

    while (get_rviz_pose != true)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}

    ros::Subscriber sub1 = nh.subscribe ("/velodyne_points", 1, velodyne_points_cb);
    ros::Subscriber sub_Imu = nh.subscribe<sensor_msgs::Imu> ("/imu", 50, imu_data_cb);


    //initialpose


    sub_map_pub = nh.advertise<sensor_msgs::PointCloud2>("sub_map_points", 1);
    transformed_points_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_points", 1);
    transform_before_points_pub = nh.advertise<sensor_msgs::PointCloud2>("transform_before_points", 1);
    pubPoseForKITTI = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    filtered_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_pointcloud",1);


    ros::Rate r(100.0);
    while(nh.ok()){

        if(odom_trans.header.stamp.toSec() != odom_time.toSec())
        {
            odom_broadcaster.sendTransform(odom_trans);
            odom_time = odom_trans.header.stamp;
        }

        ros::spinOnce();
        r.sleep();
    }
    

    return 0;
}

