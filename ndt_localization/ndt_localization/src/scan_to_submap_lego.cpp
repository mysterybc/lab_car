#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/ndt.h>
#include <nav_msgs/Odometry.h>
#include <pclomp/ndt_omp.h>
#include <sensor_msgs/Imu.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include"queue"

const int hz = 3; //这里其实是几帧激光lidar 运行几次定时器
typedef pcl::PointXYZ PointT;
double score_threshold;
double score;
double imu_stamp;
int lego_odom_cout;

using namespace message_filters;

bool newcallback;

pcl::Filter<PointT>::Ptr downsample_filter;
pcl::Filter<PointT>::Ptr outlier_removal_filter;

//局部地图的参数
int laserCloudCenX = 10;
int laserCloudCenY = 10;

const int laserCloudX = 21;
const int laserCloudY = 21;

const int laserCloudNum = laserCloudX * laserCloudY;

pcl::PointCloud<PointT>::Ptr laserCloudMapArray[laserCloudNum];
nav_msgs::Odometry lidar_odom;
Eigen::Matrix4f lego_odom_matrix;

std::queue<Eigen::Matrix4f> lego_odom_matrix_queue;
std::queue<nav_msgs::Odometry> lego_odom_queue;

double lidartime;
double legoodomtime;
double curtime;

ros::Publisher sub_map_pub,transformed_points_pub,transform_before_points_pub,pubPoseForKITTI;
//点云指针
pcl::PointCloud<PointT>::ConstPtr  prev_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr  cur_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::PointXYZ>::ConstPtr  point_cloud_map_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr sub_map(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr sliding_window[10];

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
double distance_near_thresh = 0.5;
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

double downsample_resolution = 0.1;

//点云滤波处理
pcl::PointCloud<PointT>::ConstPtr filter_point(const pcl::PointCloud<PointT>::Ptr& source_point)
{

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(source_point);
    ROS_INFO("distance");


    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;
    filtered = downsample(filtered);
    ROS_INFO("downsample");
    

    // pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
    // rad->setRadiusSearch(radius);
    // rad->setMinNeighborsInRadius(min_neighbors);
    // outlier_removal_filter = rad;
    // filtered = outlier_removal(filtered);
    // ROS_INFO("outlier_removal");

    return filtered;
}


Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
Eigen::Matrix4f cur_trans;                       // current estimated transform from keyframe


int not_first_frame;


//submap生成： 输入：1、预估的变换关系 2、完整的点云地图（或者分割好的点云cube） 输出：当前预估关系的map下的submap
void sub_map_generation(const Eigen::Matrix4f transform,const pcl::PointCloud<PointT>::Ptr laserCloudMapArray[] , pcl::PointCloud<PointT>::Ptr sub_map)
{
    int cur_I,cur_J;
    int trans_to_map_X,trans_to_map_Y;

    //lidar视域范围内(FOV)的点云集索引
    int laserCloudValidInd[125];
    //lidar周围的点云集索引
    int laserCloudSurroundInd[125];

    trans_to_map_X = transform(0,3);
    trans_to_map_Y = transform(1,3);

    //确定x,y坐标对应的cube索引
    cur_I = int((trans_to_map_X + 10.0) / 20.0) + laserCloudCenX;
    cur_J = int((trans_to_map_Y + 10.0) / 20.0) + laserCloudCenY;

    if (trans_to_map_X + 10.0 < 0) cur_I--;
    if (trans_to_map_Y + 10.0 < 0) cur_J--;

    int cubeInd = cur_I + laserCloudX * cur_J; //ind = I + laserCloudX * J


    int laserCloudValidNum = 0;

    for (int i = cur_I - 2; i <= cur_I + 2; i++) 
    {
        for (int j = cur_J - 2; j <= cur_J + 2; j++) 
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
    std::cerr << "map_PointCloud : " << sub_map->width * sub_map->height
    << " data points (" << pcl::getFieldsList(*sub_map) << ")."<<std::endl;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*sub_map, output);
    output.header.frame_id = "world";

    sub_map_pub.publish(output);
}

//匹配函数 ： 输入：1、当前点云 2、目标点云 3、变换初值 该函数是为了服务于地图与帧匹配
Eigen::Matrix4f matching(double& score,const pcl::PointCloud<PointT>::ConstPtr& source_point,const pcl::PointCloud<PointT>::ConstPtr& target_point,const Eigen::Matrix4f trans)
{
    Eigen::Matrix4f esti_trans;

    esti_trans.setIdentity();

    boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt_omp(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    
    ndt_omp->setNumThreads(omp_get_max_threads());
    ndt_omp->setTransformationEpsilon(0.01);
    ndt_omp->setMaximumIterations(50);
    ndt_omp->setResolution(1);
    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

    boost::shared_ptr<pcl::Registration<PointT, PointT>> registration;

    registration = ndt_omp;

    registration->setInputTarget(target_point);
    registration->setInputSource(source_point);

    ROS_INFO("run here");

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->align(*aligned,trans);


    if(!registration->hasConverged()) {
      ROS_INFO("scan matching has not converged!!");
      return trans;
    }

    esti_trans = registration->getFinalTransformation();

    std::cout << "Normal Distributions Transform has converged:/n" << esti_trans
            << " score: " << registration->getFitnessScore() << std::endl;

    score = registration->getFitnessScore();

    return esti_trans;
}

void RosQ2RPY(const geometry_msgs::Quaternion& quaternion ,double& roll,double& pitch,double& yaw)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}
void RPY2RosQ(geometry_msgs::Quaternion& quaternion ,const double& roll,const double& pitch,const double& yaw)
{
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);//返回四元数
}


void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) 
{

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
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


    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = tmp_roll*180/M_PI;//;
    odom.twist.twist.linear.y = tmp_pitch*180/M_PI;
    odom.twist.twist.linear.z = tmp_yaw*180/M_PI;
    //odom.twist.twist.angular.z = tmp_yaw;

    pubPoseForKITTI.publish(odom);
}

Eigen::Matrix4f error_trans;
double timeScanCur;

Eigen::Matrix4f matching_scan_to_map(double& score,const pcl::PointCloud<PointT>::ConstPtr& cur_point,
                                            const pcl::PointCloud<PointT>::ConstPtr& map_point,const Eigen::Matrix4f& dif_odom)
{

    ros::Time msg_time(timeScanCur);

    cur_trans = dif_odom * prev_trans ;

    sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

    //该函数可以读一帧，匹配一帧
    cur_trans = matching(score,cur_point,sub_map,cur_trans);


    Eigen::Matrix4f delta = cur_trans * prev_trans.inverse();

    double dx = delta.block<3, 1>(0, 3).norm();
    double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());

    if(dx > 5 || da > M_PI_2) {
        std::cout<<"too large transform!!  " << dx << "[m] " << da << "[rad]"<<std::endl;
        ROS_ERROR_STREAM("ignore this frame");

        cur_trans = dif_odom * prev_trans;
    }

    prev_trans = cur_trans;

    pcl::PointCloud<PointT>::Ptr  transformed_cloud(new pcl::PointCloud<PointT>);

    pcl::transformPointCloud (*cur_point, *transformed_cloud, cur_trans);


    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*transformed_cloud, output);
    output.header.frame_id = "world";

    transformed_points_pub.publish(output);

    publish_odometry(msg_time, cur_trans);

    return cur_trans;

}


//对地图进行分割 输入：点云地图 输出：分块后的点云地图
void map_segmentation(const pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::PointCloud<PointT>::Ptr laserCloudMapArray[])
{
    int map_points_num = cloud->points.size();
    for (int i = 0; i < map_points_num; i++) {
        
        PointT map_point = cloud->points[i];
        //按50的比例尺缩小，四舍五入，偏移laserCloudCen*的量，计算索引
        int cubeI = int((map_point.x + 10.0) / 20.0) + laserCloudCenX;
        int cubeJ = int((map_point.y + 10.0) / 20.0) + laserCloudCenY;
        

        if (map_point.x + 10.0 < 0) cubeI--;
        if (map_point.y + 10.0 < 0) cubeJ--;

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
    pcl::io::loadPCDFile ("/home/robot/catkin_ws/src/ndt_localization/add_pcl_link/2019-07-13-20-21-30.bag_map.pcd", *map_Ptr);
    ROS_INFO("point_cloud_map points is receiving ");

    //显示地图的点云数量
    std::cerr << "map_PointCloud : " << map_Ptr->width * map_Ptr->height
    << " data points (" << pcl::getFieldsList(*map_Ptr) << ")."<<std::endl;

    //对地图进行滤波
    
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());

    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;
    filtered_map_ptr = downsample(map_Ptr);
    ROS_INFO("downsample");

    pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
    rad->setRadiusSearch(radius);
    rad->setMinNeighborsInRadius(min_neighbors);
    outlier_removal_filter = rad;
    filtered_map_ptr = outlier_removal(filtered_map_ptr);
    ROS_INFO("outlier_removal");

    //显示滤波后地图的点云数量
    std::cerr << "downsample_map_PointCloud : " << filtered_map_ptr->width * filtered_map_ptr->height
    << " data points (" << pcl::getFieldsList(*filtered_map_ptr) << ")."<<std::endl;

}

void navodom2Matrix4f(const nav_msgs::Odometry& odomin ,Eigen::Matrix4f& matrix)//将odom消息转换为矩阵
{

    Eigen::Quaternionf q(odomin.pose.pose.orientation.w,odomin.pose.pose.orientation.x,odomin.pose.pose.orientation.y,odomin.pose.pose.orientation.z);//w,x,y,z
    Eigen::Matrix3f rotation;
    rotation = q.matrix();
    Eigen::Vector3f T(odomin.pose.pose.position.x,odomin.pose.pose.position.y,odomin.pose.pose.position.z);
    matrix.block(0,0,3,3) = rotation.matrix();
    
    matrix(0,3) = T(0);
    matrix(1,3) = T(1);
    matrix(2,3) = T(2);

}

Eigen::Matrix4f prev_odom;

void timerCallback(const ros::TimerEvent& event)
{
    if( newcallback == 0) //如果没有新的帧进入，则不执行定时器
    {
        return;
    }

    if(lego_odom_cout >= hz)
    {

        Eigen::Matrix4f end_odom = lego_odom_matrix_queue.back();

        Eigen::Matrix4f odom_matrix[hz];

        for(int i = 0;i < hz;i++)
        {
            odom_matrix[i] = lego_odom_matrix_queue.front();
            lego_odom_matrix_queue.pop();
        }

        Eigen::Matrix4f delta_odom;

        pcl::PointCloud<PointT>::Ptr sliding_window_point(new pcl::PointCloud<PointT>);


        delta_odom.setIdentity();
        for(int i = 1;i < hz;i++)
        {
            delta_odom = odom_matrix[hz-1-i]*odom_matrix[hz-1].inverse();
            pcl::PointCloud<PointT>::Ptr  transformed_cloud(new pcl::PointCloud<PointT>);
            pcl::transformPointCloud (*sliding_window[hz-i-1], *transformed_cloud, delta_odom.inverse());

            pcl::PointCloud<PointT>::ConstPtr fliter_cloud_Ptr = filter_point(transformed_cloud);

            *sliding_window_point += *fliter_cloud_Ptr;
        }

        

            std::cerr << "sliding_window_point : " << sliding_window_point->width * sliding_window_point->height
    << " data points (" << pcl::getFieldsList(*sliding_window_point) << ")."<<std::endl;
        //构建局部地图


        Eigen::Matrix4f dif_odom = end_odom*prev_odom.inverse();
        
        prev_odom = end_odom;

        double score;

        matching_scan_to_map(score,sliding_window_point,point_cloud_map_Ptr,dif_odom);//分数，当前帧地图，点云地图，位姿变换
        
        lego_odom_cout = 0;
    }
    newcallback = 0;
}

//读入lego_odom和原始激光lidar的数据，同时进行对齐
void maincallback(const sensor_msgs::PointCloud2::ConstPtr& msg,const nav_msgs::Odometry::ConstPtr& odomin)  //回调中包含多个消息
{
    ROS_INFO("receive bind messages!");

    lidar_odom.header = odomin->header;
    lidar_odom.child_frame_id = odomin->child_frame_id;
    lidar_odom.pose = odomin->pose;
    lidar_odom.twist = odomin->twist;

    lidar_odom.pose.pose.orientation.x = odomin->pose.pose.orientation.z;
    lidar_odom.pose.pose.orientation.y = -odomin->pose.pose.orientation.x;
    lidar_odom.pose.pose.orientation.z = -odomin->pose.pose.orientation.y;
    lidar_odom.pose.pose.orientation.w = odomin->pose.pose.orientation.w;

    lidar_odom.pose.pose.position.x = odomin->pose.pose.position.z;
    lidar_odom.pose.pose.position.y = odomin->pose.pose.position.x;
    lidar_odom.pose.pose.position.z = odomin->pose.pose.position.y;


    double roll,pitch,yaw;

    RosQ2RPY(lidar_odom.pose.pose.orientation,roll,pitch,yaw);

    pitch = -pitch;
    yaw = -yaw;
    RPY2RosQ(lidar_odom.pose.pose.orientation,roll,pitch,yaw);

    lego_odom_matrix = Eigen::Matrix4f::Identity();

    navodom2Matrix4f(lidar_odom,lego_odom_matrix);

    lego_odom_matrix_queue.push(lego_odom_matrix); 
    lego_odom_queue.push(lidar_odom);

    sensor_msgs::PointCloud2 point;
    point.header = msg->header;

    lidartime = point.header.stamp.toSec();

    

    pcl::fromROSMsg(*msg, *cur_cloud_Ptr);

    //这里可能有bug，如果超过10,就不是最新的frame了
    if(lego_odom_cout>=0 &&lego_odom_cout < hz )
    {
        sliding_window[lego_odom_cout] = cur_cloud_Ptr;
    }
    

    pcl::PointCloud<PointT>::Ptr  transformed_cloud(new pcl::PointCloud<PointT>);

    pcl::transformPointCloud (*cur_cloud_Ptr, *transformed_cloud, lego_odom_matrix);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*transformed_cloud, output);
    output.header.frame_id = "world";

    transform_before_points_pub.publish(output);

    lego_odom_cout++;

    newcallback = 1;

}
//主函数
int main (int argc, char** argv)
{
    ros::init (argc, argv, "pc_fliter");

    ros::NodeHandle nh; 
    ros::NodeHandle nh_p("~");
    not_first_frame = 0;
    score = 0;
    lego_odom_cout = 0;
    newcallback = 0;


    nh_p.param<double>("score_threshold",score_threshold,2.00);

    std::cout<<"score_threshold:"<<score_threshold<<std::endl;

    pubPoseForKITTI = nh.advertise<nav_msgs::Odometry>("/odometry_for_kitti", 100);

    sub_map_pub = nh.advertise<sensor_msgs::PointCloud2>("sub_map_points", 1);
    transformed_points_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_points", 1);
    transform_before_points_pub = nh.advertise<sensor_msgs::PointCloud2>("transform_before_points", 1);


    //消息滤波器，以期达到信息同步
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/velodyne_points", 1);   // topic1 输入
    message_filters::Subscriber<nav_msgs::Odometry> lego_odom_sub(nh, "/laser_odom_to_init", 1);   // topic2 输入
    TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(cloud_sub, lego_odom_sub, 10);       // 同步
    sync.registerCallback(boost::bind(&maincallback, _1, _2));                   // 回调


    map_input(point_cloud_map_Ptr);//读入地图并滤波

    //地图分块

    for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudMapArray[i].reset(new pcl::PointCloud<PointT>());
	}
    for (int i = 0; i < hz-1; i++)
	{
		sliding_window[i].reset(new pcl::PointCloud<PointT>());
	}
    map_segmentation(point_cloud_map_Ptr, laserCloudMapArray);

    cur_trans = Eigen::Matrix4f::Identity();
    prev_trans = Eigen::Matrix4f::Identity();   
    prev_odom = Eigen::Matrix4f::Identity();

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*point_cloud_map_Ptr, output);
    output.header.frame_id = "world";


    ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCallback);

    ros::spin();

    return 0;
}

