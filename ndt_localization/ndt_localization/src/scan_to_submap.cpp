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




typedef pcl::PointXYZ PointT;

pcl::Filter<PointT>::Ptr downsample_filter;
pcl::Filter<PointT>::Ptr outlier_removal_filter;

//局部地图的参数
int laserCloudCenX = 10;
int laserCloudCenY = 10;

const int laserCloudX = 21;
const int laserCloudY = 21;

const int laserCloudNum = laserCloudX * laserCloudY;

pcl::PointCloud<PointT>::Ptr laserCloudMapArray[laserCloudNum];



ros::Publisher sub_map_pub,flitered_pcl_points_pub,pubPoseForKITTI;
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
double distance_near_thresh = 0.5;
double distance_far_thresh = 50;

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
    

    pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
    rad->setRadiusSearch(radius);
    rad->setMinNeighborsInRadius(min_neighbors);
    outlier_removal_filter = rad;
    filtered = outlier_removal(filtered);
    ROS_INFO("outlier_removal");

    return filtered;
}

Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
Eigen::Matrix4f cur_trans;                       // current estimated transform from keyframe
Eigen::Matrix4f trans_to_init;                       // current estimated transform from keyframe

Eigen::Matrix4f cur_to_map_trans;                       // current estimated transform from mapframe
Eigen::Matrix4f prev_to_map_trans;                       // current estimated transform from mapframe

Eigen::Quaternionf cur_imu_quaternion;
Eigen::Quaternionf prev_imu_quaternion;
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

    std::cout<<"run here " <<"cubeInd: "<<cubeInd<<"cur_I"<<cur_I<<std::endl;

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
                std::cout<<"j:"<<j<<"i"<<i<<std::endl;
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
Eigen::Matrix4f matching(const pcl::PointCloud<PointT>::ConstPtr& source_point,const pcl::PointCloud<PointT>::ConstPtr& target_point,const Eigen::Matrix4f trans)
{
    Eigen::Matrix4f esti_trans;

    esti_trans.setIdentity();

    boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt_omp(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    
    ndt_omp->setNumThreads(omp_get_max_threads());
    ndt_omp->setTransformationEpsilon(0.01);
    ndt_omp->setMaximumIterations(35);
    ndt_omp->setResolution(0.5);
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
            << " score: " << registration->getFitnessScore () << std::endl;

    return esti_trans;
}

void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) 
{

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    Eigen::Matrix3f rotation_matrix = pose.block(0,0,3,3);
    Eigen::Quaternionf q = Eigen::Quaternionf(rotation_matrix);

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();


    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pubPoseForKITTI.publish(odom);
}

Eigen::Matrix4f error_trans;
double timeScanCur;
Eigen::Matrix4f matching_scan_to_map(const pcl::PointCloud<PointT>::ConstPtr& cur_point,const pcl::PointCloud<PointT>::ConstPtr& map_point)
{
    //估计当前相对于上一帧增加的变换矩阵
    if( not_first_frame != 1 )
    {
        prev_trans.setIdentity();   
        cur_trans.setIdentity();
        error_trans.setIdentity();
    } 

    ros::Time msg_time(timeScanCur);

    cur_trans = error_trans * prev_trans;

    sub_map_generation(cur_trans,laserCloudMapArray, sub_map);

    //该函数可以读一帧，匹配一帧
    cur_trans = matching(cur_point,sub_map,cur_trans);

    error_trans = cur_trans * prev_trans.inverse();

    prev_trans = cur_trans;

    prev_imu_quaternion = cur_imu_quaternion;

    pcl::PointCloud<PointT>::Ptr  transformed_cloud(new pcl::PointCloud<PointT>);

    pcl::transformPointCloud (*cur_point, *transformed_cloud, cur_trans);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*transformed_cloud, output);
    output.header.frame_id = "world";

    flitered_pcl_points_pub.publish(output);


    publish_odometry(msg_time, cur_trans);

    return cur_trans;

}

//读入点云,进行点云数据处理
void velodyne_points_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ros::Time begin = ros::Time::now();

    double timeScanCur = msg->header.stamp.toSec();

	ROS_INFO("velodyne points is receiving ");
    pcl::fromROSMsg(*msg, *cur_cloud_Ptr);

    pcl::PointCloud<PointT>::ConstPtr fliter_cloud_Ptr = filter_point(cur_cloud_Ptr);

    matching_scan_to_map(fliter_cloud_Ptr,point_cloud_map_Ptr);

    ros::Time end = ros::Time::now();

    ros::Duration cost_time = end - begin;

    std::cout<<"cost_time"<<cost_time<<std::endl;

    not_first_frame = 1;

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
    pcl::io::loadPCDFile ("/home/robot/catkin_ws/src/ndt_localization/add_pcl_link/banchao_big.pcd", *map_Ptr);
    ROS_INFO("point_cloud_map points is receiving ");

    Eigen::AngleAxisd rotzp1(M_PI/2, Eigen::Vector3d::UnitZ());

    Eigen::AngleAxisd rotxp2(M_PI/2, Eigen::Vector3d::UnitX());

    Eigen::Matrix3d rotation_matrix_w2c;

    rotation_matrix_w2c = rotxp2.matrix() * rotzp1.matrix();

    Eigen::Matrix3d rotation_matrix_c2w;

    rotation_matrix_c2w = rotation_matrix_w2c.inverse();

    Eigen::AngleAxisd rotxp3(M_PI, Eigen::Vector3d::UnitX());

    rotation_matrix_c2w = rotxp3.matrix()*rotation_matrix_c2w;

    Eigen::Matrix4d transform_matrix_c2w = Eigen::Matrix4d::Identity();


    transform_matrix_c2w.block(0,0,3,3) = rotation_matrix_c2w;

    pcl::transformPointCloud (*map_Ptr, *map_Ptr, transform_matrix_c2w);//lego-loam出来的点云地图需要进行一次坐标变换

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

//主函数
int main (int argc, char** argv)
{
    ros::init (argc, argv, "pc_fliter");

    ros::NodeHandle nh; 
    not_first_frame = 0;

    ros::Subscriber sub1 = nh.subscribe ("/velodyne_points", 1, velodyne_points_cb);

    sub_map_pub = nh.advertise<sensor_msgs::PointCloud2>("sub_map_points", 1);
    flitered_pcl_points_pub = nh.advertise<sensor_msgs::PointCloud2>("flitered_pcl_points", 1);
    pubPoseForKITTI = nh.advertise<nav_msgs::Odometry>("/odometry_for_kitti", 100);

    map_input(point_cloud_map_Ptr);//读入地图并滤波

    //地图分块

    for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudMapArray[i].reset(new pcl::PointCloud<PointT>());
	}
    map_segmentation(point_cloud_map_Ptr, laserCloudMapArray);

    prev_to_map_trans = Eigen::Matrix4f::Identity();
    cur_to_map_trans = Eigen::Matrix4f::Identity();
    trans_to_init = Eigen::Matrix4f::Identity();


    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*point_cloud_map_Ptr, output);
    output.header.frame_id = "world";


    ros::spin();

    return 0;
}

