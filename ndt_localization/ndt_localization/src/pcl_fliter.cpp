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



ros::Publisher pcl_pub1,pcl_pub2,pubPoseForKITTI;
//点云指针
pcl::PointCloud<PointT>::ConstPtr  prev_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr  cur_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::PointXYZ>::Ptr  point_cloud_map_Ptr(new pcl::PointCloud<pcl::PointXYZ>);

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
double distance_far_thresh = 30;

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

double height = 4.0;
pcl::PointCloud<PointT>::ConstPtr height_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
      [&](const PointT& p) {
        double d = p.z;
        return d > -height && d < height;
      }
    );

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
}


//点云滤波处理
pcl::PointCloud<PointT>::ConstPtr filter_point(const pcl::PointCloud<PointT>::Ptr& source_point)
{
    //ros::Time begin = ros::Time::now();

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

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pc_fliter");

    ros::NodeHandle nh; 

    //读地图pcd文件，储存到map
    pcl::io::loadPCDFile ("/home/robot/catkin_ws/src/add_pcl_link/lio_sam.pcd", *point_cloud_map_Ptr);
    ROS_INFO("point_cloud_map points is receiving ");

    std::cerr << "map_PointCloud : " << point_cloud_map_Ptr->width * point_cloud_map_Ptr->height
    << " data points (" << pcl::getFieldsList(*point_cloud_map_Ptr) << ")."<<std::endl;




    ros::spin();

    return 0;
}

