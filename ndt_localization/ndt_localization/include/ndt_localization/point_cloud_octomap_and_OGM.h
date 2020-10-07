//point_cloud_octomap_and_OGM.h
//该h文件主要是为了定义一个类，为point cloud 转换为 occupy grid map 和octomap提供转换接口
//
//
#ifndef POINT_CLOUD_OCTOMAP_AND_OGM_H
#define POINT_CLOUD_OCTOMAP_AND_OGM_H

#include <ros/ros.h>//ros

#include <pcl/point_cloud.h>//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>

#include <pcl/common/transforms.h>

#include <tf/transform_datatypes.h>//RPY2Q
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>//sensor_data
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <octomap/octomap.h>//octomap
#include <octomap/OcTreeStamped.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>

#include <thread>
#include <deque>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <filters/filter_chain.h>

typedef pcl::PointXYZI PointT;

class MapGeneration
{
    public:

    MapGeneration(ros::NodeHandle &nh_);
    ~MapGeneration();

    void allocateMemory();//初始化参数，分配内存
    
    void resetParameters();//重置参数

    void velodyne_points_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void odometry_points_cb(const nav_msgs::Odometry::ConstPtr& odomin);

    void pointcloud2octree(octomap::OcTree& tree,const pcl::PointCloud<PointT>::Ptr& temp ,const Eigen::Matrix4f& pose);

    void octree2OccupyGridMap(const octomap::OcTree& tree);

    void update2DMap(const octomap::OcTree::iterator& it, bool occupied);

//    unsigned mapIdx(const octomap::OcTreeKey& key,const octomap::OcTreeKey& MinKey){
//        return mapIdx((key[0] - MinKey[0]) ,
//                      (key[1] - MinKey[1]) );
//
//    };
    inline unsigned  mapIdx(int i, int j) const {
        return grid_map_.info.width * j + i;
    };

    inline unsigned  mapIdx(const octomap::OcTreeKey& key) const {
        return mapIdx((key[0] - minKey_[0]),
                      (key[1] - minKey_[1]));
    };


    void RosQ2RPY(const geometry_msgs::Quaternion& quaternion ,double& roll,double& pitch,double& yaw);

    void RosRPY2Q(const double& roll,const double& pitch,const double& yaw,geometry_msgs::Quaternion& quaternion);

    Eigen::Matrix4f odom2TransformMatrix(const nav_msgs::Odometry& odomin);


    private:

    ros::NodeHandle nh_;

    double res_;

    bool new_odom_; //init to false

    std::string octomap3D_pub_topic_;

    double max_range_;

    double probHit_;

    double probMiss_;

    double thresMin_;

    double thresMax_;

    double occupancyMinZ_;

    double occupancyMaxZ_;

    double treeMaxDepth_;

    octomap::OcTreeKey minKey_;

    octomap::OcTreeKey maxKey_;

    octomap::KeyRay keyRay_;

    ros::Subscriber velodyne_point_sub_;

    ros::Subscriber odom_sub_;

    nav_msgs::OccupancyGrid grid_map_;

    nav_msgs::Odometry cur_odom_;
    
    std::deque<sensor_msgs::PointCloud2> point_duque_;

    std::mutex odom_mutex_;

    octomap::OcTree* tree_;//octomap类型指针，需要分配内存

    ros::Publisher octomap3D_pub_;

    ros::Publisher map2D_pub_;

};

struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)







#endif