#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/ndt.h>
#include <nav_msgs/Odometry.h>
#include <pclomp/ndt_omp.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


typedef pcl::PointXYZ PointT;

pcl::Filter<PointT>::Ptr downsample_filter;

pcl::PointCloud<PointT>::Ptr  cur_cloud_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr  point_cloud_map_Ptr(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr  point_cloud_add_map_Ptr(new pcl::PointCloud<PointT>);
ros::Publisher pcl_pub1;

double downsample_resolution = 0.1;

int laserCloudCenX = 12;
int laserCloudCenY = 12;

const int laserCloudX = 25;
const int laserCloudY = 25;


//点云方块集合最大数量
const int laserCloudNum = laserCloudX * laserCloudY;//4851

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

    std::cout<<"run here "<<std::endl;

    for (int i = 0; i < laserCloudValidNum; i++) 
    {
        *sub_map += *laserCloudMapArray[laserCloudValidInd[i]];
    }

}

void velodyne_points_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ros::Time msg_time;
	ROS_INFO("velodyne points is receiving ");
    pcl::fromROSMsg(*msg, *cur_cloud_Ptr);

    PointT local_minPt, local_maxPt;

    pcl::getMinMax3D (*cur_cloud_Ptr, local_minPt, local_maxPt);

    std::cout<<"minPt.x"<<local_minPt.x<<"maxPt.x"<<local_maxPt.x<<std::endl;//一帧点云的范围：-90~90
    std::cout<<"minPt.y"<<local_minPt.y<<"maxPt.y"<<local_maxPt.y<<std::endl;//-68~88
    std::cout<<"minPt.z"<<local_minPt.z<<"maxPt.z"<<local_maxPt.z<<std::endl;//-1~27

}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "map_segmentation");

    ros::NodeHandle nh;

    pcl_pub1 = nh.advertise<sensor_msgs::PointCloud2>("map_segmentation_points", 1);

    ros::Subscriber sub1 = nh.subscribe ("/velodyne_points", 1, velodyne_points_cb);

    //读地图pcd文件，储存到map
    pcl::io::loadPCDFile ("/home/guozixuan/catkin_ws/src/ndt_localization/ndt_localization/lidar_imu_odom_0611_0509.bag_map.pcd_filter", *point_cloud_map_Ptr);
    ROS_INFO("point_cloud_map points is receiving ");

    // Eigen::AngleAxisd rotzp1(M_PI/2, Eigen::Vector3d::UnitZ());

    // Eigen::AngleAxisd rotxp2(M_PI/2, Eigen::Vector3d::UnitX());

    // Eigen::Matrix3d rotation_matrix_w2c;

    // rotation_matrix_w2c = rotxp2.matrix() * rotzp1.matrix();

    // Eigen::Matrix3d rotation_matrix_c2w;

    // rotation_matrix_c2w = rotation_matrix_w2c.inverse();

    // Eigen::AngleAxisd rotxp3(M_PI, Eigen::Vector3d::UnitX());

    // rotation_matrix_c2w = rotxp3.matrix()*rotation_matrix_c2w;

    // Eigen::Matrix4d transform_matrix_c2w = Eigen::Matrix4d::Identity();


    // transform_matrix_c2w.block(0,0,3,3) = rotation_matrix_c2w;

    // pcl::transformPointCloud (*point_cloud_map_Ptr, *point_cloud_map_Ptr, transform_matrix_c2w);//lego-loam出来的点云地图需要进行一次坐标变换

    std::cerr << "map_PointCloud : " << point_cloud_map_Ptr->width * point_cloud_map_Ptr->height
        << " data points (" << pcl::getFieldsList(*point_cloud_map_Ptr) << ")."<<std::endl;
    
    pcl::PointCloud<PointT>::ConstPtr filtered_Ptr;
    
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());

    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;
    filtered_Ptr = downsample(point_cloud_map_Ptr);
    ROS_INFO("downsample");

    std::cerr << "map_PointCloud : " << filtered_Ptr->width * filtered_Ptr->height
    << " data points (" << pcl::getFieldsList(*filtered_Ptr) << ")."<<std::endl;

    PointT minPt, maxPt;

    pcl::getMinMax3D (*filtered_Ptr, minPt, maxPt);

    // std::cout<<"minPt.x"<<minPt.x<<"maxPt.x"<<maxPt.x<<std::endl;//国防科技园map 范围：-144.752~80
    // std::cout<<"minPt.y"<<minPt.y<<"maxPt.y"<<maxPt.y<<std::endl;//-154~44
    // std::cout<<"minPt.z"<<minPt.z<<"maxPt.z"<<maxPt.z<<std::endl;//-3~6.84

    //建一个地图分割
    pcl::PointCloud<PointT>::Ptr laserCloudMapArray[laserCloudNum];

    for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudMapArray[i].reset(new pcl::PointCloud<PointT>());
	}
    map_segmentation(filtered_Ptr, laserCloudMapArray);

    for(int i = 0;i < laserCloudNum ;i++)
    {
        *point_cloud_add_map_Ptr += *laserCloudMapArray[i];
    }



    pcl::getMinMax3D (*point_cloud_add_map_Ptr, minPt, maxPt);

    std::cout<<"minPt.x"<<minPt.x<<"maxPt.x"<<maxPt.x<<std::endl;//国防科技园map 范围：-144.752~80
    std::cout<<"minPt.y"<<minPt.y<<"maxPt.y"<<maxPt.y<<std::endl;//-154~44
    std::cout<<"minPt.z"<<minPt.z<<"maxPt.z"<<maxPt.z<<std::endl;//-3~6.84


    //测试submap

    Eigen::Matrix4f test_trans;

    pcl::PointCloud<PointT>::Ptr  sub_map(new pcl::PointCloud<PointT>);

    test_trans.setIdentity();

    test_trans(0,3) = 113;
    test_trans(1,3) = 88;

    sub_map_generation(test_trans,laserCloudMapArray,sub_map);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*sub_map, output);
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    

    pcl_pub1.publish(output);

    pcl::getMinMax3D (*sub_map, minPt, maxPt);

    std::cout<<"minPt.x"<<minPt.x<<"maxPt.x"<<maxPt.x<<std::endl;//国防科技园map 范围：-144.752~80
    std::cout<<"minPt.y"<<minPt.y<<"maxPt.y"<<maxPt.y<<std::endl;//-154~44
    std::cout<<"minPt.z"<<minPt.z<<"maxPt.z"<<maxPt.z<<std::endl;//-3~6.84



    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        output.header.stamp = ros::Time::now();
        pcl_pub1.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

