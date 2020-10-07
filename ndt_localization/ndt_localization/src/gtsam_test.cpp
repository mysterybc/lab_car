/*
2020.8.9

    1、 自定义一个rostopic，将激光里程计配准得分信息加入到rostopic里

    2、 确定imu的预积分该怎么做，先做个实验试一下精度如何

    
*/





#include <ros/ros.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>



using namespace gtsam;
//定义一个点云类型来储存关键点
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

class GTSAM_BACKEND
{
    private:

    ros::Subscriber lidar_odom_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher key_frame_path_pub_;
    ros::Publisher after_opt_path_pub_;
    ros::Publisher after_opt_odom_pub_;   //优化完成后，与odom和imu融合的odom 发布频率要快

    ros::NodeHandle nh_;
    NonlinearFactorGraph gtSAMgraph_; // gtSAMgraph 非线性因子图
    Values initialEstimate_;          // 初始估计  插入因子图里面
    Values optimizedEstimate_;        //优化估计 
    ISAM2 *isam_;                     //使用isam2方法去优化 初始化一个指针
    Values isamCurrentEstimate_;      //isam当前估计

    noiseModel::Diagonal::shared_ptr priorNoise_; //初始噪声  vector6
    noiseModel::Diagonal::shared_ptr odomandimuNoise_; //里程计噪声  vector6 都非常小
    noiseModel::Diagonal::shared_ptr lidarodomNoise_; //回环噪声？
    noiseModel::Diagonal::shared_ptr constraintNoise_; //回环噪声？

    nav_msgs::Odometry lidar_odom_;   //当前 lidar确定的定位
    nav_msgs::Odometry odom_;

    ros::Time imu_timestamp_;
    double imu_roll_;
    double imu_pitch_;
    double imu_yaw_;

    // float last_transform_[6]; //存储上一次 xyz roll pitch yaw
    // float cur_transform_[6];  //存储当前 xyz roll pitch yaw
    // float aft_opt_transform_[6];  //存储优化后的 xyz roll pitch yaw

    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D_;             //加了一个点云来表示6D关键点

    bool newlidarodom_;
    bool newimudata_;
    bool newodomdata_;

    float lastlidardata_[6];  //x,y,z,roll,pitch,yaw
    float curlidardata_[6];   //x,y,z,roll,pitch,yaw

    public:

    GTSAM_BACKEND() 
    {
        //初始化订阅
        lidar_odom_sub_ = nh_.subscribe ("/ ", 1, &GTSAM_BACKEND::lidar_odom_cb,this);
        odom_sub_ = nh_.subscribe ("/ ", 50, &GTSAM_BACKEND::odom_cb,this);
        imu_sub_ = nh_.subscribe ("/ ", 50, &GTSAM_BACKEND::imu_cb,this);

        //初始化发送
        after_opt_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odometry_after_opt", 100);


        reset();

        //初始化非线性因子图和噪声模型   激光定位噪声模型应该得根据ndt配准得分去做
        gtsam::Vector Vector6_prior(6);
        gtsam::Vector Vector6_lidarodom(6);
        gtsam::Vector Vector6_odomandimu(6);

        Vector6_prior << 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4;       //初始位置先验噪声
        Vector6_lidarodom << 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4;   //lidar initial noise model   
        Vector6_odomandimu << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3;  //odom and imu noise model

        priorNoise_ = noiseModel::Diagonal::Variances(Vector6_lidarodom);
        lidarodomNoise_ = noiseModel::Diagonal::Variances(Vector6_lidarodom);
        odomandimuNoise_ = noiseModel::Diagonal::Variances(Vector6_odomandimu);

    }

    ~GTSAM_BACKEND() = default;

    //分配指针和变量初值
    void reset()
    {
        newlidarodom_ = false;
        newimudata_ = false;
        newodomdata_ = false;

        cloudKeyPoses6D_.reset(new pcl::PointCloud<PointType>());
    }

    void run()
    {
        if( std::abs(lidar_odom_.header.stamp.toSec()  - odom_.header.stamp.toSec()) < 0.01 &&
            std::abs(odom_.header.stamp.toSec()  - imu_timestamp_) < 0.01 &&
            std::abs(lidar_odom_.header.stamp.toSec()  - imu_timestamp_) < 0.01 && 
            newlidarodom_ && 
            newimudata_ && 
            newodomdata_) //如果来新帧进入该函数
        {
            newlidarodom_ = false; newimudata_ = false; newodomdata_ = false;


            //将激光雷达定位位置和四元数转变为位置和欧拉角    
            geometry_msgs::pose p;
            p = lidar_odom_.pose.pose;
            Eigen::Quaterniond q(p.orientation.w , p.orientation.x , p.orientation.y , p.orientation.z);//w,x,y,z
            Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);//Z-Y-X 即RPY

            curlidardata_[0] = p.position.x;
            curlidardata_[1] = p.position.y;
            curlidardata_[2] = p.position.z;
            curlidardata_[3] = eulerAngle(0);
            curlidardata_[4] = eulerAngle(1);
            curlidardata_[5] = eulerAngle(2);

            //Eigen::Affine3f lidar_odom = pcl::getTransformation(p.position.x,p.position.y,p.position.z,); // x,y,z roll ,pitch ,yaw
            //这里应该加一个判断，如果初始配准得分低于一定值，就不考虑进行图的初始化
            //if( FitnessScore > biggest total value && isam_ is empty){
            //  ROSERROR("因为配准分数过低，因子图的初始化未完成");
            //  return;
            //}

            if (cloudKeyPoses3D->points.empty()) //如果为空，设置图的初始节点
            {
                /*此处的prior noise_也可以用配准得分去做*/
                gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(curlidardata_[3], curlidardata_[4],curlidardata_[5]),
                                                            Point3(curlidardata_[0], curlidardata_[1], curlidardata_[2]), priorNoise_));

                initialEstimate_.insert(0, Pose3(Rot3::RzRyRx(curlidardata_[3], curlidardata_[4],curlidardata_[5]),
                                                            Point3(curlidardata_[0], curlidardata_[1], curlidardata_[2])));

                //将初始节点的位置进行保存
                for (int i = 0; i < 6; ++i)
                    lastlidardata_[i] = curlidardata_[i];
            }
            else       //如果不为空，设置图的下一个节点和累加节点以及中间的观测
            {

                //gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
                //                                    Point3(transformLast[5], transformLast[3], transformLast[4]));
                //gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                //                                    Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
                //gtSAMgraph_.add(BetweenFactor<Pose3>(cloudKeyPoses6D_->points.size()-1, cloudKeyPoses6D_->points.size(), poseFrom.between(poseTo), odomandimuNoise_));
                
                //此地方应该用imu和odom的融合去做

                //估计初值可以用匀速运功模型去做   如果配准得分很低，就可以将
                
                //initialEstimate_.insert(cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                //                                                                Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
            }

            /**
             * update iSAM
             */
            isam_->update(gtSAMgraph_, initialEstimate_);
            isam_->update();

            gtSAMgraph_.resize(0);
            initialEstimate_.clear();

            /**
             * save key poses
             */
            PointTypePose thisPose6D;
            Pose3 latestEstimate;

            isamCurrentEstimate = isam->calculateEstimate();
            latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);

            thisPose6D.x = latestEstimate.translation().x();
            thisPose6D.y = latestEstimate.translation().y();
            thisPose6D.z = latestEstimate.translation().z();
            thisPose6D.intensity = cloudKeyPoses6D_->points.size(); // this can be used as index
            thisPose6D.roll  = latestEstimate.rotation().roll();
            thisPose6D.pitch = latestEstimate.rotation().pitch();
            thisPose6D.yaw   = latestEstimate.rotation().yaw(); // in camera frame
            thisPose6D.time = lidar_odom_.header.stamp.toSec();
            cloudKeyPoses6D_->push_back(thisPose6D);


        }
        else ROSERROR("消息未发布或时间戳没有匹配上"); //如果没有进入就显示该错误信息

    }

    void lidar_odom_cb(const nav_msgs::Odometry::ConstPtr& lidarodomIn){ //接收到了以后转换为gtsam中的关键点
        //读入lidar的定位
        /*可以使用的信息：
        时间戳 ：odom.header.stamp
        参考系 ：odom.header.frame_id
        3D位置 ：odom.pose.pose.position
        四元数 ：odom.pose.pose.orientation

        绝对位置
        */

        lidar_odom_.header = lidarodomIn->header;
        lidar_odom_.child_frame_id = lidarodomIn->child_frame_id;
        lidar_odom_.pose = lidarodomIn->pose;
        lidar_odom_.pose = lidarodomIn->twist;


        newlidarodom_ = true;

    }
    
    void odom_cb(const nav_msgs::Odometry::ConstPtr& odomIn){
        //读入轮速计odom的定位
        /*可以使用的信息：
        时间戳 ：odom.header.stamp
        参考系 ：odom.header.frame_id
        3D位置 ：odom.pose.pose.position
        四元数 ：odom.pose.pose.orientation

        需要对odom进行差值处理，这样误差不会累计
        */
        odom_.header = odomIn->header;
        odom_.child_frame_id = odomIn->child_frame_id;
        odom_.pose = odomIn->pose;
        odom_.twist = odomIn->twist;

        newodomdata_ = true;

    }

    void imu_cb(const sensor_msgs::Imu::ConstPtr& imuIn){
        //读入imu的角度
        /*可以使用的信息：
        时间戳: imu.header.stamp
        参考系: imu.header.frame_id
        四元数 ：imu.orientation
        三轴角速度：imu.angular_velocity
        3轴线加速度：imu.linear_acceleration

        对于履带车上的imu:     
        roll = imuIn->orientation.x;
        pitch = imuIn->orientation.y;
        yaw = imuIn->orientation.z;

        只读入imu的角度信息，row，pitch，yaw 同样得进行差值处理  
        */

        imu_timestamp_ = imuIn->header.stamp;
        imu_roll_ = imuIn->orientation.x;
        imu_pitch_ = imuIn->orientation.y;
        imu_yaw_ = imuIn->orientation.z;

        newodomdata_ = true;

    }

};




int main (int argc, char** argv)
{
    ros::init (argc, argv, "gtsam_backend_node");

    GTSAM_BACKEND gtsam_backend;


    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        gtsam_backend.run();//运行主函数

        rate.sleep();
    }

    return 0;
}
