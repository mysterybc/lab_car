#include "ndt_localization/point_cloud_octomap_and_OGM.h"


MapGeneration::MapGeneration(ros::NodeHandle &nh_):
nh_(nh_),
res_(0.3),
new_odom_(false),
octomap3D_pub_topic_("/octo3Dmap"),
max_range_(30.0),
probHit_(0.7),
probMiss_(0.4),
thresMin_(0.12),
thresMax_(0.97),
occupancyMinZ_(-0.2),
occupancyMaxZ_(0.6),
treeMaxDepth_(0){
    velodyne_point_sub_ = nh_.subscribe("/velodyne_points",1, &MapGeneration::velodyne_points_cb, this);

    odom_sub_ = nh_.subscribe("/odometry",1, &MapGeneration::odometry_points_cb, this);

    octomap3D_pub_ = nh_.advertise<octomap_msgs::Octomap>(octomap3D_pub_topic_,5);

    map2D_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map",5);

    allocateMemory();
}

MapGeneration::~MapGeneration(){
    if(tree_)
    {
        delete tree_;
        tree_ = NULL;
    }
}

void MapGeneration::allocateMemory(){
    //octree init
    tree_ = new octomap::OcTree(res_);

    tree_->setProbHit(probHit_);
    tree_->setProbMiss(probMiss_);
    tree_->setClampingThresMin(thresMin_);
    tree_->setClampingThresMax(thresMax_);

    treeMaxDepth_ = tree_->getTreeDepth();

    //occupy grid map init
    grid_map_.header.frame_id = "world";
    grid_map_.info.resolution = res_;



}

void MapGeneration::resetParameters(){}

void MapGeneration::velodyne_points_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
    point_duque_.push_back(*msg);

    if(new_odom_ == true)
    {
        sensor_msgs::PointCloud2 cur_point;
        {
            std::lock_guard<std::mutex> lock1(odom_mutex_);

            while(!point_duque_.empty())
            {
                cur_point = point_duque_.front();
                double timeScanCur = cur_point.header.stamp.toSec();
                if(std::abs(cur_odom_.header.stamp.toSec()-timeScanCur)<=0.05)
                {
                    ROS_INFO("|timeScanCur - cur_odom.header.stamp.toSec()|<= 0.05");
                    break;
                }
                else if( timeScanCur - cur_odom_.header.stamp.toSec() < - 0.05)
                {
                    point_duque_.pop_front();
                    ROS_WARN("timeScanCur - cur_odom.header.stamp.toSec()< -0.05");
                }
                else if( timeScanCur - cur_odom_.header.stamp.toSec()> 0.05)
                {
                    ROS_ERROR("timeScanCur - cur_odom.header.stamp.toSec()> 0.05");

                    std::cout<<std::fixed<<"timeScanCur: "<<timeScanCur<<" "<<"cur_odom.header.stamp.toSec(): "<<cur_odom_.header.stamp.toSec()<<std::endl;
                    return;
                }
            }
        }

        if(point_duque_.empty())
        {
            ROS_ERROR("point_duque.empty()");
            return;
        }
        double timeScan = cur_point.header.stamp.toSec();

        pcl::PointCloud<PointT>::Ptr  raw_cloud_Ptr(new pcl::PointCloud<PointT>);

        pcl::fromROSMsg(cur_point, *raw_cloud_Ptr);

        Eigen::Matrix4f cur_trans;

        cur_trans = odom2TransformMatrix(cur_odom_);

        pcl::PointCloud<PointT>::Ptr  transformed_cloud_1(new pcl::PointCloud<PointT>);

        pcl::transformPointCloud(*raw_cloud_Ptr, *transformed_cloud_1, cur_trans);

        pointcloud2octree(*tree_,transformed_cloud_1,cur_trans);

        resetParameters();
        new_odom_ = false;
    }

}

void MapGeneration::odometry_points_cb(const nav_msgs::Odometry::ConstPtr& odomin){

    std::lock_guard<std::mutex> lock1(odom_mutex_);
    
    cur_odom_ = *odomin;
    cur_odom_.pose.pose.position.z = 0;

    double roll,pitch,yaw;

    RosQ2RPY(cur_odom_.pose.pose.orientation,roll,pitch,yaw);

    new_odom_ = true;
}

void MapGeneration::pointcloud2octree(octomap::OcTree& tree,const pcl::PointCloud<PointT>::Ptr& temp ,const Eigen::Matrix4f& pose){

    double x,y,z;
    x = pose(0,3);
    y = pose(1,3);
    z = pose(2,3);
    octomap::point3d base_pose(x,y,z);

    octomap::KeySet free_cells, occupied_cells; //设置一个keyset

    for (auto p:temp->points){

        octomap::point3d point(p.x,p.y,p.z);

        if ((max_range_ < 0.0) || ((point - base_pose).norm() <= max_range_) ) {
            // free cells
            if (tree.computeRayKeys(base_pose, point, keyRay_)){

                free_cells.insert(keyRay_.begin(), keyRay_.end());

            }
            // occupied endpoint
            octomap::OcTreeKey key;
            if (tree.coordToKeyChecked(point, key)){

                occupied_cells.insert(key);

            }
        } else {// ray longer than maxrange:;

            octomap::point3d new_end = base_pose + (point - base_pose).normalized() * max_range_;
            if (tree.computeRayKeys(base_pose, new_end, keyRay_)){
                free_cells.insert(keyRay_.begin(), keyRay_.end());

                octomap::OcTreeKey endKey;
                if (tree.coordToKeyChecked(new_end, endKey)){
                    free_cells.insert(endKey);

                } else{
                    ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
                }

            }
        }
    }
    // mark free cells only if not seen occupied in this cloud
    for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
        if (occupied_cells.find(*it) == occupied_cells.end()){
            tree.updateNode(*it, false);
        }
    }

    // now mark all occupied cells:
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
        tree.updateNode(*it, true);
    }

    tree.prune();

    if(octomap3D_pub_.getNumSubscribers() != 0){
        octomap_msgs::Octomap octomap_msg;
        octomap_msgs::fullMapToMsg(tree, octomap_msg);
        octomap_msg.header.frame_id = "world";
        octomap_msg.header.stamp = ros::Time::now();
        octomap3D_pub_.publish(octomap_msg);
    }
    octree2OccupyGridMap(tree);
}

void MapGeneration::octree2OccupyGridMap(const octomap::OcTree& tree){

    treeMaxDepth_ = tree.getTreeDepth();

    double gridRes = tree_->getNodeSize(treeMaxDepth_);
    grid_map_.info.resolution = gridRes;
    grid_map_.header.stamp = ros::Time::now();

    //grid_map_.info.origin.position;

    double minX, minY, minZ, maxX, maxY, maxZ;

    tree_->getMetricMin(minX, minY, minZ);
    tree_->getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);

    //map's origin is octomap's origin

    minKey_ = tree_->coordToKey(minPt, treeMaxDepth_);//这里是什么意思
    maxKey_ = tree_->coordToKey(maxPt, treeMaxDepth_);//这里是什么意思

    grid_map_.info.width = (maxKey_[0]-minKey_[0]);
    grid_map_.info.height = (maxKey_[1]-minKey_[1]);

    octomap::point3d origin = tree_->keyToCoord(minKey_, treeMaxDepth_);

    grid_map_.info.origin.position.x = origin.x() - gridRes*0.5;
    grid_map_.info.origin.position.y = origin.y() - gridRes*0.5;

    grid_map_.data.clear();
    // init to unknown:
    grid_map_.data.resize(grid_map_.info.width * grid_map_.info.height, -1);

    std::cout<<"grid_map_size: "<< grid_map_.info.width * grid_map_.info.height<<std::endl;

    for (octomap::OcTree::iterator it = tree.begin(treeMaxDepth_),
                 end = tree.end(); it != end; ++it){
        if(tree.isNodeOccupied(*it)){//node is occupied
            double z = it.getZ();
            double half_size = it.getSize() / 2.0;

            if (z + half_size > occupancyMinZ_ && z - half_size < occupancyMaxZ_){//if in z~-z range, update occupied grid
                update2DMap(it,true);
            }

        }else{// node not occupied => mark as free in 2D map if unknown so far
            double z = it.getZ();
            double half_size = it.getSize() / 2.0;
            if (z + half_size > occupancyMinZ_ && z - half_size < occupancyMaxZ_)
            {
                update2DMap(it,false);
            }
        }

    }

    map2D_pub_.publish(grid_map_);
}

void MapGeneration::update2DMap(const octomap::OcTree::iterator& it, bool occupied) {

    // update 2D map (occupied always overrides):
    if (it.getDepth() == treeMaxDepth_) {//如果是树的最大深度
        unsigned idx = mapIdx(it.getKey());//得到地图的索引
        if (occupied)//如果是占据的
            grid_map_.data[idx] = 100;
        else if (grid_map_.data[idx] == -1) {
            grid_map_.data[idx] = 0;
        }

    } else {//如果不是八叉树的最大深度呢
        int intSize = 1 << (int)(treeMaxDepth_ - it.getDepth());//
        octomap::OcTreeKey minKey = it.getIndexKey();
        for (int dx = 0; dx < intSize; dx++) {
            int i = (minKey[0] + dx - minKey_[0]) / 1;
            for (int dy = 0; dy < intSize; dy++) {
                unsigned int idx = mapIdx(i, (minKey[1] + dy - minKey_[1]) / 1);
                if (occupied)
                    grid_map_.data[idx] = 100;
                else if (grid_map_.data[idx] == -1) {
                    grid_map_.data[idx] = 0;
                }
            }
        }
    }
}


void MapGeneration::RosQ2RPY(const geometry_msgs::Quaternion& quaternion ,double& roll,double& pitch,double& yaw){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

void MapGeneration::RosRPY2Q(const double& roll,const double& pitch,const double& yaw,geometry_msgs::Quaternion& quaternion){
    tf::Quaternion quat;
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

Eigen::Matrix4f MapGeneration::odom2TransformMatrix(const nav_msgs::Odometry& odomin){
    Eigen::Matrix4f cur_trans;
    cur_trans = Eigen::Matrix4f::Identity();

    Eigen::Quaternionf q(odomin.pose.pose.orientation.w,odomin.pose.pose.orientation.x, odomin.pose.pose.orientation.y, odomin.pose.pose.orientation.z);

    cur_trans.block(0,0,3,3) = q.matrix();
    cur_trans(0,3) =  odomin.pose.pose.position.x;
    cur_trans(1,3) =  odomin.pose.pose.position.y;
    cur_trans(2,3) =  odomin.pose.pose.position.z;

    return cur_trans;
}
