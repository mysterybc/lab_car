#include "CleaningPathPlanner.h"
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include "robot_msgs/PathCoverage.h"
#include <stdio.h>
#include <room_rotator.h>
#include <selectimage.h>


/******************************
1.请划分园区的区域
2.请输入清扫区域的起始点
3.请输入清扫区域的顺序
*******************************/
namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS; 
using namespace cv; 


struct PathCoverageNode{
public:
    PathCoverageNode(costmap_2d::Costmap2DROS *lcr);
    bool on_service(robot_msgs::PathCoverage::Request & req,
                    robot_msgs::PathCoverage::Response & res);
    void draw_edge(cv::Point2d p1,cv::Point2d p2);

    vector<cv::Point> init_point_vector; //用于记录path coverage的起始位置
    ros::ServiceServer server;
    CleaningPathPlanning clr;
    //地图信息
    cv::Mat img;
    double origin_x,origin_y;  //pixel
    int size_x,size_y;      //pixel
    double resolution;
};

PathCoverageNode::PathCoverageNode(costmap_2d::Costmap2DROS *lcr): clr(lcr){
    ros::NodeHandle nh;
    server = nh.advertiseService("path_coverage",&PathCoverageNode::on_service,this);
    resolution = lcr->getCostmap()->getResolution();
    origin_x = lcr->getCostmap()->getOriginX();
    origin_y = lcr->getCostmap()->getOriginY();
    size_x = lcr->getCostmap()->getSizeInCellsX();
    size_y = lcr->getCostmap()->getSizeInCellsY();
}   

//接口说明
//传入的req已经按照先x后y，从小到大的顺序排列好
//初始位置默认为左下角，即x、y最小的点。
bool PathCoverageNode::on_service(  robot_msgs::PathCoverage::Request & req,
                                    robot_msgs::PathCoverage::Response & res) {
    //TODO 元算法支持多区域覆盖，目前就写了一个区域
    // //多区域循环
    // for(int i =0;i < init_point_vector.size();i++)
    // {
    //     clr.init_point_.clear();
    //     selectandtransform.rotatorselectedMap(org,init_point_vector[i],clr.division_region_Map_,clr.R_);
    //     namedWindow("OUTPUT"+std::to_string(i),CV_WINDOW_NORMAL);
    //     imshow("OUTPUT"+std::to_string(i),clr.division_region_Map_);
    //     cv::waitKey(0);
    //     destroyAllWindows();
    //     std::cout<<"clr.R_"<<clr.R_<<std::endl;
    //     clr.init_point_.push_back(cv::Point(clr.division_region_Map_.size().width/2,clr.division_region_Map_.size().height/2));
    //     //单写一个函数，进入利用clr.division_region_Map_,clr.R_来生成路径
    //     clr.GetPathInROS(clr.division_region_Map_,clr.R_);
    // }
    //创建path并发布

    //bc
    //初始化地图
    clr.mouse_select_region_Map_.copyTo(img);
    //绘制需要寻找的区域边界
    //这里默认已经对点进行了排序
    //需要将xy转成坐标

    // for(auto point:req.select_point.poses){
    //     std::cout << "receive point x is : " << point.pose.position.x << std::endl;
    //     std::cout << "receive point y is : " << point.pose.position.y << std::endl;
    // }
    std::vector<cv::Point> points;
    {
        //加上原点坐标，计算出图片中相对于左下角的xy坐标
        //y由于图像需要进行翻转
        for(auto &point:req.select_point.poses){
            point.pose.position.x = (point.pose.position.x + origin_x)/resolution;
            point.pose.position.y = size_y - (point.pose.position.y + origin_y)/resolution;
            points.push_back(cv::Point(point.pose.position.x,point.pose.position.y));
        }


        // for(auto point:req.select_point.poses){
        //     std::cout << "modify point x is : " << point.pose.position.x << std::endl;
        //     std::cout << "modify point y is : " << point.pose.position.y << std::endl;
        // }
        cv::Point start,end;
        cv::line(img, points[0], points[1], cv::Scalar(255),3, 4); 
        cv::line(img, points[0], points[2], cv::Scalar(255),3, 4); 
        cv::line(img, points[1], points[3], cv::Scalar(255),3, 4); 
        cv::line(img, points[2], points[3], cv::Scalar(255),3, 4);
        //test 
        // cv::namedWindow("show_img",CV_WINDOW_NORMAL);
        // cv::imshow("show_img",img);
        // cv::waitKey(0);
    }
    //设置起始点
    {
        cv::Point start(req.start_point.position.x,req.start_point.position.y);
        start.x = (req.start_point.position.x + origin_x)/resolution;
        start.y =  size_y - (req.start_point.position.y + origin_y)/resolution;
        init_point_vector.push_back(start);
        // std::cout << "start point x is : " << start.x << std::endl;
        // std::cout << "start point y is : " << start.y << std::endl;
    }
    //计算路径
    {
        clr.init_point_.clear();
        SelectAndTransform selectandtransform;
        selectandtransform.rotatorselectedMap(img,init_point_vector[0],clr.division_region_Map_,clr.R_);
        //by bc
        // namedWindow("OUTPUT",CV_WINDOW_NORMAL);
        // imshow("OUTPUT",clr.division_region_Map_);
        // cv::waitKey(0);
        // std::cout<<"clr.R_"<<clr.R_<<std::endl;
        clr.init_point_.push_back(cv::Point(clr.division_region_Map_.size().width/2,clr.division_region_Map_.size().height/2));
        //单写一个函数，进入利用clr.division_region_Map_,clr.R_来生成路径
        clr.GetPathInROS(clr.division_region_Map_,clr.R_);
    }
    //TODO 这里会发布需要的path 需要进行修改
    res.path.poses = clr.trasnsformtorospath();
    return true;
}



int main(int argc, char** argv){
    ros::init(argc, argv, "path_planning_node");
    ros::NodeHandle nh;
    //melodic devel
    // tf2_ros::Buffer tf(ros::Duration(10));
    //kinetic devel
    tf::TransformListener tf(ros::Duration(10));

    //创建costmap cleaning path需要
    costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);

    PathCoverageNode path_coverage_node(&lcr);

    ROS_WARN("planner inital finished");
    
    ros::Rate r(1);   
    while(ros::ok()){
      ros::spinOnce();
      r.sleep();
    }

    ros::shutdown();
    return 0;
}




