/***
 * @brief: cleaning robot path planning
 * @author: Wang
 * @date: 20170702
***/

#ifndef CLEANINGPATHPLANNING_H
#define CLEANINGPATHPLANNING_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <room_rotator.h>

using namespace cv;
using namespace std;

constexpr double PI =3.14159;

struct cellIndex
{
    int row;
    int col;
    double theta; //{0, 45,90,135,180,225,270,315}
};
//初始化的时候选择的感兴趣区域的坐标点

struct Astar
{
  int row;
  int col;
  int father_row;
  int father_col;
  int F;
  int G;
  int H;
};


/*************************************************
 *
 * 读取删格地图并根据占据信息获取其对应的空闲（可行走）空间，
 * 按照遍历算法规划行走路线。
 *
 * **********************************************/
class CleaningPathPlanning
{
public:
    //CleaningPathPlanning() = delete;
    CleaningPathPlanning(costmap_2d::Costmap2DROS *costmap2d_ros);

    vector<geometry_msgs::PoseStamped> GetPathInROS(const Mat& division_region_Map ,const Mat& R); 
    vector<geometry_msgs::PoseStamped> GetPathInROS();
    vector<geometry_msgs::PoseStamped> trasnsformtorospath();
    //vector<geometry_msgs::PoseStamped> GetBorderTrackingPathInROS();

    void SetCoveredGrid(double wx,double wy);
    int GetSizeOfCell(){return this->SIZE_OF_CELL;}
    bool Boundingjudge(int a,int b);

    //for visualization
    void PublishCoveragePath();
    void PublishGrid();
    bool outcloselist(int a, int b);
    bool outopenlist(int a,int b);
    
    void Astar_find_path(int a,int b,int c,int d);
    vector<Astar> openlist;
    vector<Astar> closelist;
    Mat Astarmap;
    int G_compare=0;
    vector<geometry_msgs::PoseStamped> pathVecForSend_;
    vector<cv::Point> init_point_;//初始点向量
    Mat mouse_select_region_Map_;//鼠标选择的划分地图
    Mat division_region_Map_;//分割好的地图
    Mat R_;
private:
    //helper functions.
    bool initializeMats(const Mat& division_region_Map);
    bool initializeCoveredGrid();
    void getCellMatAndFreeSpace(const Mat& srcImg, Mat &cellMat,vector<cellIndex> &freeSpaceVec);
    void initializeNeuralMat(Mat cellMat, Mat neuralizedMat);
    void writeResult(Mat resultmat,vector<cellIndex> pathVec);
    void writeResult(cv::Mat resultmat,std::vector<cv::Point2f> pathVec);
    void mainPlanningLoop(int point_img_x , int point_img_y);
    double distance(Point2i pta,Point2i ptb);
    bool findElement(vector<cv::Point2i> pointsVec,cv::Point2i pt, int&index);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    bool cellContainsPoint(cv::Point2i pt,cellIndex cell);
    void transformPointPathToPosePath(const std::vector<cv::Point2f>& point_path, std::vector<geometry_msgs::Pose2D>& pose_path);
    //void GetBorderTrackingPathInCV(vector<cv::Point2i>&resultVec);
    vector<cellIndex> GetPathInCV();


    bool initialized_;
    Mat srcMap_;
    Mat cellMat_;
    Mat neuralizedMat_;
    vector<cellIndex> freeSpaceVec_;
    vector<cellIndex> pathVec_;
    vector<cellIndex> temp_pathVec_;
    vector<geometry_msgs::PoseStamped> pathVecInROS_;
    std::vector<cv::Point2f> fov_middlepoint_path_;
    std::vector<geometry_msgs::Pose2D> path_fov_poses_;

    double resolution_;
    ros::Publisher plan_pub_;
    ros::Publisher grid_pub_;
    nav_msgs::OccupancyGrid covered_path_grid_;

    //tf::TransformListener &tf_;
    tf::Stamped<tf::Pose> initPose_;

    costmap_2d::Costmap2D* costmap2d_;
    costmap_2d::Costmap2DROS* costmap2d_ros_;

    int SIZE_OF_CELL; //must be odd number.
    int GRID_COVERED_VALUE;
};

#endif // CLEANINGPATHPLANNING_H
