#pragma once

#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
 
//OpenCV2标准头文件
#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <room_rotator.h>
 

using namespace cv;
using namespace std;

//
//定义一个两个开源代码转换的类，，主要涉及到图像旋转及坐标转化和格式匹配
class SelectAndTransform
{
private:
    cv::Mat img_;
    //cv::Point seed_point_;//init point and rotation center
    cv::Mat gray_mat_;
    cv::Mat room_map_;
    cv::Mat rotated_room_map_;
    cv::Rect bbox_;
    cv::Mat R_;
    RoomRotator roomrotator;
    double room_angle_;
    float map_resolution_;
    
public:
    SelectAndTransform(){}

    void rotatorselectedMap(const cv::Mat& img,const cv::Point& seed_point,cv::Mat& rotated_room_map,cv::Mat& R)
    {
        //img_ = cv::imread("/home/guozixuan/map_test.jpg");
        img_ = img.clone();
        Rect ccomp;
        cv::floodFill(img_,seed_point,76,&ccomp,10,10);
        //cv::cvtColor(img_, gray_mat_, CV_BGR2GRAY);//感兴趣区域灰度值为76
        // namedWindow("INPUT",CV_WINDOW_NORMAL);
        // imshow("INPUT",img_);
        // cv::waitKey(1);
        cv::inRange(img_,70,80, room_map_);//提取二值化的图像
        map_resolution_ = 1;
        room_angle_ = roomrotator.computeRoomRotationMatrix(room_map_,R_, bbox_,map_resolution_,&seed_point);
        // std::cout<<"room_angle:"<<(room_angle_*180)/CV_PI<<std::endl;
        roomrotator.rotateRoom(room_map_, rotated_room_map_,R_,bbox_);
        for(int i = 0;i < rotated_room_map_.rows;i++)
        {
            uchar *data = rotated_room_map_.ptr<uchar>(i);
           for(int j = 0;j < rotated_room_map_.cols;j++) 
           {
                if(data[j] == 0) data[j] = 255;
                else if(data[j] == 255) data[j] = 0;

            }
        }
        rotated_room_map = rotated_room_map_.clone(); 
        R = R_.clone();
    }
    ~SelectAndTransform() //析构函数
    {
        // destroyWindow("INPUT");
        // destroyWindow("OUTPUT");
    }
};
