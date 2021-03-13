#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "CleaningPathPlanner.h"
#include "tf/tf.h"
#include <room_rotator.h>
#include <selectimage.h>


#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace cv; 

cv::Mat org,dst,img,tmp;
int Clicks;

vector<cv::Point> init_point_vector;

void on_mouse(int event,int x,int y,int flags,void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号  
{  
    static Point pre_pt ;//初始坐标  
    static Point cur_pt ;//实时坐标  
    char temp[16];  
    if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆  
    {  
        org.copyTo(img);//将原始图片复制m到img中  
        sprintf(temp,"第一个点(%d,%d)",x,y);  
        pre_pt = Point(x,y);
        putText(img,temp,pre_pt,FONT_HERSHEY_SIMPLEX,4,Scalar(0,0,0,255),3,8);//在窗口上显示坐标  
        circle(img,pre_pt,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);//划圆
        imshow("img",img);
    }  
    else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))//左键没有按下的情况下鼠标移动的处理函数  
    {  
        img.copyTo(tmp);//将img复制到临时图像tmp上，用于显示实时坐标  
        sprintf(temp,"(%d,%d)",x,y);  
        cur_pt = Point(x,y);
        putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,4,Scalar(0,0,0,255),3,8);//只是实时显示鼠标移动的坐标  
        imshow("img",tmp);
    }  
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))//左键按下时，鼠标移动，则在图像上划矩形  
    {  
        img.copyTo(tmp);  
        sprintf(temp,"第二个点(%d,%d)",x,y);  
        cur_pt = Point(x,y);  
        putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,4,Scalar(0,0,0,255),3,8);  
        cv::line(tmp, pre_pt, cur_pt, cv::Scalar(255),3, 4); 
        imshow("img",tmp);  
    }  
    else if (event == CV_EVENT_MBUTTONDOWN)//中键点击，选取起始点的位置  
    {
        cur_pt = Point(x,y);
        init_point_vector.push_back(cur_pt);
        Clicks++;
        ROS_INFO("第%d个清扫区域的起始坐标为：(%d,%d)",Clicks,cur_pt.x,cur_pt.y);

    }  
    else if (event == CV_EVENT_LBUTTONUP)//左键松开，将在图像上划矩形
    {
        org.copyTo(img);  
        sprintf(temp,"(%d,%d)",x,y);
        cur_pt = Point(x,y);
        circle(img,pre_pt,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);
        cv::line(img, pre_pt, cur_pt, cv::Scalar(255),3, 4);
        imshow("img",img);
        img.copyTo(org);
    }
}  

int main(int argc, char** argv){
    ros::init(argc, argv, "cleaning_using_movebase");
    move_base_msgs::MoveBaseGoal nextGoal;

    //load the global path.
    tf::TransformListener tl_listener(ros::Duration(10));
    costmap_2d::Costmap2DROS lcr("cleaning_costmap", tl_listener);
    CleaningPathPlanning *pathPlanner = new CleaningPathPlanning(&lcr);

    //gzx
    namedWindow("img",CV_WINDOW_NORMAL);//定义一个img窗口
    Clicks = 0; 
    pathPlanner->mouse_select_region_Map_.copyTo(org);
    org.copyTo(img);
    org.copyTo(tmp); 
    setMouseCallback("img",on_mouse,0);//调用回调函数  在img这个窗口上做鼠标回调处理
    imshow("img",img);
    ROS_INFO("请用鼠标左键划分清扫区域，中键选取区域起始点，划分后输入键盘上任意按键后继续");
    cv::waitKey(0);  
    //destroyAllWindows();
    SelectAndTransform selectandtransform;
    selectandtransform.rotatorselectedMap(org,init_point_vector[0],pathPlanner->division_region_Map_,pathPlanner->R_);
    imshow("OUTPUT",pathPlanner->division_region_Map_);
    pathPlanner->init_point_.push_back(cv::Point(pathPlanner->division_region_Map_.size().width/2,pathPlanner->division_region_Map_.size().height/2));
    std::cout<<pathPlanner->init_point_<<std::endl;
    cv::waitKey(0);
    //gzx

    //full coverage path.
    std::vector<geometry_msgs::PoseStamped> fullCoverPath = pathPlanner->GetPathInROS();
    int beginNum = fullCoverPath.size();

    pathPlanner->PublishCoveragePath();

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

  //main loop
  ros::Rate r(5);
  for(int i = 0; i < fullCoverPath.size(); i++)
  {
      nextGoal.target_pose.header.frame_id = "map";
      nextGoal.target_pose.header.stamp = ros::Time::now();

      geometry_msgs::PoseStamped posestamped = fullCoverPath[i];

      //call move base to plan a long distance.
      nextGoal.target_pose.pose.position.x = posestamped.pose.position.x;
      nextGoal.target_pose.pose.position.y = posestamped.pose.position.y;
      nextGoal.target_pose.pose.position.z = 0;
      nextGoal.target_pose.pose.orientation.w = posestamped.pose.orientation.w;
      nextGoal.target_pose.pose.orientation.x = posestamped.pose.orientation.x;
      nextGoal.target_pose.pose.orientation.y = posestamped.pose.orientation.y;
      nextGoal.target_pose.pose.orientation.z = posestamped.pose.orientation.z;

      ROS_INFO("Sending next goal!");
      ac.sendGoal(nextGoal);
      ac.waitForResult();
      ros::Duration(0.01).sleep(); 

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          ROS_INFO("Hooray, the base moved a point forward in full path!");
          pathPlanner->SetCoveredGrid(posestamped.pose.position.x,posestamped.pose.position.y);
          pathPlanner->PublishGrid();
      }
      else
      {
          ROS_INFO("The base failed to move forward to the next path for some reason!");
          continue;
      }

      pathPlanner->PublishCoveragePath();
      ros::spinOnce();
      r.sleep();
    }

  delete pathPlanner;
  return 0;
}
