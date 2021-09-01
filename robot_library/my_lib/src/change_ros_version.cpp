#include "iostream"
#include "fstream"
#include "ros/ros.h"
#include "ros/package.h"
#include "string"
#include <vector>

using namespace std;


//argv as ros version Kinetic or Melodic
int main(int argc, char** argv){
    ros::init(argc,argv,"change_ros_version");
    ros::NodeHandle nh;

    //选择需要修改的版本
    if(argc != 2){
        ROS_WARN("please input ros version as cpp arg input !!!!");
        ROS_WARN("please input ros version as cpp arg input !!!!");
        return 0;
    }
    string VERSION = argv[1];

    //get file path
    string package_path = ros::package::getPath("service_node");
    vector<string> cpp_path;
    cpp_path.push_back(package_path+ "/path_coverage/src/CleaningPathPlanner.cpp");
    cpp_path.push_back(package_path + "/path_coverage/src/LocalGridVelocityComputer.cpp");
    cpp_path.push_back(package_path + "/path_coverage/src/PathPlanningNode.cpp");


    //file
    fstream file;

    //file1
    //open file
    file.open(cpp_path[0],ios_base::in | ios_base::out);
    if(!file.is_open()){
        ROS_WARN("file1 open failed !!!! please modify this file by your self!!!");
        ROS_WARN("file1 open failed !!!! please modify this file by your self!!!");
        return 0;
    }

    //find place
    char temp[1024];
    string file_content="";
    int line_num{0};
    while(file.getline(temp,sizeof(temp)) && line_num <= 65){
        line_num++;
        // if(line_num <= 50){
        //     file_content += string(temp);
        //     continue;
        // }
        string line_content(temp);
        line_content += "\n";
        if(VERSION == "Kinetic"){
            if(line_num == 57){
                file_content += "    // geometry_msgs::PoseStamped pose;\n";
                continue;
            }
            if(line_num == 58){
                file_content += "    // bool isok = costmap2d_ros_->getRobotPose(pose);\n";
                continue;
            }
            if(line_num == 59){
                file_content += "    // tf::poseStampedMsgToTF(pose,initPose_);\n";
                continue;
            }
            if(line_num == 61){
                file_content += "    bool isok = costmap2d_ros_->getRobotPose(initPose_);   \n";
                continue;
            }
        }
        else{
            if(line_num == 57){
                file_content += "    geometry_msgs::PoseStamped pose;   \n";
                continue;
            }
            if(line_num == 58){
                file_content += "    bool isok = costmap2d_ros_->getRobotPose(pose);   \n";
                continue;
            }
            if(line_num == 59){
                file_content += "    tf::poseStampedMsgToTF(pose,initPose_);   \n";
                continue;
            }
            if(line_num == 61){
                file_content += "    // bool isok = costmap2d_ros_->getRobotPose(initPose_)\n";
                continue;
            }
        }
        file_content += line_content;
    }
    file.close();
    file.open(cpp_path[0],ios_base::in | ios_base::out);
    if(file.is_open()){
        file << file_content;
    }
    file.close();


    //file2
    //open file
    file.open(cpp_path[1],ios_base::in | ios_base::out);
    if(!file.is_open()){
        ROS_WARN("file2 open failed !!!! please modify this file by your self!!!");
        ROS_WARN("file2 open failed !!!! please modify this file by your self!!!");
        return 0;
    }

    //find place
    file_content.clear();
    line_num = 0;
    while(file.getline(temp,sizeof(temp))){
        line_num++;
        // if(line_num <= 50){
        //     file_content += string(temp);
        //     continue;
        // }
        string line_content(temp);
        line_content += "\n";
        if(VERSION == "Kinetic"){
            if(line_num == 53){
                file_content += "    tf::Stamped<tf::Pose> pose;   \n";
                continue;
            }
            if(line_num == 54){
                file_content += "        if(costmap2d_ros_->getRobotPose(pose))   \n";
                continue;
            }
            if(line_num == 55){
                file_content += "    tf::poseStampedTFToMsg(pose, current_pose_);   \n";
                continue;
            }
            if(line_num == 57){
                file_content += "    // costmap2d_ros_->getRobotPose(current_pose_);\n";
                continue;
            }
        }
        else{
            if(line_num == 53){
                file_content += "    // tf::Stamped<tf::Pose> pose;\n";
                continue;
            }
            if(line_num == 54){
                file_content += "    //     if(costmap2d_ros_->getRobotPose(pose))\n";
                continue;
            }
            if(line_num == 55){
                file_content += "    // tf::poseStampedTFToMsg(pose, current_pose_);\n";
                continue;
            }
            if(line_num == 57){
                file_content += "    costmap2d_ros_->getRobotPose(current_pose_);   \n";
                continue;
            }
        }
        file_content += line_content;
    }
    file.close();
    file.open(cpp_path[1],ios_base::in | ios_base::out);
    if(file.is_open()){
        file << file_content;
    }
    file.close();


    //file3
    //open file
    file.open(cpp_path[2],ios_base::in | ios_base::out);
    if(!file.is_open()){
        ROS_WARN("file3 open failed !!!! please modify this file by your self!!!");
        ROS_WARN("file3 open failed !!!! please modify this file by your self!!!");
        return 0;
    }

    //find place
    file_content.clear();
    line_num = 0;
    while(file.getline(temp,sizeof(temp))){
        line_num++;
        string line_content(temp);
        line_content += "\n";
        if(VERSION == "Kinetic"){
            if(line_num == 145){
                file_content += "    // tf2_ros::Buffer buffer(ros::Duration(10));\n";
                continue;
            }
            if(line_num == 146){
                file_content += "    // tf2_ros::TransformListener tf(buffer);\n";
                continue;
            }
            if(line_num == 148){
                file_content += "    tf::TransformListener buffer(ros::Duration(10));   \n";
                continue;
            }
        }
        else{
            if(line_num == 145){
                file_content += "    tf2_ros::Buffer buffer(ros::Duration(10));   \n";
                continue;
            }
            if(line_num == 146){
                file_content += "    tf2_ros::TransformListener tf(buffer);\n";
                continue;
            }
            if(line_num == 148){
                file_content += "    // tf::TransformListener buffer(ros::Duration(10));\n";
                continue;
            }
        }
        file_content += line_content;
    }
    file.close();
    file.open(cpp_path[2],ios_base::in | ios_base::out);
    if(file.is_open()){
        file << file_content;
    }
    file.close();


    ROS_INFO("change version to %s success!!",argv[1]);
    return 0;



}