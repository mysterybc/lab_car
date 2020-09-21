/** brief:头文件声明了global_planner节点类，GlobalPlannerNode
 *  note:
 *  
 */

#ifndef GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_NODE_H
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "robot_msgs/Planning.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "yaml-cpp/yaml.h"
#include "AStar.hpp"


class GlobalPlannerNode{
public:
    GlobalPlannerNode();
    ~GlobalPlannerNode()=default;
    //接口函数
    bool LoadConfig(std::string file);
    bool Check(std::string algorithm_name);
    bool Prepare(std::string algorithm_name);
    bool IsComplete(std::string algorithm_name);

    //TODO 目前对于astar算法使用了AStar::Vec2i，之后可以改成模板
    bool CalPath(robot_msgs::Planning::Request &req,
                 robot_msgs::Planning::Response &res);
    void OdomCallback(const nav_msgs::OdometryConstPtr &msg);



    ros::Subscriber robot_location_sub;
    ros::ServiceServer palnning_service; 
    geometry_msgs::Point my_pose;
    geometry_msgs::Point goal;
    nav_msgs::Path path;
    AStar::Generator a_star_planner;
    std::string map_path;
    double map_resplution;
    bool new_goal;
    bool is_complete;
    std::string node_name;
    
};


#endif