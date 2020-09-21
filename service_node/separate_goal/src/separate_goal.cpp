#include "separate_goal.h"


SeparateGoal::SeparateGoal():robots_info(4){
    std::string name = "/robot_";
    for(int i = 0;i < 4; i++){
        std::string robot_name = name + std::to_string(i);
        robots_info[i].pose_sub = nh.subscribe(robot_name+"/odom",10,&RobotInfo::PoseCallback,&robots_info[i]);
    }

    map_sub = nh.subscribe("map",10,&SeparateGoal::MapCallback,this);
    separate_service = nh.advertiseService("separate_goal",&SeparateGoal::CalGoal,this);
    nh.getParam("car_id",robot_id);
}

bool SeparateGoal::CalGoal(robot_msgs::Separate::Request &req,
                           robot_msgs::Separate::Response &res){
    goal_point = req.goal;  
    int x_flag{0}, y_flag{0};
    for(auto it : robots_info){
        if(robots_info[robot_id].robot_pose.position.x > it.robot_pose.position.x){
            x_flag++;
        }
        if(robots_info[robot_id].robot_pose.position.y > it.robot_pose.position.y){
            y_flag++;
        }
    }
    if(x_flag >= 2){
        goal_point.position.x += 0.5;
    }
    else{
        goal_point.position.x -= 0.5;
    }
    if(y_flag >= 2){
        goal_point.position.y += 0.5;
    }
    else{
        goal_point.position.y -= 0.5;
    }
    res.goal = goal_point;
    return true;

}

//point should be in meters not pixels
bool SeparateGoal::IsOccupied(geometry_msgs::Point p){
    // p.x = p.x / map.info.resolution;
    // p.y = p.y / map.info.resolution;
    // return map.data[map.info.width*p.x + p.y];
}

void RobotInfo::PoseCallback(const nav_msgs::OdometryConstPtr &msg){
    robot_pose = msg->pose.pose;

}

void SeparateGoal::MapCallback(const nav_msgs::OccupancyGridConstPtr &msg){
    map.data = msg->data;
    map.header = msg->header;
    map.info = msg->info;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "separate_goal_node");
    ros::NodeHandle nh;
    SeparateGoal separate_goal_node;
    ros::Rate loop(100);
    while(ros::ok()){
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}
