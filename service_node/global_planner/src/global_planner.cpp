#include "global_planner.h"

GlobalPlannerNode::GlobalPlannerNode(){
    ros::NodeHandle nh;
    std::string path = "config file path";
    is_complete = true;
    new_goal = false;
    if(!LoadConfig(path)){
        ROS_WARN("global_planning_node: Config failed");
    }
    a_star_planner.LoadMap(map_path,a_star_planner);
    a_star_planner.setHeuristic(&AStar::Heuristic::euclidean);
    palnning_service = nh.advertiseService("global_planning",&GlobalPlannerNode::CalPath,this);
    robot_location_sub = nh.subscribe("odom",10,&GlobalPlannerNode::OdomCallback,this);
}

bool GlobalPlannerNode::LoadConfig(std::string file){
    ROS_INFO("service_node: load config");
    std::string config_path = "/home/lovezy/catkin_car/src/service_node/global_planner/config/planning_config.yaml";
    YAML::Node config = YAML::LoadFile(config_path);
    node_name = config["node_name"].as<std::string>();
    map_path = config["map_path"].as<std::string>();
    map_resplution = config["resolution"].as<double>();
    return true;
}

void GlobalPlannerNode::OdomCallback(const nav_msgs::OdometryConstPtr &msg){
    my_pose.x = msg->pose.pose.position.x;
    my_pose.y = msg->pose.pose.position.y;
}

bool GlobalPlannerNode::Check(std::string algorithm_name){
    if(algorithm_name!="A_Star"){
        return false;
    }
    return true;
}

bool GlobalPlannerNode::Prepare(std::string algorithm_name){
    return Check(algorithm_name);
}

bool GlobalPlannerNode::IsComplete(std::string algorithm_name){
    return is_complete;
}

bool GlobalPlannerNode::CalPath(robot_msgs::Planning::Request  &req,
                                robot_msgs::Planning::Response &res){
    std::vector<AStar::Vec2i> temp_path;
    AStar::Vec2i end_pose;
    AStar::Vec2i start_pose;
    goal.x = req.goal.position.x;
    goal.y = req.goal.position.y;
    new_goal = true;
    double start_time = ros::WallTime::now().toSec();
    ROS_INFO("get goal!  the pose is [%f,%f],the goal is [%f,%f]",my_pose.x,my_pose.y,goal.x,goal.y);
    start_pose.x = my_pose.x / map_resplution;
    start_pose.y = my_pose.y / map_resplution;
    end_pose.x = goal.x / map_resplution;
    end_pose.y = goal.y / map_resplution;
    temp_path = a_star_planner.findPath(start_pose,end_pose);
    res.path.header.frame_id = "map";
    double cost_time = ros::WallTime::now().toSec() - start_time;
    ROS_INFO("planning cost time is: %f",cost_time);
    for(int i = temp_path.size() -1; i >0 ; i--){
        geometry_msgs::PoseStamped pose_;
        pose_.header.frame_id = "map";
        pose_.pose.position.x = (float)temp_path.at(i).x * map_resplution;
        pose_.pose.position.y = temp_path.at(i).y * map_resplution;
        res.path.poses.push_back(pose_);  
    }
    {
        //最后一个点带有角度要求
        geometry_msgs::PoseStamped pose_;
        pose_.header.frame_id = "map";
        pose_.pose.position.x = (float)temp_path.front().x * map_resplution;
        pose_.pose.position.y = temp_path.front().y * map_resplution;
        pose_.pose.orientation = req.goal.orientation;
        res.path.poses.push_back(pose_);  
    }
    
    return true;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_node");
    GlobalPlannerNode planning_node;
    ros::Rate loop(100);
    while(ros::ok()){
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}
