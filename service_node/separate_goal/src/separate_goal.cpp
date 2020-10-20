#include "separate_goal.h"
#include "tf/transform_listener.h"


SeparateGoal::SeparateGoal(){
    ros::NodeHandle nh;
    nh.param<int>("car_id",car_id,1);
    robots_state_sub = nh.subscribe("robot_states",10,&SeparateGoal::RobotStateCallback,this);
    map_sub = nh.subscribe("map",10,&SeparateGoal::MapCallback,this);
    separate_service = nh.advertiseService("separate_goal",&SeparateGoal::CalGoal,this);
}

bool SeparateGoal::CalGoal(robot_msgs::Separate::Request &req,
                           robot_msgs::Separate::Response &res){
    goal_point = req.goal;  
    int x_flag{0}, y_flag{0};
    int online_car{1};
    my_pose = GetMyPose();
    for(auto it : robots_info){
        if(it.car_id == 0){
            continue;
        }
        online_car++;
        std::cout << "other car x" << it.robot_pose.position.x << std::endl;
        if(my_pose.position.x > it.robot_pose.position.x){
            x_flag++;
        }
        if(my_pose.position.y > it.robot_pose.position.y){
            y_flag++;
        }
    }
    if(online_car == 1){
        res.goal = goal_point;
        return true;
    }
    else{
        if(x_flag >= online_car/2){
            goal_point.position.x += 1;
        }
        else{
            goal_point.position.x -= 1;
        }
        if(online_car > 2){
            if(y_flag >= online_car/2){
                goal_point.position.y += 1;
            }
            else{
                goal_point.position.y -= 1;
            }
        }
    }
    
    
    res.goal = goal_point;
    return true;

}

void RobotInfo::SetState(const robot_msgs::RobotState state){
    car_id = state.car_id;
    robot_pose = state.robot_pose;
}

//point should be in meters not pixels
bool SeparateGoal::IsOccupied(geometry_msgs::Point p){
    // p.x = p.x / map.info.resolution;
    // p.y = p.y / map.info.resolution;
    // return map.data[map.info.width*p.x + p.y];
}

void SeparateGoal::RobotStateCallback(const robot_msgs::RobotStatesConstPtr &msg){
    while(msg->online_robot_number > robots_info.size()){
        RobotInfo robot_info;
        robots_info.push_back(robot_info);
    }
    for(int i = 0; i < msg->online_robot_number; i++){
        robots_info[i].SetState(msg->robot_states[i]);
    }
}

geometry_msgs::Pose SeparateGoal::GetMyPose(){
    geometry_msgs::Pose pose;
    tf::TransformListener listerner;
    tf::StampedTransform trans;
    while(ros::ok()){
        try{
            listerner.lookupTransform("map","base_link",ros::Time(0),trans);
        }
        catch(tf::TransformException &exception) {
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            continue;                
        }
        tf::quaternionTFToMsg(trans.getRotation(),pose.orientation);
        pose.position.x = trans.getOrigin().x();
        pose.position.y = trans.getOrigin().y();
        pose.position.z = trans.getOrigin().z();
        std::cout << "pose x is" << pose.position.x << std::endl;
        std::cout << "pose y is" << pose.position.y << std::endl;
        break;
    }
    return pose;
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
