#include "separate_goal.h"
#include "tf/transform_listener.h"


SeparateGoal::SeparateGoal(){
    ros::NodeHandle nh;
   //获取group空间名
    std::string namespace_;
    namespace_ = nh.getNamespace();
    //获取group下的参数
    if(!nh.getParam(namespace_+"/car_id",car_id)){
        car_id = 1;
        ROS_WARN("SEPARATE SERVICE FAILED TO GET CAR ID");
        ROS_WARN("SEPARATE SERVICE RESET CAR ID TO 1");
    }
    if(!nh.getParam(namespace_+"/tf_ns",tf_ns)){
        ROS_WARN("SEPARATE SERVICE FAILED TO GET TF FRAME");
    }
    debug_pub = nh.advertise<robot_msgs::DebugInfo>("/debug_info",10);
    robots_state_sub = nh.subscribe("robot_states",10,&SeparateGoal::RobotStateCallback,this);
    map_sub = nh.subscribe("/map",10,&SeparateGoal::MapCallback,this);
    separate_service = nh.advertiseService("separate_goal",&SeparateGoal::CalGoal,this);
    //Debug info
    robot_msgs::DebugInfo info;
    std_msgs::String data;
    data.data = "car " + std::to_string(car_id) + " separate goal service Initialized";
    info.info.push_back(data);
    data.data.clear();
    data.data = "car " + std::to_string(car_id) + " tf frame is " + tf_ns;
    info.info.push_back(data);
    debug_pub.publish(info);
}

bool SeparateGoal::CalGoal(robot_msgs::Separate::Request &req,
                           robot_msgs::Separate::Response &res){
    goal_point = req.goal;  
    ROS_INFO("separate start!!");
    int x_flag{0}, y_flag{0};
    int online_car{2};
    my_pose = GetMyPose();
    ROS_INFO("get my pose!!");
    for(auto it : robots_info){
        if(it.car_id == 0){
            continue;
        }
        // online_car++;
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
            listerner.lookupTransform("map",tf_ns+"base_link",ros::Time(0),trans);
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
