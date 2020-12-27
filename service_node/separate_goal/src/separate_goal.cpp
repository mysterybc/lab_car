#include "separate_goal.h"
#include "tf/transform_listener.h"
#include "algorithm"
#include "debug_info.h"

Debug::DebugLogger logger;

SeparateGoal::SeparateGoal(){
    ros::NodeHandle nh;
   //获取group空间名
    std::string namespace_;
    namespace_ = nh.getNamespace();
    //获取group下的参数
    if(!nh.getParam(namespace_+"/car_id",car_id)){
        car_id = 1;
        logger.WARNINFO(car_id,"SEPARATE SERVICE FAILED TO GET CAR ID");
        logger.WARNINFO(car_id,"SEPARATE SERVICE RESET CAR ID TO 1");
    }
    if(!nh.getParam(namespace_+"/tf_ns",tf_ns)){
        logger.WARNINFO(car_id,"SEPARATE SERVICE FAILED TO GET TF FRAME");
    }
    robots_state_sub = nh.subscribe("robot_states",10,&SeparateGoal::RobotStateCallback,this);
    map_sub = nh.subscribe("/map",10,&SeparateGoal::MapCallback,this);
    separate_service = nh.advertiseService("separate_goal",&SeparateGoal::CalGoal,this);
    //Debug info
    logger.init_logger(car_id);

}

bool SeparateGoal::CalGoal(robot_msgs::Separate::Request &req,
                           robot_msgs::Separate::Response &res){
    goal_point = req.goal;  
    logger.DEBUGINFO(car_id,"separate start!!");
    int x_flag{0}, y_flag{0};
    //确定需要进行散点的车辆数
    int online_car{1};
    my_pose = GetMyPose();
    logger.DEBUGINFO(car_id,"get my pose!!");
    //统计在线车辆，并判定散点方式
    for(auto it : robots_info){
        if(it.car_id == 0){
            continue;
        }
        if(std::find(req.idList.begin(),req.idList.end(),it.car_id) == req.idList.end() ){
            continue ;
        }
        online_car++;
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


//point should be in meters not pixels
bool SeparateGoal::IsOccupied(geometry_msgs::Point p){
    // p.x = p.x / map.info.resolution;
    // p.y = p.y / map.info.resolution;
    // return map.data[map.info.width*p.x + p.y];
}

/**
 * 获取其它机器人位姿 
 */
void RobotInfo::SetState(const robot_msgs::RobotState state){
    car_id = state.car_id;
    robot_pose = state.robot_pose;
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

/** 获取自己的位姿
 * 
 * */
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

/** 
 * 获取地图信息 用于获取障碍物，目前没用 
 * */

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
