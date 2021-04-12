#include "separate_goal.h"
#include "tf/transform_listener.h"
#include "algorithm"
#include "my_param_server.h"

Debug::DebugLogger logger;

SeparateGoal::SeparateGoal(){
    ros::NodeHandle nh;
    //获取group空间名
    my_lib::GetParam("path_follow",&car_id,NULL,&tf_ns);
    robots_pose_sub = nh.subscribe("odom",10,&SeparateGoal::OnNewPose,this);
    robots_state_sub = nh.subscribe("robot_states",10,&SeparateGoal::RobotStateCallback,this);
    map_sub = nh.subscribe("/map",10,&SeparateGoal::MapCallback,this);
    separate_goal_server = nh.advertiseService("separate_goal",&SeparateGoal::CalGoal,this);
    separate_area_server = nh.advertiseService("separate_area",&SeparateGoal::CalArea,this);
    //Debug info
    logger.init_logger(car_id);

}

bool SeparateGoal::CalArea( robot_msgs::SeparateArea::Request &req,
                            robot_msgs::SeparateArea::Response & res){
    logger.DEBUGINFO(car_id,"separate area start!!");
    int x_flag{0}, y_flag{0};
    //确定需要进行散点的车辆数
    int online_car{1};
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
    switch(online_car){
        case 1: res.area = req.area; break;
        case 2:
        {
            geometry_msgs::PoseStamped mid_pose1,mid_pose2;
            mid_pose1.pose.position.x = (req.area[0].pose.position.x+req.area[1].pose.position.x)/2;
            mid_pose1.pose.position.y = (req.area[0].pose.position.y+req.area[1].pose.position.y)/2;
            mid_pose2.pose.position.x = (req.area[2].pose.position.x+req.area[3].pose.position.x)/2;
            mid_pose2.pose.position.y = (req.area[2].pose.position.y+req.area[3].pose.position.y)/2;

            if(x_flag < 1){
                res.area.push_back(req.area[0]);
                res.area.push_back(mid_pose1);
                res.area.push_back(req.area[2]);
                res.area.push_back(mid_pose2);
            }else{
                res.area.push_back(mid_pose1);
                res.area.push_back(req.area[1]);
                res.area.push_back(mid_pose2);
                res.area.push_back(req.area[3]);
            }
            break;
        }
        case 3:
        {
            return false;
            break;
        }
        case 4:
        {
            geometry_msgs::PoseStamped mid_pose1,mid_pose2,mid_pose3,mid_pose4,mid_pose;
            mid_pose1.pose.position.x = (req.area[0].pose.position.x+req.area[1].pose.position.x)/2;
            mid_pose1.pose.position.y = (req.area[0].pose.position.y+req.area[1].pose.position.y)/2;
            mid_pose2.pose.position.x = (req.area[2].pose.position.x+req.area[3].pose.position.x)/2;
            mid_pose2.pose.position.y = (req.area[2].pose.position.y+req.area[3].pose.position.y)/2;
            mid_pose3.pose.position.x = (req.area[0].pose.position.x+req.area[2].pose.position.x)/2;
            mid_pose3.pose.position.y = (req.area[0].pose.position.y+req.area[2].pose.position.y)/2;
            mid_pose4.pose.position.x = (req.area[1].pose.position.x+req.area[3].pose.position.x)/2;
            mid_pose4.pose.position.y = (req.area[1].pose.position.y+req.area[3].pose.position.y)/2;
            mid_pose.pose.position.x = (req.area[0].pose.position.x+req.area[3].pose.position.x)/2;
            mid_pose.pose.position.y = (req.area[0].pose.position.y+req.area[3].pose.position.y)/2;
            if(x_flag < 2){
                if(y_flag >= 2){
                    res.area.push_back(req.area[0]);
                    res.area.push_back(mid_pose1);
                    res.area.push_back(mid_pose3);
                    res.area.push_back(mid_pose);
                }
                else{
                    res.area.push_back(mid_pose3);
                    res.area.push_back(mid_pose);
                    res.area.push_back(req.area[2]);
                    res.area.push_back(mid_pose2);
                }

            }else{
                if(y_flag >= 2){
                    res.area.push_back(mid_pose1);
                    res.area.push_back(req.area[1]);
                    res.area.push_back(mid_pose);
                    res.area.push_back(mid_pose4);
                }
                else{
                    res.area.push_back(mid_pose);
                    res.area.push_back(mid_pose4);
                    res.area.push_back(mid_pose2);
                    res.area.push_back(req.area[3]);
                }   
                
            }
            break;
        }
    }
    return true;

}

bool SeparateGoal::CalGoal(robot_msgs::Separate::Request &req,
                           robot_msgs::Separate::Response &res){
    goal_point = req.goal;  
    logger.DEBUGINFO(car_id,"separate start!!");
    int x_flag{0}, y_flag{0};
    //确定需要进行散点的车辆数
    int online_car{1};
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
void SeparateGoal::OnNewPose(const nav_msgs::OdometryConstPtr &msg){
    my_pose = msg->pose.pose;
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
