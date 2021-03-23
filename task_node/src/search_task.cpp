#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "robot_msgs/SearchAction.h"
#include "robot_msgs/PathFollowAction.h"
#include "robot_msgs/PathCoverage.h"
#include "robot_msgs/RobotStates.h"
#include "my_debug_info.h"
#include "tf/transform_listener.h"
#include "my_param_server.h"
#include "robot_msgs/SeparateArea.h"
#include "nav_msgs/Odometry.h"


Debug::DebugLogger logger;
class RobotInfo{
public:
    RobotInfo():car_id(0){};
    void SetState(const robot_msgs::RobotState state){
        car_id = state.car_id;
        robot_pose = state.robot_pose;
    }
    geometry_msgs::Pose robot_pose;
    int car_id;
};


/**
 * 这个类用于实现通用的actiob client
 * 假设action名字为xxxx
 * Action: xxxxAction
 * Goal: xxxxGoal
 * Result: xxxxResultConstPtr
 * Feedback: xxxxFeedbackConstPtr
 * ！！！在调用on_action之前，需要自己设定goal！！！
 */
template <typename Action,typename Goal,typename Result,typename Feedback>
class TaskClientRealize{
public:
    TaskClientRealize(std::string action_name_):goal_state(actionlib::SimpleClientGoalState::PENDING),action_name(action_name_){
        action_client = new actionlib::SimpleActionClient<Action>(nh,action_name_,true);
    }
    void on_action(){
        logger.DEBUGINFO("%s waiting for server !!!",action_name);
        action_client->waitForServer();
        action_client->sendGoal( action_goal,
                                        boost::bind(&TaskClientRealize::DoneCallback,this,_1,_2),
                                        boost::bind(&TaskClientRealize::ActiveCallback,this),
                                        boost::bind(&TaskClientRealize::FeedbackCallback,this,_1));
        goal_state = actionlib::SimpleClientGoalState::ACTIVE;
        logger.DEBUGINFO("%s task send goal!!",action_name);
    }
    void FeedbackCallback(const Feedback &feedback){
        if(feedback->error_occured){
            goal_state = actionlib::SimpleClientGoalState::ABORTED;
            logger.WARNINFO("task error!!");
        }
    }
    void DoneCallback(const actionlib::SimpleClientGoalState &state,const Result &result){goal_state = state;}
    void ActiveCallback(void){}

    actionlib::SimpleActionClient<Action>*  action_client;
    actionlib::SimpleClientGoalState goal_state;
    Goal action_goal;
    ros::NodeHandle nh;
    std::string action_name;
};


/**
 * 1.create search action server(search_action) (later)
 * 2.create path_coverage service client (path_coverage_client)
 * 3.add path_tracking action
 */
class SearchAction{
public:
    SearchAction();
    void RobotStateCallback(const robot_msgs::RobotStatesConstPtr &msg);
    void SearchExcuteCB(const robot_msgs::SearchGoalConstPtr &goal);
    geometry_msgs::Pose GetStartPoint(const std::vector<geometry_msgs::PoseStamped> &area);
    void OnNewPose(const nav_msgs::OdometryConstPtr &odom);

    int car_id;
    std::vector<RobotInfo> robots_info;
    ros::NodeHandle nh;
    std::string tf_ns;
    ros::Subscriber robot_pose_sub;
    ros::Subscriber robots_state_sub;
    ros::ServiceClient path_coverage_client;
    ros::ServiceClient separate_area_client;
    actionlib::SimpleActionServer<robot_msgs::SearchAction> search_server;
    TaskClientRealize<robot_msgs::PathFollowAction,robot_msgs::PathFollowGoal,
                      robot_msgs::PathFollowResultConstPtr,robot_msgs::PathFollowFeedbackConstPtr> path_follow_client;
    geometry_msgs::Pose robot_pose;
};

/**
 * 构造
 */
SearchAction::SearchAction():
    search_server(nh,"search_action",boost::bind(&SearchAction::SearchExcuteCB,this,_1),false),
    path_follow_client("PathFollow_action_laser")
{
    ros::NodeHandle nh;
    my_lib::GetParam("laser march task",&car_id,NULL,&tf_ns);
    robots_state_sub = nh.subscribe("robot_states",10,&SearchAction::RobotStateCallback,this);
    robot_pose_sub = nh.subscribe("odom",1,&SearchAction::OnNewPose,this);
    separate_area_client = nh.serviceClient<robot_msgs::SeparateArea>("separate_area");
    path_coverage_client = nh.serviceClient<robot_msgs::PathCoverage>("path_coverage");
    //Debug info
    logger.init_logger(car_id);
    search_server.start();
}

/**
 * 任务执行函数
 */
void SearchAction::SearchExcuteCB(const robot_msgs::SearchGoalConstPtr &goal){
    logger.DEBUGINFO(car_id,"get search goal");
    //首先对需要搜索的区域进行划分
    robot_msgs::SeparateArea new_goal;
    new_goal.request.area = goal->area;
    new_goal.request.idList = goal->idList;
    if(separate_area_client.call(new_goal)){
        logger.DEBUGINFO(car_id,"call divide area");
    }
    else{
        logger.DEBUGINFO(car_id,"fail to divide area");
        return ;
    }
    //call path coverage
    robot_msgs::PathCoverage path_coverage;
    if(new_goal.response.area.size() != 4){
        logger.WARNINFO(car_id,"area edge point size wrong!!");
    }
    for(auto point:new_goal.response.area){
        // logger.DEBUGINFO(car_id,"point is : %f %f",point.pose.position.x,point.pose.position.y);
        path_coverage.request.select_point.poses.push_back(point);
    }
    path_coverage.request.start_point = GetStartPoint(new_goal.response.area);
    logger.DEBUGINFO(car_id,"start point is : %f %f",path_coverage.request.start_point.position.x,path_coverage.request.start_point.position.y);
     while(!ros::service::waitForService("path_coverage",ros::Duration(1.0))){
        logger.DEBUGINFO(car_id,"waiting for service path coverage");
        ros::spinOnce();
    }
    if(path_coverage_client.call(path_coverage)){
        logger.DEBUGINFO(car_id,"call path coverage");
    }
    else{
        logger.DEBUGINFO(car_id,"fail to call path");
        return ;
    }
    // path_follow_client
    path_follow_client.action_goal.idList = goal->idList;
    path_follow_client.action_goal.path = path_coverage.response.path;
    path_follow_client.on_action();
    ros::Rate loop(20);
    robot_msgs::SearchResult result;
    while(ros::ok()){
        if(path_follow_client.goal_state == actionlib::SimpleClientGoalState::ABORTED){
            result.succeed = false;
            logger.DEBUGINFO(car_id,"search action failed!!!");
            search_server.setSucceeded(result,"failed");
            return;
        }
        if(path_follow_client.goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
            result.succeed = true;
            search_server.setSucceeded(result,"success");
            logger.DEBUGINFO(car_id,"search action success!!!");
            return;
        }
    }
}

/**
 * 判断机器人是否在探索区域内
 * 如果在，则机器人未知作为出发点
 * 如果不在，机器人所在位置最近的定点作为出发点
 */
geometry_msgs::Pose SearchAction::GetStartPoint(const std::vector<geometry_msgs::PoseStamped> &area){
    double max_x(-1e5),max_y(-1e5),min_x(1e5),min_y(1e5);
    for(auto point:area){
        max_x = std::max(point.pose.position.x,max_x);
        max_y = std::max(point.pose.position.y,max_y);
        min_x = std::min(point.pose.position.x,min_x);
        min_y = std::min(point.pose.position.y,min_y);
    }
    //如果机器人在区域内
    if(     robot_pose.position.x < max_x && robot_pose.position.x > max_x 
        &&  robot_pose.position.y < max_y && robot_pose.position.y > max_y ){
        return robot_pose;
    }
    //如果不在区域内
    else{
        double min_delta(1e5);
        geometry_msgs::Pose start_point;
        for(auto point:area){
            double delta_xy = fabs(point.pose.position.x-robot_pose.position.x) + fabs(point.pose.position.y-robot_pose.position.y);
            if(delta_xy < min_delta){
                start_point = point.pose;
                min_delta = delta_xy; 
            }
        }
        //起始点需要向内一些
        if(start_point.position.x > (min_x+max_x)/2){
            start_point.position.x -= 0.5;
        }else{
            start_point.position.x += 0.5;
        }
        if(start_point.position.y > (min_y+max_y)/2){
            start_point.position.y -= 0.5;
        }else{
            start_point.position.y += 0.5;
        }
        return start_point;
    }
    
}

void SearchAction::RobotStateCallback(const robot_msgs::RobotStatesConstPtr &msg){
    while(msg->online_robot_number > robots_info.size()){
        RobotInfo robot_info;
        robots_info.push_back(robot_info);
    }
    for(int i = 0; i < msg->online_robot_number; i++){
        robots_info[i].SetState(msg->robot_states[i]);
    }
}

void SearchAction::OnNewPose(const nav_msgs::OdometryConstPtr &odom){
    robot_pose = odom->pose.pose;
}


int main(int argc,char **argv){
    ros::init(argc,argv,"test_pathcoverage");
    SearchAction search_action;
    ros::Rate loop(20);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}