#include "ros/ros.h"
#include "my_debug_info.h"
#include "actionlib/client/simple_action_client.h"
#include "robot_msgs/SearchAction.h"
#include "geometry_msgs/PointStamped.h"

Debug::DebugLogger logger;

class TaskRealize{
public:
    TaskRealize(std::string ns);
    void Search();
    void Search_FeedbackCallback(const robot_msgs::SearchFeedbackConstPtr &feedback);
    void Search_DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::SearchResultConstPtr &result);
    void Search_ActiveCallback(void);
    void OnNewPoint(const geometry_msgs::PointStampedConstPtr &point);

    actionlib::SimpleActionClient<robot_msgs::SearchAction>*  search_action_client;
    ros::Subscriber point_sub;
    robot_msgs::SearchGoal search_goal;
    ros::NodeHandle nh;
};


TaskRealize::TaskRealize(std::string ns){
    search_action_client = new actionlib::SimpleActionClient<robot_msgs::SearchAction>(nh,ns+"search_action",true);
    logger.init_logger(1);
    search_goal.idList.push_back(1);
    search_goal.idList.push_back(2);
    point_sub = nh.subscribe("/clicked_point",10,&TaskRealize::OnNewPoint,this);
}




void TaskRealize::Search()
{
    logger.DEBUGINFO("waiting for search server !!!");
    search_action_client->waitForServer();
    search_action_client->sendGoal(   search_goal,
                                boost::bind(&TaskRealize::Search_DoneCallback,this,_1,_2),
                                boost::bind(&TaskRealize::Search_ActiveCallback,this),
                                boost::bind(&TaskRealize::Search_FeedbackCallback,this,_1));
    logger.DEBUGINFO("search task send goal!!");
}

void TaskRealize::Search_FeedbackCallback(const robot_msgs::SearchFeedbackConstPtr &feedback){
    if(feedback->error_occured)
        logger.WARNINFO("task error!!");
    return ;
}

void TaskRealize::Search_ActiveCallback(void){
    return ;
}

void TaskRealize::Search_DoneCallback(const actionlib::SimpleClientGoalState &state, const robot_msgs::SearchResultConstPtr &result){
    logger.DEBUGINFO("Search Done");
    return ;
}

void TaskRealize::OnNewPoint(const geometry_msgs::PointStampedConstPtr &point){
    if(search_goal.area.size() >= 4){
        search_goal.area.clear();
    }
    geometry_msgs::PoseStamped pose;
    pose.header = point->header;
    pose.pose.position.x = point->point.x;
    pose.pose.position.y = point->point.y;
    pose.pose.position.z = point->point.z;
    search_goal.area.push_back(pose);
    logger.DEBUGINFO("get new point!!!");
}

int main(int argc,char** argv){
    ros::init(argc,argv,"test_search");
    ros::NodeHandle nh;
    TaskRealize task_realize1("robot_0/");
    TaskRealize task_realize2("robot_1/");
    TaskRealize task_realize3("robot_2/");
    TaskRealize task_realize4("robot_3/");
    ros::Rate loop(20);
    while(ros::ok()){
        if(task_realize1.search_goal.area.size() == 4){
            task_realize1.Search();
            task_realize2.Search();
            // task_realize3.Search();
            // task_realize4.Search();
            task_realize1.search_goal.area.clear();
            task_realize2.search_goal.area.clear();
            // task_realize3.search_goal.area.clear();
            // task_realize4.search_goal.area.clear();
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;

}