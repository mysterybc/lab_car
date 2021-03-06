#ifndef _BEHAVIAC_BLACKBOARD_H_
#define _BEHAVIAC_BLACKBOARD_H_

#include "behaviac_headers.h"
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "robot_msgs/HostCmd.h"
#include "robot_msgs/HostCmdArray.h"
#include "robot_msgs/RobotStates.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "string"
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include "GroupAsBasicLogic.h"
#include "TaskRealize.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "robot_msgs/CurrentTask.h"
#include "algorithm"
#include "vector"
#include "my_debug_info.h"
#include<utility>


class GroupAsBasicLogic;
class TaskRealize;
extern GroupAsBasicLogic* g_GroupAsBasicLogicAgent;
extern TaskRealize* g_TaskRealizeAgent;
extern bool set_behavior_tree;
class BlackBoard : public behaviac::Agent
{
public:
	BlackBoard();
	virtual ~BlackBoard();

	BEHAVIAC_DECLARE_AGENTTYPE(BlackBoard, behaviac::Agent)

	public: int car_id;

    public:void BackgrdFuncProcessing(TaskIndividual backgrdfunc);

    public: void CmdCallback(const robot_msgs::HostCmdArrayConstPtr &msg);
    public: void GroupStateCallback(const robot_msgs::RobotStatesConstPtr &msg); 
	public: void TagDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg);
	public:	void PubDecisionState();
	public:	void PubMembers();
	public: void PubTagPose();
    public: std::vector<geometry_msgs::Pose> GetGoal();//每一个任务可能有多个点的goal
	public: std::vector<robot_msgs::HostCmd> msgs;//一次发布多个任务
    public: std::vector<robot_msgs::HostCmd> TaskList;
	public: std::vector<geometry_msgs::Pose> goal;
	public:  std::vector<geometry_msgs::Pose> tag_pose;
	public: std::vector<int32_t> tag_id;
	private: ros::Subscriber group_state_sub;
	private: ros::Subscriber cmd_sub;
	private: ros::Subscriber tag_detection_sub;
	private: ros::Publisher decision_state_pub;
	private: ros::Publisher members_pub;
	private: ros::Publisher tag_pose_pub;
    private: ros::Publisher current_task_pub;
	private: ros::Publisher tag_id_pub;
};

#endif
