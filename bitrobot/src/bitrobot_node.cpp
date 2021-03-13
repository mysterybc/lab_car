#include <cstdio>
#include <BIT.h>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include "ros/package.h"


inline double m2cm(double x) { return x * 100; }
inline double cm2m(double x) { return x / 100; }
inline double deg2rad(double x) { return x / 180 * 3.141592654; }
inline double rad2deg(double x) { return x * 180 / 3.141592654; }


struct ConfigBIT {
	int robotID = 1;
	std::string config_dir;
	std::string log_dir;
	int debug_info = DEBUG_INFO_STATES | DEBUG_INFO_FUNCTION;
	
	float target_velocity = 0.3;
};
struct StateBIT {
	StateInfo me;
	std::string my_message;
	std::map<int, StateInfo> others;
	std::map<int, std::string> message;
	
	bool new_path = false;
	std::vector<PathPoint> path;
	
	std::vector<ObstacleInfo> obstacles;
};

ConfigBIT myconfig;
StateBIT  mydata;



void init_controller(const ConfigBIT& config, StateBIT& state) {
	puts("Initializing BIT Controller");
	ControllerSetConfigDir(config.config_dir.c_str());
	ControllerSetLogDir(config.log_dir.c_str());
	ControllerInit(EXP_GENERAL, config.robotID);
	ControllerSetDebugInfo(config.debug_info, 1);
	
	state.me.ID = config.robotID;
	state.me.x = 0;
	state.me.y = 0;
	state.me.heading = 0;
	state.me.v = 0;
	state.me.w = 0;
}


void on_new_pos(const nav_msgs::Odometry& state) {
	auto pos = state.pose.pose.position;	
	tf::Quaternion q(
		state.pose.pose.orientation.x,
		state.pose.pose.orientation.y,
		state.pose.pose.orientation.z,
		state.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	
	mydata.me.ID = myconfig.robotID;
	mydata.me.x = (float)m2cm(pos.x);
	mydata.me.y = (float)m2cm(pos.y);
	mydata.me.heading = (float)rad2deg(yaw);
	mydata.me.v = 0;  // TBD
	mydata.me.w = 0;  // TBD
	
	ROS_DEBUG("Get new state (x, y, z): %.2f, %.2f, %.2f\n", mydata.me.x, mydata.me.y, mydata.me.heading);
}

void on_new_goal(const geometry_msgs::PoseStamped& goal) {
	auto pos = goal.pose.position;
	
	mydata.path.clear();  // currently, only one point
	PathPoint me, target;
	me.x = mydata.me.x;
	me.y = mydata.me.y;
	me.v = m2cm(myconfig.target_velocity);
	target.x = m2cm(pos.x);
	target.y = m2cm(pos.y);
	target.v = m2cm(myconfig.target_velocity);
	mydata.path = {me, target};
	mydata.new_path = true;
	
	
	ROS_INFO("Get new goal (x, y, z): %.2f, %.2f, %.2f\n", pos.x, pos.y, pos.z);
}

void on_new_scan(const sensor_msgs::LaserScan& scan) {
	auto& data = scan.ranges;
	int num = (int)data.size();
	
	double range_max = std::min(scan.range_max, 2.0f);
	double thMe = deg2rad(mydata.me.heading);
	mydata.obstacles.clear();
	for (int i = 0; i < num; ++i) {
		float v = data[i];
		if (v >= scan.range_min && v <= range_max) {
			double th = thMe + scan.angle_min + scan.angle_increment * i;
			double cc = std::cos(th);
			double ss = std::sin(th);
			double len = m2cm(v);	
			ObstacleInfo one;
			one.x = float(mydata.me.x + cc * len);  
			one.y = float(mydata.me.y + ss * len);  
			one.radius = 50;      // Fixed as 10 cm	
			mydata.obstacles.push_back(one);	
		}
	}
}



// Program Usage
// 
//

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "bitctrl");
	ros::NodeHandle node;
	
	// Channge the config_dir to a path containing simpleConfig.txt
	
	std::string package_path = ros::package::getPath("robot_library");
	myconfig.config_dir = package_path + "/bitrobot/config";
	init_controller(myconfig, mydata);  // If it fails, it will crash.
										// So, don't worry.
	
	ros::Subscriber pos_sub = node.subscribe("odom", 100, &on_new_pos);
	ros::Subscriber goal_sub = node.subscribe("move_base_simple/goal", 10, &on_new_goal);
	ros::Subscriber scan_sub = node.subscribe("base_scan", 2, &on_new_scan);
	ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	

	TimeInfo tm;
	tm.timeElapsed = 0;
	tm.loopCounter = 0;
	tm.globalTime  = 0;
	
	std::vector<StateInfo> others;
	
	printf("Entering Main Loop\n");
	ros::Rate loop_rate(20);
	while (node.ok()) {
		//printf("Loop %d starts.\n", tm.loopCounter);
	
		// Check for new tasks
		if (mydata.new_path) {
			ROS_INFO("Starting a new path following task\n");
			ControllerSetPath(mydata.path.size(), mydata.path.data());
			ControllerSetFunction(FUNC_TRACKING_SINGLE, 1);
			mydata.new_path = false;
		}
		
		// Handle Information from other robots
		for (auto& one: mydata.message) {
			int srcID = one.first;
			ControllerHandleMsg(srcID, (void*)one.second.c_str(), one.second.size());
		}
		mydata.message.clear();  // Clear handled messages
		
		// Fill the information of other robots
		others.clear();
		for (auto& one: mydata.others) others.push_back(one.second);
		
		// Setup Obstacles
		ControllerSetObstacles(mydata.obstacles.size(), mydata.obstacles.data());
		
		// Compute Data
		ControlInfo u;
		u = ControllerCompute(tm, mydata.me, others.size(), others.data());
		
		
		std::string msg_out(80, '\0');             // Always reserver 80bits for algo message
		ControllerGetMsg((void*)msg_out.c_str());     // msg_out should be broadcast to other robots
		
		
		// Convert to Published Data
		geometry_msgs::Twist u_pub;
		u_pub.linear.x = cm2m(u.v);
		u_pub.angular.z = deg2rad(u.w);  // is this yaw?
		cmd_pub.publish(u_pub);
	
		// Update
		tm.timeElapsed += 50;
		tm.loopCounter += 1;
		tm.globalTime += 50;
		
		//printf("Loop %d ends.\n", tm.loopCounter - 1);
	
		ros::spinOnce();
		loop_rate.sleep();
	}	
	ControllerStop();
	return 0;
}
