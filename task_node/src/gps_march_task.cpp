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
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "robot_msgs/MarchAction.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <algorithm>
#include "ros/package.h"
#include "robot_msgs/RobotStates.h"
#include "tf/transform_listener.h"
#include "robot_msgs/BuildUpAction.h"
#include "my_debug_info.h"
#include "time.h"
#include "sensor_msgs/Imu.h"
#include "my_param_server.h"



inline double m2cm(double x) { return x * 100; }
inline double cm2m(double x) { return x / 100; }
inline double deg2rad(double x) { return x / 180 * 3.141592654; }
inline double rad2deg(double x) { return x * 180 / 3.141592654; }


//sub and pubs
ros::Subscriber pos_sub;
ros::Subscriber goal_sub;
ros::Subscriber scan_sub;
ros::Publisher cmd_pub;
ros::Publisher  msg_pub;
Debug::DebugLogger logger;

std::string robot_frame;

struct ActionConfig{
	ActionConfig(ros::NodeHandle &node):
		march_action(node,"march_action_gps",boost::bind(&ActionConfig::on_new_action,this,_1),false),
		buildup_action(node,"build_up_action",true)
	{
		march_action.registerPreemptCallback(boost::bind(&ActionConfig::cancel_action_request,this));
		march_action.start();
		tm.timeElapsed = 0;
		tm.loopCounter = 0;
		tm.globalTime  = 0;
	}
	int run_march_action();
	void on_new_action(const robot_msgs::MarchGoalConstPtr &goal);
	void cancel_action_request();
	void config_controller(const robot_msgs::MarchGoalConstPtr &goal);
	actionlib::SimpleActionServer<robot_msgs::MarchAction> march_action;
	actionlib::SimpleActionClient<robot_msgs::BuildUpAction> buildup_action;
	TimeInfo tm;
};


// Should use std::optional, but....
template<class T>
struct IsNew {
	IsNew() {}
	IsNew(const T& val): data(val) {}
	T& operator = (const T& val) { data = val; return data; }
	T& operator () () { return data; }
	const T& operator () () const { return data; }
	
	void update(const T& value) { data = value; is_new = true; }
	void update() { is_new = false; }	
	
	bool is_new = false;
	T data;
};

//读取其他车的状态
struct RobotHandler {
	void subscribe(ros::NodeHandle& nh) {
		sub_state = nh.subscribe("robot_states",    10,  &RobotHandler::on_new_pos, this);
		sub_msg   = nh.subscribe("algomsg_others", 10,  &RobotHandler::on_new_msg, this);
	}

	void on_new_pos(const robot_msgs::RobotStatesConstPtr& states);
	void on_new_msg(const std_msgs::UInt8MultiArray& str);
	
	
	int robotID;
	std::map<int,IsNew<StateInfo> > id2state;
	std::map<int,IsNew<std::string> > id2msg;
	
	ros::Subscriber sub_state;
	ros::Subscriber sub_msg;
};


struct ConfigBIT {
	int robotID = 1;
	std::string config_dir;
	std::string log_dir;
	int debug_info = DEBUG_INFO_STATES | DEBUG_INFO_FUNCTION;
	
	float target_velocity = 0.3f;
	
	std::vector<int> idlist, idform;
	std::vector<float> dx, dy;
	float edge_scaling = 1.6f;
};
struct StateBIT {
	StateInfo mean(const std::vector<int>& id_list);

	StateInfo me;
	std::string my_message;
	
	RobotHandler others;
	
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
	//ControllerSetDebugInfo(config.debug_info, 1);
	ControllerSetDebugInfo(DEBUG_INFO_ALL, 0);

	ControllerSetDebugInfo(DEBUG_INFO_STATES, 0);
	ControllerSetDebugInfo(DEBUG_INFO_COMPUTE, 0);
	ControllerSetDebugInfo(DEBUG_INFO_FUNCTION, 0);
	ControllerSetDebugInfo(DEBUG_INFO_COM_SEND, 0);
	ControllerSetDebugInfo(DEBUG_INFO_COM_RECV, 0);
	// printf("Turn off Debug Info99999!!!\n");

	state.me.ID = config.robotID;
	state.me.x = 0;
	state.me.y = 0;
	state.me.heading = 0;
	state.me.v = 0;
	state.me.w = 0;
}


//read pos from msg
StateInfo pose2stateinfo(const robot_msgs::RobotState &state) {
	auto pos = state.robot_pose;	
	tf::Quaternion q(
		pos.orientation.x,
		pos.orientation.y,
		pos.orientation.z,
		pos.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);


	StateInfo one;
	one.ID = state.car_id;
	one.x = (float)m2cm(pos.position.x);
	one.y = (float)m2cm(pos.position.y);
	one.heading = (float)rad2deg(yaw);
	one.v = 0;  // TBD
	one.w = 0;  // TBD
	//printf("robot %d - x y yaw is %lf %lf %lf\n",one.ID,one.x,one.y,one.heading);
	return one;
}

//transform array 2 algomsg
std::string multiarr2algomsg(const std_msgs::UInt8MultiArray& msg) {
	std::string data(80, '\0');
	if (msg.layout.dim.size() == 1) {
		unsigned len = msg.layout.dim[0].size;
		unsigned offset = msg.layout.data_offset;
		//total 81 byte = id + 80 byte msg 
		for (unsigned i =0;i<len && i< 80;++i) {
			data[i] = (char)msg.data[offset + i + 1];
		}
	}
	return data;
}

//transform algomsg 2 array
void algomsg2multiarr(const std::string& msg, std_msgs::UInt8MultiArray& data) {
	data.layout.dim.resize(1);
	data.layout.data_offset = 0;
	data.layout.dim[0].size = msg.size();
	data.layout.dim[0].stride = 1;

	data.data.resize(msg.size());
	for (unsigned i=0;i<msg.size();++i) {
		data.data[i] = (uint8_t)msg[i];
	}
}

//use tf instead
// void on_new_pos(const nav_msgs::Odometry& state) {
// 	mydata.me = odm2stateinfo(myconfig.robotID, state);
// 	ROS_DEBUG("Get new state (x, y, z): %.2f, %.2f, %.2f\n", mydata.me.x, mydata.me.y, mydata.me.heading);
// }

//get other robot pos
void RobotHandler::on_new_pos(const robot_msgs::RobotStatesConstPtr& states) {
	for(auto robot_state : states->robot_states){
		int id = robot_state.car_id;
		this->id2state[robot_state.car_id].update(pose2stateinfo(robot_state));
	}
	//this->id2state.update(odm2stateinfo(robotID, state));
}

//get other robot algomsg
void RobotHandler::on_new_msg(const std_msgs::UInt8MultiArray& data) {
	int id = data.data[0];
	this->id2msg[id].update(multiarr2algomsg(data));
	//this->msg.update(multiarr2algomsg(data));
}

// Utility Function: Compute the average position of robots in id_list
StateInfo StateBIT::mean(const std::vector<int>& id_list) {
	StateInfo info;
	info.x = 0;
	info.y = 0;
	int nitem = 0;
	for (int id: id_list) {
		if (id == myconfig.robotID) {
			info.x += me.x;
			info.y += me.y;
			++nitem;
		}
		else {
			auto it = others.id2state.find(id);
			if (it != others.id2state.end()) {
				StateInfo& q = it->second();
				info.x += q.x;
				info.y += q.y;
				++nitem;
			}
		}
	}
	info.x /= nitem;
	info.y /= nitem;
	return info;
}

void on_new_goal(const geometry_msgs::Pose& goal) {
	auto pos = goal.position;
	
	mydata.path.clear();  // currently, only one point
	PathPoint start, target;
	StateInfo center = mydata.mean(myconfig.idform);
	start.x = center.x;
	start.y = center.y;
	start.v = m2cm(myconfig.target_velocity);
	target.x = m2cm(pos.x);
	target.y = m2cm(pos.y);
	target.v = m2cm(myconfig.target_velocity);
	mydata.path = {start, target};
	mydata.new_path = true;
	
	
	logger.DEBUGINFO(myconfig.robotID,"Get new goal  (x, y, z): %.2f, %.2f, %.2f\n", pos.x, pos.y, pos.z);
	logger.DEBUGINFO(myconfig.robotID,"start pose is (x, y): %.2f, %.2f\n", start.x, start.y);
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


void on_new_gps_pos(const nav_msgs::Odometry& msg){
	double yaw,roll,pitch;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg.pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getEulerYPR(yaw,pitch,roll);
    StateInfo one;
	one.ID = myconfig.robotID;
	one.x = (float)m2cm(msg.pose.pose.position.x);
	one.y = (float)m2cm(msg.pose.pose.position.y);
	one.heading = (float)rad2deg(yaw);
	one.v = 0;  // TBD
	one.w = 0;  // TBD
	mydata.me = one;
}


//bit ctrl
int ActionConfig::run_march_action(){


	// Temporary Variable
	std::vector<StateInfo> others;
	
	printf("Entering Main Loop\n");

	ros::NodeHandle node;
	ros::Rate loop_rate(20);
	while (node.ok()) {
		//printf("Loop %d starts.\n", tm.loopCounter);

		if(!march_action.isActive()){
			ControllerSetFunction(FUNC_ALL, 0);
			return -1;
		}
	
		// Check for new tasks
		if (mydata.new_path) {
			logger.DEBUGINFO(myconfig.robotID,"Starting a new path following task\n");
			

			ControllerSetFormationGroup(0);
			
			uint num = myconfig.idform.size();
			const int *pID = myconfig.idform.data();
			const float *pDx = myconfig.dx.data();
			const float *pDy = myconfig.dy.data();
			//ControllerSetFormationShape(num, pID, pDx, pDy, myconfig.edge_scaling);
			// or, leave pDx, pDy as nullptr, then will use polygon as the shape
			ControllerSetFormationShape(num, pID, nullptr, nullptr, myconfig.edge_scaling);

			ControllerSetPath(mydata.path.size(), mydata.path.data());

			ControllerSetFunction(FUNC_TRACKING_GROUP, 1);
			mydata.new_path = false;
		}
		
		// Handle Information from other robots
		for (auto& one: mydata.others.id2msg) {
			int srcID = one.first;
			auto& rec = one.second;
			if (rec.is_new) {
				ControllerHandleMsg(srcID, (void*)rec.data.c_str(), rec.data.size());
				rec.update();	// Flag is_new as false
			}
		}
		
		// Fill the information of other robots
		// also compute the center of all robots
		others.clear();
		for (auto& one: mydata.others.id2state) {
			int srcID = one.first;
			auto& rec = one.second;
			if (rec.is_new) {
				others.push_back(rec.data);
			}
		}
		
		// Setup Obstacles
		ControllerSetObstacles(mydata.obstacles.size(), mydata.obstacles.data());
		
		// Compute Data
		ControlInfo u;
		u = ControllerCompute(tm, mydata.me, others.size(), others.data());
		
		// Convert to Published Data
		geometry_msgs::Twist u_pub;
		u_pub.linear.x = cm2m(u.v);
		u_pub.angular.z = deg2rad(u.w);  // is this yaw?
		cmd_pub.publish(u_pub);
	
		// Send Message To Other Robots
		std::string msg_out(80, '\0');             // Always reserver 80bits for algo message
		ControllerGetMsg((void*)msg_out.c_str());     // msg_out should be broadcast to other robots
		std_msgs::UInt8MultiArray arr;
		//send msg with id(int not string)
		std::string msg_withid = " " + msg_out;
		msg_withid[0] = myconfig.robotID;
		algomsg2multiarr(msg_withid, arr);
		msg_pub.publish(arr);

		// Update
		tm.timeElapsed += 50;
		tm.loopCounter += 1;
		tm.globalTime += 50;
		
		//判断任务是否结束
		if(ControllerTaskProgress() >= 0.99){
			logger.DEBUGINFO(myconfig.robotID,"march task finish!");
			break;
		}
		//printf("Loop %d ends.\n", tm.loopCounter - 1);
	
		ros::spinOnce();
		loop_rate.sleep();
	}	
	ControllerSetFunction(FUNC_ALL, 0);
    return 0;
}

//action CB
void ActionConfig::on_new_action(const robot_msgs::MarchGoalConstPtr &goal){
    logger.DEBUGINFO(myconfig.robotID,"get new march goal!");
	config_controller(goal);
	on_new_goal(goal->goal);
    robot_msgs::MarchResult result;
    if(this->run_march_action()){
		if(!march_action.isActive()){
			return ;
		}
        result.succeed = false;
        march_action.setAborted(result,"action fail");
    }
    else{
			
		result.succeed = true;
		march_action.setAborted(result,"action success");
	}
}

void ActionConfig::cancel_action_request(){
	logger.DEBUGINFO(myconfig.robotID,"get cancel request!");
    if(march_action.isPreemptRequested()){
        robot_msgs::MarchResult result;
        result.succeed = 2;
        march_action.setPreempted(result,"goal cancel");
		buildup_action.cancelAllGoals();
		logger.DEBUGINFO(myconfig.robotID,"cancel cation!");
    }
}

void ActionConfig::config_controller(const robot_msgs::MarchGoalConstPtr &goal){
	myconfig.idlist.clear();
	myconfig.idform.clear();
	// mydata.others.id2msg.clear();
	// mydata.others.id2state.clear();
	for(auto number:goal->idList){
		myconfig.idlist.push_back(number);
		myconfig.idform.push_back(number);
	}
}



// Program Usage
// 
//

int main(int argc, char* argv[]) {
	// -----------------------
	// Hard coded Parameters
	// ----------------------- 
	std::string package_path = ros::package::getPath("robot_library");
	myconfig.config_dir = package_path + "/bitrobot/config";
	myconfig.debug_info = DEBUG_INFO_STATES | DEBUG_INFO_FUNCTION | DEBUG_INFO_COMPUTE;
	myconfig.target_velocity = 0.5; // m/s
	myconfig.idlist = {1, 2, 3, 4};  // These robots are all connected
	myconfig.idform = {1, 2, 3, 4};  // These robots will be in a formation
	myconfig.edge_scaling = 1.6;                // when not specifying dx, dy, default edge length = 1m (which is too small)
	myconfig.dx = {0.5, 0.5, -0.5, -0.5 };   // optional, meter
	myconfig.dy = {0.5, -0.5, -0.5, 0.5 };   // optional, meter

	int nrobot = (int)myconfig.idlist.size();
	int myID = -1;
	

	// ----------------------- 
	// Read Params
	// ----------------------- 
	ros::init(argc, argv, "bitform");
	ros::NodeHandle node;
    my_lib::GetParam("path_follow",&myID);
	myconfig.robotID = myID;
	printf("This is Robot %d\n", myID);
	logger.init_logger(myID);
	
	
	// ----------------------- 
	// Initializing PUB/SUB
	// ----------------------- 
	goal_sub = node.subscribe("formation_goal", 2, &on_new_goal);
	ros::Subscriber gps_pos_sub  = node.subscribe("gps_odom", 100, &on_new_gps_pos);
	scan_sub = node.subscribe("base_scan", 2, &on_new_scan);
	cmd_pub  = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	msg_pub  = node.advertise<std_msgs::UInt8MultiArray>("algomsg_my", 10);
	// Subscribe to data of the neighbours
	mydata.others.subscribe(node);
	


	// ----------------------- 
	// Initializing Controller
	// ----------------------- 
	
	// init_controller(myconfig, mydata);  // If it fails, it will crash.
	// 									// So, don't worry.

	ActionConfig marchconfig(node);
	init_controller(myconfig, mydata);
	ros::Rate loop(50);
	while(ros::ok()){
		loop.sleep();
		ros::spinOnce();
	}
	ControllerStop();

	
	return 0;
}
