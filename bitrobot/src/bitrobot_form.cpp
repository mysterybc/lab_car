#include <cstdio>
#include <BIT.h>
#include <ssnet/proto/proto_bit.hpp>
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
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <algorithm>


inline double m2cm(double x) { return x * 100; }
inline double cm2m(double x) { return x / 100; }
inline double deg2rad(double x) { return x / 180 * 3.141592654; }
inline double rad2deg(double x) { return x * 180 / 3.141592654; }

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


struct RobotHandler {
	void subscribe(ros::NodeHandle& nh, const std::string& base) {
		sub_state = nh.subscribe(base + "base_pose_ground_truth",    10,  &RobotHandler::on_new_pos, this);
		sub_msg   = nh.subscribe(base + "algomsg", 10,  &RobotHandler::on_new_msg, this);
	}

	void on_new_pos(const nav_msgs::Odometry& state);
	void on_new_msg(const std_msgs::UInt8MultiArray& str);
	
	
	int robotID;
	IsNew<StateInfo> state;
	IsNew<std::string> msg;
	
	ros::Subscriber sub_state;
	ros::Subscriber sub_msg;
};


struct ConfigBIT {
	int robotID = 1;
	std::string config_dir;
	std::string log_dir;
	int debug_info = DEBUG_INFO_STATES | DEBUG_INFO_FUNCTION;
	
	float target_velocity = 0.3f;
	
	std::map<int, std::string> id2ns;
	std::vector<int> idlist, idform;
	std::vector<float> dx, dy;
	float edge_scaling = 1.6f;
};
struct StateBIT {
	StateInfo mean(const std::vector<int>& id_list);

	StateInfo me;
	std::string my_message;
	
	std::map<int, RobotHandler> others;
	
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



StateInfo odm2stateinfo(int robotID, const nav_msgs::Odometry& state) {
	auto pos = state.pose.pose.position;	
	tf::Quaternion q(
		state.pose.pose.orientation.x,
		state.pose.pose.orientation.y,
		state.pose.pose.orientation.z,
		state.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);


	StateInfo one;
	one.ID = robotID;
	one.x = (float)m2cm(pos.x);
	one.y = (float)m2cm(pos.y);
	one.heading = (float)rad2deg(yaw);
	one.v = 0;  // TBD
	one.w = 0;  // TBD
	return one;
}
std::string multiarr2algomsg(const std_msgs::UInt8MultiArray& msg) {
	std::string data(80, '\0');
	if (msg.layout.dim.size() == 1) {
		unsigned len = msg.layout.dim[0].size;
		unsigned offset = msg.layout.data_offset;
		for (unsigned i =0;i<len && i< 80;++i) {
			data[i] = (char)msg.data[offset + i];
		}
	}
	return data;
}
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

void on_new_pos(const nav_msgs::Odometry& state) {
	mydata.me = odm2stateinfo(myconfig.robotID, state);
	ROS_DEBUG("Get new state (x, y, z): %.2f, %.2f, %.2f\n", mydata.me.x, mydata.me.y, mydata.me.heading);
}
void RobotHandler::on_new_pos(const nav_msgs::Odometry& state) {
	this->state.update(odm2stateinfo(robotID, state));
}
void RobotHandler::on_new_msg(const std_msgs::UInt8MultiArray& data) {
	this->msg.update(multiarr2algomsg(data));
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
			auto it = others.find(id);
			if (it != others.end()) {
				StateInfo& q = it->second.state();
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

void on_new_goal(const geometry_msgs::PoseStamped& goal) {
	auto pos = goal.pose.position;
	
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
	// -----------------------
	// Hard coded Parameters
	// ----------------------- 
	myconfig.config_dir = "/home/lovezy/packs/repo-v0.0.4/config";
	myconfig.debug_info = DEBUG_INFO_STATES | DEBUG_INFO_FUNCTION | DEBUG_INFO_COMPUTE;
	myconfig.target_velocity = 0.3; // m/s
	myconfig.id2ns.clear();
	myconfig.id2ns[1] = "robot_0/";
	myconfig.id2ns[2] = "robot_1/";
	myconfig.id2ns[3] = "robot_2/";
	myconfig.id2ns[4] = "robot_3/";
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
	ros::NodeHandle node, nh("~");
	if (!nh.getParam("myid", myID)) {
		ROS_WARN("Failed to get myid. quit");
		return -1;
	}
	printf("This is Robot %d\n", myID);
	
	
	// ----------------------- 
	// Initializing PUB/SUB
	// ----------------------- 
	ros::Subscriber goal_sub = node.subscribe("formation_goal", 2, &on_new_goal);
	ros::Subscriber pos_sub  = node.subscribe(myconfig.id2ns[myID] + "base_pose_ground_truth", 100, &on_new_pos);
	ros::Subscriber scan_sub = node.subscribe(myconfig.id2ns[myID] + "base_scan", 2, &on_new_scan);
	ros::Publisher  cmd_pub  = node.advertise<geometry_msgs::Twist>(myconfig.id2ns[myID] + "cmd_vel", 1);
	ros::Publisher  msg_pub  = node.advertise<std_msgs::UInt8MultiArray>(myconfig.id2ns[myID] + "algomsg", 10);

	// Subscribe to data of the neighbours
	for (int i=0; i < nrobot; ++i) {
		int id = myconfig.idlist[i];
		if (id == myID) continue;
		
		auto& one = mydata.others[i];
		one.robotID = id;
		one.subscribe(node, myconfig.id2ns[id]);
	}
	


	// ----------------------- 
	// Initializing Controller
	// ----------------------- 
	myconfig.robotID = myID;
	init_controller(myconfig, mydata);  // If it fails, it will crash.
										// So, don't worry.

	TimeInfo tm;
	tm.timeElapsed = 0;
	tm.loopCounter = 0;
	tm.globalTime  = 0;
	
	// Temporary Variable
	std::vector<StateInfo> others;
	
	printf("Entering Main Loop\n");

	ros::Rate loop_rate(20);
	while (node.ok()) {
		//printf("Loop %d starts.\n", tm.loopCounter);
	
		// Check for new tasks
		if (mydata.new_path) {
			ROS_INFO("Starting a new path following task\n");
			
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
		for (auto& one: mydata.others) {
			int srcID = one.first;
			auto& rec = one.second;
			if (rec.msg.is_new) {
				ControllerHandleMsg(srcID, (void*)rec.msg().c_str(), rec.msg().size());
				rec.msg.update();	// Flag is_new as false
			}
		}
		
		// Fill the information of other robots
		// also compute the center of all robots
		others.clear();
		for (auto& one: mydata.others) {
			int srcID = one.first;
			auto& rec = one.second;
			if (rec.state.is_new) {
				others.push_back(rec.state());
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
		algomsg2multiarr(msg_out, arr);
		msg_pub.publish(arr);

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
