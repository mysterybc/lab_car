//c++ standard
#include "map"
//third party lib
#include "jsoncpp/json/json.h"
//ros
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include "robot_msgs/ReportPath.h"
#include "std_msgs/Int8MultiArray.h"
#include "iostream"
#include "robot_msgs/robot_states_enum.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
//my lib
#include "zmq_lib.h"
#include "msgs.h"
#include "my_debug_info.h"
#include "my_param_server.h"

Debug::DebugLogger logger;
//通过tf寻找机器人位置
void getPose(std::string map_frame,std::string robot_frame,StateMsg &state_msg ){
    tf::TransformListener listener;
    tf::StampedTransform trans;
    while(ros::ok){
        try{
            listener.lookupTransform(map_frame,robot_frame,ros::Time(0),trans);
        }
        catch(tf::TransformException){
            ros::Duration(0.1).sleep();
            continue;
        }
        break;
    }
    tf::Quaternion quat = trans.getRotation();
    double roll,yaw,pitch;
    tf::Matrix3x3(quat).getEulerYPR(yaw,pitch,roll);
    yaw = yaw / 3.1415926 * 180;
    state_msg.x = trans.getOrigin().x();
    state_msg.y = trans.getOrigin().y();
    state_msg.yaw = yaw;
}


//service 上报路径
bool ReportPath(robot_msgs::ReportPath::Request  &req,
                robot_msgs::ReportPath::Response &res,
                PathMsg &path_msg, zmq_lib::Sender &path_sender){
    std::string msg = path_msg.fmtMsg(req.Path);
    path_sender.sendMsg(msg);
    res.cmd.data = true;
    return true;
}

//接收组员消息
void GroupSubCB(const std_msgs::Int8MultiArrayConstPtr &msg, StateMsg &state_msg){
    state_msg.group.clear();
    for(auto data:msg->data){
        state_msg.group.push_back(data);
    }
}

//接收状态信息
void DecisionSubCB(const robot_msgs::robot_states_enumConstPtr &msg, StateMsg &state_msg){
    state_msg.state = msg->robot_states_enum;
}

//send algo msg
void OnNewAlgomsg(const std_msgs::UInt8MultiArrayConstPtr &msg, zmq_lib::Sender &sender){
    std::string send_msg = multiArray2json(msg);
    sender.sendMsg(send_msg);
}

void OnNewRobotPose(const nav_msgs::OdometryConstPtr &msg,RobotStateMsg& robot_state_msg){
    static double last_vx{0},last_vy{0},last_w{0};
    robot_state_msg.vx = msg->twist.twist.linear.x;
    robot_state_msg.vy = msg->twist.twist.linear.y;
    robot_state_msg.w = msg->twist.twist.angular.z;
    
    //acc 目前是假的
    if(robot_state_msg.vx - last_vx > 0.05){
        robot_state_msg.acc_x = 0.1;
    }
    else if(robot_state_msg.vx - last_vx < -0.05){
        robot_state_msg.acc_x = 0.1;
    }
    else{
        robot_state_msg.acc_x = 0;
    }

    if(robot_state_msg.vy - last_vy > 0.05){
        robot_state_msg.acc_y = 0.1;
    }
    else if(robot_state_msg.vy - last_vy < -0.05){
        robot_state_msg.acc_y = 0.1;
    }
    else{
        robot_state_msg.acc_y = 0;
    }


    if(robot_state_msg.w - last_w > 0.05){
        robot_state_msg.acc_yaw = 0.1;
    }
    else if(robot_state_msg.w - last_w < -0.05){
        robot_state_msg.acc_yaw = 0.1;
    }
    else{
        robot_state_msg.acc_yaw = 0;
    }

    last_vx = robot_state_msg.vx;
    last_vy = robot_state_msg.vy;
    last_w = robot_state_msg.w;

}

void OnNewTask(const robot_msgs::CurrentTaskConstPtr &msg, RobotTaskMsg& robot_task_msg){
    robot_task_msg.current_task_int = msg->current_task;
    
}
void OnNewScan(const sensor_msgs::LaserScanConstPtr &msg, RobotPerceptionMsg& robot_perception_msg){
    robot_perception_msg.points.clear();
    double pi = 3.1415926;
    double angle = -3.1415927;
    double angle_inc = 0.0175;
    for(auto range:msg->ranges){
        if(range <= 3){
            Point point;
            point.y = sin(pi-angle)*range;
            point.x = cos(pi-angle)*range;
            robot_perception_msg.points.push_back(point);
        }
        angle += angle_inc;
    }
}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"publish_robot_state");
    ros::NodeHandle nh;
    std::string ip_address;

    //config id & ip
    int car_id;
    std::string tf_frame;
    my_lib::GetParam("publish_robot_state",&car_id,NULL,&tf_frame,&ip_address,NULL,NULL,NULL);

    //zmq_init
    zmq::context_t ctx(1);
    zmq_lib::Sender sender(ip_address,ctx);



    //message init
    StateMsg state_msg(car_id);
    PathMsg path_msg(car_id);
    //TODO 完成这几个
    RobotStateMsg robot_state_msg(car_id);
    RobotTaskMsg robot_task_msg(car_id);
    RobotPerceptionMsg robot_perception_msg(car_id);


    //sub 
    ros::Subscriber state_subs;
    ros::Subscriber group_subs;
    ros::Subscriber algomsg_subs;
    ros::ServiceServer report_path_server;
    ros::Subscriber robot_pose_sub;
    ros::Subscriber robot_task_sub;
    ros::Subscriber laser_sub;
    report_path_server = nh.advertiseService<robot_msgs::ReportPath::Request,robot_msgs::ReportPath::Response>\
                            ("report_path",boost::bind(&ReportPath,_1,_2,std::ref(path_msg),std::ref(sender)));
    state_subs = nh.subscribe<robot_msgs::robot_states_enum>("decision_state",10,boost::bind(&DecisionSubCB,_1,std::ref(state_msg)));
    group_subs = nh.subscribe<std_msgs::Int8MultiArray>("my_group_member",10,boost::bind(&GroupSubCB,_1,std::ref(state_msg)));
    algomsg_subs = nh.subscribe<std_msgs::UInt8MultiArray>("algomsg_my",10,boost::bind(&OnNewAlgomsg,_1,std::ref(sender)));
    robot_pose_sub = nh.subscribe<nav_msgs::Odometry>("odom",1,boost::bind(OnNewRobotPose,_1,std::ref(robot_state_msg)));
    robot_task_sub = nh.subscribe<robot_msgs::CurrentTask>("current_task",1,boost::bind(OnNewTask,_1,std::ref(robot_task_msg)));
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>("base_scan",1,boost::bind(OnNewScan,_1,std::ref(robot_perception_msg)));


    //loop
    //in loop send state message
    ros::Rate loop(20);
    while(ros::ok())
    {
        getPose(std::string("map"),tf_frame+std::string("base_link"),state_msg);
        robot_state_msg.x = state_msg.x;
        robot_state_msg.y = state_msg.y;
        robot_state_msg.yaw = state_msg.yaw;
        sender.sendMsg(state_msg.fmtMsg());
        sender.sendMsg(robot_state_msg.fmtMsg());
        sender.sendMsg(robot_task_msg.fmtMsg());
        sender.sendMsg(robot_perception_msg.fmtMsg());
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}