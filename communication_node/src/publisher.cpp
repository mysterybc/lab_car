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
//my lib
#include "zmq_lib.h"
#include "msgs.h"
#include "debug_info.h"

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



int main(int argc,char **argv)
{
    ros::init(argc,argv,"publish_robot_state");
    ros::NodeHandle nh;
    std::string ip_address;

    //config id & ip
    int car_id;
    //获取group空间名
    std::string namespace_;
    std::string tf_frame;
    namespace_ = nh.getNamespace();
    //获取group下的参数
    nh.getParam(namespace_+"/car_id",car_id);
    nh.getParam(namespace_+"/my_ip_address",ip_address);
    nh.getParam(namespace_+"/tf_ns",tf_frame);
    //获取group下的参数
    if(!nh.getParam(namespace_+"/car_id",car_id)){
        car_id = 1;
        logger.WARNINFO("PUBLISHER FAILED TO GET CAR ID");
        logger.WARNINFO("PUBLISHER RESET CAR ID TO 1");
    }
    logger.init_logger(car_id);
    if(!nh.getParam(namespace_+"/my_ip_address",ip_address)){
        ip_address = "127.0.0.1:6661";
        logger.WARNINFO(car_id,"PUBLISHER FAILED TO GET ip_address");
        logger.WARNINFO(car_id,"PUBLISHER RESET CAR ID TO 127.0.0.1");
    }
    if(!nh.getParam(namespace_+"/tf_ns",tf_frame)){
        logger.WARNINFO(car_id,"PUBLISHER FAILED TO GET TF FRAME");
    }

    //zmq_init
    zmq::context_t ctx(1);
    zmq_lib::Sender sender(ip_address,ctx);



    //message init
    StateMsg state_msg(car_id);
    PathMsg path_msg(car_id);

    //sub 
    ros::Subscriber state_subs;
     ros::Subscriber group_subs;
    ros::Subscriber algomsg_subs;
    ros::ServiceServer report_path_server;
    report_path_server = nh.advertiseService<robot_msgs::ReportPath::Request,robot_msgs::ReportPath::Response>\
                            ("report_path",boost::bind(&ReportPath,_1,_2,std::ref(path_msg),std::ref(sender)));
    state_subs = nh.subscribe<robot_msgs::robot_states_enum>("decision_state",10,boost::bind(&DecisionSubCB,_1,std::ref(state_msg)));
    group_subs = nh.subscribe<std_msgs::Int8MultiArray>("my_group_member",10,boost::bind(&GroupSubCB,_1,std::ref(state_msg)));
    algomsg_subs = nh.subscribe<std_msgs::UInt8MultiArray>("algomsg_my",10,boost::bind(&OnNewAlgomsg,_1,std::ref(sender)));

    //loop
    //in loop send state message
    ros::Rate loop(20);
    while(ros::ok())
    {
        getPose(std::string("map"),tf_frame+std::string("base_link"),state_msg);
        sender.sendMsg(state_msg.fmtMsg());
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}