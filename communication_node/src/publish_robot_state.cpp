#include "zmq_test.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include "robot_msgs/ReportPath.h"
#include "memory"
#include "tf/transform_listener.h"




//用于发送机器人状态
class RobotState{
public:
    RobotState(int car_id);
    void GetPose();
    void SetState(std_msgs::String msg);
    std::string GetMessage();
    Json::Value json;
    tf::TransformListener listener;
    tf::StampedTransform trans;
    
};
RobotState::RobotState(int car_id){
    json["message_type"] = "state";  //string
    json["id"] = car_id;             //int
    json["state"] = "stand_by";
    for (int i=0;i<3;i++)
    {
        json["pose"].append(0.0);
    }
}
void RobotState::GetPose(){
    while(ros::ok){
        try{
            listener.lookupTransform("map","base_link",ros::Time(0),trans);
        }
        catch(tf::TransformException){
            continue;
        }
        break;
    }
    tf::Quaternion quat = trans.getRotation();
    double roll,yaw,pitch;
    tf::Matrix3x3(quat).getEulerYPR(yaw,pitch,roll);
    yaw = yaw / 3.1415926 * 180;
    json["pose"].clear();
    json["pose"].append(trans.getOrigin().x());
    json["pose"].append(trans.getOrigin().y());
    json["pose"].append(yaw);
}
void RobotState::SetState(std_msgs::String msg){
    //pose
    GetPose();
    //state
    json["state"] = msg.data;
}
std::string RobotState::GetMessage(){
    return json.toStyledString();
}

class ConfigPath{
public:
    ConfigPath(int car_id, std::shared_ptr<ZMQ_TEST> zmq);
    void SetPath(nav_msgs::Path msg);
    std::string GetMessage();
    bool ReportPath(robot_msgs::ReportPath::Request  &req,
                    robot_msgs::ReportPath::Response &res);
private:
    Json::Value json;
    std::shared_ptr<ZMQ_TEST> host_zmq;
};
ConfigPath::ConfigPath(int car_id, std::shared_ptr<ZMQ_TEST> zmq){
    json["message_type"] = "path";
    json["id"] = car_id;
    json["path_x"].append(0.0);
    json["path_y"].append(0.0);
    host_zmq = zmq;
}
void ConfigPath::SetPath(nav_msgs::Path msg){
    json["path_x"].clear();
    json["path_y"].clear();
    for(auto i : msg.poses){
        json["path_x"].append(i.pose.position.x);
        json["path_y"].append(i.pose.position.y);
    }
}
std::string ConfigPath::GetMessage(){
    return json.toStyledString();
}


//service
bool ConfigPath::ReportPath(robot_msgs::ReportPath::Request  &req,
                            robot_msgs::ReportPath::Response &res){
    SetPath(req.Path);  //转json格式
    host_zmq->send_data(GetMessage()); //发送路径信息
    res.cmd.data = true;
    return true;
}
//topic callback
void DecisionSubCB(const std_msgs::StringConstPtr &msg, RobotState &state){
    state.SetState(*msg);
}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"publish_robot_state");
    ros::NodeHandle nh;
    std::string ip_address;

    //config id & ip
    int car_id;
    nh.getParam("car_id",car_id);
    nh.getParam("my_ip_address",ip_address);

    //zmq_init
    std::shared_ptr<ZMQ_TEST> host_zmq;
    host_zmq = std::make_shared<ZMQ_TEST>(ZMQ_TEST());
    host_zmq->zmq_init(0,1,6666,ip_address);


    //message init
    RobotState state_message(car_id);
    ConfigPath config_message(car_id,host_zmq);

    //sub 
    ros::Subscriber state_subs;
    ros::ServiceServer report_path_server;
    report_path_server = nh.advertiseService("report_path",&ConfigPath::ReportPath,&config_message);
    state_subs = nh.subscribe<std_msgs::String>("decision_state",10,boost::bind(&DecisionSubCB,_1,std::ref(state_message)));



    //loop
    //in loop send state message
    ros::Rate loop(20);
    while(ros::ok())
    {
        host_zmq->send_data(state_message.GetMessage());
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}