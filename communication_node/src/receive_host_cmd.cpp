#include "zmq_test.h"

//指令解码
class HostMessage{
public:
    HostMessage();
    ~HostMessage();
    bool GetConfigCmdServer(robot_msgs::GetConfigCmd::Request  &req,
                            robot_msgs::GetConfigCmd::Response &res);
    void DecodeMsg(Json::Value json);
    void run();
private:
    Json::Value json;
    Json::Reader jsonread;
    ros::NodeHandle nh;
    robot_msgs::HostCmd cmd;
    ros::Publisher host_cmd_pub;
    ros::ServiceServer server;
    ZMQ_TEST *zmq_test;

    string host_cmd_state;
    std::string ip_recv;
    int car_id;
    double pose_separate_factor;
};

HostMessage::~HostMessage(){
    delete zmq_test;
}

HostMessage::HostMessage(){
    host_cmd_pub = nh.advertise<robot_msgs::HostCmd>("/host_cmd",10);
    server = nh.advertiseService("get_config_cmd",&HostMessage::GetConfigCmdServer,this);
    nh.param<int>("car_id",car_id,1);
    if(car_id == 1){
        pose_separate_factor = 1;
    }
    else{
        pose_separate_factor = -1;
    }
    host_cmd_state = "not receive";
    nh.getParam("host_ip_address",ip_recv);
    zmq_test = new ZMQ_TEST;
    zmq_test->zmq_init(1,0,6666,ip_recv);
}
void HostMessage::DecodeMsg(Json::Value json){
    if(json["message_type"].asString() == "mission"){
        if(json["instruction"].isArray())
        {
            cmd.car_id.clear();
            if(json["id"].isArray()){
                Json::Value json_id = json["id"];
                for(int i = 0; i < json_id.size(); i++){
                    cmd.car_id.push_back(json_id[i].asInt());
                }
            }
            cmd.mission.mission = json["type"].asUInt();
            cmd.goal.header.stamp = ros::Time().now();
            cmd.goal.header.frame_id = "map";
            cmd.goal.pose.position.x = json["instruction"][0].asDouble();
            cmd.goal.pose.position.y = json["instruction"][1].asDouble();
            cmd.goal.pose.position.z = 0;
            double yaw = json["instruction"][2].asDouble();
            //目前输入角度，手动坐标-1
            yaw = yaw / 180.0 * 3.1415926;
            cmd.goal.pose.position.x += pose_separate_factor;
            cmd.goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            host_cmd_pub.publish(cmd);
        }
        else{
            ROS_INFO("cmd error!!");
        }
    }
    else if(json["message_type"].asString() == "cmd"){
        int type = json["type"].asUInt();
        if(type == 0){
            host_cmd_state = "no";
        }
        else{
            host_cmd_state = "ok";
        }
        
    }
    
}
void HostMessage::run()
{
    char topic_name[256]={0}; //用于接收订阅的主题名
    char payload[1024]={0};   //用于接收订阅主题的内容
    while(ros::ok())
    {
        memset(topic_name,0,sizeof(topic_name));
        memset(payload,0,sizeof(payload));

        int size = zmq_recv (zmq_test->subscriber, topic_name, sizeof(topic_name), 0); //接收订阅的主题名称
        if (size == -1)
        {
            ROS_INFO("recv topic error!!");
        }
        size = zmq_recv (zmq_test->subscriber, payload, sizeof(payload), 0); //接收订阅的消息
        if (size == -1)
        {
            ROS_INFO("recv payload error!!");
        }
        if(!jsonread.parse(payload,json))
        {
            ROS_INFO("json_error");
        }
        else
        {
            cout << "get command !!!" ;
            cout << json.toStyledString();
            DecodeMsg(json);
        }
        ros::spinOnce();
    }
}
bool HostMessage::GetConfigCmdServer(robot_msgs::GetConfigCmd::Request  &req,
                                     robot_msgs::GetConfigCmd::Response &res){
    res.cmd.data = host_cmd_state;
    if(host_cmd_state != "not receive"){
        host_cmd_state = "not receive";
    }
    return true;
}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"receive_host_cmd");
    ros::NodeHandle nh;
    HostMessage host_message;
    host_message.run();
    return 0;
}

