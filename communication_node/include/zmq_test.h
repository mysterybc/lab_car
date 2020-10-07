#ifndef ZMQ_TEST_H
#define ZMQ_TEST_H

#include "zmq.hpp"
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <jsoncpp/json/json.h>
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"
#include "robot_msgs/HostCmd.h"

using namespace std;


class ZMQ_TEST
{
public:
    ZMQ_TEST();
    ~ZMQ_TEST();
    void run();

    void zmq_init(int server_or_client, int sub_or_pub, int port, string ip);
    void send_data(string payload);

private:
    void *context;
    void *subscriber; //订阅
    void *publisher; //发布
    robot_msgs::HostCmd cmd;
    ros::Publisher host_cmd_pub;
    double pose_separate_factor;
    ros::NodeHandle nh;
};

ZMQ_TEST::ZMQ_TEST()
{
    host_cmd_pub = nh.advertise<robot_msgs::HostCmd>("/host_cmd",10);
    int car_id;
    nh.param<int>("car_id",car_id,1);
    if(car_id == 1){
        pose_separate_factor = 1;
    }
    else{
        pose_separate_factor = -1;
    }
    context = NULL;
    subscriber = NULL;
    publisher = NULL;
}

ZMQ_TEST::~ZMQ_TEST()
{
    if(subscriber)
        zmq_close(subscriber);    //退出时调用
    if(publisher)
        zmq_close(publisher);    //退出时调用
    zmq_ctx_destroy(context);
}

void ZMQ_TEST::run()
{
    char topic_name[256]={0}; //用于接收订阅的主题名
    char payload[1024]={0};   //用于接收订阅主题的内容
    Json::Value json1;
    Json::Reader jsonread1;
    while(ros::ok())
    {
        memset(topic_name,0,sizeof(topic_name));
        memset(payload,0,sizeof(payload));

        int size = zmq_recv (subscriber, topic_name, sizeof(topic_name), 0); //接收订阅的主题名称
        if (size == -1)
        {
            ROS_INFO("recv topic error!!");
        }
        size = zmq_recv (subscriber, payload, sizeof(payload), 0); //接收订阅的消息
        if (size == -1)
        {
            ROS_INFO("recv payload error!!");
        }
        if(!jsonread1.parse(payload,json1))
        {
            ROS_INFO("json_error");
        }
        else
        {
            cout<<json1.toStyledString()<<endl;
            if(json1["instruction"].isArray())
            {
                cmd.car_id.clear();
                if(json1["ID"].isArray()){
                    Json::Value json_id = json1["ID"];
                    for(int i = 0; i < json_id.size(); i++){
                        cmd.car_id.push_back(json_id[i].asInt());
                    }
                }
                cmd.mission.mission = json1["type"].asUInt();
                cmd.goal.header.stamp = ros::Time().now();
                cmd.goal.header.frame_id = "map";
                cmd.goal.pose.position.x = json1["instruction"][0].asDouble();
                cmd.goal.pose.position.y = json1["instruction"][1].asDouble();
                cmd.goal.pose.position.z = 0;
                double yaw = json1["instruction"][2].asDouble();
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
        ros::spinOnce();
    }
}

void ZMQ_TEST::zmq_init(int server_or_client, int sub_or_pub, int port, string ip) //ZMQ初始化
{
    context = zmq_ctx_new();
    if(context)
    {
        if(sub_or_pub == 0)  //订阅
        {
            if(0 == server_or_client)    //服务端
            {
                subscriber = zmq_socket(context, ZMQ_SUB);
                // string sub_str = "tcp://*:" + to_string(port);
                string sub_str = ip + to_string(port);
                zmq_bind(subscriber, sub_str.data());
            }
            else            //客户端
            {
                subscriber = zmq_socket(context, ZMQ_SUB);
                // string sub_str = "tcp://localhost:" + to_string(port);
                string sub_str = ip + to_string(port);
                zmq_connect(subscriber, sub_str.data());
            }
            zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "TEST", 4);  //允许订阅多个频道
        }
        else        //发布
        {
            if(0 == server_or_client)    //服务端
            {
                publisher = zmq_socket(context, ZMQ_PUB);
                // string pub_str = "tcp://10.1.76.159:" + to_string(port);
                string pub_str = ip + to_string(port);
                zmq_bind(publisher, pub_str.data());
            }
            else            //客户端
            {
                publisher = zmq_socket(context, ZMQ_PUB);
                // string pub_str = "tcp://10.1.76.159:" + to_string(port);
                string pub_str = ip + to_string(port);
                zmq_connect(publisher, pub_str.data());
            }
        }

    }
}

void ZMQ_TEST::send_data(string payload)  //向某个主题发送数据
{
    if(publisher)
    {
        zmq_send (publisher, "TEST", 4, ZMQ_SNDMORE); //指定要发布消息的主题
        zmq_send (publisher, payload.data(), payload.length(), 0);   //向设置的主题发布消息
    }
}

#endif // ZMQ_TEST_H