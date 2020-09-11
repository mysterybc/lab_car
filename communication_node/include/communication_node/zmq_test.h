#ifndef ZMQ_TEST_H
#define ZMQ_TEST_H

#include "zmq.h"
#include <iostream>
#include <string>
#include "ros/ros.h"
#include <string.h>
#include <unistd.h>

using namespace std;
using std::to_string;

class ZMQ_TEST
{
public:
    ZMQ_TEST();
    ~ZMQ_TEST();
    void run(string &str, bool &new_data);

    void zmq_init(int server_or_client, int sub_or_pub, int port);
    void send_data(string payload);

private:
    void *context;
    void *subscriber; //订阅
    void *publisher; //发布
    int run_flag;
};

ZMQ_TEST::ZMQ_TEST()
{
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

void ZMQ_TEST::run(string &str, bool &new_data)
{
    run_flag = 1;
    char topic_name[256]={0}; //用于接收订阅的主题名
    char payload[1024]={0};   //用于接收订阅主题的内容
    while(ros::ok())
    {
        memset(topic_name,0,sizeof(topic_name));
        memset(payload,0,sizeof(payload));

        int size = zmq_recv (subscriber, topic_name, sizeof(topic_name), 0); //接收订阅的主题名称
        if (size == -1)
        {
            // qDebug("recv topic error!!\n");
            cout<<"recv topic error!!\n";
        }
        size = zmq_recv (subscriber, payload, sizeof(payload), 0); //接收订阅的消息
        if (size == -1)
        {
            // qDebug("recv payload error!!\n");
            cout<<"recv payload error!!\n";
        }
        str = payload;
        new_data = true;
        // qDebug("Topic:%s  Msg:%s\n",topic_name, payload);
        cout<<"Topic:"<<topic_name<<" Msg:"<<payload<<endl;
        usleep(100*1000);
        ros::spinOnce();
    }
}

void ZMQ_TEST::zmq_init(int server_or_client, int sub_or_pub, int port) //ZMQ初始化
{
    context = zmq_ctx_new();
    if(context)
    {
        if(sub_or_pub == 0)  //订阅
        {
            if(0 == server_or_client)    //服务端
            {
                subscriber = zmq_socket(context, ZMQ_SUB);
                // QString sub_str = QString("tcp://*:") + QString::number(port);
                string sub_str = "tcp://*:" + std::to_string(port);
                // qDebug()<<"sub_str:"<<sub_str;
                // zmq_bind(subscriber, sub_str.toLocal8Bit().data());
                zmq_bind(subscriber, sub_str.data());
            }
            else            //客户端
            {
                subscriber = zmq_socket(context, ZMQ_SUB);
                // QString sub_str = QString("tcp://localhost:") + QString::number(port);
                string sub_str = "tcp://*:" + std::to_string(port);
                // qDebug()<<"sub_str:"<<sub_str;
                // zmq_connect(subscriber, sub_str.toLocal8Bit().data());
                zmq_connect(subscriber, sub_str.data());
            }
            zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "TEST", 4);  //允许订阅多个频道
            // qDebug()<<"sub topic:"<< "TEST";
        }
        else        //发布
        {
            if(0 == server_or_client)    //服务端
            {
                publisher = zmq_socket(context, ZMQ_PUB);
                // QString pub_str = QString("tcp://*:") + QString::number(port);
                string pub_str = "tcp://192.168.0.104:" + std::to_string(port);
                // qDebug()<<"pub_str:"<<pub_str;
                // zmq_bind(publisher, pub_str.toLocal8Bit().data());
                zmq_bind(publisher, pub_str.data());
            }
            else            //客户端
            {
                publisher = zmq_socket(context, ZMQ_PUB);
                // QString pub_str = QString("tcp://localhost:") + QString::number(port);
                string pub_str = "tcp://192.168.0.104:" + std::to_string(port);
                // qDebug()<<"pub_str:"<<pub_str;
                // zmq_connect(publisher, pub_str.toLocal8Bit().data());
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
        // qDebug()<<"pub msg:"<<payload;
    }
}

#endif // ZMQ_TEST_H