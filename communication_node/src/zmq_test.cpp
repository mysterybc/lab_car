#include "zmq_test.h"

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

void ZMQ_TEST::zmq_init(int server_or_client, int sub_or_pub, string ip) //ZMQ初始化
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
                zmq_bind(subscriber, ip.data());
            }
            else            //客户端
            {
                subscriber = zmq_socket(context, ZMQ_SUB);
                // string sub_str = "tcp://localhost:" + to_string(port);
                zmq_connect(subscriber, ip.data());
            }
            zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "TEST", 4);  //允许订阅多个频道
        }
        else        //发布
        {
            if(0 == server_or_client)    //服务端
            {
                publisher = zmq_socket(context, ZMQ_PUB);
                // string pub_str = "tcp://10.1.76.159:" + to_string(port);
                zmq_bind(publisher, ip.data());
            }
            else            //客户端
            {
                publisher = zmq_socket(context, ZMQ_PUB);
                // string pub_str = "tcp://10.1.76.159:" + to_string(port);
                zmq_connect(publisher, ip.data());
                
            }
            int hwm = 10;
            zmq_setsockopt(publisher,ZMQ_SNDHWM,&hwm,sizeof(hwm));
        }

    }
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
            
        }
        ros::spinOnce();
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

