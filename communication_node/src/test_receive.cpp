#include <ros/ros.h>
#include "zmq_test.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_receive", ros::init_options::AnonymousName);

    ZMQ_TEST *zmq_test;
    zmq_test = new ZMQ_TEST();
    string ip_recv = "tcp://192.18.1.110:";
    // string ip_recv = "tcp://10.1.76.171:";
    zmq_test->zmq_init(1,0,6666,ip_recv);       //connect,sub,port,ip
    zmq_test->run();

    return 0;
}