#include "zmq_test.h"



int main(int argc,char **argv)
{
    ros::init(argc,argv,"receive_cmd");
    ros::NodeHandle nh;
    ZMQ_TEST *zmq_test;
    zmq_test = new ZMQ_TEST();
    string ip_recv ;
    nh.getParam("host_ip_address",ip_recv);
    zmq_test->zmq_init(1,0,6666,ip_recv);
    zmq_test->run();

    return 0;
}

