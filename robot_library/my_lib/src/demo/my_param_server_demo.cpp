#include "my_param_server.h"

int main(int argc,char** argv){
    ros::init(argc,argv,"my_param_server_demo");
    ros::NodeHandle nh;

    //init param
    std::string node_name = "test name server";
    int car_id ;
    int total_car_num ;
    std::string tf_ns ;
    std::string my_ip ;
    std::string host_ip ;
    std::vector<int> debug_id ;
    std::vector<std::string> total_ip;

    //get param
    my_lib::GetParam(node_name,&car_id,&total_car_num,&tf_ns,&my_ip,&host_ip,&debug_id,&total_ip);

    std::cout << " car id is : " << car_id << std::endl;
    std::cout << " total car num is : " << total_car_num << std::endl;
    std::cout << " tf ns is : " << tf_ns << std::endl;
    std::cout << " my ip is : " << my_ip << std::endl;
    std::cout << " host ip is : " << host_ip << std::endl;
    for(auto id:debug_id){
        std::cout << " deug id is : " << id << std::endl;
    }
    for(auto ip:total_ip){
        std::cout << " total car ip is : " << ip << std::endl;
    }
    while(ros::ok());

    return 0;
    

}