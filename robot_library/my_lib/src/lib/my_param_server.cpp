#include "my_param_server.h"


void my_lib::GetParam(  std::string node_name,
                        int* car_id,
                        int* total_car_num,
                        std::string* tf_ns,
                        std::string* my_ip,
                        std::string* host_ip,
                        std::vector<std::string>* total_ip,
                        bool* is_simulation)
{
    Debug::DebugLogger logger;

    //获取group空间名
    ros::NodeHandle nh;
    std::string namespace_;
    namespace_ = nh.getNamespace();

    //依次获取所有param
    //1. car id
    if(car_id != NULL){
        if(!nh.getParam(namespace_+"/car_id",*car_id)){
            *car_id = 1;
            logger.WARNINFO("%s FAILED TO GET CAR ID",node_name.c_str());
            logger.WARNINFO("%s RESET CAR ID TO 1",node_name.c_str());
        }
    }
    logger.init_logger(*car_id);

    //2.car num
    if(total_car_num != NULL){
        if(!nh.getParam("/total_car_number",*total_car_num)){
                logger.WARNINFO(*car_id,"%s FAILED TO GET TOTAL CAR NUMBER",node_name.c_str());
            }
    }

    //3.tf ns
    if(tf_ns != NULL){
        if(!nh.getParam(namespace_+"/tf_ns",*tf_ns)){
                logger.WARNINFO(*car_id,"%s FAILED TO GET TF FRAME",node_name.c_str());
            }
    }

    //4.my ip
    if(my_ip != NULL){
        if(!nh.getParam(namespace_+"/my_ip_address",*my_ip)){
                logger.WARNINFO(*car_id,"%s FAILED TO GET MY IP",node_name.c_str());
            }
    }


    //5. host ip
    if(host_ip != NULL){
        if(!nh.getParam("/host_ip_address",*host_ip)){
                logger.WARNINFO(*car_id,"%s FAILED TO GET HOST IP",node_name.c_str());
            }
    }

    //6. total ip
    if(total_ip != NULL){
        if(!nh.getParam("/total_robot_ip",*total_ip)){
                logger.WARNINFO(*car_id,"%s FAILED TO GET TOTAL ROBOT IP",node_name.c_str());
            }
    }

    //7. is simulation
    if(is_simulation != NULL){
        if(!nh.getParam("/is_simulation",*is_simulation)){
                *is_simulation = true;
                logger.WARNINFO(*car_id,"%s FAILED TO GET SIMULATION FLAG",node_name.c_str());
                logger.WARNINFO(*car_id,"DEFAULT SET SIMULATION MODE",node_name.c_str());
            }
    }

}