#pragma once
//ros
#include "ros/ros.h"
//c++
#include "string"
//my debug header
#include "my_debug_info.h"


namespace my_lib{

/**
 * @description: base class of service class，shouldn't be use directly!!
 *               inherit the base class to implement your service, and SetRequest() function should bt rewrite
 *               demo service in my_service_demo.cpp
 * @param Service : xxxxService,xxxx represent the name of the service
 */
template <typename Service>
class ServiceClientRealize{
public:

    /**
     * @description: constructor
     * @param action_name_ : action name used in ROS
     * @param car_id_      : the id of the agent
     */
    ServiceClientRealize(std::string service_name_,int car_id_):
        service_name(service_name_),car_id(car_id_)
    {
        logger.init_logger(car_id_);
        service_client = nh.serviceClient<Service>(service_name_);
    }

    /**
     * @description: call service server
     */
    void CallServer(){
        Service service;
        SetRequest(service);
        logger.DEBUGINFO(car_id,"[%s client] waiting for server!!",service_name.c_str());
        if(service_client.call(service)){
            logger.DEBUGINFO(car_id,"[%s client] call server success!!",service_name.c_str());
        }
        else{
            logger.DEBUGINFO(car_id,"[%s client] call server failed!!",service_name.c_str());
        }
    }

    /**
     * @description: set service request,need rewrite
     */
    virtual void SetRequest(Service &service){}

    ros::ServiceClient service_client;
    ros::NodeHandle nh;
    std::string service_name;
    int car_id;
    Debug::DebugLogger logger;

};
//end of class ServiceClientRealize



/**
 * @description: base class of service class，shouldn't be use directly!!
 *               inherit the base class to implement your service, and SetRequest() function should bt rewrite
 *               demo service in my_service_demo.cpp
 * @param Service : xxxxService,xxxx represent the name of the service
 */
template <typename Service>
class ServiceServerRealize{
public:

    /**
     * @description: constructor
     * @param action_name_ : action name used in ROS
     * @param car_id_      : the id of the agent
     */
    ServiceServerRealize(std::string service_name_,int car_id_):
        service_name(service_name_),car_id(car_id_)
    {
        service_server = nh.advertiseService(service_name,&ServiceServerRealize::OnNewService,this);
        logger.init_logger(car_id_);
    }

    /**
     * @description: call service server,need rewrite
     */
    virtual bool OnNewService(typename Service::Request & req,typename Service::Response &res ){
        return true;
    }

    ros::ServiceServer service_server;
    ros::NodeHandle nh;
    std::string service_name;
    int car_id;
    Debug::DebugLogger logger;

};
//end of class ServiceClientRealize





};
//end of namespace my_lib