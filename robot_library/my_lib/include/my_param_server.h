#pragma once
//ros
#include <ros/ros.h>
//c++
#include <string>
#include <vector>
#include <iostream>
//my lib
#include "my_debug_info.h"

namespace my_lib{

/**
 * @brief  get param from param server
 * @param: node_name
 *         car_id
 *         total_car_num
 *         tf_ns
 *         my_ip
 *         host_ip
 *         total_ip
 */


void GetParam(  std::string node_name,
                int* car_id = NULL,
                int* total_car_num = NULL,
                std::string* tf_ns = NULL,
                std::string* my_ip = NULL,
                std::string* host_ip = NULL,
                std::vector<std::string>* total_ip = NULL,
                bool* is_simulation = NULL);



};
//end of namespace my_lib