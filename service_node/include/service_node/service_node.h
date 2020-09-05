/** brief:头文件声明了service节点的基类，ServiceNode
 *  note:
 *  
 */

#ifndef SERVICE_NODE_H
#define SERVICE_NODE_H
#include <string>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "ros/ros.h"


class ServiceNode{
public:
    ServiceNode()=default;
    ~ServiceNode()=default;
    virtual bool  LoadConfig(std::string file){};
    std::string node_name;
};


#endif