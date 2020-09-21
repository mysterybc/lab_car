#pragma once
#include "geometry_msgs/Point.h"
#include "yaml-cpp/yaml.h"

namespace YAML{
  template<>
  struct convert<geometry_msgs::Point>{
    static bool decode(const Node& node,geometry_msgs::Point& cType){ 
       cType.x = node["x"].as<double>();
       cType.y = node["y"].as<double>();
       cType.z = 0;
       return true;
    }
  };
}