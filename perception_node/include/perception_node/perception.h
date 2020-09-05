/** brief:头文件声明了source节点的基类，SourceNode
 *  note:
 *  
 */

#ifndef PERCEPTION_NODE_H
#define PERCEPTION_NODE_H
#include <thread>
#include <string>
#include <iostream>
#include "robot_msgs/PerceptionNodeMsg.h"
#include "robot_msgs/Cmd.h"
#include "yaml-cpp/yaml.h"
#include "ros/ros.h"

enum class State{
    NOT_CONFIG    = 0,
    HAVE_CONFIG   = 1,
    CONNECTING    = 2,
    CONNECTED     = 3,
    RUNNING       = 4,
    PAUSED        = 5,
    ERROR_RUNNING = 6,
    EXIT          = 7,
    STOP          = 8
};

enum class Cmd{
    START   = 0,
    PAUSE   = 1,
    STOP    = 2,
    RESUME  = 3,
    EXIT    = 4,
};


class PerceptionNode{
public:
    PerceptionNode()=default;
    ~PerceptionNode()=default;
    virtual bool  LoadConfig(std::string file){};
    //这些函数应该在cmd的callback中被调用
    virtual State Start(){};
    virtual State Stop(){};
    virtual State Exit(){};
    virtual State Pause(){};
    virtual State Resume(){};
    virtual void  UpdateState(){};
    virtual void CmdCallback(const robot_msgs::CmdConstPtr &msg){};
    ros::Publisher  state_pub;
    ros::Subscriber cmd_sub;
    State node_state;
    uint8_t update_frequence;
    std::string node_name;
};


#endif