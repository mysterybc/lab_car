/** brief:头文件声明了source节点的基类，SourceNode
 *  note:
 *  
 */

#ifndef SOURCE_NODE_H
#define SOURCE_NODE_H
#include <thread>
#include <string>
#include "robot_msgs/SourceNodeMsg.h"
#include "robot_msgs/Cmd.h"
#include "ros/ros.h"

#define pi 3.14

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

enum class Error{
    NO_ERROR = 0,
    CONFIG_FAILED = 1
};


class SourceNode{
public:
    SourceNode()=default;
    ~SourceNode()=default;
    virtual bool  LoadConfig(std::string file){}
    //这些函数应该在cmd的callback中被调用
    virtual State Start(){}
    virtual State Stop(){}
    virtual State Exit(){}
    virtual State Pause(){}
    virtual State Resume(){}
    virtual void  UpdateState(){}
    virtual void CmdCallback(const robot_msgs::CmdConstPtr &msg){}
    ros::Publisher  state_pub;
    ros::Subscriber cmd_sub;
    State node_state;
    Error node_error;
    uint8_t update_frequence;
    std::string node_name;
};


#endif