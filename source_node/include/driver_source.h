/** brief:头文件声明了驱动器资源节点，继承source_node
 *  note:
 *  
 */
#ifndef DRIVER_SOURCE_H
#define DRIVER_SOURCE_H
#include "source_node.h"

class DriverSource : public SourceNode{
public:
    DriverSource();
    ~DriverSource(){};
    bool  LoadConfig(std::string file);
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  UpdateState();
    void CmdCallback(const robot_msgs::CmdConstPtr &msg);
};

#endif