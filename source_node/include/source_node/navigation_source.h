/** brief:头文件声明了组合导航资源节点，继承source_node
 *  note:
 *  
 */

#ifndef NAVIGATION_SOURCE_H
#define NAVIGATION_SOURCE_H

#include "source_node.h"
#include "gps.h"
#include "imu.h"

class NavigationSource : public SourceNode{
public:
    NavigationSource();
    ~NavigationSource(){};
    bool  LoadConfig(std::string file);
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  UpdateState();
    void CmdCallback(const robot_msgs::CmdConstPtr &msg);
    GPS gps_;
    IMU imu_;
};



#endif
