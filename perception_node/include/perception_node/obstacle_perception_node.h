/** brief:头文件声明了障碍物感知节点，ObstraclePerceptionNode
 *        目前该节点还没有分配
 *  note:
 *  
 */

#ifndef OBSTACLE_PERCEPTION_NODE
#define OBSTACLE_PERCEPTION_NODE
#include "perception.h"

class ObstaclePerceptionNode:public PerceptionNode{
public:
    ObstaclePerceptionNode()=default;
    ~ObsraclePerceptionNode()=default;
    bool  LoadConfig(std::string file);
    //这些函数应该在cmd的callback中被调用
    State Start();
    State Stop();
    State Exit();
    State Pause();
    State Resume();
    void  UpdateState();
    void CmdCallback(const perception_node::CmdConstPtr &msg);
};


#endif