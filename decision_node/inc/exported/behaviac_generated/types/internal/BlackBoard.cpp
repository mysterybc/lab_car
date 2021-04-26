#include "BlackBoard.h"

Debug::DebugLogger logger;

BlackBoard::BlackBoard()
{
    ros::NodeHandle nh;
    car_id = 0;
    cmd_sub = nh.subscribe("host_cmd",10,&BlackBoard::CmdCallback,this);                                                            //1、2
    decision_state_pub = nh.advertise<robot_msgs::robot_states_enum>("decision_state",10);
    members_pub = nh.advertise<std_msgs::Int8MultiArray>("my_group_member",10);
    tag_pose_pub=nh.advertise<geometry_msgs::Pose>("tag_pose",10);
    tag_id_pub=nh.advertise<std_msgs::Int8MultiArray>("tag_id",10);
    group_state_sub=nh.subscribe("robot_states",10,&BlackBoard::GroupStateCallback,this);
    tag_detection_sub=nh.subscribe("/tag_detections",10,&BlackBoard::TagDetectionsCallback,this);
    current_task_pub=nh.advertise<robot_msgs::CurrentTask>("current_task",10);
    std::string namespace_=nh.getNamespace();
    nh.getParam(namespace_+"/car_id",car_id);
    logger.init_logger(car_id);

}

BlackBoard::~BlackBoard()
{

}


void BlackBoard::BackgrdFuncProcessing(TaskIndividual backgrdfunc)
{
    switch(backgrdfunc)
    {
        case Stop:
            g_GroupAsBasicLogicAgent->ActionCancel();
            g_GroupAsBasicLogicAgent->CurrentTask=NonTask;
            behaviac::vector<int>().swap(g_GroupAsBasicLogicAgent->GroupMember);
            std::vector<robot_msgs::HostCmd> ().swap(TaskList);
            break;
        case Pause:
            g_GroupAsBasicLogicAgent->ActionCancel();
            break;
        case Resume:
            g_GroupAsBasicLogicAgent->SendGoal();
            break;
        default:
            break;


    }

}

void BlackBoard::GroupStateCallback(const robot_msgs::RobotStatesConstPtr &msg)
{
    g_GroupAsBasicLogicAgent->wait_for_CB=false;

    auto iter1=g_GroupAsBasicLogicAgent->GroupMember.begin();
    auto iter2=msg->robot_states.begin();
    auto iter3=g_GroupAsBasicLogicAgent->MembersFromOtherMembers.begin();

    behaviac::vector<ForeFuncState>().swap(g_GroupAsBasicLogicAgent->GroupState);
    behaviac::vector<behaviac::vector<int>>().swap(g_GroupAsBasicLogicAgent->MembersFromOtherMembers);

    for(int i=0;i<g_GroupAsBasicLogicAgent->GroupMember.size();i++)
    {
        if(*(iter1+i)==car_id)//本车状态不在msg中，但要加入groupstates
            g_GroupAsBasicLogicAgent->GroupState.push_back(g_TaskRealizeAgent->fore_func_state);
        
        for(int j=0;j<msg->robot_states.size();j++)//在robot_states中找GroupMember的状态，加入group_states
        {
            if(*(iter1+i)==(iter2+j)->car_id)//carid==GroupMember[i]
            {
                g_GroupAsBasicLogicAgent->GroupState.push_back((ForeFuncState)(iter2+j)->robot_state.robot_states_enum);

                behaviac::vector<int> temp_vec;
                for(auto member : (iter2+j)->my_group_member)
                {
                    temp_vec.push_back(member);
                }
                g_GroupAsBasicLogicAgent->MembersFromOtherMembers.push_back(temp_vec) ;
            }
                        
        }

    }

}


void BlackBoard::CmdCallback(const robot_msgs::HostCmdArrayConstPtr &msg){
    logger.DEBUGINFO(car_id,"Cmd received:%d",msg->host_cmd_array.begin()->mission);

    //1.清空msgs、list
    //2.反向初始化，复制局部变量vector到类内变量
    //3.本车任务+数据类型转换
    //4.置位ToInitialize状态位
    //5.交给任务列表
    
    std::vector<robot_msgs::HostCmd>().swap(msgs);//1

    msgs.assign(msg->host_cmd_array.rbegin(),msg->host_cmd_array.rend());//2反向初始化：rbegin->rend.
    std::vector<robot_msgs::HostCmd>::iterator lr1=msgs.begin();
    std::vector<robot_msgs::HostCmd>::iterator lr2=msgs.begin();
    do
    {
        for(int i=0;i<lr1->car_id.size();i++){
            if(lr1->car_id.at(i)==car_id){
                (*lr2++)=(*lr1);
                break;
            }
        }
        if(lr1>=lr2)
            msgs.erase(lr1);
        else
            lr1++;
    }while(lr1!=msgs.end());//3

    if(!msgs.empty())//有本车命令
    {
        auto iter= msgs.begin();
        for(int i=0;i<msgs.size();i++)
        {
            bool meet_forefuncs=false;
                if(g_GroupAsBasicLogicAgent->IsForegrdFunc((TaskIndividual)(iter+i)->mission.mission)!=true)//Is backgrdFunc
                {
                    if(!meet_forefuncs)//before first foregrdFunc
                    {
                        BackgrdFuncProcessing((TaskIndividual)(iter+i)->mission.mission);
                        msgs.erase(iter+i);
                    }
                    else//After first foregrdFunc
                        msgs.erase(iter+i);
                }
                else
                    meet_forefuncs=true;

        }

        if(!msgs.empty())//有ForegrdFunc
        {
            std::vector<robot_msgs::HostCmd>().swap(TaskList);//1
            TaskList.assign(msgs.begin(),msgs.end());//5
            set_behavior_tree=true;//4
            logger.DEBUGINFO(car_id,"set_behavior_tree is True");
        }
    }

}


void BlackBoard::TagDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg ){
    std::vector<int>().swap(tag_id);
    std::vector<geometry_msgs::Pose>().swap(tag_pose);//识别结束后不会再pub"tag_pose". 

    for(auto i:msg->detections)
    {
        tag_pose.pushback(i.pose.pose.pose);
        tag_id.pushback(i.id);
    }
       
}

void BlackBoard::PubDecisionState(){
	robot_msgs::robot_states_enum state_;
    if(g_GroupAsBasicLogicAgent->CurrentTask!=NonTask)
        g_TaskRealizeAgent->fore_func_state=ForeFuncState::Running;//本句可以被ActiveCB代替，但是目前没有ActiveCB所以暂时使用。
    // else
        // g_TaskRealizeAgent->fore_func_state=ForeFuncState::Idle;
    
	state_.robot_states_enum = g_TaskRealizeAgent->fore_func_state;
	decision_state_pub.publish(state_);
    robot_msgs::CurrentTask current_task;
    current_task.current_task = g_GroupAsBasicLogicAgent->CurrentTask;
    current_task_pub.publish(current_task);
}

void BlackBoard::PubMembers(){
    std_msgs::Int8MultiArray msg;

    for(auto i:g_GroupAsBasicLogicAgent->GroupMember)
        msg.data.push_back(i);
    members_pub.publish(msg);
}


void BlackBoard::PubTagPose(){
    tag_pose_pub.publish(tag_pose);
    tag_id_pub.publish(tag_id);
}

std::vector<geometry_msgs::Pose> BlackBoard::GetGoal(){
    return goal;
}


