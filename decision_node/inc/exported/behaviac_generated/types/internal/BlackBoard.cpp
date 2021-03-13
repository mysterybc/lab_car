#include "BlackBoard.h"

Debug::DebugLogger logger;

// void PubCurrentTask(){
//     ros::Rate loop(10);
//     while(ros::ok()){
//         //pub current task
//         robot_msgs::CurrentTask current_task;
//         current_task.current_task = g_GroupAsBasicLogicAgent->CurrentTask;
//         g_BlackBoardAgent->current_task_pub.publish(current_task);
//         ros::spinOnce();
//         loop.sleep();
//     }
    
// }


BlackBoard::BlackBoard()
{
    ros::NodeHandle nh;
    car_id = 0;
    cmd_sub = nh.subscribe("host_cmd",10,&BlackBoard::CmdCallback,this);                                                            //1、2
    decision_state_pub = nh.advertise<robot_msgs::robot_states_enum>("decision_state",10);
    members_pub = nh.advertise<std_msgs::Int8MultiArray>("my_group_member",10);
    group_state_sub=nh.subscribe("robot_states",10,&BlackBoard::GroupStateCallback,this);
    current_task_pub = nh.advertise<robot_msgs::CurrentTask>("current_task",1);
    std::string namespace_=nh.getNamespace();
    // std::thread pub_task(PubCurrentTask);
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

    		if(g_BlackBoardAgent->car_id==2&&g_GroupAsBasicLogicAgent->wait_for_CB==true)
			logger.DEBUGINFO(g_BlackBoardAgent->car_id,"In CB");
            
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
            // if(g_BlackBoardAgent->car_id==2)
            // for(auto k:g_GroupAsBasicLogicAgent->GroupState)
            // 	logger.DEBUGINFO(g_BlackBoardAgent->car_id,"the car %d's state is:%d",*(iter1+i),k);

			behaviac::vector<int> temp_vec;
            for(auto member : (iter2+j)->my_group_member){
                temp_vec.push_back(member);
			}
			g_GroupAsBasicLogicAgent->MembersFromOtherMembers.push_back(temp_vec) ;
            // if(g_BlackBoardAgent->car_id==1)
            // for(auto k:g_GroupAsBasicLogicAgent->MembersFromOtherMembers)
            //     for(auto m:k)
            //     ROS_INFO("the car %d's member is:car%d",*(iter1+i),m);
		}
		            
    }

}

// if(g_BlackBoardAgent->car_id==1)
// ROS_INFO("car 1 in CB:%d",g_GroupAsBasicLogicAgent->wait_for_CB);
}


void BlackBoard::CmdCallback(const robot_msgs::HostCmdArrayConstPtr &msg){
    if(g_BlackBoardAgent->car_id==2)
    logger.DEBUGINFO(car_id,"Cmd received");

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
            if(lr1>=lr2){
            msgs.erase(lr1);
            }
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

            //test
            // if(g_BlackBoardAgent->car_id==1)
            // {
            //     for(auto i:TaskList)
            //         ROS_INFO("car1'sTaskList:%d",i.mission.mission);
            //     for(auto j:TaskList)
            //         for(auto k:j.car_id)
            //             ROS_INFO("car1'TaskMember:%d",k);
            //}
            //test
            set_behavior_tree=true;//4
            if(g_BlackBoardAgent->car_id==2)
                logger.DEBUGINFO(car_id,"set_behavior_tree is True");
        }
    }

}

void BlackBoard::PubDecisionState(){
    if(g_BlackBoardAgent->car_id==2&&g_GroupAsBasicLogicAgent->CurrentTask==NonTask)
        logger.DEBUGINFO(car_id,"PubNonTask");
	robot_msgs::robot_states_enum state_;
    if(g_GroupAsBasicLogicAgent->CurrentTask==NonTask)
        g_TaskRealizeAgent->fore_func_state=ForeFuncState::Idle;
    else
        g_TaskRealizeAgent->fore_func_state=ForeFuncState::Running;
    
	state_.robot_states_enum = g_TaskRealizeAgent->fore_func_state;

//test
    // if(g_BlackBoardAgent->car_id==2)
    //     logger.DEBUGINFO(car_id,"car2's fore_func_state is: %d",g_TaskRealizeAgent->fore_func_state);
//test
	decision_state_pub.publish(state_);
}

void BlackBoard::PubMembers(){
    // if(g_BlackBoardAgent->car_id==2)
    //     logger.DEBUGINFO(car_id,"PubMembers");
    std_msgs::Int8MultiArray msg;
    for(auto i:g_GroupAsBasicLogicAgent->GroupMember)
        msg.data.push_back(i);
    members_pub.publish(msg);
}

	geometry_msgs::Pose BlackBoard::GetGoal(){
        return goal;
    }

