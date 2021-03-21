//c++ standard
#include "algorithm"
//third party lib
#include "jsoncpp/json/json.h"
//ros
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "robot_msgs/RobotStates.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/UInt8MultiArray.h"
//my lib
#include "zmq_lib.h"
#include "msgs.h"
#include "my_debug_info.h"
#include "my_param_server.h"


int main(int argc, char** argv){
    ros::init(argc,argv,"test_receiver");
    ros::NodeHandle nh;

    //config id & ip
    std::string my_ip = "127.0.0.1:6661";


    //zmq_init
    zmq::context_t ctx(1);
    zmq_lib::Receiver state_receive(std::vector<std::string>{my_ip},ctx);

    //data
    RobotStateMsg robot_state(1);
    RobotTaskMsg robot_task(1);
    RobotPerceptionMsg robot_percepyion(1);


    ros::Rate loop(20);
    while(ros::ok()){
        std::vector<std::string> msgs;
        state_receive.receiveMsg(msgs);
        for(auto msg:msgs){
            Json::Value json;
            Json::Reader reader;
            reader.parse(msg.c_str(),json);
            if(json["message_id"].type() != Json::nullValue){
                switch(json["message_id"].asInt()){
                    case ROBOT_STATE: robot_state.GetDataFromMsg(json);std::cout << "机器人状态信息为: " << json.toStyledString() << std::endl;break;
                    case RRBOT_TASK: robot_task.GetDataFromMsg(json);std::cout << "机器人任务状态为: " << json.toStyledString() << std::endl;break;
                    case ROBOT_PERCEPTION: robot_state.GetDataFromMsg(json);std::cout << "障碍物信息太长了不显示" << std::endl;break;
                }
                
            }
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}