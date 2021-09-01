#include "my_action.h"
#include "my_lib/WashDishAction.h"
#include "iostream"

using namespace my_lib;

//implement of WashDish action client
template <typename Action>
class WashDishClientRealize :public ActionClientRealize<Action>{
public:
    WashDishClientRealize(std::string action_name_,int car_id_,ros::NodeHandle &nh):ActionClientRealize<Action>(action_name_,car_id_,nh){}
    void SetGoal(){
        this->action_goal.dish_num = 10;
    }
};


//implement of WashDish server client
template <typename Action>
class WashDishServerRealize :public ActionServerRealize<Action>{
public:
    WashDishServerRealize(std::string action_name_,int car_id_,ros::NodeHandle &nh):ActionServerRealize<Action>(action_name_,car_id_,nh){}
    bool CustomExcuteAction(const GoalConstPtr<Action>& goal){
        ros::Rate loop(1);
        int count{0};
        while(ros::ok()){
            if(goal->dish_num == count){
                return 1;
            }
            count++;
            loop.sleep();
            this->logger.DEBUGINFO(this->car_id,"[%s server]: clean dish num %d",this->action_name.c_str(),count);
        }
    }
};

int main(int argc,char** argv){
    ros::init(argc,argv,"my_action_demo");
    ros::NodeHandle nh;
    //create client and server
    WashDishClientRealize<my_lib::WashDishAction> wash_dish_client("wash_dish",1,nh);
    WashDishServerRealize<my_lib::WashDishAction> wash_dish_server("wash_dish",1,nh);
    //excute once
    wash_dish_client.SetGoal();
    wash_dish_client.StartAction();
    ros::Rate loop(20);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}