#include "my_service.h"
#include "my_lib/AddNum.h"
#include "thread"

using namespace my_lib;

template <typename Service>
class AddNumServiceClient: public ServiceClientRealize<Service>{
public:
    AddNumServiceClient(std::string service_name_,int car_id_):ServiceClientRealize<Service>(service_name_,car_id_){}
    void SetRequest(Service &service){
        service.request.num1 = 10;
        service.request.num2 = 10;
    }
};

template <typename Service>
class AddNumServiceServer: public ServiceServerRealize<Service>{
public:
    AddNumServiceServer(std::string service_name_,int car_id_):ServiceServerRealize<Service>(service_name_,car_id_){}
    bool OnNewService(typename Service::Request & req,typename Service::Response &res ){
        res.sum = req.num1 + req.num2;
        this->logger.DEBUGINFO(this->car_id,"[%s server] calculate sum is %d",this->service_name.c_str(),res.sum);
        return true;
    }
};

/**
 * @brief if i create server thread, I don't know how to quit without extra signal.
 */
// void ServerThread(){
//     AddNumServiceServer<my_lib::AddNum> add_num_server("add_num",1);
//     ros::Rate loop(10);
//     while(ros::ok()){
//         ros::spinOnce();
//         loop.sleep();
//     }
// }

void ClientThread(){
    AddNumServiceClient<my_lib::AddNum> add_num_client("add_num",1);
    add_num_client.CallServer();
}


int main(int argc,char ** argv){
    ros::init(argc,argv,"my_service_demo");
    ros::NodeHandle nh;
    //define service & client
    /**
     * @brief : client call service will block the main thread,
     *          so I create client and server in different thread.
     */
    AddNumServiceServer<my_lib::AddNum> add_num_server("add_num",1);
    std::thread client_thread(&ClientThread);

    ros::Rate loop(10);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
    client_thread.join();
    return 0;

}