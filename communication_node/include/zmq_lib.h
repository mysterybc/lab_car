#pragma once

#include "zmq.hpp"
#include "string"
#include "vector"
#include "map"
#include "jsoncpp/json/json.h"

namespace zmq_lib{
    struct Addr{
        Addr(){}
        Addr(std::string ip_,int port_){
            ip = ip_;
            port = port_;
        }
        std::string ip;
        int port;
    };

    std::string fmtaddr(Addr& addr){
        return std::string("tcp://") + addr.ip + std::string(":") + std::to_string(addr.port);
    }

    std::string fmtaddr(std::string ip_port){
        return std::string("tcp://") + ip_port;
    }

    struct Sender{
        Sender(){}
        Sender(Addr& addr,zmq::context_t &ctx):
            pub(ctx,ZMQ_PUB){
                pub.bind(fmtaddr(addr));
        }
        Sender(std::string& addr,zmq::context_t &ctx):
            pub(ctx,ZMQ_PUB){
                pub.bind(fmtaddr(addr));
        }
        //发送string
        int sendMsg(const std::string& str){
            zmq::message_t message(str.c_str(),str.size());
            pub.send(message);
        }
         //发送json
        int sendMsg(const Json::Value& json){
            std::string data = json.toStyledString();
            sendMsg(data);
        
        }
        //动态添加
        void addIp(const std::string ip){
            pub.bind(ip);
        }
        //动态移除
        void removeIp(const std::string ip){
            pub.unbind(ip);
        }

    private:
        zmq::socket_t pub;

    };


    struct Receiver{
        Receiver(){}
        Receiver(std::vector<std::string> ips,zmq::context_t &ctx):
            sub(ctx,ZMQ_SUB){
            sub.setsockopt(ZMQ_SUBSCRIBE,"",0);
            for(auto ip:ips){
                sub.connect(fmtaddr(ip));
            }
        }
        ~Receiver(){
            sub.close();
        }
        //TODO 接收topic有用么？
        int receiveMsg(std::vector<std::string>& vec){
            zmq::message_t recv_msg;
            while(sub.recv(&recv_msg,ZMQ_DONTWAIT)){
                std::string str{recv_msg.to_string()};
                vec.push_back(str);
            }
        }
        int receiveMsg(std::string& vec){
            zmq::message_t recv_msg;
            while(sub.recv(&recv_msg,ZMQ_DONTWAIT)){
                vec = recv_msg.to_string();
            }
        }
        //动态添加
        void addIp(const std::string ip){
            sub.connect(fmtaddr(ip));
        }
        //动态移除
        void removeIp(const std::string ip){
            sub.disconnect(fmtaddr(ip));
        }


    private:
        zmq::socket_t sub;
    };
}//end namespace zmq_lib
