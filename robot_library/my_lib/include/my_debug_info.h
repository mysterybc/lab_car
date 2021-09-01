#pragma once
#include "iostream"
#include "cstdio"
#include "string"
#include "vector"
#include <stdarg.h>
#include "ros/ros.h"
#include "time.h"

/**
 *  @description: this file is used for debug infomation 
 * */
namespace Debug{
    
    struct DebugLogger{
        void init_logger(int id){
            ros::NodeHandle nh;
            should_i_output = should_output(id);
        }

        std::string format(const char *pszFmt, ...)
        {
            std::string str;
            va_list args;
            va_start(args, pszFmt);
            {
                char data[100];
                vsnprintf(data, 100, pszFmt, args);
                str.assign(data);
            }
            va_end(args);
            return str;
        }

        std::string format_debug_msg(int id,const std::string &msg){
            return "\033[40;37m[car " + std::to_string(id) + "] : " + msg + "\033[0m\n";
        }

        std::string format_warn_msg(int id,const std::string &msg){
            return "\033[40;31m[car " + std::to_string(id) + "] : " + msg + "\033[0m\n";
        }

        std::string format_debug_msg(const std::string &msg){
            return "\033[40;37m[unknow] : " + msg + "\033[0m\n";
        }

        std::string format_warn_msg(const std::string &msg){
            return "\033[40;31m[unknow] : " + msg + "\033[0m\n";
        }

        void DEBUGINFO(int id, const char *pszFmt, ...){
            if(!should_i_output){
                return ;
            }
            std::string str;
            va_list args;
            va_start(args, pszFmt);
            {
                char data[100];
                vsnprintf(data, 100, pszFmt, args);
                str.assign(data);
            }
            va_end(args);
            printf("%s",format_debug_msg(id,str).c_str());
        }

        void WARNINFO(int id,const char *pszFmt, ...){
            if(!should_i_output){
                return ;
            }
            std::string str;
            va_list args;
            va_start(args, pszFmt);
            {
                char data[100];
                vsnprintf(data, 100, pszFmt, args);
                str.assign(data);
            }
            va_end(args);
            printf("%s",format_warn_msg(id,str).c_str());
        }

        void DEBUGINFO(const char *pszFmt, ...){
            std::string str;
            va_list args;
            va_start(args, pszFmt);
            {
                char data[100];
                vsnprintf(data, 100, pszFmt, args);
                str.assign(data);
            }
            va_end(args);
            printf("%s",format_debug_msg(str).c_str());
        }

        void WARNINFO(const char *pszFmt, ...){
            std::string str;
            va_list args;
            va_start(args, pszFmt);
            {
                char data[100];
                vsnprintf(data, 100, pszFmt, args);
                str.assign(data);
            }
            va_end(args);
            printf("%s",format_warn_msg(str).c_str());
        }

        bool should_output(int id){
            ros::NodeHandle nh;
            std::vector<int> idList;
            nh.getParam("/debug_output_id",idList);
            if(std::find(idList.begin(),idList.end(),id) != idList.end()){
                return true;
            }
            return false;
        }
        int should_i_output;
    };


};