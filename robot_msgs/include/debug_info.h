#pragma once
#include "iostream"
#include "cstdio"
#include "string"
#include "vector"
#include <stdarg.h>
#include "ros/ros.h"

/**
 *  this file is used for debug infomation 
 * */

namespace Debug{
    
    struct DebugLogger{
        /**
         * 调用这四个
         * 
         * */
        void DEBUGINFO(int id, const char *pszFmt, ...);

        void WARNINFO(int id,const char *pszFmt, ...);

        void DEBUGINFO(const char *pszFmt, ...);

        void WARNINFO(const char *pszFmt, ...);
        /**
         * 调用这四个
         * 
         * */
        /**
         * 用于初始化哪个车输出debug信息
         */
        void init_logger(int id);

        private:

        std::string format(const char *pszFmt, ...);

        std::string format_debug_msg(int id,const std::string &msg);

        std::string format_warn_msg(int id,const std::string &msg);

        std::string format_debug_msg(const std::string &msg);

        std::string format_warn_msg(const std::string &msg);

        bool should_output(int id);
        bool should_i_output;
    };


}

extern Debug::DebugLogger logger;