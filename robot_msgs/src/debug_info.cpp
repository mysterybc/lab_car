#include "debug_info.h"
#include "time.h"


namespace Debug{
    void DebugLogger::init_logger(int id){
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

    std::string DebugLogger::format_debug_msg(int id,const std::string &msg){
        return "\033[40;37m[car " + std::to_string(id) + "] : " + msg + "\033[0m\n";
    }

    std::string DebugLogger::format_warn_msg(int id,const std::string &msg){
        return "\033[40;31m[car " + std::to_string(id) + "] : " + msg + "\033[0m\n";
    }

    std::string DebugLogger::format_debug_msg(const std::string &msg){
        return "\033[40;37m[unknow] : " + msg + "\033[0m\n";
    }

    std::string DebugLogger::format_warn_msg(const std::string &msg){
        return "\033[40;31m[unknow] : " + msg + "\033[0m\n";
    }

    void DebugLogger::DEBUGINFO(int id, const char *pszFmt, ...){
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

    void DebugLogger::WARNINFO(int id,const char *pszFmt, ...){
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

    void DebugLogger::DEBUGINFO(const char *pszFmt, ...){
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

    void DebugLogger::WARNINFO(const char *pszFmt, ...){
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

    bool DebugLogger::should_output(int id){
        ros::NodeHandle nh;
        std::vector<int> idList;
        nh.getParam("/debug_output_id",idList);
        if(std::find(idList.begin(),idList.end(),id) != idList.end()){
            return true;
        }
        return false;
    }

}