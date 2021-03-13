#ifndef _LOGGING_CONFIG_H
#define _LOGGING_CONFIG_H


#ifndef DISABLE_LOGGING
//#define DISABLE_LOGGING
#endif
//#define DISABLE_INFO_LOGGING
//#define DISABLE_DEBUG_LOGGING
//#define DISABLE_WARN_LOGGING


/**
 ** Implementation...
 **/

#if  defined(DISABLE_LOGGING) || defined(DISABLE_INFO_LOGGING)
#define LOGINFO(...) 
#else
#define LOGINFO(...) loginfo(__VA_ARGS__)
#endif

#if  defined(DISABLE_LOGGING) || defined(DISABLE_DEBUG_LOGGING)
#define LOGDEBUG(...) 
#else
#define LOGDEBUG(...) loginfo(__VA_ARGS__)
#endif

#if  defined(DISABLE_LOGGING) || defined(DISABLE_DEBUGLV2_LOGGING)
#define LOGDEBUGLV2(...) 
#else
#define LOGDEBUGLV2(...) loginfo_if(__VA_ARGS__)
#endif

#if  defined(DISABLE_LOGGING) || defined(DISABLE_WARN_LOGGING)
#define LOGWARN(...) 
#else
#define LOGWARN(...) loginfo(__VA_ARGS__)
#endif


#ifndef DISABLE_FILE_LOGGING
#ifdef DISABLE_LOGGING
#define DISABLE_FILE_LOGGING
#endif
#endif

//#define ENABLE_TRACE_LOGGING
//#define ENABLE_DEBUGMSG_LOGGING

//#define _HAS_ZMQ_					// Enable ZMQ
//#define _HAS_REMOTE_TOOLS_		// Enable Remote Input
//#define ENABLE_STDOUT_LOGGING		// print debug info to stdout if ext_loggers're set

//#define ENABLE_LATENCYTEST_LOGGING

#endif