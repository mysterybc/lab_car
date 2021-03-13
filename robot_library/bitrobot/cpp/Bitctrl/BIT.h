#ifndef BIT_CONTROLLER_H
#define BIT_CONTROLLER_H

#if defined _WIN32
#ifdef _BITCTRL_EXPORT
#define DLL_PUBLIC  __declspec(dllexport)
#elif defined(_BITCTRL_STATIC)
#define DLL_PUBLIC 
#else
#define DLL_PUBLIC __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#ifdef _BITCTRL_EXPORT
#define DLL_PUBLIC  __attribute__ ((visibility ("default")))
#else
#define DLL_PUBLIC 
#endif
#else
#define DLL_PUBLIC 
#endif
#endif


#ifdef __cplusplus
extern "C" {
#endif

/*
 * 演示实验接口:
 * - ControllerInit    : 实验开始前调用, 设置实验内容, 当前平台的编号(四个平台对应于: 1, 2, 3, 4)
 * - ControllerState   : 实验过程中调用, 用于判断是否已完成预定任务, 是否存在错误
 * - ControllerCompute : 实验过程中调用, 计算控制量
 * - ControllerSetHumanInput : 实验过程中调用, 用于传递人为干预信息 (只在人为干预实验中使用)
 * - ControllerStop    : 实验结束后调用, 控制器保存数据、清理等
 *
 * 演示实验中暂时不使用的接口:
 * - ControllerSetPath : 实验过程中调用, 用于更新轨线
 * - ControllerSetObstacles : 实验过程中调用, 用于更新当前机器人附近的障碍物信息
 *
 * 测试用接口: (此接口为方便线下仿真设计 具体使用见 example)
 * - ControllerGetPtr : 获得当前控制器所用的对象指针
 * - ControllerSetPtr : 设置当前控制器所用的对象指针
 * - ControllerGetMsg : 获取当前控制器返回的调试信息(4个float)
 * - ControllerHandleMsg : 处理其他控制器发来的消息
 *
 * 辅助程序接口:
 * - GPS2Local : 用于将经纬度坐标系统转换为局部坐标系统
 *
 * 需要平台提供的信息 
 * - 平台状态 : 编号、位置坐标、角度、速度
 * - 邻居平台状态 : 同上
 * - 当前的人为干预信息 : 人的期望线速度、期望角速度
 * - 当前环境中的障碍信息 : 中心点坐标及半径
 * - 当前的时间信息 : 实验进行了多久
 *
 * 提供给平台的量
 * - 计算得到的控制量: 期望的线速度和角速度
 * - 控制器当前的状态: 是否出错、是否可以结束实验
 */
 
 
/*
 * 各信息单位
 * - 位置坐标: cm 
 * - 角度量: 角度(-180 ~ +180)
 * - 线速度: cm/s
 * - 角速度: 角度/s
 */

 /* 
 * 基本数据类型说明
 * - int   为32位有符号整型
 * - uint  为32位无符号整形
 * - float 为32位浮点数
 */
typedef unsigned int uint;
typedef float        real_t;

/**** 演示实验编号设置 ****/
#define EXP_GENERAL        0	// 最终实验ver 0.1
#define EXP_RENDEZVOUS     1    // 会聚定姿实验
#define EXP_FORMATION      2    // 编队行进实验
#define EXP_INTERVENTION   3    // 人为干预实验
#define EXP_FAULT_DETECT   4    // 故障检测实验
#define EXP_TRACEHUMAN	   5	// 人的跟随实验
#define EXP_TRACKING       6    // 单体行进实验
#define EXP_TESTID_BEG     10	// 在此范围内的ID为测试ID
#define EXP_TESTID_END	   20
#define EXP_TEST_EIGHT     21   //
#define EXP_TEST_LINE      22   //

/**** ExpGeneral的功能编号 ****/
#define FUNC_UNSET           0  // 功能未设置
#define FUNC_RENDEZVOUS      1	// 聚集到一起
#define FUNC_TRACKING_SINGLE 2	// 单独沿轨线前进
#define FUNC_TRACKING_GROUP  3	// 编队沿轨线前进
#define FUNC_TRACE_HUMAN     4  // 编队跟随人绕过障碍
#define FUNC_TURN_TO         5  // 原地转向目标
#define FUNC_TELE_BASIC      6  // 最基本的遥控控制
#define FUNC_TELE_DXY        7  // 一个通过指定偏移量进行遥控的方法
#define FUNC_TEST_EIGHT      11 // 绕八的任务
#define FUNC_TEST_MOTION     12 // 运动测试任务
#define FUNC_FAULT_DETECT    (1<<8)   // 进行故障检测
#define FUNC_HUMAN_TELE      (1<<9)   // 允许人为干预
#define FUNC_ACT_FAULTY      (1<<10)  // 假装出现故障
#define FUNC_LATENCY_TEST    (1<<11)  // 进行通讯延迟测试
#define FUNC_ALL             0xffff	  // 用于关闭所有功能

/**** Error 编号设置 *****/
#define INVALID_EXPID        1
#define INVALID_ROBOTID      2
#define OTHER_INIT_ERROR     3

#define STATE_GOOD           0	// 正常运行
#define STATE_TARGET_REACHED 1	// 到达目标
#define OTHER_STATE_ERROR    2	// 异常错误
#define STATE_FAULT_DETECTED 3	// 检测出本机故障(故障检测模块返回)

#define ERR_INVALID_FUNCID     1
#define ERR_FUNC_UNIMPLEMENTED 2
#define ERR_FUNC_INIT_ERROR    3

/*********Debug 输出设置***********/
#define DEBUG_INFO_ALL			 0xff	// 所有Debug信息
#define DEBUG_INFO_FUNCTION		(1<<0)	// 调用其他函数时显示相应调试信息
#define DEBUG_INFO_COMPUTE		(1<<1)  // 调用ControllerCompute时显示计算结果
#define DEBUG_INFO_TELE			(1<<2)  // 调用ControllerSetHumanInput时显示收到的数据
#define DEBUG_INFO_HUMAN_POS	(1<<3)  // 调用ControllerSetHumanPosition时显示收到的数据
#define DEBUG_INFO_STATES       (1<<4)  // 调用ControllerCompute时打印所有人的状态
#define DEBUG_INFO_AVOID_ALL    (1<<5)  // 完全的避障算法调试信息
#define DEBUG_INFO_AVOID_SIM    (1<<6)  // 简易的避障算法调试信息
#define DEBUG_INFO_COM_SEND     (1<<7)  // 发送内容的debug信息
#define DEBUG_INFO_COM_RECV     (1<<8)  // 接收内容时的debug信息
#define DEBUG_INFO_FDETECT      (1<<9)  // 故障检测的调试信息

/*********File Logging Option***********/
#define LOG_FILE_INFO    (1<<0)  // 将打印的log信息也保存到文件中
#define LOG_FILE_TRACE   (1<<1)  // 将状态信息保存到文件中
#define LOG_FILE_DEBUG   (1<<2)  // 将Debug信息保存到文件中
#define LOG_FILE_ALL     0xff

/************************
 ****接口数据结构定义****
 ************************/

// 状态信息
// - 坐标及速度均是全局坐标系下的
typedef struct {
	uint ID;        // 机器人的编号 
	real_t x,  y;   // 当前位置
	real_t heading; // 当前朝向角
	real_t v,  w;   // 当前线速度, 角速度
}StateInfo;

// 时间, 进度信息
// - loopCounter = 0 当且仅当 timeElapsed = 0 
typedef struct {
	uint timeElapsed;   // 实验进行了多久 单位: ms 
	uint loopCounter;   // 控制环路计数器 (从0开始计数)
	uint globalTime;	// GPS时间
}TimeInfo;

// 控制输出
typedef struct {
	real_t v;    // 期望线速度
	real_t w;    // 期望角速度
}ControlInfo;

// 干预信息
typedef struct {
	real_t  v;  // 期望的线速度
	real_t  w;  // 期望的角速度
}InterventionInfo; 

// 规划的路径信息
typedef struct{
	real_t x,  y; // 路点的局部坐标
	real_t v;	  // 路点的速度
}PathPoint;

// 环境, 障碍信息
// 障碍物统一视为圆
typedef struct {
	real_t  x, y;      // 障碍中心点坐标
	real_t  radius;    // 障碍的半径, 单位同位置量
}ObstacleInfo;


/*
 * 坐标转换函数 GPS2Local
 * 功能: 将输入的GPS坐标、航向转换为某局部坐标系统下的坐标、航向
 *       并将所得结果存入res.x, res.y, res.heading中
 *       所得结果
 * 输入: lat 维度, lon 精度, heading 航向角(北为0, 东为90)
 * 输出: x, y        局部坐标系下的坐标
 *       headingNew  局部坐标系下的航向角
 */
DLL_PUBLIC void GPS2Local(double lat, double lon, double heading, real_t* x, real_t* y, real_t* headingNew);

/*
 * 坐标转换函数设置 (测试用)
 * 功能: 获取、设置当前GPS2Local函数所用的坐标原点信息
 */
DLL_PUBLIC void GPSSetAnchor(double lat, double lon, double heading);
DLL_PUBLIC void GPSGetAnchor(double* lat, double* lon, double* heading);

/*
 * GPS辅助功能
 * 功能: 计算两GPS点间距离(GSPDistance), 显示给定点附近的GPS信息
 */
DLL_PUBLIC double GPSDistance(double lat1, double lon1, double lat2, double lon2);
DLL_PUBLIC void GPSShowStats(double lat, double lon);
 
 /*
  * 测试接口
  * 功能: 获取、设置控制器内部对象指针
  *       可实现在同一个程序中调用多个不同的控制器
  * 输入: newPtr 必须为某次 ControllerGetPtr 的返回值 或 0
  * 输出: ControllerGetPtr: 当前控制器的内部对象指针
  *       ControllerSetPtr: 设置之后控制器的内部对象指针
  */
DLL_PUBLIC void* ControllerGetPtr();
DLL_PUBLIC void* ControllerSetPtr(void* newPtr);


/*
 * I/O配置接口:
 * 功能: 设置控制器默认读取配置的路径
 *		 默认输出log信息的路径
 * 要求: 输入路径是C字符串, 即以'\0'结尾
 *       输入路径是有效路径, 且以'\\' or '/' 结尾
 *       例 C:\\config\\  
 *          /home/config/
 */
DLL_PUBLIC void ControllerSetConfigDir(const char* conf_dir);
DLL_PUBLIC void ControllerSetLogDir(const char* log_dir);
DLL_PUBLIC int  ControllerSetConfigString(const char* config_string, int length);

/* 
 * Debug 接口: 仅用于调试  十分危险  请勿调用
 */
DLL_PUBLIC void ControllerSetDebugItem(uint item_id, void* item_ptr, void* args);
DLL_PUBLIC void ControllerGetDebugItem(uint item_id, void* item_ptr, void* args);

/*
 * 获取当前控制器要发送的消息
 * 输入: buf 不小于 4个float大小 (32个字节)
 * 返回: 写入了多少字节
 */ 
DLL_PUBLIC int  ControllerGetMsg(void* buf);
DLL_PUBLIC void ControllerHandleMsg(int srcID, void* data, uint size);
DLL_PUBLIC int  ControllerGetDebugMsg(void* buf);


DLL_PUBLIC int ControllerCommand(void* str, uint length);

/*
 * 控制器初始化函数 ControllerInit
 * 功能: 为expID所表示的实验, robotID所表示的机器人 初始化控制器
 * 输入: expID    见开头定义的实验编号
 *       robotID  1 or 2 or 3 or 4 表示四个机器人
 * 输出: 0  正常
 *       1  expID不识别
 *       2  robotID不识别
 *       3  其他错误
 */
DLL_PUBLIC int ControllerInit(int expID, int robotID);

/*
 * 设置控制器的功能
 * 输入: functionID, 见以 FUNC_ 开头的宏定义
 *       on_off,     !=0, 开启功能
 *                   ==0, 关闭功能
 * 返回: 是否设置有错误, 0为成功 非零为设置失败
 */
DLL_PUBLIC int ControllerSetFunction(int funciotnID, int on_off);


/*
 * 设置控制器的功能
 * setFormationGroup, 设置编队的分组, 让四个车可以分成多组编队
 * setFormationShape, 设置默认编队队形, 如果dx, dy为NULL, 则按边长为1 meter的正多边形进行编队
 *                    用scaling可以设置队形的伸缩系数
 *                    注意: dx, dy单位是米
 */
DLL_PUBLIC void ControllerSetFormationGroup(int groupIndex);
DLL_PUBLIC void ControllerSetFormationShape(uint nrobot, const int* robotID, const real_t* dx, const real_t* dy, float scaling);


/*
 * 设置控制器Debug信息输出
 * 输入: debug_info, 见以 DEBUG_INFO_ 开头的宏定义
 *       on_off,     !=0, 开启此类信息的输出
 *                   ==0, 关闭此类信息的输出
 */
DLL_PUBLIC void ControllerSetDebugInfo(int debug_info, int on_off);

/* 设置控制器暂停/恢复运动
 * 输入: pause,   != 0  暂停运动
 *                == 0  恢复运动
 */
DLL_PUBLIC void ControllerPause(int pause);

/*
 * 获取控制器的状态 ControllerState
 * 功能: 获取控制器的状态
 * 输出: 0 正常
 *       1 正常, 但已到达目的地/完成预定任务, 可以停止运行
 *       2 其他错误, 需要立即终止运行       
 *       3 检测出本机故障 ( 故障检测时 )
 */
DLL_PUBLIC int ControllerState();

/*
 * 获取控制器当前执行任务的进度 ControllerTaskProgress
 * 输出: 0~1 间的小数, 0表示任务刚开始, 1表示任务已完成
 * 说明: 当控制器未实现此功能时, 默认始终返回0
 */
DLL_PUBLIC real_t ControllerTaskProgress();

/*
 * 轨线设置/更新函数 ControllerSetPath
 * 功能: 通过路径点设置/更新期望轨线
 * 输入: num     路点数量
 *       path    路点数组, 见PathPoint的定义
 *       pathvel 期望路径上的速度
 * 说明: 会在内部对path中的数组进行复制
 *       路点坐标采用cm为单位 pathvel采用cm/s为单位
 */
DLL_PUBLIC void ControllerSetPath(uint num, const PathPoint* path);


/* 
 * 设置目标点
 * 输入: target.x, target.y, 目标点的位置
 *       target.v            未使用
 */
DLL_PUBLIC void ControllerSetTargetPoint(PathPoint target);


/*
 * 障碍物设置/更新函数 ControllerSetObstacles
 * 功能: 设置更新当前机器人周围的障碍物
 * 输入: num        障碍物数量
 *       obstacles  障碍物信息, 见ObstacleInfo定义
 * 说明: 会在内部对path中的数组进行复制
 */
DLL_PUBLIC void ControllerSetObstacles(uint num, const ObstacleInfo* obstacles);


/*
 * 人为干预设置函数 ControllerSetHumanInput
 * 功能: 将人为干预信息传给控制器
 * 输入: humanInput 干预信息, 见InterventionInfo的定义
 */
DLL_PUBLIC void ControllerSetHumanInput(InterventionInfo humanInput);

/*
 * 设置人的位置坐标
 * x, y是经纬度坐标经GPS2Local转换而来的局部坐标
 */
DLL_PUBLIC void ControllerSetHumanPosition(real_t x, real_t y);

/*
 * 计算控制量 ControllerCompute
 * 功能: 计算控制量
 * 输入: tm 当前的时间信息, 见TimeInfo的定义
 *       me 被控制的机器人的状态信息, 见StateInfo的定义
 *       nOthers 收到了多少个邻居的信息
 *       others  邻居的状态信息, 见StateInfo的定义
 * 输出: 期望的线速度和角速度
 * 说明: 会在内部对other数组进行复制
 */
DLL_PUBLIC ControlInfo ControllerCompute(TimeInfo tm, StateInfo me, uint nOthers, const StateInfo* others);

/* 路径规划 (仅供测试, 待修改) 
 * 功能: 规划路径(基于8向矩形栅格网络的A*算法)
 */
DLL_PUBLIC int ControllerComputePath(PathPoint q0, PathPoint q1, real_t safe, real_t vel, PathPoint* path, int max_length);



/*
 * 指示控制器结束 ControllerStop
 * 功能: 指示控制器实验结束(控制器开始保存数据、销毁临时变量etc)
*/
DLL_PUBLIC void ControllerStop();

 

#ifdef __cplusplus
}   // extern "C"
#endif

#endif  // BIT_CONTROLLER_H
