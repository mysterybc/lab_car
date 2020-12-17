#ifndef _BASE_H
#define _BASE_H

#include "../BIT.h"
#include "helper.hpp"
#include <vector>
#include <deque>
#include <string>
#include <fstream>
#include <set>
#include "./fmt/format.h"
#include "logconfig.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include "Action.hpp"
#include "ssconfig.hpp"
#include "Obstacles.hpp"
#include "toolbox/LoggingUtil.hpp"
#if defined(ENABLE_SSNETWORK)
#include "tcpcom.hpp"
#endif

#ifdef __GNUC__
#if __GNUC__ < 4
	#ifndef nullptr
    #define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
	#endif
#elif __GNUC__ == 4 && __GNUC_MINOR__ < 6
    #ifndef nullptr
    #define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
    #endif
#endif
#endif

namespace impl{

// Forward Declare
class ShapeTrace;

inline ControlInfo saturate(ControlInfo u, real_t limitV, real_t limitW){
	u.v = saturate(u.v, limitV);
	u.w = saturate(u.w, limitW);
	return u;
}

/*
 * 输入的StateInfo信息都被放到了这里面
 * 所有数组的第一个是me 后面的按顺序来
 */
struct DataSet{
	DataSet(){}
	DataSet(const StateInfo& me, uint nOthers, const StateInfo* others){
		setData(me, nOthers, others);
	}
	
	void setData(const StateInfo& me, uint nOthers, const StateInfo* others){
		size_t sz = 1 + nOthers;
		//reserveAndResize(sz, xy, vw, vxy, th, ID);
		xy.resize(sz);
		vw.resize(sz);
		vxy.resize(sz);
		th.resize(sz);
		ID.resize(sz);

		setData(0, me);
		for (size_t i = 1; i < sz; ++i){
			setData(i, others[i - 1]);
		}
	}
	size_t size() const { return ID.size(); }
	bool empty() const { return size() == 0; }
	bool has(int robotID) const {
		return std::find(ID.begin(), ID.end(), robotID) != ID.end();
	}
	size_t indexOf(int robotID) const{
		return std::find(ID.begin(), ID.end(), robotID) - ID.begin();
	}

	void filterOut(int id){
		std::vector<int>::iterator iter = std::find(ID.begin(), ID.end(), id);
		if (iter != ID.end()){
			size_t k = iter - ID.begin();

			xy.erase(xy.begin() + k);
			vxy.erase(vxy.begin() + k);
			vw.erase(vw.begin() + k);
			th.erase(th.begin() + k);
			ID.erase(ID.begin() + k);
		}
	}
	void filterOut(const std::vector<int>& black_list){
		for (size_t i = 0; i < black_list.size(); ++i){
			filterOut(black_list[i]);
		}
	}
	void filterOut(const std::set<int>& black_list){
		std::set<int>::const_iterator iter = black_list.begin();
		while (iter != black_list.end()){
			filterOut(*iter);
			iter++;
		}
	}


	std::vector<pt2D>   xy;
	std::vector<pt2D>   vxy;
	std::vector<pt2D>   vw;
	std::vector<double> th;
	std::vector<int>    ID;
protected:
	void setData(size_t i, const StateInfo& it){
		ID[i] = it.ID;
		xy[i].x = it.x; xy[i].y = it.y;
		vw[i].x = it.v; vw[i].y = deg2rad(it.w);
		th[i] = anglediff(deg2rad(it.heading), 0);
		vxy[i].x = it.v * std::cos(th[i]);
		vxy[i].y = it.v * std::sin(th[i]);
	}
	
	template<class T>
	void swapWrap(std::vector<T>& v, size_t i, size_t j){
		std::swap(v[i], v[j]);
	}
	void swap(size_t i, size_t j){
		swapWrap(xy,  i, j);
		swapWrap(vxy, i, j);
		swapWrap(vw,  i, j);
		//swapWrap(dxy, i, j);
		swapWrap(th, i, j);
		swapWrap(ID, i, j);
	}

};

typedef void(*log_f)(void*, const char*);

class Controller{
public:
	Controller(int robotID);
	virtual ~Controller();

	// This is the external interface
	virtual size_t getCtrlMsg(void* buf);
	virtual void   handleMsg(int srcID, void* dat, size_t size);
	virtual size_t getDebugMsg(void* buf);
	virtual size_t getDebugMsgExt(void* buf){ return getDebugMsg(buf); }
	virtual int   handleCommand(void* str, size_t length);


	// This is for debug.... A dangerous interface...
	// Use with cautious...
	virtual void setDebugItem(uint item_id, void* item_ptr, void* args){}
	virtual void getDebugItem(uint item_id, void* item_ptr, void* args){}

	// Get Controller State
	int getMyID() const { return ID; }
	virtual int state();
	virtual real_t task_progress();
	
	// These two interfaces MUST be equivalent to each other
	virtual ControlInfo compute(TimeInfo tm, StateInfo me, uint nOthers, const StateInfo* others);
	virtual ControlInfo compute(TimeInfo tm, DataSet& dat);

	// Interface Added By Chengsi Shang 2020/10/22 to support path planning
	virtual bool computePath(std::vector<PathPoint>& path, pt2D q0, pt2D q1, real_t v, real_t safe);
	virtual bool computePath(std::vector<PathPoint>& path, const std::vector<PathPoint>& target, real_t safe, bool startAtMe);

	// Interface Added By Chengsi Shang 2020/08/25 to use tcpcom
	virtual void before_compute();
	virtual void after_compute();
	virtual const char* controllerName() const { return "Base"; }

	virtual int  setFunction(int funcID, bool is_on){ return ERR_FUNC_UNIMPLEMENTED; }
	virtual bool setFormationShape(const std::vector<int>& id, std::vector<float> dx, std::vector<float> dy, float dxy_scaling) { return false; }
	virtual void setFormationGroup(int groupID) {}
	virtual void setPause(int paused);
	virtual void setHumanInput(InterventionInfo humanInput);
	//virtual void setHumanInputEx(int type, InterventionInfo humanInput);
	virtual void setHumanPoisition(pt2D pos);
	virtual void setObstacles(uint num, const ObstacleInfo* obstacles);
	virtual void setPath(uint num, const PathPoint* path);  // (External) cm, cm/s
	virtual void setPath(const ShapeTrace& trace);          // (Internal) m, s
	virtual void setTargetPoint(PathPoint pt){}

	virtual void stop();
	void setExtLog(void* ptr, log_f extLog);

	// Functions used to implement the compute
	virtual pt2D        innerCompute2D(TimeInfo tm, DataSet& dat);
	virtual ControlInfo innerCompute(TimeInfo tm, DataSet& dat);
	virtual ControlInfo getCompensate(){ ControlInfo u= {0,0}; return u; }
	//virtual pt2D collisionAvoid(DataSet& dat, pt2D currU);
	
	virtual void recording(TimeInfo tm, const DataSet& dat, ControlInfo u);

	void disableLogging(int option);
	void enableLogging(int option, const std::string& prefix, bool withTimeStamp);
	
	// base class function
	void setConfigDir(const std::string& dir) { configDir = dir; }
	int loadconfig(const std::string& file_name = "");
	virtual int loadconfig(sscfg::ConfigFile& file);
	int loadmapdata(bool overwrite);
	int initialize_extcom(sscfg::ConfigFile& file);

	// Config Debug Info
	void setDebugInfoOption(int log_option, bool is_on);
	int  currLogOption() const;

protected:
	// derived class's maker to the base
	virtual bool useCollisionAvoid() { return true;  }
	virtual bool innerUse2D()	     { return true; }
	//virtual uint timeBeforeStart()   { return 1000;  }
	
	// base's helper functions
	//virtual pt2D dAvoidPF(pt2D err, double aoivdmin);
	virtual ControlInfo fromVxy2VW(double th, pt2D vxy);

	virtual void filterData(double dt, DataSet& last, DataSet& now);

public:
	void loginfo(const char* str){
		//if (ID != 1) return;
		if (str) { 
			if (ext_log) ext_log(ext_ptr, str);
			else{
#ifdef ENABLE_STDOUT_LOGGING
				printf("[%d][%d] %s\n", ID, lastTime.loopCounter, str);
#endif
			}
#ifndef DISABLE_FILE_LOGGING
			if (ext_file){
				fprintf(ext_file, "[%d][%d] %s\n", ID, lastTime.loopCounter, str);
				if (lastTime.loopCounter % 10 == 0) {
					fflush(ext_file);
				}
			}
#endif
		}
	}

	void loginfo(const std::string& str){
		loginfo(str.c_str());
	}

	template<class ...Arg>
	void loginfo(const char* fmtStr, Arg... args){
		loginfo(fmt::format(fmtStr, args...));
	}
	void loginfo_if(int level, const char* str){
		if (level & currLogOption()){
			if (level & DEBUG_INFO_COMPUTE || level & DEBUG_INFO_AVOID_ALL || level & DEBUG_INFO_AVOID_SIM){
				if (lastTime.loopCounter % 10 != 0)
					return;
			}
			else if (level & DEBUG_INFO_STATES){
				if (lastTime.loopCounter % 10 != 0)
					return;
			}
			else if (level & DEBUG_INFO_COM_SEND){
				if (lastTime.loopCounter % 10 != 0)
					return;
			}
			else if (level & DEBUG_INFO_COM_RECV){
				if (lastTime.loopCounter % 50 != 0)
					return;
			}
			else if (level & DEBUG_INFO_FDETECT){
				if (lastTime.loopCounter % 10 != 0)
					return;
			}
			loginfo(str);
		}
	}
	void loginfo_if(int level, const std::string& str){
		loginfo_if(level, str.c_str());
	}
	template<class ...Arg>
	void loginfo_if(int level, const char* fmtStr, Arg... args){
		if (level & currLogOption()){
			loginfo_if(level, fmt::format(fmtStr, args...));
		}
	}

	virtual void logtrace(const TimeInfo& t, const DataSet& dat, const ControlInfo& u){
		if (ext_trace){
			fprintf(ext_trace, "%6d ", t.timeElapsed);
			fprintf(ext_trace, "%7.2f %7.2f %6.2f ", dat.xy[0].x, dat.xy[0].y, rad2deg(dat.th[0]));
			fprintf(ext_trace, "%6.2f %6.2f ", dat.vw[0].x, rad2deg(dat.vw[0].y));
			fprintf(ext_trace, "%6.2f %6.2f ", u.v, u.w);
			fprintf(ext_trace, "\n");
			if (t.loopCounter % 10 == 0)
				fflush(ext_trace);
		}
	}
	virtual void logdebugmsg(){
		if (ext_debug){
			float buf[6] = {0, 0, 0, 0, 0};
			size_t sz = getDebugMsgExt(buf);
			if (sz == 4 * sizeof(float)){
				fprintf(ext_debug, "%2d %7.2f %7.2f %7.2f %7.2f\n",
					(int)sz,
					buf[0], buf[1], buf[2], buf[3]);
			}
			else if (sz == 6 * sizeof(float)){
				fprintf(ext_debug, "%2d %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",
					(int)sz,
					buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
			}
		}
	}

protected:
	// For derived class's use
	int ID;
	TimeInfo lastTime;
	DataSet  lastData;
	std::vector<pt2D>  obXY;	// 新增障碍物位置
	std::vector<real_t> obR;	// 新增障碍物半径
	wValid<TimeInfo> lastObValid;	// last_time a obXY is set
	//std::vector<pt2D> obLineA;	// 线段障碍物的起点
	//std::vector<pt2D> obLineB;	// 线段障碍物的终点
	StateInfo bufferMe;
	MapData	  mapinfo;
	
	std::string logDir;
	std::string configDir;
	//double scaleBase;
	//bool limitVByTh;
	//bool limitWByV;
	//int  defaultMethodVxy2VW;

	sscfg::ConfigSet config;
	int  log_option;
	int  DebugMode;
	int  tracelog_index;

#if defined(ENABLE_SSNETWORK)
	ExtTCPCom tcpcom;
	ExtStrCom strcom;
	bool strcomDontBind;
#endif

public:
	void setLogOutDir(const std::string& dir){
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Output dir for log files is {}", dir);
		logDir = dir;
	}
	void setParaVelLimit(real_t v, real_t w){
		limitV = v;
		limitW = w;
	}
	//void setParaObAvoid(real_t dis_safe, real_t dis_min, real_t K){
	//	obAvoidMin = dis_min;
	//	obAvoidSafe = dis_safe;
	//	obAvoidK = K;
	//}
	void setParaLinearDistance(real_t dis_cm) { linearDis = dis_cm; }
	real_t getParaLinearDistance() const { return linearDis; }
	real_t maxVel() const      { return  limitV; }
	real_t maxRotate() const   { return  limitW; }
	//real_t robotRadius() const { return  obAvoidMin; }

private:
	// A derived class cannot access this
	// Obstacle Avoidance And Speed Limit is Locked by the Base Controller
	//real_t obAvoidK;
	real_t limitV;		 // 1m/s
	real_t limitW;		 // 90deg/s
	//real_t obAvoidSafe;	 // Desired Distance 1.5m
	//real_t obAvoidMin;	 // Minimum Distance 1.0m
	real_t linearDis;	     // Linear Distance used to calculate VW from Vxy
	real_t filterUpdateW;     // 0~100 (0 -> 1)
	bool useProjectionFilter;
	bool useDataFilter;
	bool filterOnlyMe;
	bool iampaused;

	log_f  ext_log;
	void*  ext_ptr;		

	FILE*  ext_file;
	FILE*  ext_trace;
	FILE*  ext_debug;
	
	ActionSleep sleepAtBeg;	// Skip the first serveral loops to ensure network stability
	wValid<TimeInfo> t0;	// Used to ensure that tm satisfies the requirement
	TimeInfo dt0;			// Used to implement pause/resume
							// t = t_in - t0 + dt0
protected:
	FILE* getExtLogFile(){ return ext_file; }
	FILE* getExtTraceFile(){ return ext_trace; }
	FILE* getExtDebugFile(){ return ext_debug; }

public:
	// Added 2020/10/15, external state_info sources
	std::map<int, StateInfo> ext_stateinfo;
};

// Helper Function
//void toLocal(StateInfo base, std::vector<StateInfo>& others);

inline bool checkDataSetID(const DataSet& dat, int IDmax = 4, int IDmin = 1){
	for (size_t i = 0; i < dat.size(); ++i){
		if (dat.ID[i] < IDmin || dat.ID[i] > IDmax)
			return false;
	}
	return true;
}


inline int  Controller::currLogOption() const{
	return log_option;
}


}	   // namespace impl
#endif // _BASE_H
