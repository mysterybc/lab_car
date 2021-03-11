#pragma once
#include "Base.hpp"
#include "Algo.hpp"
#include "toolbox/NSAvoidAlgo.hpp"
#include "toolbox/CtrlBuffer.hpp"
#include "Obstacles.hpp"
#include "astarplan.hpp"
#include <set>

#if defined(_HAS_ZMQ_)
#include "zmq.hpp"
#endif

#if defined(ENABLE_SSNETWORK)
#include "tcpcom.hpp"
#endif

namespace impl{
	// Forward Declaration of Communication
	class CommandHandler;
	class StrHandler;
	class ExtTCPCom;
	class ExtStrCom;

	struct TestEightConfig{
		TestEightConfig() :enabled(false), all_prepared(false), R(2), vel(0), theta(0), tStay(2), nRound(2), nPoint(20) {}
		bool enabled;
		bool all_prepared;
		real_t R, vel, theta, tStay;
		int nRound, nPoint;
	};

	void append_eight(pt2D* p0,
		std::vector<double>& tlist, std::vector<double>& xlist, std::vector<double>& ylist,
		double R, double vLine, int nRound = 2, int nPoint = 20, double rotTheta = 0, double tStay = 3);

	class Combined : public Controller{
	public:
		friend class CommandHandler;
		friend class ExtTCPCom;
		friend class StrHandler;
		friend class ExtStrCom;

		Combined(int robotID);
		const char* controllerName() const { return "Combined"; }

		// Current, 80 bytes
		size_t getCtrlMsg(void* buf);
		void   handleMsg(int srcID, void* dat, size_t size);

		// Inteface Added 2020/10/25, explictly set Ctrl Msg
		conf::CtrlPack& updateMyCtrlPack();
		void handleCtrlPack(int srcID, const conf::CtrlPack& msg, std::string& log_msg);

		size_t getCtrlMsgBackup(void* buf);
		void   handleMsgBackup(int srcID, void* dat, size_t size);

		size_t getDebugMsg(void* buf){ return 0; }
		void setDebugItem(uint item_id, void* item_ptr, void* args){}
		void getDebugItem(uint item_id, void* item_ptr, void* args);

		int  loadconfig(sscfg::ConfigFile& file);
		int  setFunction(int funcID, bool is_on);
		void setHumanInput(InterventionInfo humanInput);
		void setHumanPoisition(pt2D pos);
		void setPath(uint num, const PathPoint* path);
		void setPath(const ShapeTrace& trace);
		void setTargetPoint(PathPoint pt);
		void setTargetHeading(real_t th, bool isRelative);
		bool setFormationShape(const std::vector<int>& id, std::vector<float> dx, std::vector<float> dy, float dxy_scaling);
		void setFormationGroup(int groupID) { formation_group = groupID;  LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Set FM_Group {}", groupID);  }
		int  getFormationGroup()            { return formation_group; }
		bool computePath(std::vector<PathPoint>& path, pt2D q0, pt2D q1, real_t v, real_t safe);
		bool computePath(std::vector<PathPoint>& path, const std::vector<PathPoint>& target, real_t safe, bool startAtMe);

	public:
		int state(){ return initState != 0 ? initState : runtimeState; }
		real_t task_progress(){
			return current_progress;
		}

		ControlInfo innerCompute(TimeInfo tm, DataSet& dat);
		pt2D innerCompute2D(TimeInfo tm, const DataSet& dat);

		void logtrace(const TimeInfo& t, const DataSet& dat, const ControlInfo& u);

	protected:
		// Helper Functions
		bool useCollisionAvoid() { return false; }
		bool innerUse2D()        { return false;  }

	private:
		// IMPORTATNT: Call this when dxy is changed or initialized
		void onFormationChange();
		void selectShape(std::map<int, pt2D>& dxy, const DataSet& dat);

		bool needShapeSelection(int base_func) const { 
			return (base_func == FUNC_TRACKING_GROUP || base_func == FUNC_RENDEZVOUS || base_func == FUNC_TRACE_HUMAN);
		}
		bool needShapeSelection() const { return needShapeSelection(base_function()); }
		bool enableFDetect() const     { return (functionState & FUNC_FAULT_DETECT) != 0; }
		bool enableLatencyTest() const { return true || (functionState & FUNC_LATENCY_TEST) != 0; }
		bool enableHField() const      { return (functionState & FUNC_HUMAN_TELE) != 0; }
		bool enableHFieldMulti() const { return enableHField() && (base_function() == FUNC_TRACKING_GROUP); }
		bool isfaulty() const          { return (functionState & FUNC_ACT_FAULTY) != 0; }
		int base_function() const      { return (functionState & 0xff); }
	
		// Return if the shape has been confirmed
		void initShapeSelection();
		bool onSelectFormation(const DataSet& dat);
		char getMyShapeIndex();

		void generateTestEightPath(const FormationInfo& info, const TimeInfo& tm);
	private:
		int loadconfig_debug(sscfg::ConfigFile& file);
		int loadconfig_shape(sscfg::ConfigFile& file);
		int loadconfig_planning(sscfg::ConfigFile& file);
	private:
		// Helper functions related to using nsavoid
        //ControlInfo avoid_obstacle(const TimeInfo& tm, DataSet& dat, const ControlInfo& u);

        ControlInfo project_safe(const ControlInfo& u, const pt2D& q, double theta);
        pt2D project_safe(const pt2D& u, const pt2D& q);
        void refresh_obstacles(const DataSet& dat);
		void log_nsavoid(const DataSet& dat, const ControlInfo& u, const ControlInfo& u2, const pt2D& u2D);

	private:
		typedef wValid<HumanInput> wHumanInput;
		typedef wValid<InterventionInfo> wIntervention;
		typedef wValid<DirectTeleData> wDirectTeleData;
		typedef wValid<DirectTeleVxyData> wDirectTeleVxyData;

		//int  nsavoid_fd;	// What for ???
		ControlInfo fwcomp;
		int initState;
		int runtimeState;
		int functionState;
		real_t current_progress;
		wValid<ControlInfo> lastControl;

		// Communication protocols (Old method, using ZMQ)
		conf::CtrlPack compack;

		// Data For Fault Detect
		wValid<StateInfo> last_predict;
		std::set<int> errList;
		bool iamfaulty;

		// ID list that never enables fault detect
		std::set<int> dont_detect;
		std::set<int> auto_faulty;

		// For Tracer
		wValid<pt2D> human_pos;
		bool human_virtual;

		// Data For Tracking and Rendezvous
		FormationInfo       info;
		std::map<int, pt2D> dxy_base, dxy_desire; // in cm
		std::map<int, int>  dxyRemap;             // dxy_deisre[id] = dxy[dxyRemap[id]]
		
		wValid<pt2D>   target_point;
		wValid<real_t> target_angle;
		spline2D     lineXY;
		real_t       targetTime;
		ConditionChecker reach_position;
		ConditionChecker reach_heading;
		
		// Data For Formation Select
		std::map<int, char> shapeReceive;
		bool shape_selected;
		int  selected_counter;
		int  shape_mismatch_counter;
		int  shape_missing_counter;
		int  formation_group;  // Default Formation Group, added 2020/10/16

		// Data For Human Tele (using Intention Field)
		wIntervention ivLast;
		wHumanInput   hiLast;
		HumanField	  hfMe;
		real_t        humanScale;   // in cm

		std::map<int, HumanField> hfNet;
		std::set<int>	  myNetHumanNeb;

		// Raw Tele Data
		wDirectTeleData teleDirect; // Added: 0824 by Chengsi Shang
		wDirectTeleVxyData teleDirectVxy; // Added: 0913 by Chengsi Shang

		// Algorithms
		typedef NSAvoid::AlgoNSAvoid<real_t> fAlgoNSAvoid;
		//AlgoPFAvoidPower power_avoid;
		AlgoTracking     tracking,   tracking_single;
		AlgoConsensus    consenRend, consenTrack;
		AlgoHeadingAlign alignRend,
						 alignLocal, alignTarget;
		AlgoHumanField   human_field;
		AlgoFaultDetect<pt2D>    fault_xy;	   // In cm
		AlgoFaultDetect<double>  fault_th;	   // In degree
		AlgoUnicycleMode predict_next;
		//AlgoTracer       tracer;
		fAlgoNSAvoid     nsavoid;
		pt2D             tracer_default_shift;
		AlgoLatencyTest  latency_test;
		AlgoSlowDown     slow_down;

		// Algorithm swithces
		bool use_nsavoid;       // If use projection based obstacle avoidance
		bool use_pfavoid;       // If use potential field based obstacle avoidance
		bool use_autoselect;    // If automatically select shape based on current location
		bool use_autofault;     // If turn on act_faulty when detecting faults
		bool use_fakevel;       // If use control signal as the velocity at t+1
		bool dumpNSAvoid;		// If dumpNSAvoid Info
		//bool useAvoid;		// ??
		bool use_slowdown;		// If use AlgoSlowDown
		bool nsavoid_2D;		// If use nsavoid on (vx, vy) instead of (v, w)
		bool nsavoid_vw;		// If use nsavoid on (v, w)

		// For act faulty
		real_t wFault;
		TimeInfo tFautlStart;

		// For path tracking
		real_t default_vpath;	// m/s, the path velocity

		// For Testing Eight
		TestEightConfig testEight;	// Additional Test info for Eight-shape tracking

		// Task Completion Threshold
		real_t thresh_consen;
		real_t thresh_tracking;
		real_t thresh_th;

		// For NSAvoid and related stuff
		real_t ob_size_robot;	// radius of a robot agent
		real_t ob_size_human;	// radius of a human agent
		real_t ob_size_envob;	// If >= 0, overwrites obR
		real_t nsavoid_lamV;	// lamV_overW;

		// Faulty Actions
		int ftd_fault_type;  // 0, 1, 2 (Fault 0, dont move)
		real_t ftd_tp1_v, ftd_tp1_w;  // Fault 1 Fixed v (cm/s), w (deg/s)
		real_t ftd_tp2_v, ftd_tp2_t;  // Fault 2 Forward/Backward Move, speed v (cm/s), period t (s)

		// Planning
		PlanAstar planner;
		ObGrid obgrid;

		// For Debug Info control options
		bool DebugShowAllState;
		int BadMsgCounter;
		int MisSizeCounter;
		int GoodMsgCounter;
		void print_com_stat();
	};

    template<class T>
    NSAvoid::AlgoNSAvoid<real_t>::Vec2X
    to2X(const T& p) { return NSAvoid::AlgoNSAvoid<real_t>::Vec2X((real_t)p.x, (real_t)p.y); }

    template<class T>
    NSAvoid::AlgoNSAvoid<real_t>::Mat2X
    toMat2X(const std::vector<T>& p) {
		NSAvoid::AlgoNSAvoid<real_t>::Mat2X m(2, p.size());
		for (size_t i = 0; i < p.size(); ++i)
			m.col(i) = to2X(p[i]);
		return m;
	}

} // namespace impl

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif
