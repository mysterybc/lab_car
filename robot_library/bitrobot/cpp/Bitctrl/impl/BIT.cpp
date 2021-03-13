#include <vector>
#include "../BIT.h"
#include "Base.hpp"
#include "ssconfig.hpp"
#include "Combined.hpp"
#include "BasicTest.hpp"
#include <fstream>
#include "toolbox/LoggingUtil.hpp"
#include "tcpcom.hpp"

using namespace impl;
using sscfg::ConfigFile;
static Controller* ctrl = nullptr;
static std::string config_dir;
static std::string logfile_dir;
logger::Logger<16, 8> loginfo;

RangeCheck<real_t> pos_check;
RangeCheck<real_t> vel_check;
RangeCheck<real_t> rot_check;
RangeCheck<uint>   id_check;

std::vector<int> ID_Disabled;
bool is_disabled(int id) {
	return std::find(ID_Disabled.begin(), ID_Disabled.end(), id) != ID_Disabled.end();
}

void* ControllerGetPtr(){
	return (void*)ctrl;
}
void* ControllerSetPtr(void* newCtrlPtr){
	ctrl = (Controller*)newCtrlPtr;
	return (void*)ctrl;
}

bool loadssconfig(ConfigFile& conf){
	std::string fname = config_dir + "simpleConfig.txt";
	std::ifstream file(fname.c_str());
	if (!file.is_open()){
		printf("[Ctrl] Error: Failed to open config file %s\n", fname.c_str());
		return false; 
	}
	else{
		real_t q_size = 2*1e5;
		real_t v_size = 1*1e3;
		real_t w_size = 2*1e2;
		
		conf = ConfigFile::load(file);
		pos_check.set_range(q_size);
		vel_check.set_range(v_size);
		rot_check.set_range(w_size);
        id_check.set_range(1, 20);
		conf.get("IDBad", ID_Disabled);
		printf("[Ctrl] Config file loaded from %s\n", fname.c_str());
        printf("[Ctrl] Allowed ID range is [%u, %u)\n", id_check.lb, id_check.ub);
		if (!ID_Disabled.empty()) {
			printf("[Ctrl] Disabled IDs are ");
			for (size_t i = 0; i < ID_Disabled.size(); ++i) {
				printf("%d ", ID_Disabled[i]);
			}
			printf("\n");
		}
		return true;
	}
}

void ControllerSetConfigDir(const char* dir){
	config_dir = dir;
	if (!config_dir.empty()){
		char c = config_dir[config_dir.size() - 1];
		if (c != '/' && c != '\\'){
			config_dir.append("/");
		}
	}
}

int ControllerSetConfigString(const char* config_string, int length) {
	return -1;
}

void ControllerSetLogDir(const char* log_dir){
	logfile_dir = log_dir;
	if (!logfile_dir.empty()){
		char c = logfile_dir[logfile_dir.size() - 1];
		if (c != '/' && c != '\\'){
			logfile_dir.append("/");
		}
	}
	if (ctrl){
		ctrl->setLogOutDir(log_dir);
	}
}

void ControllerSetDebugItem(uint item_id, void* item_ptr, void* args){
	if (ctrl)
		ctrl->setDebugItem(item_id, item_ptr, args);
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}

void ControllerGetDebugItem(uint item_id, void* item_ptr, void* args){
	if (ctrl)
		ctrl->getDebugItem(item_id, item_ptr, args);
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}

// Declare TraceInitialziation Function in TraceInit.cpp
namespace impl {
    bool setUpTrace(Controller* ctrl, int expID, int robotID, const std::string& config_dir);
}


int ControllerInit(int expID, int robotID){
    bool conf_valid;
    ConfigFile conf;
    conf_valid = loadssconfig(conf);

    if (!conf_valid){
        loginfo(logger::INFO_ERROR, "BITCTRL ERROR: failed to load config");
        return OTHER_INIT_ERROR;
    }

    // Load config
	bool valid = true;
#define fast_get(type, name) \
    type name; \
    if (conf_valid) { \
        valid = valid && conf.get( #name, name ); \
	} \
	if (!valid) { \
        loginfo(logger::INFO_ERROR, "[Ctrl] Failed to load config %s\n", #name ); \
	}
    fast_get(double, ref_lat);
    fast_get(double, ref_lon);
    fast_get(double, ref_th);
#undef fast_get

    // Set Up GPS Anchor
    GPSSetAnchor(ref_lat, ref_lon, ref_th);
	GPSShowStats(ref_lat, ref_lon);

    if (!valid || !conf_valid){
        printf("[Ctrl] Error: Invalid config file or GPS Anchor invalid!\n");
        return OTHER_INIT_ERROR;
    }

    if (ctrl != nullptr){
        printf("[Ctrl] Error: Some controller already existed!\n");
        return OTHER_INIT_ERROR;
    }
	
#define setAndLoad( pbase, pderived )   \
    pbase = pderived; \
	pderived->setConfigDir(config_dir);  \
	pderived->setLogOutDir(logfile_dir); \
    if ( pbase->loadconfig(conf) != 0 ){ \
        delete pbase; \
        pbase = nullptr;  \
        printf("[Ctrl] Error: Failed to load experiment configs!"); \
        return OTHER_INIT_ERROR; \
    }

    if (expID == EXP_GENERAL){
        Combined* p = new Combined(robotID);
        //puts("[Ctrl] Loading Config for ExpGeneral");
        loginfo(logger::INFO_SIGNAL, "[Ctrl] Loading Config for ExpGeneral");
		
		setAndLoad(ctrl, p);
    }
    else if (expID == EXP_RENDEZVOUS){
        Combined* p = new Combined(robotID);
		puts("[Ctrl] Loading Config for Rendezvous");
        setAndLoad(ctrl, p);

        p->setFunction(FUNC_RENDEZVOUS, true);
    }
    else if (expID == EXP_FORMATION){
		puts("[Ctrl] Loading Config for Formation");
        Combined* p = new Combined(robotID);
        setAndLoad(ctrl, p);

        p->setFunction(FUNC_TRACKING_GROUP, true);
		if (!impl::setUpTrace(p, expID, robotID, config_dir)){
			delete p;
			ctrl = nullptr;
			return OTHER_INIT_ERROR;
		}
    }
    else if (expID == EXP_TRACKING){
		puts("[Ctrl] Loading Config for Tracking");
        Combined* p = new Combined(robotID);
        setAndLoad(ctrl, p);

        p->setFunction(FUNC_TRACKING_SINGLE, true);
		if (!impl::setUpTrace(p, expID, robotID, config_dir)){
			delete p;
			ctrl = nullptr;
			return OTHER_INIT_ERROR;
		}
    }
    else if (expID == EXP_INTERVENTION){
		puts("[Ctrl] Loading Config for Share");
        Combined* p = new Combined(robotID);
        setAndLoad(ctrl, p);

        p->setFunction(FUNC_TRACKING_GROUP, true);
        p->setFunction(FUNC_HUMAN_TELE, true);
		if (!impl::setUpTrace(p, expID, robotID, config_dir)){
			delete p;
			ctrl = nullptr;
			return OTHER_INIT_ERROR;
		}
    }
    else if (expID == EXP_TRACEHUMAN){
		puts("[Ctrl] Loading Config for Tracer");
        Combined* p = new Combined(robotID);
        setAndLoad(ctrl, p);

        p->setFunction(FUNC_TRACE_HUMAN, true);
    }
    else if (expID == EXP_TEST_EIGHT || expID == EXP_TEST_LINE){
		puts("[Ctrl] Loading Config for TestEight/Line");
        Combined* p = new Combined(robotID);
        setAndLoad(ctrl, p);

        p->setFunction(FUNC_TRACKING_GROUP, true);
		if (!impl::setUpTrace(p, expID, robotID, config_dir)){
			delete p;
			ctrl = nullptr;
			return OTHER_INIT_ERROR;
		}
    }
    else if (expID >= EXP_TESTID_BEG && expID <= EXP_TESTID_END){
		puts("[Ctrl] Loading Config for BasicTest");
        BasicTest* p = new BasicTest(robotID);
        setAndLoad(ctrl, p);

        if (expID == EXP_TESTID_BEG + 0) p->shape = BasicTest::circle_shape;
        if (expID == EXP_TESTID_BEG + 1) p->shape = BasicTest::eight_shape;
        if (expID == EXP_TESTID_BEG + 2) p->shape = BasicTest::w_test;
        if (expID == EXP_TESTID_BEG + 3) p->shape = BasicTest::th_test;
        if (expID == EXP_TESTID_BEG + 4) p->shape = BasicTest::v_test;
        if (expID == EXP_TESTID_BEG + 5) p->shape = BasicTest::wv_test;
        if (expID == EXP_TESTID_BEG + 6){
            p->shape = BasicTest::th_test;
            p->val_testTh = 0;
        }
        if (expID == EXP_TESTID_BEG + 7){
            p->shape = BasicTest::th_test;
            p->val_testTh = 90;
        }
        if (expID == EXP_TESTID_BEG + 8){
            p->shape = BasicTest::th_test;
            p->val_testTh = -90;
        }
        if (expID == EXP_TESTID_BEG + 9){
            p->shape = BasicTest::th_test;
            p->val_testTh = 180;
        }
    }
#undef setAndLoad
    if (ctrl == nullptr){
        return INVALID_EXPID;
    }

    // Return current controller's state
    return ControllerState();
}

int ControllerState(){
	if (ctrl) 
		return ctrl->state();
	printf("[Ctrl] Error: Controller unintialized!\n");
	return OTHER_STATE_ERROR;
}

real_t ControllerTaskProgress(){
	if (ctrl)
		return ctrl->task_progress();
	printf("[Ctrl] Error: Controller unintialized!\n");
	return 0;
}

int ControllerGetDebugMsg(void* buf){
	if (ctrl){
		return (int)(ctrl->getDebugMsg(buf));
	}
	printf("[Ctrl] Error: Controller unintialized!\n");
	return 0;
}

int  ControllerGetMsg(void* buf){
	if (ctrl){
		return ctrl->getCtrlMsg(buf);
	}
	printf("[Ctrl] Error: Controller unintialized!\n");
	return 0;
}

void ControllerHandleMsg(int srcID, void* data, uint size){
	if (ctrl){
		if (!is_disabled(srcID)) {
			ctrl->handleMsg(srcID, data, size);
		}
	}
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}

void ControllerSetPath(uint num, const PathPoint* path){
	if (ctrl) {
		bool valid = true;
		int invalid_counter = 0;
		for (uint i = 0; i < num; ++i) {
			if (!pos_check(path[i].x, path[i].y)) {
				valid = false;
				++invalid_counter;
				if (valid) {
					ctrl->loginfo("[Ctrl] FATAL: Pathpoint ({:.4f}, {:.4f}) is out of range !!", path[i].x, path[i].y);
				}
			}
		}
		if (valid) {
			return ctrl->setPath(num, path);
		}
		else {
			ctrl->loginfo("[Ctrl] -- Get {} path points, {} of them are invalid.", num, invalid_counter);
		}
	}
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}

void ControllerSetTargetPoint(PathPoint target){
	if (ctrl) {
		bool valid = true;
		if (!pos_check(target.x, target.y)) {
			valid = false;
			ctrl->loginfo("[Ctrl] FATAL: TargetPoint ({:.4f}, {:.4f}) is out of range !!", target.x, target.y);
		}
		if (valid) {
			return ctrl->setTargetPoint(target);
		}
	}
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}

void ControllerSetObstacles(uint num, const ObstacleInfo* obstacles){
	if (ctrl) {
		bool valid = true;
		for (uint i = 0; i < num; ++i) {
			if (!pos_check(obstacles[i].x, obstacles[i].y)) {
				valid = false;
				ctrl->loginfo("[Ctrl] FATAL: ObstaclePoint ({:.4f}, {:.4f}) is out of range !!", obstacles[i].x, obstacles[i].y);
			}
		}
		if (valid) {
			return ctrl->setObstacles(num, obstacles);
		}
	}
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}

void ControllerSetHumanInput(InterventionInfo humanInput){
	if (ctrl) {
		bool valid = true;
		if (valid) {
			return ctrl->setHumanInput(humanInput);
		}
	}
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}

void ControllerSetHumanPosition(real_t x, real_t y){
	if (ctrl) {
		if (!pos_check(x, y)) {
			//ctrl->loginfo("[Ctrl] FATAL: HumanPosition ({:.4f}, {:.4f}) is out of range !!", x, y);
		}
		else {
			return ctrl->setHumanPoisition({ x, y });
		}
	}
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}

int ControllerSetFunction(int funciotnID, int on_off){
	if (ctrl){
		ctrl->loginfo("[Ctrl] [Raw] Turn {} function {}", (on_off ? "ON" : "OFF"), funciotnID);
		return ctrl->setFunction(funciotnID, on_off != 0);
	}
	printf("[Ctrl] Error: Controller unintialized!\n");
	return -1;
}

void ControllerPause(int pause){
	if (ctrl)
		return ctrl->setPause(pause);
	printf("[Ctrl] Error: Controller unintialized!\n");
}

bool check_neb_state(const StateInfo& one) {
	bool one_valid = true;
	if (!id_check(one.ID)) {
		one_valid = false;
		ctrl->loginfo("[Ctrl] WARN: Neb ID {} is out of range !!", one.ID);
	}
	if (!pos_check(one.x, one.y)) {
		one_valid = false;
		ctrl->loginfo("[Ctrl] WARN: Neb(ID={})'s position ({:.4f}, {:.4f}) is out of range !!", one.ID, one.x, one.y);
	}
	if (!vel_check(one.v) || !rot_check(one.w)) {
		one_valid = false;
		ctrl->loginfo("[Ctrl] WARN: Neb(ID={})'s velocity ({:.4f}, {:.4f}) is out of range !!", one.ID, one.v, one.w);
	}
	return one_valid;
}

// Added 2020/10/16...
int ControllerCommand(void* str, uint length) {
	if (ctrl) {
		return ctrl->handleCommand(str, length);
	}
	return 0;
}

// Added 2020/11/03
void ControllerSetFormationGroup(int groupIndex) {
	if (ctrl) {
		ctrl->setFormationGroup(groupIndex);
	}
}
void ControllerSetFormationShape(uint nrobot, const int* robotID, const real_t* dx, const real_t* dy, float scaling) {
	if (ctrl) {
		std::vector<float> x, y;
		std::vector<int> id;
		if (dx && dy) {
			x.resize(nrobot);
			y.resize(nrobot);
			for (uint i = 0; i < nrobot; ++i) {
				x[i] = dx[i];
				y[i] = dy[i];
			}
		}
		id.resize(nrobot);
		for (uint i = 0; i < nrobot; ++i) {
			id[i] = robotID[i];
		}
		ctrl->setFormationShape(id, x, y, scaling);
	}
}


ControlInfo ControllerCompute(TimeInfo tm, StateInfo me, uint nOthers, const StateInfo* others){
	static ControlInfo zero = {0, 0};
	if (ctrl) {
		ctrl->before_compute();  // Added 08/25, to use tcpcom

		bool valid = !is_disabled(me.ID);
        if (!id_check(me.ID)) {
            valid = false;
            ctrl->loginfo("[Ctrl] FATAL: My ID {} is out of range !!", me.ID);
        }
		if (!pos_check(me.x, me.y)) {
			valid = false;
			ctrl->loginfo("[Ctrl] FATAL: My position ({:.4f}, {:.4f}) is out of range !!", me.x, me.y);
		}
		if (!vel_check(me.v) || !rot_check(me.w)) {
			valid = false;
			ctrl->loginfo("[Ctrl] FATAL: My velocity ({:.4f}, {:.4f}) is out of range !!", me.v, me.w);
		}

		ControlInfo u = zero;
		if (valid) {
			std::vector<StateInfo> filter_others;
			filter_others.reserve(nOthers);
			for (uint i = 0; i < nOthers; ++i) {
				if (!is_disabled(others[i].ID) && check_neb_state(others[i])) {
					filter_others.push_back(others[i]);
				}
			}

			// BUG Fix: 2020/12/09
			if (filter_others.size() > 0)
				u = ctrl->compute(tm, me, filter_others.size(), filter_others.data());
			else
				u = ctrl->compute(tm, me, 0, nullptr);

			if (!vel_check(u.v) || !rot_check(u.w)) {
				ctrl->loginfo("[Ctrl] FATAL: Velocity computed ({:.4f}, {:.4f}) is out of range !!", u.v, u.w);
				u = zero;
			}
		}

		ctrl->after_compute();	// Added 08/25, to use tcpcom
		return u;
	}
	printf("[Ctrl] Error: Controller unintialized!\n");
	return zero;
}

int ControllerComputePath(PathPoint q0, PathPoint q1, real_t safe, real_t vel, PathPoint* path, int max_length) {
	if (ctrl) {
		pt2D mq0(q0.x, q0.y), mq1(q1.x, q1.y);
		std::vector<PathPoint> p;
		if (ctrl->computePath(p, mq0, mq1, vel, safe)) {
			int npoint = (int)p.size();
			if (path && npoint <= max_length) {
				for (int i = 0; i < npoint; ++i) {
					path[i] = p[i];
				}
				return npoint;
			}
			return 0;
		}
	}
	return -1;
}


void ControllerSetDebugInfo(int debug_info, int on_off){
	if (ctrl){
		ctrl->setDebugInfoOption(debug_info, on_off != 0);
	}
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}
void ControllerStop(){
	if (ctrl){
		ctrl->stop();
		delete ctrl;
		ctrl = nullptr;
	}
	else
		printf("[Ctrl] Error: Controller unintialized!\n");
}


#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geodesic.hpp>
//GPS At Beijing
//static double reflat     = 39.9641617569; 
//static double reflon     = 116.3103568554;
static double refHeading = 0;
static double reflat =  41.761128182994;
static double reflon = 123.441580684147;
static double cn = 1.0;
static double sn = 0.0;
static GeographicLib::LocalCartesian coord = GeographicLib::LocalCartesian(reflat, reflon);

void GPSSetAnchor(double lat, double lon, double heading){
	reflat = lat;
	reflon = lon;
	refHeading = heading;
	coord = GeographicLib::LocalCartesian(reflat, reflon);

	if (heading == 90){
		cn = 0;
		sn = 1.0;
	}
	else{
		double th = heading / 180.0 * 3.1415926535897932384;
		cn = std::cos(th);
		sn = std::sin(th);
	}

	printf("[Ctrl] GPS Anchor set to lat=%.10f, lon=%.10f\n", coord.LatitudeOrigin(), coord.LongitudeOrigin());
}

void GPSGetAnchor(double* lat, double* lon, double *heading){
    if (lat) *lat = coord.LatitudeOrigin();
    if (lon) *lon = coord.LongitudeOrigin();
	if (heading) *heading = refHeading;
}

void GPS2Local(double lat, double lon, double heading, real_t* x, real_t* y, real_t* headingNew){
	using GeographicLib::LocalCartesian;
	
	double dx, dy, dz;
	coord.Forward(lat, lon, 0, dx, dy, dz);    // We don't count the height, so z is useless

	std::swap(dx, dy);

	double dx2 =  cn * dx + sn * dy;
	double dy2 =  -sn * dx + cn * dy;
	real_t x2, y2, th2;

	x2 = static_cast<real_t>(dx2 * 100);
	y2 = static_cast<real_t>(dy2 * 100);
	th2 = static_cast<real_t>(heading - refHeading);
	if (x) *x = x2;
	if (y) *y = y2;
	if (headingNew) *headingNew = th2;
}
double GPSDistance(double lat1, double lon1, double lat2, double lon2){
	using GeographicLib::Geodesic;
	static const Geodesic& earth = Geodesic::WGS84();
	double s12;
	earth.Inverse(lat1, lon1, lat2, lon2, s12);
	return s12 * 100;
}

void GPSShowStats(double lat, double lon) {
	GeographicLib::LocalCartesian coord_test = GeographicLib::LocalCartesian(lat, lon);
	printf("GPS Stats at (lat, lon)=(%.8f, %.8f) are:\n", lat, lon);

	double dlat = GPSDistance(lat + 1e-6, lon, lat, lon) / 100;
	double dlon = GPSDistance(lat, lon + 1e-6, lat, lon) / 100;

	printf("\t1e-6 dlat = %.4fm, i.e 1m = %.4f 1e-6 dlat\n", dlat, 1 / dlat);
	printf("\t1e-6 dlon = %.4fm, i.e 1m = %.4f 1e-6 dlon\n", dlon, 1 / dlon);

	printf("\tStat string: %.8f, %.8f, %.6f, %.6f\n", lat, lon, 1/dlat, 1/dlon);
}
