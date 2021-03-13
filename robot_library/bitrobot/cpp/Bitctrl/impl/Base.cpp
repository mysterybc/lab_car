#include "Base.hpp"
#include <iterator>
#include <algorithm>
#include <cstdio>
#include <ctime>
#include "Geotools.hpp"

#if defined(ENABLE_SSNETWORK)
#if defined(__QNX__) || defined(__linux__)
#include <sys/signal.h>
#endif
#endif

namespace impl{

static TimeInfo time_diff(const TimeInfo& a, const TimeInfo& b){
	TimeInfo c;
	c.globalTime  = a.globalTime  - b.globalTime;
	c.loopCounter = a.loopCounter - b.loopCounter;
	c.timeElapsed = a.timeElapsed - b.timeElapsed;
	return c;
}

static TimeInfo time_add(const TimeInfo& a, const TimeInfo& b){
	TimeInfo c;
	c.globalTime = a.globalTime + b.globalTime;
	c.loopCounter = a.loopCounter + b.loopCounter;
	c.timeElapsed = a.timeElapsed + b.timeElapsed;
	return c;
}


Controller::Controller(int robotID){
	ID = robotID;

	// Function Pointers
	ext_log    = nullptr;
	log_option = 0;

	// File Pointers
	ext_file = nullptr;
	ext_trace = nullptr;
	ext_debug = nullptr;
	tracelog_index = 0;

	// Config Para For Base
	limitV = 100;
	limitW = 90;
	linearDis = 10;
	useProjectionFilter = true;
	useDataFilter = true;
	filterOnlyMe = false;
	filterUpdateW = 0.8f;
	strcomDontBind = false;

	iampaused = false;
	dt0.loopCounter = 0;
	dt0.globalTime  = 0;
	dt0.timeElapsed = 0;
	lastTime = dt0;
	DebugMode = 0;

	// Add parameter to ConfigSet
	config.addarg(limitW,      "wmax");
	config.addarg(limitV,      "vmax");
	config.addarg(linearDis,   "lin_dis");
	config.addarg(useDataFilter,	   "filter_enable");
	config.addarg(useProjectionFilter, "filter_proj");
	config.addarg(filterOnlyMe,        "filter_onlyme");
	config.addarg(filterUpdateW,       "filter_w");
	config.addarg(DebugMode, "DebugMode");

	// Refresh external logger
	// It's preferred that in future, the logging is completed using this external logger,
	// which provides logging util for things outside a Controller, e.g. Algo, or Interface functions
	// However, this is currently not finished...
	::loginfo.tick(0);
	::loginfo.config(DEBUG_INFO_FUNCTION, logger::LEVEL_DEFAULT, 1);
	::loginfo.config(DEBUG_INFO_COMPUTE, logger::LEVEL_COMPUTE, 10);
	::loginfo.enable(DEBUG_INFO_FUNCTION);
	::loginfo.enable(DEBUG_INFO_COMPUTE);
	::loginfo.__prefix = fmt::format("[{}]", ID);

#ifdef __QNXNTO__
	logDir = "/home/hzy/log/";
#else
	logDir = "./";
#endif

	setDebugInfoOption(DEBUG_INFO_FUNCTION, true);
	lastObValid.setValid(false);
}


Controller::~Controller(){
	if (ext_file)
		fclose(ext_file);
	if (ext_trace)
		fclose(ext_trace);
	if (ext_debug)
		fclose(ext_debug);
}

int Controller::state(){
	return 0;
}
size_t Controller::getDebugMsg(void* buf){
	return 0;
}

void Controller::disableLogging(int option) {
	if (ext_file && (option & LOG_FILE_INFO)){
		fclose(ext_file);
		ext_file = nullptr;
	}
	if (ext_trace && (option & LOG_FILE_TRACE)){
		fclose(ext_trace);
		ext_trace = nullptr;
	}
	if (ext_debug && (option & LOG_FILE_DEBUG)) {
		fclose(ext_debug);
		ext_debug = nullptr;
	}
}

void Controller::enableLogging(int option, const std::string& prefix, bool withTimeStamp)  {
	time_t tnow;
	time(&tnow);
	tm* local = localtime(&tnow);
	
#ifdef ENABLE_FILE_LOGGING
	if (option & LOG_FILE_INFO) {
		std::string logName = logDir + fmt::format("{}-{}-{:02}{:02}.txt",
			ID, prefix, local->tm_hour, local->tm_min);
		if (!withTimeStamp)
			logName = logDir + fmt::format("{}-{}-tmp.txt", ID, prefix);
		if (ext_file) 
			fclose(ext_file);
		ext_file = fopen(logName.c_str(), "w");
	}
#endif

	if (option & LOG_FILE_TRACE) {
		std::string traceName = logDir + fmt::format("{}-{}-trace-{:02}{:02}.txt",
			ID, prefix, local->tm_hour, local->tm_min);
		if (!withTimeStamp)
			traceName = logDir + fmt::format("{}-{}-trace-{}.txt", ID, prefix, tracelog_index);
		if (ext_trace)
			fclose(ext_trace);
		ext_trace = fopen(traceName.c_str(), "w");
	}

	if (option & LOG_FILE_DEBUG) {
		std::string debugName = logDir + fmt::format("{}-{}-debug-{:02}{:02}.txt",
			ID, prefix, local->tm_hour, local->tm_min);
		if (!withTimeStamp)
			debugName = logDir + fmt::format("{}-{}-debug-tmp.txt", ID, prefix);
		if (ext_debug)
			fclose(ext_debug);
		ext_debug = fopen(debugName.c_str(), "w");
	}
}

int Controller::loadmapdata(bool overwrite) {
	std::string obfile = configDir + "obstacles.txt";
	LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "Loading MapData from {}", obfile);
	if (mapinfo.load(obfile, !overwrite) > 0) {
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "Loaded {} Map Obstacles", mapinfo.size());
	}
	else {
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "Map Empty");
	}
	return (int)mapinfo.size();
}

int Controller::loadconfig(const std::string& file_name) {
	if (file_name == ""){
		return loadconfig(configDir + "simpleConfig.txt");
	}

	LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "Loading config file at {}", file_name);

	std::ifstream file(file_name.c_str());
	if (file.is_open()){
		sscfg::ConfigFile cfgfile = sscfg::ConfigFile::load(file);

		// This calls the child's loadconfig on ConfigFile
		if (loadconfig(cfgfile) != 0){
			LOGWARN("[Warn] Failed to load config file {}", file_name);
			return -1;
		}
		return 0;
	}
	else{
		LOGWARN("[Warn] Failed to open config file {}", file_name);
	}
	return -1;
};

int Controller::handleCommand(void* str, size_t length) {
#if defined(ENABLE_SSNETWORK)
	return strcom.onMessage((char*)str, length);
#endif
	return -1;
}

int Controller::initialize_extcom(sscfg::ConfigFile& cfgfile) {
#if defined(ENABLE_SSNETWORK) // Added by Chengsi Shang, 2020/08/25
	// Diabling SIGPIPE
#if defined(__QNX__) || defined(__linux__)
	LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Disabling SIGPIPE");
	signal(SIGPIPE, SIG_IGN);
#endif

	int port_shift = 0;
	std::vector<int> m_id, m_port, m_port_str;
	std::vector<std::string> m_ip;
	cfgfile.get("tcpcom_id", m_id);
	cfgfile.get("tcpcom_ip", m_ip);
	cfgfile.get("tcpcom_port", m_port);
	cfgfile.get("strcom_port", m_port_str);
	cfgfile.get("port_shift", port_shift);
	cfgfile.get("strcom_dontbind", strcomDontBind);

	if (strcomDontBind) {
		// Perform a simple initialization
		tcpcom.setController(this, controllerName());
		strcom.setController(this, controllerName());
		return 0;
	}

	tcpcom.server_ip.clear();
	strcom.server_ip.clear();
	for (size_t i = 0; i < m_id.size(); ++i) {
		if (m_id[i] == getMyID() && i < m_ip.size() && i < m_port.size() && i < m_port_str.size()) {
			tcpcom.server_ip = m_ip[i];
			tcpcom.server_port = m_port[i] + port_shift;
			strcom.server_ip = m_ip[i];
			strcom.server_port = m_port_str[i] + port_shift;
		}
	}

	// server_ip empty means that it is already initialized
	if (!tcpcom.server_ip.empty()) {
		tcpcom.setController(this, controllerName());  // Will return child's name
		tcpcom.bind_and_listen();
		if (tcpcom.is_binded()) {
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] [TCPCom] Binded on {}:{}", tcpcom.server_ip.c_str(), tcpcom.server_port);
		}
		else {
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Error] [TCPCom] TCP Bind Failure on {}:{}", tcpcom.server_ip.c_str(), tcpcom.server_port);
		}

		strcom.setController(this, controllerName());  // Will return child's name
		strcom.bind_and_listen();
		if (strcom.is_binded()) {
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] [StrCom] Binded on {}:{}", strcom.server_ip.c_str(), strcom.server_port);
		}
		else {
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Error] [StrCom] TCP Bind Failure on {}:{}", strcom.server_ip.c_str(), strcom.server_port);
		}
	}
	else {
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Error] Failed to load configs for TCPCom/StrCom, Cannot find myID");
	}
#endif
	return 0;
}

int Controller::loadconfig(sscfg::ConfigFile& cfgfile){
	// The childs should call me, TODO, move base's confi operation to a new function to avoid confusion

	//printf("[Ctrl] Loading Config from Base\n");
	LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "Loading config from Base. --- Version 2020-1211-1400");
	loadmapdata(true);

	// Modified by Chengsi Shang 2020/09/07
	LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "Initializing TCPCom/StrCom");
	initialize_extcom(cfgfile);

	LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "Base Loading configset");
	int ret = config.load(cfgfile);  // Note, this will then load parameters of Child since config is also set by a Child
	if (ret != 0){
		printf("[Ctrl] The following items failed to load:\n");
		for (size_t i = 0; i < config.last_invalid.size(); ++i){
			printf("\t%s\n", config.last_invalid[i].c_str());
		}
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Warn] Failed to load configs for Base, {} items are missing", ret);
	}
	return ret;
}

void Controller::recording(TimeInfo tm, const DataSet& dat, ControlInfo u){
	lastTime = tm;
	lastData = dat;
	logtrace(tm, dat, u);
	logdebugmsg();
}
ControlInfo Controller::compute(TimeInfo tm, DataSet& dat){
	static const ControlInfo zero_u = { 0, 0 };

	// 1. Check State
	if (iampaused){
		return zero_u;
	}
	if (state() == OTHER_STATE_ERROR){
		return zero_u;
	}
	
	// 2. Check Beg Sleep Timer
	if (sleepAtBeg.unset()){
		//sleepAtBeg.resetTime(500, tm);
		sleepAtBeg.resetLoops(10, tm);
	}
	sleepAtBeg.update(tm);
	if (!sleepAtBeg.completed()){
		lastTime.loopCounter = 0;
		lastTime.globalTime  = 0;
		lastTime.timeElapsed = 0;

		lastData = dat;
		return zero_u;
	}

	// 3. Shift tm to ensure tm criterion
	if (!t0.valid()){
		t0 = tm;
	}
	tm = time_add(time_diff(tm, t0), dt0);

	// 4. Initialize lastXXX at loop 0
	if (tm.loopCounter == 0){
		lastTime = tm;
		lastData = dat;
	}
	
	// 5. Compute 
	ControlInfo ualgo, ucomp;
	ControlInfo u;
	::loginfo.tick(tm.loopCounter);

	// 5.1 Add a simple filter to position
	if (useDataFilter){
		double dt = (tm.timeElapsed - lastTime.timeElapsed) / 1000.0;
		filterData(dt, lastData, dat);
	}
	
	// 5.2 Clear obselete obstacles
	double ob_valid_max = 2.0; // sec
	if (lastObValid.valid()) {
		double tmLastOb = tm.timeElapsed / 1000.0 - lastObValid.timeElapsed / 1000.0;
		if (tmLastOb > ob_valid_max) {
			loginfo("[Ctrl] Obstacle cleared");
			obXY.clear();
			obR.clear();
			lastObValid.setValid(false);
		}
	}

	// Compute
	if (innerUse2D()){
		pt2D uf = innerCompute2D(tm, dat);
		ualgo = fromVxy2VW(dat.th[0], uf);
	}
	else{
		ualgo = innerCompute(tm, dat);
	}

	// ucomp is always retrived after innerCompute
	ucomp = getCompensate();
	u.v = ualgo.v + ucomp.v;
	u.w = ualgo.w + ucomp.w;
	u = saturate(u, limitV, limitW);

	// 6. Recording
	recording(tm, dat, u);
	
	// 7. Return the computed Result
	return u;
}

void Controller::before_compute() {
#if defined(ENABLE_SSNETWORK) // Added by Chengsi Shang, 2020/08/25
	if (strcomDontBind) return;
	if (tcpcom.is_binded()) {
		tcpcom.check_status();
		tcpcom.accept_and_recv();
	}
	else {
		LOGWARN("[Ctrl] TCPCom is not binded");
	}
	if (strcom.is_binded()) {
		strcom.check_status();
		strcom.accept_and_recv();
	}
	else {
		LOGWARN("[Ctrl] StrCom is not binded");
	}
#endif
}
void Controller::after_compute() {
#if defined(ENABLE_SSNETWORK) // Added by Chengsi Shang, 2020/08/25
	if (strcomDontBind) return;
	if (tcpcom.is_binded()) {
		tcpcom.broadcast_state();
		LOGINFO(DEBUG_INFO_STATES, "[Ctrl] TCPCom Clients {}", tcpcom.get_server().client_num());
	}
	if (strcom.is_binded()) {
		strcom.broadcast_state();
		LOGINFO(DEBUG_INFO_STATES, "[Ctrl] StrCom Clients {}", strcom.get_server().client_num());
	}
#endif
}

ControlInfo Controller::compute(TimeInfo tm, StateInfo me, uint nOthers, const StateInfo* others){
	if (DebugMode == 1) {
		nOthers = 0; others = nullptr;
	}

	// Modified 2020/10/15
	std::vector<StateInfo> tmpinfo;
	std::map<int, StateInfo>::iterator it = ext_stateinfo.begin();
	for (; it != ext_stateinfo.end(); ++it) {
		tmpinfo.push_back(it->second);
	}

	int n_ext = (int)tmpinfo.size();
	for (uint i = 0; i < nOthers; ++i) {
		int index = -1;
		for (int k = 0; k < n_ext; ++k) {
			if (tmpinfo[k].ID == others[i].ID) {
				index = k;
				break;
			}
		}
		if (index == -1) {
			tmpinfo.push_back(others[i]);
		}
		else {
			tmpinfo[index] = others[i];
		}
	}

	// BUG Fix: 2020/12/09
	DataSet dat(me, tmpinfo.size(), (tmpinfo.empty() ? nullptr : &tmpinfo[0]));

	// Always clear it
	ext_stateinfo.clear();
	return compute(tm, dat);
}

void Controller::setObstacles(uint num, const ObstacleInfo* obstacles){
	obXY.resize(num);
	obR.resize(num);

	for (size_t i = 0; i < num; ++i){
		obXY[i].x = obstacles[i].x;
		obXY[i].y = obstacles[i].y;
		obR[i] = obstacles[i].radius;
	}
	lastObValid = lastTime;
	lastObValid.setValid(true);
}

ControlInfo Controller::innerCompute(TimeInfo tm, DataSet& dat){
	static const ControlInfo zero_u = { 0, 0 };
	LOGWARN("Wow, unintended error..");
	return zero_u;
}

pt2D        Controller::innerCompute2D(TimeInfo tm, DataSet& dat){
	LOGWARN("Wow, unintended error..");
	return pt2D();
}

ControlInfo Controller::fromVxy2VW(double th, pt2D vxy){
	ControlInfo u;
	//int method = defaultMethodVxy2VW;
	
	double thNow = rad2deg(th);
	double thDesire = rad2deg(std::atan2(vxy.y, vxy.x));

	if (norm2(vxy) > limitV)
		vxy = normalize(vxy) * limitV;

	pt2D dir = pt2D(std::cos(th), std::sin(th));
	pt2D nor = pt2D(-std::sin(th), std::cos(th));
	u.v = (real_t)dot(dir, vxy);
	u.w = (real_t)(dot(nor, vxy)/ linearDis);    // 1/100 is for cm->m
	u.w = (real_t)rad2deg(u.w);

	return u;
}

void Controller::filterData(double dt, DataSet& last, DataSet& now){
	if (dt < 0.0001)
		return;

	size_t i = 0;
	size_t imax = filterOnlyMe ? 1 : now.size();
	for (; i < imax; ++i){
		pt2D x1 = now.xy[i];
		int  id = now.ID[i];
		size_t j = 0;
		for (; j < last.size(); ++j){
			if (last.ID[j] == id) break;
		}
		if (j == last.size()) continue;
		
		// Has this data for now and the last time
		pt2D x0 = last.xy[j];
		pt2D v0 = last.vxy[j];
		double th0 = last.th[j];

		pt2D xp;
		if (useProjectionFilter){
			pt2D dir;
			dir.x = std::cos(th0);
			dir.y = std::sin(th0);
			xp = dot(x1 - x0, dir)*dir + x0;
		}
		else{
			xp = x0 + v0*dt;
		}

		double r = filterUpdateW / 100.0;
		r = std::pow(r, dt);
		x1 = xp * r + x1 * (1 - r);
		now.xy[i] = x1;
	}
}


void Controller::setExtLog(void* ptr, log_f extLog){
	ext_log = extLog;  ext_ptr = ptr;
}
size_t Controller::getCtrlMsg(void* buf) { return 0; }
void Controller::handleMsg(int srcID, void* dat, size_t size){}
void Controller::stop(){
	LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[CTRL] Controller Stopped");
}
void Controller::setHumanInput(InterventionInfo humanInput){}
void Controller::setHumanPoisition(pt2D pos){}
void Controller::setPath(uint num, const PathPoint* path){}
void Controller::setPath(const ShapeTrace& trace) {}
bool Controller::computePath(std::vector<PathPoint>& path, pt2D q0, pt2D q1, real_t v, real_t safe) { return false; }
bool Controller::computePath(std::vector<PathPoint>& path, const std::vector<PathPoint>& target, real_t safe, bool startAtMe) { return false; }
real_t Controller::task_progress(){ return 0; }
void Controller::setPause(int paused){
	if (paused != 0){
		if (!iampaused){
			iampaused = true;
			dt0 = lastTime;
			t0.setValid(false);
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Controller Paused");
		}
	}
	else{
		if (iampaused){
			iampaused = false;
			dt0.globalTime += 100;
			dt0.loopCounter++;
			dt0.timeElapsed += 100;
			LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Controller Resumed");
		}
	}
}


static std::string LogOptName(int option){
	std::string s;
	if (option & DEBUG_INFO_COMPUTE)  s += "Compute, ";
	if (option & DEBUG_INFO_FUNCTION) s += "Function, ";
	if (option & DEBUG_INFO_HUMAN_POS)s += "Human_Pos, ";
	if (option & DEBUG_INFO_TELE)     s += "Tele, ";
	if (option & DEBUG_INFO_STATES)   s += "States, ";
	if (option & DEBUG_INFO_AVOID_ALL) s += "Avoid(all), ";
	if (option & DEBUG_INFO_AVOID_SIM) s += "Avoid(sim), ";
	if (s.empty()){
		return "(Invalid Log Option)";
	}
	else{
		return s.substr(0, s.size() - 2);
	}
}

void Controller::setDebugInfoOption(int option, bool is_on){
	if (is_on){
		log_option |= option;
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Enable Logging on {}", LogOptName(option));
	}
	else{
		log_option &= ~option;
		LOGDEBUGLV2(DEBUG_INFO_FUNCTION, "[Ctrl] Disable Logging on {}", LogOptName(option));
	}
}


}	// namesapce impl
