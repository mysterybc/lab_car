#ifndef _OLD_TEST
#include "BIT.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Existing functions
void fprintInitError(FILE* out, int expID, int ID, int ret) {
	fprintf(out, "Failed to init robot %d for exp %d, errno is %d: ", ID, expID, ret);
	if (ret == INVALID_EXPID) 	 fprintf(out, "Invalid ExpID\n");
	if (ret == INVALID_ROBOTID)  fprintf(out, "Invalid RobotID\n");
	if (ret == OTHER_INIT_ERROR) fprintf(out, "Other Error(Failed to load configurations)\n");
}

void check(int ret, const char* err_msg) {
	if (ret != 0) {
		printf("Exiting on Error. %s\n", err_msg);
		exit(-1);
	}
}

// Global Configurations
char* config_path = "D:/Coding/cpp/RobotNet/src/RobotNet/expProject/";
char* logtxt_path = "";
int nRobot = 4;
int buffer_size = 80;

// Global Variables
void        **pRobot;
StateInfo   *state, *neighbour;
ControlInfo *ctrl;
TimeInfo    tmSimu;
char **com_buffer;

void* initRobot(int expID, int ID) {
	void* ptr;
	ControllerSetConfigDir(config_path);
	ControllerSetLogDir(logtxt_path);
	int ret = ControllerInit(expID, ID);
	if (ret != 0) {
		fprintInitError(stdout, expID, ID, ret);
		return NULL;
	}
	ptr = ControllerGetPtr();
	ControllerSetPtr(NULL);
	return ptr;
}

int initRobotAll(int expID) {
	int i;
	pRobot = malloc(sizeof(void*)*nRobot);
	state = (StateInfo*)malloc(sizeof(StateInfo)*nRobot);
	neighbour = (StateInfo*)malloc(sizeof(StateInfo)*nRobot);
	ctrl = (ControlInfo*)malloc(sizeof(ControlInfo)*nRobot);
	com_buffer = (char**)malloc(sizeof(char*)*nRobot);
	for (i = 0; i<nRobot; ++i) {
		pRobot[i] = initRobot(expID, i + 1);
		com_buffer[i] = (char*)malloc(sizeof(char)*buffer_size);
		if (pRobot[i] == NULL) {
			printf("Failed to init robot %d\n", i);
			return -1;
		}
	}
	return 0;
}

int checkConfigPath() {
	char fname[256] = { 0 };
	int len = strlen(config_path);
	strcpy(fname, config_path);
	if (fname[len - 1] != '/' || fname[len - 1] != '\\') {
		fname[len++] = '/';
	}
	strcpy(fname + len, "simpleConfig.txt");
	FILE* in_file = fopen(config_path, "r");
	int ret = (in_file != 0) ? -1 : 0;
	if (in_file) {
		fclose(in_file);
	}
	return ret;
}

void set_states(float* x, float* y, float* th) {
	int i = 0;
	for (; i < nRobot; ++i) {
		state[i].ID = i + 1;
		state[i].heading = th[i];
		state[i].x = x[i];
		state[i].y = y[i];
		state[i].v = 0;
		state[i].w = 0;
	}
}

StateInfo state_advance(float dt, StateInfo s, ControlInfo u) {
	float pi = 3.14159265f;
	float th = s.heading * pi / 180.0f;
	StateInfo s2 = s;
	s2.x += u.v * (float)cos(th) * dt;
	s2.y += u.v * (float)sin(th) * dt;
	s2.heading += u.w * dt;
	while (s2.heading > 180) s2.heading -= 360;
	while (s2.heading < -180) s2.heading += 360;
	return s2;
}

void setup_formation() {
	float _x0[] = { 5500, 5700, 5900, 5500 };
	float _y0[] = { -1500, -1500, -1500, -1300 };
	float _th0[] = { 90, 0, -90, 135 };
	set_states(_x0, _y0, _th0);

	PathPoint path[2];
	path[0].x = 56.73 * 100;
	path[0].y = -15.93 * 100;
	path[0].v = 80;
	path[1].x = 62.06 * 100;
	path[1].y = -42.75 * 100;
	path[1].v = 80;

	int i;
	for (i = 0; i < nRobot; ++i) {
		ControllerSetPtr(pRobot[i]);
		ControllerSetPath(2, path);
		ControllerSetFunction(FUNC_TRACKING_GROUP, 1);
		//ControllerSetDebugInfo(DEBUG_INFO_COMPUTE | DEBUG_INFO_FUNCTION | DEBUG_INFO_STATES, 1);
	}
}

void setup_test8() {
	float _x0[] = { 5500, 5700, 5900, 5500 };
	float _y0[] = { -1500, -1500, -1500, -1300 };
	float _th0[] = { 90, 0, -90, 135 };
	set_states(_x0, _y0, _th0);

	int i;
	for (i = 0; i < nRobot; ++i) {
		ControllerSetPtr(pRobot[i]);
		ControllerSetFunction(FUNC_TRACE_HUMAN, 1);
		//ControllerSetDebugInfo(DEBUG_INFO_COMPUTE | DEBUG_INFO_FUNCTION | DEBUG_INFO_STATES, 1);
	}
}

int simuFormation(int dt_ms) {
	uint TMax = 240 * 1000; // 120s
	//setup_formation();
	setup_test8();

	TimeInfo tm;
	tm.globalTime = 0;
	tm.loopCounter = 0;
	tm.timeElapsed = 0;
	int i, j, k;
	int ret;
	int nFinished = 0;

	while (tm.timeElapsed < TMax && nFinished < nRobot) {
		nFinished = 0;

		// Get control info
		for (i = 0; i < nRobot; ++i) {
			ControllerSetPtr(pRobot[i]);

			// Build neighbour states
			k = 0;
			for (j = 0; j < nRobot; ++j) {
				if (i == j) continue;
				neighbour[k++] = state[j];
			}

			// Compute
			ctrl[i] = ControllerCompute(tm, state[i], nRobot - 1, neighbour);

			// Check States
			ret = ControllerState();
			if (ret != 0) {
				if (ret == STATE_TARGET_REACHED) {
					nFinished++;
				}
				else {
					printf("Error! Robot %d is in invalid state %d\n", i + 1, ret);
					exit(-1);
				}
			}
		}

		// Communication: Get MSG
		for (i = 0; i < nRobot; ++i) {
			ControllerSetPtr(pRobot[i]);
			ControllerGetMsg(com_buffer[i]);
		}

		// Communication: Handle MSG
		for (i = 0; i < nRobot; ++i) {
			ControllerSetPtr(pRobot[i]);
			for (j = 0; j < nRobot; ++j) {
				if (j == i) continue;
				ControllerHandleMsg(j + 1, com_buffer[j], buffer_size);
			}
		}

		// Advance Robot
		for (i = 0; i < nRobot; ++i) {
			state[i] = state_advance(dt_ms / 1000.0f, state[i], ctrl[i]);
		}

		// Advance time
		tm.loopCounter++;
		tm.timeElapsed += dt_ms;
		tm.globalTime += dt_ms;
	}

	return nFinished == nRobot ? 0 : -1;
}

int main(int argc, char* argv[]){
	if (argc > 1) {
		config_path = argv[1];
	}
	check(checkConfigPath(), "Failed to find simpleConfig.txt at given dir");
	check(initRobotAll(0), "Initialization failed.");
	check(simuFormation(100), "Failed to simulate the formation task");
	printf("All good\n");
	return 0;
}



#endif


#ifdef _OLD_TEST
#include "BIT.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


typedef struct {
	double dt;
	double obSigTh;
	double obSigPos;
	double mdSigTh;
	double mdSigPos;
	int	   useOneStepDelay;	 // 是否采用一步时滞
}SimuOptions;

double norm2(double x, double y){
	return sqrt(x*x + y*y);
}

double deg2rad(double deg){
	return deg * 3.141592653589793 / 180.0;
}
double rad2deg(double rad){
	return rad / 3.141592653589793 * 180.0;
}

double inragne(double deg, double half){
	while (deg > half) deg -= 2*half;
	while (deg < -half) deg += 2*half;
	return deg;
}

double round_angle(double angle){
	const double pi = 3.141592653589793;
	while (angle > pi){
		angle -= pi * 2;
	}
	while (angle < -pi){
		angle += pi * 2;
	}
	return angle;
}

StateInfo simulateRobot(StateInfo st, ControlInfo u, double dt){
	const static int simuOrder = 4;
	if (simuOrder == 4){
		double th = deg2rad(st.heading);
		double w  = deg2rad(u.w);
		double dth = w * dt;
		double dv  = u.v*dt;

		double a = 1 - dth*dth / 6;
		double b = (1 - dth*dth / 12) * dth / 2;
		double cn = cos(th);
		double sn = sin(th);

		StateInfo q1 = st;
		q1.w = u.w;
		q1.v = u.v;
		q1.heading = (real_t)rad2deg(round_angle(th + dth));
		q1.heading = (real_t)inragne(q1.heading, 180);
		q1.x = (real_t)(st.x + dv * (cn*a - sn*b));
		q1.y = (real_t)(st.y + dv * (sn*a + cn*b));
		return q1;
	}
	else {
		double th = deg2rad(st.heading);
		st.x += (real_t)(st.v * cos(th) * dt);
		st.y += (real_t)(st.v * sin(th) * dt);
		st.heading += (real_t)(st.w * dt);        // Both are in degrees... so we're good
		st.heading = (real_t)inragne(st.heading, 180);
		st.v = u.v;
		st.w = u.w;
		return st;
	}
}

void stopExitingControllers(void* ctrlptr[], int num){
	int i;
	for (i=0;i<num;++i){
		ControllerSetPtr(ctrlptr[i]);
		ControllerStop();
	}
}

int buildMeAndOthers(
	int myID,
	StateInfo* me, StateInfo* others, 
	StateInfo* allagents, int nID, int* IDList)
{
   int i;
   for (i=0;i<nID;++i) {
	   if (IDList[i] == myID){
			*me = allagents[i];
			me->ID = myID;
			break;
	   }
   }
   if (i==nID) return -1;
   
   int idx=0;
   for (i=0; i<nID; ++i){
	   if (IDList[i] != myID){
			others[idx] = allagents[i];
			idx++;
	   }
   }
   return 0;
}

int buildMeAndOthersWithOffset(int myID,
	StateInfo* me, StateInfo* others,
	StateInfo* allagents, int nID, int* IDList, float* thetaOffset)
{
	int i;
	for (i = 0; i<nID; ++i) {
		if (IDList[i] == myID){
			*me = allagents[i];
			me->heading += thetaOffset[myID];
			me->ID = myID;
			break;
		}
	}
	if (i == nID) return -1;

	int idx = 0;
	for (i = 0; i<nID; ++i){
		if (IDList[i] != myID){
			others[idx] = allagents[i];
			others[idx].heading += thetaOffset[IDList[i]];
			idx++;
		}
	}
	return 0;
}

double Rand(){
	double r = rand();
	r /= RAND_MAX;
	r *= 2;
	r -= 1;
	return r;
}
double RandEx(double mean, double sigma){
	double r = Rand();
	return r*sigma + mean;
}

void addnoise2xy(StateInfo* it, double sigmaPos, double sigmaTh){
	it->x += (real_t)(sigmaPos*Rand());
	it->y += (real_t)(sigmaPos*Rand());
	it->heading += (real_t)(sigmaTh*Rand());
}

typedef struct {
	real_t total_x;
	real_t total_y;
	size_t acc_len;
}NoiseDataAcc;

void addnoise2tn(NoiseDataAcc* noise_state, StateInfo* it, double dTan, double dNor, size_t accMaxLen){
	noise_state->acc_len++;
	if (noise_state->acc_len == accMaxLen){
		noise_state->acc_len = 0;
		return;
	}
	else{
		// 总是正的
		noise_state->total_x += (real_t)dTan;
		noise_state->total_y += (real_t)dNor;
		//it->x 
		double th = deg2rad(it->heading);
		real_t cn = (real_t)cos(th);
		real_t sn = (real_t)sin(th);
		
		// Add Noise To Tagent Direction
		it->x += cn*noise_state->total_x;
		it->y += sn*noise_state->total_x;

		// Add Noise To Normal Direction
		it->x += -sn*noise_state->total_y;
		it->y +=  cn*noise_state->total_y;
	}
}

void addAccNoise2StateList(size_t num, NoiseDataAcc* noise_states, StateInfo* states, double dTan, double dNor, size_t accMaxLen){
	size_t i = 0;
	for (i = 0; i < num; ++i){
		addnoise2tn(noise_states + i, states + i, dTan, dNor, accMaxLen);
	}
}

void addNoise2States(size_t num, StateInfo* states, double sigPos, double sigTh){
	size_t i = 0;
	for (i = 0; i < num; ++i){
		addnoise2xy(states + i, sigPos, sigTh);
	}
}

int buildMeAndOthersWithNoise(int myID,
	StateInfo* me, StateInfo* others,
	StateInfo* allagents, int nID, int* IDList, double sigPos, double sigTh)
{
	int i;
	for (i = 0; i<nID; ++i) {
		if (IDList[i] == myID){
			*me = allagents[i];
			me->ID = myID;
			addnoise2xy(me, sigPos, sigTh);
			break;
		}
	}
	if (i == nID) return -1;

	int idx = 0;
	for (i = 0; i<nID; ++i){
		if (IDList[i] != myID){
			others[idx] = allagents[i];
			addnoise2xy(others + idx, sigPos, sigTh);
			idx++;
		}
	}
	return 0;
}

void strcat2(char* buffer, const char* dir, const char* name){
	strcat(buffer, dir);
	strcat(buffer, name);
}

int runMulti(int expID, int nloop, int nID, int IDList[], SimuOptions opt){
	int i, j, k;
	FILE *out, *outDebug;
	double dt = opt.dt;     // 0.1s
	TimeInfo  tm;
	StateInfo agents[4];
	StateInfo agentsLastTime[4];
	StateInfo tmp[4];
	StateInfo me;
	StateInfo others[4];
	NoiseDataAcc noiseAccState[4];
	typedef real_t CtrlMsg[4];
	CtrlMsg ctrlmsg[4], cmsgtmp[4];
	int ctrlmsgValid[4] = { 0, 0, 0, 0 };
	int cmsgtmpValid[4] = { 0, 0, 0, 0 };

	void* ctrlptr[4];    // pointers to the controller
	int taskReached[4] = {0, 0, 0, 0};
	int taskReachedSum;
	ControlInfo u;
	int errorFlag;

	// Although we only have 4 agents (at most)
	// since ID starts with 1, we use 5 spots here...
	float initialX[5] =  {0,  0,   50, -100,   0};
	float initialY[5] =  {0, 500, 180, -100, -200};
	float initialTh[5] = {0, 90,   30, -10,  -90};
	float thetaOffset[5] = { 0, 1, -1, 2, 3 };

	// Initial States
	for (i=0;i<nID;++i){
		int id = IDList[i];
		agents[i].ID = id;
		agents[i].x  = initialX[id];
		agents[i].y  = initialY[id];
		agents[i].heading = initialTh[id];
		agents[i].v = 0;
		agents[i].w = 0;
		agentsLastTime[i] = agents[i];
	}
	
	// Result Data File
	char outname[14] = "res-00.txt";
	char outnameDebug[20] = "res-00-debug.txt";
	char foutname[64] = "";
	char foutnameDebug[64] = "";

	outname[4] = '0' + (expID / 10);
	outname[5] = '0' + (expID % 10);
	outnameDebug[4] = '0' + (expID / 10);
	outnameDebug[5] = '0' + (expID % 10);

#ifdef __QNXNTO__
	char dirname[32] = "/home/hzy/log/";
#else
	char dirname[4] = "";
#endif
	strcat2(foutname, dirname, outname);
	strcat2(foutnameDebug, dirname, outnameDebug);

	out = fopen(foutname, "w");
	outDebug = fopen(foutnameDebug, "w");
	printf("output: %s\n", foutname);
	printf("output: %s\n", foutnameDebug);

	// Initialize controllers
	for (i=0; i<nID; ++i){
		ControllerSetPtr(0);    // Default is 0, 
								// but if it detects that a none 0 ptr exists, it won't init
		if (ControllerInit(expID, IDList[i]) !=0 ){
			stopExitingControllers(ctrlptr, i);
			return -1;
		}
		else{
			ControllerSetDebugInfo(DEBUG_INFO_ALL, 1);
			ctrlptr[i] = ControllerGetPtr();
		}
	}
	//puts("init good");
	
	// Run Experiment
	tm.timeElapsed = 0;
	tm.loopCounter = 0;
	tm.globalTime = 10000;
	errorFlag = 0;
	for (i=0; i < nloop; ++i){
		taskReachedSum = 0;
		for (j=0; j<nID; ++j){
			taskReachedSum += taskReached[j];
		}
		if (taskReachedSum == nID){
			printf("All done\n");
			break;
		}

		if (expID == EXP_INTERVENTION){
			// The intervention experiment
			// Add User Input to some agent and some point
			if (tm.timeElapsed == 30 * 1000){
				ControllerSetPtr(ctrlptr[0]);

				InterventionInfo input = {1, 2};
				ControllerSetHumanInput(input);
			}

			if (tm.timeElapsed == 40* 1000){
				ControllerSetPtr(ctrlptr[nID - 1]);

				InterventionInfo input = { -1, -2 };
				ControllerSetHumanInput(input);
			}
		}

		for (j=0; j<nID; ++j){
			ControllerSetPtr(ctrlptr[j]);
			if ( ControllerState() == STATE_TARGET_REACHED){
				// This controller think it's done
				taskReached[j] = 1;
			}
			else if (ControllerState() != 0){
				// Something wrong... Stop Experiment
				printf("Error... expID %d, robotID %d, loopCounter %d\n", expID, IDList[j], i);
				errorFlag = 1;
				break;
			}
			// Controller is in good state 
			// or thinks it's done (which is also valid)
			// Handle Controller Msg
			for (k = 0; k < nID; ++k){
				if (k == j) continue;
				if (ctrlmsgValid[k]){
					ControllerHandleMsg(IDList[k], ctrlmsg[k], sizeof(CtrlMsg));
				}
			}



			// Build State Struct
			int ID = IDList[j];
			int nOthers = nID - 1;
			StateInfo* agentUse = (opt.useOneStepDelay == 1) ? agentsLastTime : agents;
			me = agents[j];	// My State is always the latest
			buildMeAndOthersWithNoise(ID, &me, others, agentUse, nID, IDList, opt.obSigPos, opt.obSigTh);
			
			if (0){
				me.heading += thetaOffset[ID];
				int zz;
				for (zz = 0; zz < nOthers; ++zz)
					others[zz].heading += thetaOffset[others[zz].ID];
			}
			
			// Compute control inputs
			u = ControllerCompute(tm, me, nOthers, others);

			// Get Controller Msg
			cmsgtmpValid[j] = ControllerGetMsg(cmsgtmp[j]);


			me = agents[j];
			tmp[j] = simulateRobot(me, u, dt);  // Simulate robot j, the new state is temporarily in tmp[j]
			addnoise2xy(tmp + j, opt.mdSigPos, opt.mdSigTh);
		}
		
		//puts("good ctrl");

		// Check Error
		if (errorFlag == 1){
			break; 
		}

		// Advance 
		memcpy(agentsLastTime, agents, sizeof(StateInfo) * 4);
		memcpy(agents, tmp, sizeof(StateInfo) * 4);
		memcpy(ctrlmsgValid, cmsgtmpValid, sizeof(int) * 4);
		memcpy(ctrlmsg, cmsgtmp, sizeof(CtrlMsg) * 4);
		
		// Save Data To File
		for (j=0;j<nID;++j){
			StateInfo* st = agents + j;
			fprintf(out, "%d %d %f %f %d %f %d\n", 
				tm.loopCounter, IDList[j], 
				st->x, st->y, (int)(st->heading), st->v, (int)(st->w));
		}
		fprintf(out, "\n");

		// Save Debug Info To File
		float buffer[16];
		ControllerSetPtr(ctrlptr[0]);
		int debugSize = ControllerGetDebugMsg(buffer);
		if (debugSize != sizeof(float) * 4 && debugSize != 0){
			printf("Weird debug info size....\n");
		}
		else if (debugSize > 0){
			fprintf(outDebug, "%7.2f %7.2f %7.2f %7.2f\n",
				buffer[0], buffer[1], buffer[2], buffer[3]);
		}

		// Advance Time
		tm.loopCounter++;
		tm.timeElapsed += (uint)(dt * 1000);
		tm.globalTime += (uint)(dt * 1000);
	}
	fclose(out);
	fclose(outDebug);
	stopExitingControllers(ctrlptr, nID);
	return 0;
}

void testGPSUnit(int incLat, int incLon, double reflat, double reflon, double th){
	double testlat, testlon;
	double disLocal, disReal;
	real_t x, y, heading;

	testlat = reflat + incLat * 10e-6;
	testlon = reflon + incLon * 10e-6;
	GPS2Local(testlat, testlon, th, &x, &y, &heading);
	disLocal = norm2(x, y);
	disReal = GPSDistance(reflat, reflon, testlat, testlon);
	printf("Lat Lon Th = (%2d, %2d) %5.2f : (x, y, th) = (%10.6f, %10.6f, %5.2f), distance error = %.6f\n", 
		incLat, incLon, th,
		x, y, heading, disReal - disLocal);
}

void testGPS(){
	puts("");
	printf("Testing GPS2Local\n");

	double reflat, reflon, refTh;
	//double testlat, testlon, testTh;
	
	GPSGetAnchor(&reflat, &reflon, &refTh);
	printf("Anchor (lat, lon)-Theta: %.10f, %.10f, %f\n", reflat, reflon, refTh);
	printf("Input Theta is set to 0, which corresponds to north\n");

	puts("");
	puts("Basic Test");
	testGPSUnit(1, 0, reflat, reflon, 0);
	testGPSUnit(0, 1, reflat, reflon, 0);
	testGPSUnit(1, 1, reflat, reflon, 0);
	testGPSUnit(1, 1, reflat, reflon, 45);
	
	puts("");
	puts("Setting the 0 heading direction to East");
	GPSSetAnchor(reflat, reflon, 90);
	testGPSUnit(1, 0, reflat, reflon, 0);
	testGPSUnit(0, 1, reflat, reflon, 0);
	testGPSUnit(1, 1, reflat, reflon, 45);
	
	puts("");
	puts("Setting the 0 heading direction to North East");
	GPSSetAnchor(reflat, reflon, 45);
	testGPSUnit(1, 1, reflat, reflon, 0);
}

int main(int argc, char* argv[]){
	if (argc == 1)
		testGPS();
	
	puts("");
	printf("Testing Controllers\n");
	int expID = 0;
	if (argc > 1){
		int t = atoi(argv[1]);
		if (t >= 1)
			expID = t;
	}
	while (expID == 0){
		printf("Input expID (>0): ");
		scanf("%d", &expID);
		if (expID < 1){
			expID = 0;
			puts("Input Invalid");
		}
	}


	printf("Running Experiment %d\n", expID);
	int nID = 4;
	int nloop = 3000;
	int IDList[4] = { 1, 2, 3, 4 };
	
	SimuOptions opt;
	opt.dt = 0.1;
	opt.mdSigPos = 2;
	opt.mdSigTh = 0.1;
	opt.obSigPos = 5;
	opt.obSigTh = 0.01;
	opt.useOneStepDelay = 1;

	if (runMulti(expID, nloop, nID, IDList, opt) !=0 )    {
		puts("Ooops....");
	}
	else{
		puts("It seems to work, plot the trace");
	}
	//system("pause");
	return 0;
}
#endif