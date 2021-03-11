#pragma once
#include "../sstcp.hpp"
#include "proto_base.hpp"
#include "proto_util.hpp"
#include <vector>
#include <cstdio>


namespace protobit {

// Use packID() >= PackBegin && packID() < PackEnd 
// to test if this package is a part of protobit
enum {
	PackNone = -1,
	PackBegin = 0,
	PackSTAT = PackBegin,
	PackCOMD,
	PackPATH,
	PackTURN,
	PackOBST,
	PackTELE,
	PackTALG,
	PackTDEG,
	PackFORM,
	PackEIGT,
	PackALGO,
	PackPLAN,
	PackMPLN,
	PackEnd
};
PackBase* new_pack(int packID);

typedef RangeInt<-1, 20> ID;
typedef RangeInt<-90000, 90000> XY;

#define __impl_protobit_declare(name) \
	int packID() const { return Pack##name; } \
	const char* pack_name() const { return "protbit::"#name; } \
	void print_data(std::FILE* out, const char* line_prefix = "") const; \
	bool valid() const; \
	int write(char* buffer, int size); \
	int read(const char* buffer, int size); \
	void reset() { *this = name(); } \
	bool operator == (const PackBase& other) const
	

struct STAT: public PackBase {
	__impl_protobit_declare(STAT);

	typedef RangeInt<0, 3>   Status;
	typedef RangeInt<0, 128> TaskType;
	enum { Running = 0, Completed = 1, Error = 2, ErrorGPS = 3 };
	enum { TaskNone = 0, TaskTrack = 1, TaskTurn = 2 };

	ID id;
	XY x, y;
	RangeInt<-360, 360> th;
	RangeInt<-200, 200> v;
	RangeInt<-360, 360> w;
	Status   status;
	TaskType type;
	RangeInt<0, 100> progress;
};

struct COMD: public PackBase {
	__impl_protobit_declare(COMD);

	enum { Stop = 0, Pause = 1, Resume = 2, ConfigReload, LogTraceBegin, LogTraceStop, DetectFault, ActFaulty, CMDNum };

	ID id;
	RangeInt<0, CMDNum> cmd;
};
struct PATH: public PackBase {
	__impl_protobit_declare(PATH);

	typedef RangeInt<1, 1000> PathVel;
	ID id;
	RangeInt<0, 128> n;
	std::vector<XY> x, y;
	std::vector<PathVel> v;
};
struct FORM : public PackBase {
	__impl_protobit_declare(FORM);

	enum {	Basic = 0, TestEight = 1 };
	typedef RangeInt<1, 1000> PathVel;

	RangeInt<0, 100>    type;
	RangeInt<0, 10000>  elen;

	RangeInt<0, 10> fmgroup;  // formation Group
	RangeInt<1, 10> nrobot;
	std::vector<ID> robot;

	RangeInt<1, 128> npoint;
	std::vector<XY> x, y;
	std::vector<PathVel> v;
};
struct EIGT: public PackBase {
	__impl_protobit_declare(EIGT);

	RangeInt<0, 10000>  len;
	RangeInt<-180, 180> dir;
	RangeInt<0, 1000>   v;
	RangeInt<0, 1000>   r;
	RangeInt<1, 10>     n;
	std::vector<ID>     ids;
};

struct TURN: public PackBase {
	__impl_protobit_declare(TURN);

	ID id;
	XY xd, yd;
};
struct TDEG : public PackBase {
	__impl_protobit_declare(TDEG);

	enum { Relative = 0, Absolute = 1 };
	ID id;
	RangeInt<0, 1> type;
	RangeInt<-360, 360> theta;
};
struct OBST: public PackBase {
	__impl_protobit_declare(OBST);

	typedef RangeInt<1, 1000> Radius;
	enum { Circle = 1, Rect = 2 };
	
	ID id;
	RangeInt<1, 2> type;
	RangeInt<0, 100> n;
	std::vector<XY>  x, y;
	std::vector<Radius> r;
};
struct TELE: public PackBase {
	__impl_protobit_declare(TELE);

	enum { Direct = 0, Shared = 1, ByAlgoVW = 2, ByAlgoVxVy = 3 };

	ID id;
	RangeInt<0, 3> type;
	RangeInt<-1000, 1000> v;
	RangeInt<-360, 360>   w;
};

struct TALG : public PackBase {
	__impl_protobit_declare(TALG);
	enum { ByVxy = 0, ByVW = 1 };

	void set_vw(float v, float w) {
		type() = ByVW; 
		v1 = int(v * 10); v2 = int(w * 10);
	}
	void set_vxy(float vx, float vy) {
		type() = ByVxy;
		v1 = int(vx * 10); v2 = int(vy * 10);
	}
	bool get_vw(float& v, float& w) {
		if (type() == ByVW) { 
			v = float(v1() / 10.0); w = float(v2() / 10.0);
			return true;
		}
		return false;
	}
	bool get_vxy(float& vx, float& vy) {
		if (type() == ByVxy) {
			vx = float(v1() / 10.0); vy = float(v2() / 10.0);
			return true;
		}
		return false;
	}


	ID id;
	RangeInt<0, 1> type;
	RangeInt<-99999, 99999> v1, v2;
};

// Algorithm Data 
// ALGO,myID,subType,DATA
// subType = shp: DAT=num,gpID,shpIndex,priority
struct ALGO : public PackBase {
	__impl_protobit_declare(ALGO);

	enum {
		shp = 0, info = 1, iamfault = 2, iamgood = 3
	};
	typedef RangeInt<-9999, 9999> Int4;

	ID id;
	RangeInt<0, 100> subType;

	// subType = shp
	Int4 nrobot, gpID, shpIndex, priority;
	
	// subType = info
	RangeInt<0, 100> task;
	RangeInt<0, 100> state;
};

struct PLAN : public PackBase {
	__impl_protobit_declare(PLAN);
	enum { Basic = 0, PlanSelf = 1, PlanSelfAndGO = 2 };

	ID id;
	RangeInt<0, 2> type;
	RangeInt<0, 9000> v;
	RangeInt<0, 9000> d;
	XY xd, yd;
	XY x0, y0;
};

struct MPLN : public PackBase {
	__impl_protobit_declare(MPLN);

	typedef RangeInt<1, 1000> PathVel;
	enum { Basic = 0, PlanSelf = 1, PlanSelfAndGO = 2 };

	ID id;
	RangeInt<0, 2>  type;
	RangeInt<0, 9000> d;
	RangeInt<0, 10> n;
	std::vector<XY> x;
	std::vector<XY> y;
	std::vector<PathVel> v;
};

#undef __impl_protobit_declare

} // namespace protobit 





