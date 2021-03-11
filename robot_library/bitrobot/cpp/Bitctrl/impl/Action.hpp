#ifndef _CONTROLLER_ACTION_H
#define _CONTROLLER_ACTION_H
#include "helper.hpp"
#include "BIT.h"

namespace impl{

class ActionSleep{
public:
	ActionSleep(){
		byLoop = false;
		byTime = false;
	}
	void disable() { byLoop = false; byTime = false; }
	bool unset() const { return !(byLoop || byTime); }
	void resetLoops(uint nloop, const TimeInfo& t0){
		byLoop = true;	byTime = false;
		this->t0 = t0;
		max_loop = nloop;
	}
	void resetTime(uint ms, const TimeInfo& t0){
		byTime = true; byLoop = false;
		this->t0 = t0;
		max_ms = ms;
	}

	void update(const TimeInfo& tm){
		tnow = tm;
	}
	bool completed(){
		if (byLoop){
			return (tnow.loopCounter - t0.loopCounter) >= max_loop;
		}
		if (byTime){
			return (tnow.timeElapsed - t0.timeElapsed) >= max_ms;
		}
		return true;
	}

protected:
	bool byLoop, byTime;
	TimeInfo t0;
	TimeInfo tnow;
	uint max_loop, max_ms;
};



}



#endif
