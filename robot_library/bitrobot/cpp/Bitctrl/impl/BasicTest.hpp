#ifndef _BASIC_TEST_HPP
#define _BASIC_TEST_HPP

#include "Base.hpp"
#include "Action.hpp"

namespace impl{

class BasicTest:public Controller{
public:
	BasicTest(int robotID);

	int state(){
		return initState != 0 ? initState :
			runtimeState;
	}
	ControlInfo innerCompute(TimeInfo tm, DataSet& dat);
	ControlInfo eightShape(TimeInfo tm, DataSet& dat);
	ControlInfo wTest(TimeInfo tm, DataSet& dat);
	ControlInfo vTest(TimeInfo tm, DataSet& dat);
	ControlInfo thTest(TimeInfo tm, DataSet& dat);
	ControlInfo wvTest(TimeInfo tm, DataSet& dat);
	size_t getDebugMsg(void* buf){
		float* dest = (float*)buf;
		dest[0] = (float)val_testV;
		dest[1] = (float)val_testW;
		dest[2] = (float)circleRadius;
		dest[3] = (float)circleLineVel;
		return sizeof(float) * 4;
	}

	void configCircle(double rad, double lineV){
		circleRadius = rad;
		circleLineVel = lineV;
	}
	enum AvaiableShape{
		circle_shape, eight_shape, w_test, v_test, wv_test, th_test
	}shape;

	double val_testW;
	double val_testV;
	double val_testTh;

protected:
	// Helper Functions
	bool useCollisionAvoid() { return useAvoid; }
	bool innerUse2D()        { return use2D; }

protected:
	bool useAvoid;
	bool use2D;

	int  initState;
	int  runtimeState;

	double circleRadius;	// in cm
	double circleLineVel;   // in cm/s

	int circleTime;
	ActionSleep sleepCounter;

private:
	real_t th_start;
};

}



#endif