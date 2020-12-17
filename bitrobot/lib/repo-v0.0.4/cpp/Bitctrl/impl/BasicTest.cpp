#include "BasicTest.hpp"


namespace impl{


BasicTest::BasicTest(int robotID) :Controller(robotID){
	initState = 0;
	runtimeState = 0;

	use2D = false;
	useAvoid = true;

	enableLogging(LOG_FILE_ALL, "BasicTest", false);	

	val_testW  = 50;
	val_testV  = 20;
	val_testTh = 0;

	// Add parameter to ConfigSet
	config.addarg(circleRadius,  "circle_radius");
	config.addarg(circleLineVel, "circle_lineV");
	config.addarg(val_testW,  "test_w");
	config.addarg(val_testV,  "test_v");
	config.addarg(val_testTh, "test_Th");
}

ControlInfo BasicTest::eightShape(TimeInfo tm, DataSet& dat){
	uint   Tcircle = static_cast<uint>(2 * circleRadius * pi / circleLineVel * 1000);	// ms
	real_t w = (real_t)rad2deg(circleLineVel / circleRadius);

	if (tm.loopCounter == 0){
		sleepCounter.resetTime(Tcircle, tm);
		circleTime = 0;
	}
	sleepCounter.update(tm);
	if (sleepCounter.completed()){
		circleTime++;
		sleepCounter.resetTime(Tcircle, tm);
	}

	ControlInfo u;
	u.v = (real_t)circleLineVel;
	if (shape == eight_shape)
		u.w = (circleTime % 2 == 0) ? w : -w;
	else
		u.w = w;
	
	double seconds = 5 * 60;
	int counter = int(seconds * 1000 / Tcircle + 0.5);
	if (circleTime == counter){
		runtimeState = STATE_TARGET_REACHED;
		u.v = u.w = 0;
	}

	return u;
}

ControlInfo BasicTest::wTest(TimeInfo tm, DataSet& dat){
	ControlInfo u;
	u.w = (real_t)val_testW;
	u.v = (real_t)0;
	
	LOGINFO("{} {:.2f} {:.2f} {:.2f}",   tm.timeElapsed, 
		rad2deg(dat.th[0]), u.w, dat.vw[0].y);
	return u;
}
ControlInfo BasicTest::vTest(TimeInfo tm, DataSet& dat){
	ControlInfo u;
	u.w = (real_t)0;
	u.v = (real_t)val_testV;
	LOGINFO("{} {:.2f} {:.2f} {:.2f} {:.2f}", tm.timeElapsed, 
		dat.xy[0].x, dat.xy[0].y, u.v, dat.vw[0].x);
	return u;
}

ControlInfo BasicTest::wvTest(TimeInfo tm, DataSet& dat){
	ControlInfo u;
	u.w = (real_t)val_testW;
	u.v = (real_t)val_testV;
	LOGINFO("{} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}", tm.timeElapsed, 
		dat.xy[0].x, dat.xy[0].y, rad2deg(dat.th[0]), u.v, u.w, dat.vw[0].x, dat.vw[0].y);
	return u;
}

ControlInfo BasicTest::thTest(TimeInfo tm, DataSet& dat){
	ControlInfo u;
	u.w = (real_t)rad2deg(anglediff(deg2rad(val_testTh), dat.th[0]));
	u.v = (real_t)0;
	LOGINFO("{} {:.2f} {:.2f} {:.2f}", tm.loopCounter, rad2deg(dat.th[0]), val_testTh, u.w);
	return u;
}

ControlInfo BasicTest::innerCompute(TimeInfo tm, DataSet& dat){
	switch (shape)
	{
	case impl::BasicTest::circle_shape:
		return eightShape(tm, dat);
		break;
	case impl::BasicTest::eight_shape:
		return eightShape(tm, dat);
		break;
	case impl::BasicTest::w_test:
		return wTest(tm, dat);
		break;
	case impl::BasicTest::v_test:
		return vTest(tm, dat);
		break;
	case impl::BasicTest::wv_test:
		return wvTest(tm, dat);
		break;
	case impl::BasicTest::th_test:
		return thTest(tm, dat);
		break;
	default:
		return {0, 0};
		break;
	}
}



}
