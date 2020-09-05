#pragma once
#include <vector>
#include "Tracking/tkspline.h"
#include "ros/ros.h"
#include "Tracking/helper.hpp"


namespace impl{

class spline2D{
public:
    spline2D(){ setup(); }
    spline2D(const  std::vector<double>& t, const  std::vector<double>& x, const  std::vector<double>& y){
        setpoints(t, x, y);
    }

    void setpoints(const std::vector<double>& t, const  std::vector<double>& x, const  std::vector<double>& y){
		recLastT = t; recLastX = x; recLastY = y;
		setup();
		if (t.size() <= 2){
			size_t n = t.size();
			std::vector<double> t2 = t;
			std::vector<double> x2 = x;
			std::vector<double> y2 = y;
			for (int i = 1; i <= 2; ++i){
				t2.push_back(t[n - 1] + i);
				x2.push_back(x[n - 1]);
				y2.push_back(y[n - 1]);
			}
			setpoints(t2, x2, y2);
		}
		else{
			lineX.set_points(t, x);
			lineY.set_points(t, y);
			tmax = t.back();
			pts_size = t.size();
		}
    }
	void clear() {
		lineX = tk::spline();
		lineY = tk::spline();
	}

    void   setT0(double t, double t_ref = 0) { t0 = t - t_ref; }
    double getT0() const { return t0; }
    void   setAutoLoop(bool useAutoLoop, int maxloop = -1) { autoloop = useAutoLoop; loopMax = maxloop; }
    void   setAutoStop(bool useAutoStop) {
        if (useAutoStop)    autoloop = false;
        autostop = useAutoStop;
    }

    int loopPassed() const { return loopcount; }
    bool reachEnd() const   { return reachend; }

    pt2D operator() (double t){
        t = pseudo(t);
        return pt2D( lineX(t), lineY(t) );
    }
    pt2D deriv(int n, double t){
        t = pseudo(t);
        if (reachEnd()){
            return pt2D();
        }
        return pt2D( lineX.deriv(n, t), lineY.deriv(n, t) );
    }

    double percentage(double t) const{
        double r = (t - t0) / tmax;
        if (r <= 0) r = 0;
        if (r >= 1) r = 1;
        return r;
    }

    double maxTime() const {
        return tmax;
    }
    size_t size() const{
        return pts_size;
    }
    bool empty() const{
        return size() == 0;
    }

	std::vector<double> recLastT, recLastX, recLastY;

    void setup(){
        t0 = 0;
        tmax = 0;
        autoloop = false;
        autostop = true;
        loopcount = 0;
        reachend = false;
        loopMax = -1;
		lineX = tk::spline();
		lineY = tk::spline();
    }
    double pseudo(double t){
        t -= t0;
		if (t < 0) 
			t = 0;
		loopcount = 0;
		reachend = false;
        if (autoloop){
            while (t > tmax * (loopcount + 1)){
                loopcount++;
            }
            while (t > tmax)
                t -= tmax;

            if (loopMax > 0 && loopcount >= loopMax){
                reachend = true;
            }
        }
        else if (autostop){
            if (t >= tmax){
                t = tmax;
                reachend = true;
            }
			else {
				reachend = false;
			}
        }

        return t;
    }

private:
    bool autoloop;  // if automatically loop back
    int  loopcount; // how many loops have be tranversed
    int  loopMax;   // how many loops can be tranversed before stop
    bool autostop;  // if automatically stop when t is beyond end
    bool reachend;  // if endtime is reached

    double t0;      // time shift
    double tmax;    // tmax
    tk::spline lineX, lineY;
    size_t pts_size;
};
}