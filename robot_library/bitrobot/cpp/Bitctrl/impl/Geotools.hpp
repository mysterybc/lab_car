#pragma once
#include "helper.hpp"
#include "tkspline.h"
#include <iterator>
#include <algorithm>
#include <map>

namespace impl{
#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

struct DataSet;

struct FormationInfo{
    struct ErrorStat{
        double max;
        double min;
        double mean;
        double meansq2;
    };

    //static FormationInfo selectFormation(std::map<int, pt2D>& dxy, const DataSet& dat);
    static void selectFormation(const std::map<int, pt2D>& dxy, const DataSet& dat, std::map<int, int>& idremap);
    //static double evaluate_formation(const DataSet& dat, const pt2D& center, const std::vector<pt2D>& sdxy);


    FormationInfo(){}
    FormationInfo(const std::map<int, pt2D>& dxy, const DataSet& dat){
        setData(dxy, dat);
    }
    /*
    FormationInfo(const std::vector<pt2D>&  sdxy, const DataSet& dat){
        setData(sdxy, dat);
    }
    */
    void setData(const std::map<int, pt2D>& dxy, const DataSet& dat);
    //void setData(const std::vector<pt2D>&  sdxy, const DataSet& dat);


    std::vector<pt2D> pts;
    std::vector<double> eth, epos;
    std::vector<int> ID;
    pt2D center;
    pt2D centerPhy;
    double centerTh;
    ErrorStat thError, posError;
protected:
    static void getStats(ErrorStat& stat, const std::vector<double>& e){
        stat.max = max(e);
        stat.min = min(e);
        stat.mean = mean(e);
        stat.meansq2 = meansquare(e);
    }
    static std::vector<pt2D> getSequenceDxy(const std::map<int, pt2D>& dxy, const DataSet& dat);
};

class ShapeTrace{
    template <typename T>
    static void Append(std::vector<T>& a, const std::vector<T>& b){
        a.reserve(a.size() + b.size());
        a.insert(a.end(), b.begin(), b.end());
    }
    template <typename T>
    static void Reverse(std::vector<T>& a){
        std::reverse(a.begin(), a.end());
    }
    template <typename T>
    static void Shift(std::vector<T>& a, T dval){
        typename std::vector<T>::iterator it = a.begin();
        for (;it != a.end(); ++it) *it += dval;
    }

public:
    static ShapeTrace Line(double headingInDegree, double length, double vline);
    static ShapeTrace Circle(int npoints, double R, double vline, double startAt = -90);


    bool empty() const{
        if (*this){
            return tlist.empty();
        }
        return true;
    }
    operator bool() const {
        return tlist.size() == xlist.size() && xlist.size() == ylist.size();
    }
    ShapeTrace append(const ShapeTrace& tr) const{
        ShapeTrace res = *this;
        if (tr){
            if (this->empty())  return tr;

            ShapeTrace tr2;
            tr2.tlist = std::vector<double>(tr.tlist.begin() + 1, tr.tlist.end());
            tr2.xlist = std::vector<double>(tr.xlist.begin() + 1, tr.xlist.end());
            tr2.ylist = std::vector<double>(tr.ylist.begin() + 1, tr.ylist.end());
            Shift(tr2.tlist, tlist.back());

            Append(res.tlist, tr2.tlist);
            Append(res.xlist, tr2.xlist);
            Append(res.ylist, tr2.ylist);
        }
        return res;
    }

    ShapeTrace reverse() const{
        ShapeTrace res = *this;
        Reverse(res.xlist);
        Reverse(res.ylist);
        return res;
    }
    ShapeTrace shiftTime(double dt)const{
        ShapeTrace res = *this;
        Shift(res.tlist, dt);
        return res;
    }
    ShapeTrace shift(double dx, double dy)const{
        ShapeTrace res = *this;
        Shift(res.xlist, dx);
        Shift(res.ylist, dy);
        return res;
    }

    std::vector<double> tlist, xlist, ylist;
};

inline ShapeTrace operator + (const ShapeTrace& a, const ShapeTrace& b){
    return a.append(b);
}

class spline2D{
public:
    spline2D(){ setup(); }
    spline2D(const ShapeTrace& shape){  setpoints(shape); }
    spline2D(const  std::vector<double>& t, const  std::vector<double>& x, const  std::vector<double>& y){
        setpoints(t, x, y);
    }

    void setpoints(const ShapeTrace& shape){
		setpoints(shape.tlist, shape.xlist, shape.ylist);
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

protected:
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
