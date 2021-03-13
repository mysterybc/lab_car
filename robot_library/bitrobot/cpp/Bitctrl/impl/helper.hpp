#ifndef _HELPER_MAS_H
#define _HELPER_MAS_H

#include <cmath>
#include <vector>
#include <cstddef>

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

namespace impl{

/******* Simple Math Functions **********/

inline double m2cm(const double& v1){ return v1 * 100; }
inline float  m2cm(const float& v1){ return v1 * 100; }

inline double cm2m(const double& v1){ return v1 / 100; }
inline float  cm2m(const float& v1){ return v1 / 100.0f; }

const double pi = 3.141592653589793;
template<class T>
inline T degInRange(T degree){
	while (degree > 180) degree -= 360;
	while (degree <= -180) degree += 360;
	return degree;
}

inline double rad2deg(double radian) {
	return radian * 180.0 / pi;
	//return degInRange(radian * 180.0 / pi);
}
inline float rad2deg(float radian){
	return static_cast<float>(radian * 180.0 / pi);
	//return degInRange(static_cast<float>(radian * 180.0 / pi));
}

inline double deg2rad(double degree){
	return degree * pi / 180.0;
	//return degInRange(degree) * pi / 180.0;
}

inline float deg2rad(float degree){
	return static_cast<float>(degree * pi / 180.0f);
	//return static_cast<float>(degInRange(degree) * pi / 180.0f);
}

inline double mm2cm(double mm){
	return mm / 10.0;
}
inline double cm2mm(double cm){
	return cm * 10.0;
}

inline double anglediff(double th1, double th2){
	th1 -= th2;
	while (th1 <= -pi) th1 += 2 * pi;
	while (th1 >= pi) th1 -= 2 * pi;
	return th1;
}
inline float anglediff(float th1, float th2){
	th1 -= th2;
	while (th1 <= -pi) th1 += float(2 * pi);
	while (th1 >= pi) th1 -= float(2 * pi);
	return th1;
}


template<class T>
inline T saturate(T x, T xmin, T xmax){
	if (x > xmax) return xmax;
	if (x < xmin) return xmin;
	return x;
}

template<class T>
inline T saturate(T x, T xmax){
	return saturate(x, -xmax, xmax);
}

template<class T>
inline T dead_zone(T x, T xmax, T xmin){
	if (x > xmax)      return x - xmax;
	else if (x < xmin) return x - xmin;
	else return 0;
}

template<class T>
inline T dead_zone(T x, T xmax){
	return dead_zone(x, xmax, -xmax);
}
/*
class VaryingDZone{
public:
	VaryingDZone(){}
	real_t lb, ub;
};
*/

inline double sumsquare(double x, double y){
	return x*x + y*y;
}

/**** Pt2D ****/
struct pt2D{
	pt2D():x(0),y(0){}
	pt2D(const double& x, const double& y):x(x), y(y){}
	inline pt2D& operator += (const pt2D& p){
		x += p.x; y += p.y;
		return *this;
	}
	inline pt2D& operator -= (const pt2D& p){
		x -= p.x; y -= p.y;
		return *this;
	}
	inline pt2D& operator *= (double k){
		x *= k; y *= k;
		return *this;
	}
	inline pt2D& operator /= (double k){
		x /= k; y /= k;
		return *this;
	}
	inline pt2D operator - (){
		return pt2D(-x, -y);
	}
	
	double x, y;
};
inline pt2D operator * (const pt2D& p, double k){
	return pt2D(k*p.x, k*p.y);
}
inline pt2D operator * (double k, const pt2D& p){
	return pt2D( k*p.x, k*p.y );
}
inline pt2D operator / (const pt2D& p, double k){
	return pt2D( p.x / k, p.y / k );
}
inline pt2D operator + (const pt2D& p1, const pt2D& p2){
	return pt2D( p1.x + p2.x, p1.y + p2.y );
}
inline pt2D operator - (const pt2D& p1, const pt2D& p2){
	return pt2D( p1.x - p2.x, p1.y - p2.y );
}

inline pt2D rot90(const pt2D& p){
	return pt2D( -p.y, p.x );
}

inline pt2D rot90neg(const pt2D& p){
	return pt2D( p.y, -p.x );
}

inline pt2D rot180(const pt2D& p){
	return pt2D( -p.x, -p.y);
}

// Rotate the Vector anti-clockwise
inline pt2D rot(const pt2D& p, double th){
	double cn = std::cos(th);
	double sn = std::sin(th);
	pt2D res = p;
	res.x = p.x * cn - p.y * sn;
	res.y = p.x * sn + p.y * cn;
	return res;
}

// Rotate the Vector anti-clockwise
inline pt2D rotDeg(const pt2D& p, double thDeg){
	return rot(p, deg2rad(thDeg));
}

inline double dot(const pt2D& a, const pt2D& b){
	return a.x * b.x + a.y * b.y;
}

inline double square(const pt2D& p){
	return dot(p, p);
}

inline double norm2(const pt2D& p){
	return std::sqrt(square(p));
}

inline pt2D normalize(const pt2D& p){
	double sz = norm2(p);
	if (sz > 0.000001){
		return p / sz;
	}
	else{
		return pt2D();
	}
}

inline double cos(const pt2D& p, const pt2D& q){
	return dot(normalize(p), normalize(q));
}

inline pt2D saturate(const pt2D& p, double sz_max){
	double sz = norm2(p);
	if (sz > sz_max)
		return p / sz * sz_max;
	return p;
}

inline pt2D dead_zone(pt2D x, double sz_dzone){
	double sz = norm2(x);
	double sz2 = dead_zone(sz, sz_dzone);
	if (sz2 > 0){
		return x * sz2 / sz;
	}
	return pt2D();
}

/*****Line Object****/
struct line2D{
	line2D(){}
	line2D(double x, double y, double th){ p.x = x; p.y = y; this->th = th; }
	line2D(pt2D start, pt2D end){
		p = start;
		end -= start;
		th = std::atan2(end.y, end.x);
	}
	pt2D dir() const {
		return pt2D(std::cos(th), std::sin(th));// { std::cos(th), std::sin(th) };
	}
	pt2D normal() const{
		return pt2D(-std::sin(th), std::cos(th));// { -std::sin(th), std::cos(th) };
	}

	pt2D   p;
	double th;	// in radians
};

inline pt2D project(pt2D pt, const line2D& line){
	pt2D e = pt - line.p;
	pt2D lineNormal = line.normal();
	double dist = dot(e, lineNormal);

	return pt - dist*lineNormal;
}

inline double distance(pt2D pt, const line2D& line){
	pt2D e = pt - line.p;
	pt2D lineNormal = line.normal();
	return dot(e, lineNormal);
}


// A Modified Sigmoid that
// is about 1  when x>=1
// is about 0  when x<=-1
// is 0.5 when x = 0
// is about 0.9 when x = 0.6
inline double sigmoid(double x){
	return 1.0 / (1 + std::exp(-x*5.0));
}

inline double sigmoid2(double x){
	return 2.0 / (1 + std::exp(-x*5.0)) - 1;
}

// is about +- 1 when x = += scale
inline double sigmoid(double x, double scale){
	if (scale != 0)
		return sigmoid(x / scale);

	// WARNING !!! THIS SHOULD NOT HAPPEN
	return 0;
}

// is about 0  when x = lb
// is about +1 when x = ub
inline double sigmoid(double x, double lb, double ub){
	double scale = (ub - lb) / 2;
	double zero = (ub + lb) / 2;
	if (lb != ub)
		return sigmoid(x - zero, scale);

	// WARNING !!! THIS SHOULD NOT HAPPEN
	return 0;
}


/*******Vector Tools*********/

template<class T>
inline T ZeroOf() { return static_cast<T>(0.0); }

template<>
inline pt2D ZeroOf<pt2D>() { return pt2D(); }


template<class T>
void reserveAndResize(size_t sz, std::vector<T>& v){
	v.reserve(sz);
	v.resize(sz);
}

template<class T, class... T2>
void reserveAndResize(size_t sz, std::vector<T>& v, T2&... rest){
	v.reserve(sz);
	v.resize(sz);
	reserveAndResize(sz, rest...);
}

template<class T>
bool checksize(size_t sz, std::vector<T>& v){
	return v.size() == sz;
}

template<class T, class... T2>
bool checksize(size_t sz, std::vector<T>& v, T2&... rest){
	return v.size() == sz && checksize(sz, rest...);
}

template<class T>
T sum(const std::vector<T>& v, T v0 = ZeroOf<T>()){
	typename std::vector<T>::const_iterator it = v.begin();

	for (; it != v.end(); ++it){
		v0 = v0 + *it;
	}
	return v0;
}

template<class T>
T max(const std::vector<T>& v){
	if (v.size() == 0) return 0;

	typename std::vector<T>::const_iterator it = v.begin();
	T v0 = *it;
	it++;

	for (; it != v.end(); ++it){
		if (v0 < *it) v0 = *it;
	}
	return v0;
}

template<class T>
T min(const std::vector<T>& v){
	if (v.size() == 0) return 0;

	typename std::vector<T>::const_iterator it = v.begin();
	T v0 = *it;
	it++;

	for (; it != v.end(); ++it){
		if (v0 > *it) v0 = *it;
	}
	return v0;
}

template<class T>
T mean(const std::vector<T>& v, T zero = ZeroOf<T>()){
	if (v.empty()) return zero;
	typename std::vector<T>::const_iterator it = v.begin();
	for (; it != v.end(); ++it){
		zero = zero + *it;
	}
	return zero / v.size();
}

template<class T>
T meansquare(const std::vector<T>& v, T zero = ZeroOf<T>()){
	if (v.empty()) return zero;
	typename std::vector<T>::const_iterator it = v.begin();
	for (; it != v.end(); ++it){
		zero = zero + *it * *it;
	}
	return zero / v.size();
}

template<class Base>
class wValid : public Base{
public:
	explicit wValid(bool valid = false) : _valid(valid) {}
	wValid(const Base& base) : _valid(true), Base(base){}

	wValid& operator = (const Base& base){
		Base* pbase = dynamic_cast<Base*>(this);
		*pbase = base;
		this->_valid = true;
		return *this;
	}

	bool operator !() const { return !_valid; }
	operator bool() const{ return _valid; }
	bool valid() const { return _valid; }
	void setValid(bool valid = true) { _valid = valid; }
private:
	bool _valid;
};
template<>
class wValid<float> {
public:
	explicit wValid(bool valid = false) : value(0), _valid(valid) {}
	wValid(const float& value) : value(value), _valid(true) {}

	wValid& operator = (const float& other) {
		value = other;
		_valid = true;
		return *this;
	}
	operator float() const { return value; }

	bool operator !() const { return !_valid; }
	operator bool() const { return _valid; }
	bool valid() const { return _valid; }
	void setValid(bool valid = true) { _valid = valid; }

	float value;
private:
	bool _valid;
};

template<class Scalar>
class RangeCheck {
public:
	RangeCheck() : lb(0), ub(0){}
	RangeCheck(const Scalar& lb, const Scalar& ub) :lb(lb), ub(ub) {}
	
	void set_range(const Scalar& _sz) {
		lb = -_sz; ub = _sz;
	}
	
	void set_range(const Scalar& _lb, const Scalar& _ub) {
		lb = _lb; ub = _ub;
	}

	bool operator() (const Scalar& v) { return v >= lb && v < ub; }
	bool operator() (const Scalar& v1, const Scalar& v2) {
		return operator()(v1) && operator()(v2);
	}

	template<class T>
	bool operator() (const std::vector<T>& v) {
		std::size_t n = v.size();
		for (std::size_t i = 0; i < n; ++i) {
			if (!operator()(v[i])) return false;
		}
		return true;
	}

	template<class Iter>
	bool operator() (Iter beg, const Iter& end) {
		for (; beg != end; ++beg) {
			if (!operator()(*beg)) return false;
		}
		return true;
	}

	Scalar lb, ub;
};

}      // namespace impl
#endif // _HELPER_MAS_H
