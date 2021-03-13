#pragma once
#include "NSAvoid.hpp"
#include "NSAvoidUtil.hpp"
#include <iostream>
//extern bool print_debug;
namespace NSAvoid {

/*
* A config class used by
*/
template<class Scalar>
struct NSConsConfig{
	typedef NSBound<Scalar> Bound;
	typedef Bound(*BoundFunction)(Scalar, void*);

	explicit NSConsConfig(BoundFunction f, void* f_arg) {
		set_limit_func(f, f_arg);
		penalty = (Scalar)-1;
		value_min = 0;
		value_max = Bound::inf();
	}
	bool range_check(const Scalar& value) const {
		return value >= value_min && value < value_max;
	}
	Bound limit(const Scalar& value) const {
		if (lim) return lim(value, arg);
		throw std::runtime_error("NSConsConifg.lim=NULL");
	}
	void set_limit_func(BoundFunction f, void* f_arg) {
		lim = f;
		arg = f_arg;
	}

	Scalar penalty;		// If penalty > 0, this constraint will be relaxed and violations will be penalized by this.
	Scalar value_min;
	Scalar value_max;
private:
	BoundFunction lim;
	void   *arg;
};

/*
* A helper base class for constructing NSConstraints
* it delegates the computation of vlim, penalty to the NSConsConfig
*/
template<class Scalar>
class NSConsDelegate : public NSConstraint<Scalar> {
public:
	typedef NSBound<Scalar> Bound;
	typedef NSConsConfig<Scalar> Config;
	typedef typename impl::util<Scalar>::VecX VecX;
	typedef typename NSConstraint<Scalar>::ConsInfo ConsInfo;

	explicit NSConsDelegate(const Config& conf)
		: _vob(0), //one_sided(true),
		_config(conf) {}
	~NSConsDelegate() {}

	NSConsDelegate& set_static() { _vob.resize(0); return *this; }
	NSConsDelegate& set_dynamic(const VecX& vob) { _vob = vob; return *this; }
	//NSConsDelegate& set_onsided(bool one_side = true) { one_sided = one_side; return *this; }

	virtual ConsInfo compute(const VecX& x) const {
		ConsInfo info = part_compute(x, _vob);
		if (info.valid) {
			if (info.inside || _config.range_check(info.value)) {
				info.penalty = _config.penalty;
				info.vlim = _config.limit(info.value);
			}
			else{
				info.valid = false;
			}
		}
		return info;
	}
	const Config& config() const { return _config; }

	// Delegate the rest of compute to childs
	// Should init for valid, gx, and Dg
	virtual ConsInfo part_compute(const VecX& x, const VecX& vob) const = 0;

private:
	VecX _vob;
	//	bool one_sided;
	const Config& _config;
};

/*
* A Line segment constraint for 2D-plane
* the line is specified by q0->q1,
* the "interior" is on the right when walking from q0 to q1
*/
template<class Scalar>
class NSConsLine : public NSConsDelegate<Scalar> {
public:
	typedef NSConsDelegate<Scalar> Base;
	typedef NSConsConfig<Scalar>   Config;
	typedef impl::util<Scalar> util;
	typedef typename util::Vec2X Vec2X;
	typedef typename util::VecX VecX;
	typedef typename NSConstraint<Scalar>::ConsInfo ConsInfo;

	enum ConsSide {
		LeftSide, RightSide, BothSides
	};

	explicit NSConsLine(const Vec2X& q0, const Vec2X& q1, const Config& conf, int side = BothSides)
		: Base(conf), side(side)
	{
		v_tan = q1 - q0;
		seg_len = v_tan.norm();
		v_tan.normalize();      // in place normalization
		v_nor.resize(2);
		v_nor << -v_tan(1), v_tan(0);
		x0 = q0;
	}

	ConsInfo part_compute(const VecX& x, const VecX& v_ob) const {
		ConsInfo ret(false);

		Vec2X dx = util::to2X(x) - x0;
		Scalar L = v_tan.dot(dx);
		if (L >= 0 && L < seg_len) {
			ret.value = v_nor.dot(dx);
			ret.grad = util::toX(v_nor);
			ret.grad_dt = (v_ob.size() > 0) ? (-ret.grad.dot(v_ob)) : 0;

			ret.valid = true;
			if (side == LeftSide) {
				//ret.valid = ret.value >= 0;
				ret.inside = ret.value < 0;
			}
			else if (side == RightSide) {
				//ret.valid   = ret.value <= 0;
				ret.inside = ret.value > 0;
				ret.value = -ret.value;
				ret.grad = -ret.grad;
				ret.grad_dt = -ret.grad_dt;
			}
			else if (side == BothSides) {
				//ret.valid   = true;
				ret.inside = false;
				if (ret.value < 0) {
					ret.value = -ret.value;
					ret.grad = -ret.grad;
					ret.grad_dt = -ret.grad_dt;
				}
			}
		}
		else {
			// BUG Fix: 2020/12/09
			Scalar d0 = dx.norm();
			Vec2X dx1 = dx - v_tan * seg_len;
			Scalar d1 = dx1.norm();
			if (d0 < d1) {
				ret.valid = true;
				ret.value = d0;
				ret.inside = false;
				ret.grad = util::toX(dx.normalized());
				ret.grad_dt = (v_ob.size() > 0) ? (-ret.grad.dot(v_ob)) : 0;
			}
			else {
				ret.valid = true;
				ret.value = d1;
				ret.inside = false;
				ret.grad = util::toX(dx1.normalized());
				ret.grad_dt = (v_ob.size() > 0) ? (-ret.grad.dot(v_ob)) : 0;
			}
		}
		return ret;
	}

private:
	int side;
	Vec2X  x0;
	Vec2X  v_tan, v_nor;
	Scalar seg_len;
};

/*
* A point/circular arc constraint
* The basic construction is the (center, radius) pair, which specifies a circle
* When an extra (theta_min, theta_max) pair is given, it specifies a circular arc
* the arc starts at theta_MAX, walking CLOCKWISE, and ends at theta_MIN
* the righthand side is the interior.
*/
template<class Scalar>
class NSConsPoint : public NSConsDelegate<Scalar> {
public:
	typedef NSConsDelegate<Scalar> Base;
	typedef NSConsConfig<Scalar>   Config;
	typedef impl::util<Scalar> util;
	typedef typename util::Vec2X Vec2X;
	typedef typename util::VecX VecX;
	typedef typename NSConstraint<Scalar>::ConsInfo ConsInfo;

	explicit NSConsPoint(const Vec2X& center, Scalar radius, const Config& conf)
		: Base(conf), x0(center), r(radius), th_min(-util::inf()), th_max(util::inf())
	{
	}
	explicit NSConsPoint(const Vec2X& center, Scalar radius, Scalar th_min, Scalar th_max, const Config& conf)
		: Base(conf), x0(center), r(radius), th_min(th_min), th_max(th_max)
	{
		while (th_max <= th_min) th_max += util::TwoPi;
	}

	ConsInfo part_compute(const VecX& x, const VecX& vob) const {
		ConsInfo ret(false);
		Vec2X dx = util::to2X(x) - x0;

		ret.value = dx.norm() - r;
		Scalar theta = std::atan2(dx(1), dx(0));
		if (th_max == util::inf() || util::in_between(theta, th_min, th_max) || theta == th_max) {
			ret.valid = true;
			ret.grad = util::toX(dx.normalized());
			ret.grad_dt = (vob.size() > 0) ? (-ret.grad.dot(vob)) : 0;
			ret.inside = ret.value < 0;
		}
		return ret;
	}

private:
	Vec2X  x0;
	Scalar r;
	Scalar th_min, th_max;
};

template<class Scalar>
class NSConsConvexPoly : public NSConsDelegate<Scalar> {
public:
	typedef NSConsDelegate<Scalar> Base;
	typedef NSConsConfig<Scalar>   Config;
	typedef NSConsLine<Scalar>     NSLine;
	typedef NSConsPoint<Scalar>    NSPoint;
	typedef impl::util<Scalar> util;
	typedef typename util::Vec2X Vec2X;
	typedef typename util::Mat2X Mat2X;
	typedef typename util::VecX VecX;
	typedef typename NSConstraint<Scalar>::ConsInfo ConsInfo;

	struct NSMultiLineHint{
		NSMultiLineHint(int idx = -1, bool is_line = true)
			: index(idx), isLine(is_line)
		{}
		int  index;
		bool isLine;
	};
public:
	// The points, when connecting them sequentially, forms a convex polygon on each edge's right
	// Furthermore, it's required that pN = p0
	// i.e. for points = [p0, p1, p2, .., pN], the right hand side of p1-p0, p2-p1, ..., pN - p(N-1) is the inner of the convex polygon
	explicit NSConsConvexPoly(const Mat2X& points, const Config& conf)
		: Base(conf), node(points)
	{
		npts = (int)node.cols() - 1;
		if (!util::is_closed(points)) { throw std::invalid_argument("[NSConsConvexPoly] pN should equal p0"); }
		if (!util::is_convex(points)) {
			std::cout << points << std::endl;
			throw std::invalid_argument("[NSConsConvexPoly] Not a convex polygon");
		}
		heading = util::heading_angles(node);   // compute heading of each
	}

	ConsInfo part_compute(const VecX& x, const VecX& vob) const {
		ConsInfo ret;
		Config conf = Base::config();
#ifdef _WIN32_xxx
		if (false) {
			for (int i = 0; i < npts; ++i) {
				Scalar th = util::round_angle(heading(i));
				th *= 180 / 3.141592654;
				printf("Heading %d: %.2f\n", i, th);
			}
			for (int i = 0; i < npts; ++i) {
				Scalar th0 = util::round_angle(heading(i + 1) + util::Pi / 2);  th0 *= 180 / 3.141592654;
				Scalar th1 = util::round_angle(heading(i) + util::Pi / 2);		 th1 *= 180 / 3.141592654;
				printf("Corner %d ranges: %.2f -> %.2f\n", i, th0, th1);
			}
		}
#endif
		static const int log_name = 2;
		std::vector<ConsInfo> lineRet(npts);
		for (int i = 0; i < npts; ++i) {
			lineRet[i] = NSLine(node.col(i), node.col(i + 1), conf, NSLine::LeftSide).part_compute(x, vob);
			if (lineRet[i].valid && !lineRet[i].inside) {
				//printf("At line %d\n", i);
				return lineRet[i];
			}
		}

		// Check if it's at corners
		for (int i = 0; i < npts; ++i) {
			Scalar th0 = heading(i + 1) + util::Pi / 2;
			Scalar th1 = heading(i) + util::Pi / 2;
			NSPoint corner(node.col(i + 1), 0, th0, th1, conf);
			ret = corner.part_compute(x, vob);
			if (ret.valid) {
				//printf("At Corner %d\n", i);
				return ret;
			}
		}

		// Check inside
		int bestID = -1;
		Scalar max = -std::numeric_limits<Scalar>::infinity();
		for (int i = 0; i < npts; ++i) {
			if (lineRet[i].inside) {
				if (max < lineRet[i].value) {
					max = lineRet[i].value;
					bestID = i;
				}
			}
		}
		if (bestID != -1) {
			return lineRet[bestID];
		}
		//puts("WARN: NSAvoid::NSConsConvexPoly, could not find ")
		return ret;

		/*
		for (int i = 0; i < npts; ++i){
		NSLine  line(node.col(i), node.col(i+1), conf, NSLine::LeftSide);
		ret = line.part_compute(x, vob);
		//ret = line.set_dynamic(vob).compute(x);
		if (ret.valid) {
		//if (print_debug) printf("%d-th line valid\n", i);
		//loginfo(log_name, "{}-th line valid", i);
		return ret;
		}

		Scalar th0 = heading(i + 1) + util::Pi / 2;
		Scalar th1 = heading(i) + util::Pi / 2;
		NSPoint corner(node.col(i + 1), 0, th0, th1, conf);
		ret = corner.part_compute(x, vob);
		//ret = corner.set_dynamic(vob).compute(x);
		if (ret.valid) {
		//if (print_debug) printf("%d-th corner valid\n", i);
		//loginfo(log_name, "{}-th corner valid", i);
		return ret;
		}
		}
		//printf("NONE is valid\n");
		return ret;
		*/
	}

private:
	int   npts;		// If it's a square, it's 4.
	Mat2X node;     // Node of the polygon (npts+1 nodes, with the first equal to the last)
	VecX  heading;  // Heading angle of n edges
};



}
