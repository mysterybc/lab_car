#pragma once
#include "NSAvoid.hpp"
#include "NSAvoidUtil.hpp"

namespace NSAvoid {

template<class Scalar>
class NSBoundFunctor {
public:
	typedef NSBound<Scalar> Bound;
	NSBoundFunctor() {
		//one_sided = true;
		K = (Scalar)1;
		v0 = (Scalar)0;
		power = (Scalar)1;
		min_avoid = (Scalar)0;
	}
	Bound operator ()(Scalar distance) const {
		Bound ret;
		ret.vmin = -vel_approach(distance);
		/*
		if (one_sided || distance > 0) {
			// Note: g(x) is the distance away
			// thus, dot(g) >= v   <==>   -dot(g) <= -v   <==>   v_approach <= v
			ret.vmin = -vel_approach(distance);
		}
		else {
			distance = -distance;
			// Note: for a two-sided constraint, and when g(x) < 0
			//       the constraint requries that g(x) remains < 0
			//       such that the speed limit is given by dot(g(x)) <= v_approach
			ret.vmax = vel_approach(distance);
		}
		*/
		return ret;
	}

	//bool   one_sided;
	Scalar K;
	Scalar v0;
	Scalar power;
	Scalar min_avoid;
private:
	Scalar vel_approach(Scalar d) const {
		d -= min_avoid;
		Scalar sgn = (d >= 0) ? (Scalar)1 : (Scalar)(-1);
		return sgn * (K * std::pow(std::abs(d), power) + v0);
	}
};

// Since this code should be able to run on GCC 4.8
// and we don't have a good lambda there
// Therefore the following warppers will be used

template<class Scalar>
NSBound<Scalar> linear_thresh(Scalar distance, void* limit_config) {
	const NSBoundFunctor<Scalar>& thresh = *((NSBoundFunctor<Scalar>*)limit_config);
	return thresh(distance);
}

template<class Scalar>
NSBound<Scalar> linear_thresh_rev(Scalar distance, void* limit_config) {
	NSBound<Scalar> r1 = linear_thresh(distance, limit_config);
	NSBound<Scalar> r2;
	if (r1.max_valid()) {
		r2.vmin = r1.vmax;
	}
	if (r1.min_valid()) {
		r2.vmax = r1.vmin;
	}
	return r2;
}

}
