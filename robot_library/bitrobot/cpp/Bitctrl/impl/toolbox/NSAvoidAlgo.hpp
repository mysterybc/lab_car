#pragma once
#include <map>
#include <vector>
#include "NSAvoid.hpp"
#include "NSAvoidUtil.hpp"
#include "NSAvoidBound.hpp"
#include "NSAvoidItem.hpp"
#include "LoggingUtil.hpp"

namespace NSAvoid {

template<class Scalar>
class AlgoNSAvoid;

template<class Scalar>
class ObstacleMan {
public:
	typedef NSConsDelegate<Scalar>     ConsDelegate;
	typedef std::vector<ConsDelegate*> ConsVec;
	typedef NSBoundFunctor<Scalar> BoundFunc;
	typedef NSConsConfig<Scalar> ConsConfig;
	typedef typename NSConstraint<Scalar>::ConsInfo ConsInfo;
	typedef impl::util<Scalar> util;
	typedef typename util::Vec2X Vec2X;
	typedef typename util::Mat2X Mat2X;
	typedef typename util::VecX VecX;
	typedef typename ConsConfig::BoundFunction BoundFunction;

	ObstacleMan(BoundFunction f_bound= linear_thresh<Scalar>)
		:_config(f_bound, &_bound)
	{}

	~ObstacleMan() {
		delete_all();
	}

	// Configurations for all constraints here
	ConsConfig& config() { return _config; }
	BoundFunc&  bound()  { return _bound;  }

	ObstacleMan& bound_function(BoundFunction f_bound) {
		config().set_limit_func(f_bound, &_bound);
		return *this;
	}
	ObstacleMan& range_valid(Scalar ymax, Scalar ymin = 0) {
		config().value_max = ymax;
		config().value_min = ymin;
		return *this;
	}
	ObstacleMan& relax(Scalar penalty = (Scalar)-1) {
		config().penalty = penalty;
		return *this;
	}
	ObstacleMan& min_avoid(Scalar min_avoid = 0) {
		bound().min_avoid = min_avoid;
		return *this;
	}
	/*
	ObstacleMan& one_sided(bool on = true) {
		bound().one_sided = on;
		return *this;
	}
	*/
	ObstacleMan& vlimit(Scalar K, Scalar v0 = 0) {
		bound().K  = K;
		bound().v0 = v0;
		return *this;
	}
	ObstacleMan& set_velocity(int k, const Vec2X& vob) {
		(*this)[k].set_dynamic(util::toX(vob));
		return *this;
	}

	// Vector like operations
	size_t size() const  { return cons.size(); }
	ConsDelegate& operator [] (int k) {
		int n = (int)size();
		if (k < 0) k += n;
		if (k < 0 || k >= n) throw std::out_of_range("ObstacleMan, [] invalid");
		return *(cons[k]);
	}
	ConsDelegate& back() {
		if (cons.empty()) throw std::out_of_range("ObstacleMan, back invalid");
		return *(cons.back());
	}
	void delete_all(){
		typename ConsVec::iterator iter = cons.begin();
		for (; iter != cons.end(); ++iter) {
			if (*iter != nullptr) delete *iter;
		}
		cons.clear();
	}

	void set_static() {
		typename ConsVec::iterator iter = cons.begin();
		for (; iter != cons.end(); ++iter) {
			iter->set_static();
		}
	}
    ObstacleMan& add_line(const Vec2X& q0, const Vec2X& q1, int side = NSConsLine<Scalar>::BothSides) {
        NSConsLine<Scalar>* one = new NSConsLine<Scalar>(q0, q1, config(), side);
		cons.push_back(one);
		return *this;
	}
	ObstacleMan& add_point(const Vec2X& center, Scalar radius) {
		NSConsPoint<Scalar>* one = new NSConsPoint<Scalar>(center, radius, config());
		cons.push_back(one);
		return *this;
	}
	ObstacleMan& add_point(const Vec2X& center, Scalar radius, Scalar th_min, Scalar th_max) {
		NSConsPoint<Scalar>* one = new NSConsPoint<Scalar>(center, radius, th_min, th_max, config());
		cons.push_back(one);
		return *this;
	}
	ObstacleMan& add_poly(const Mat2X& points) {
		NSConsConvexPoly<Scalar>* one = new NSConsConvexPoly<Scalar>(points, config());
		cons.push_back(one);
		return *this;
	}

	size_t add_to(std::vector<NSConstraint<Scalar>*>& out) {
		typename ConsVec::iterator iter = cons.begin();
		for (; iter != cons.end(); ++iter) {
			ConsDelegate* one = *iter;
			out.push_back(one);
		}
		return size();
	}

	ConsInfo min_dis(Vec2X q) {
		ConsInfo best;
		typename ConsVec::iterator iter = cons.begin();
		for (; iter != cons.end(); ++iter) {
			ConsDelegate* one = *iter;
			ConsInfo temp = one->compute(q);
			if (!best.valid || temp.value < best.value) {
				best = temp;
			}
		}
		return best;
	}
	std::vector<ConsInfo> get_valid(Vec2X q) {
		std::vector<ConsInfo> r;
		typename ConsVec::iterator iter = cons.begin();
		for (; iter != cons.end(); ++iter) {
			ConsDelegate* one = *iter;
			ConsInfo temp = one->compute(q);
			if (temp.valid) {
				r.push_back(temp);
			}
		}
		return r;
	}

private:
	ObstacleMan(const ObstacleMan& other) {}
	ObstacleMan& operator = (const ObstacleMan& other) { return *this; }

	ConsConfig _config;
	BoundFunc  _bound;
	ConsVec    cons;    // I own these things
						// but I don't have smart pointers here
};

/*
 * An AlgoXXX component manages every part of the algorithm
 * and provide high level management/control of the algorithm
 */
enum {	
	OBGroup_Static   = 0,
	OBGroup_DynAgent = 1,
	OBGroup_DynEnv   = 2,
	OBGroup_Target   = 3,
	OBGroup_Sensor   = 4,
	OBGroup_Num
};

template<class Scalar>
class AlgoNSAvoid{
public:
	typedef impl::util<Scalar> util;
	typedef typename util::Vec2X Vec2X;
	typedef typename util::Mat2X Mat2X;
	typedef typename util::VecX VecX;
	typedef NSConstraint<Scalar>  Costraint;
	typedef NSProblem<Scalar>     Problem;
	typedef NSResult<Scalar>      Result;
	typedef NSSolver<Scalar>      Solver;
	typedef ObstacleMan<Scalar>   ObManager;
	typedef typename NSConstraint<Scalar>::ConsInfo ConsInfo;

	struct UnicyclePoint {
		UnicyclePoint(Scalar len = 1, Scalar th = 0) :len(len), theta(th) {}
		Scalar len, theta;	// theta: in radians, 0 is the current heading
							// len in meters
	};

	AlgoNSAvoid()
		: prob(2, 2)//, ob_list(4)
	{
		ob_target().bound_function(linear_thresh_rev<Scalar>).relax(1);
	}

	// Set a the nonlinear system model
	void set_unicycle_point(Scalar len, Scalar theta = 0) {
		pt.len = len;
		pt.theta = theta;
	}

	void set_metric(const Scalar& aV, const Scalar& aW) {
		prob.w.resize(2);
		prob.w << aV, aW;
	}

	// Clear limit on u
	void set_ulimit() {
		prob.MIu.resize(2, 0);
		prob.miu.resize(0);
	}
	void set_ulimit(const Scalar& vmax, const Scalar& wmax) {
		prob.MIu.setZero(2, 4);
		prob.miu.resize(4);
		prob.MIu.leftCols(2).diagonal().setConstant(1);
		prob.MIu.rightCols(2).diagonal().setConstant(-1);
		prob.miu << vmax, wmax, vmax, wmax;
	}

	void update_state(const Vec2X& q, const Scalar& th) {
		prob.J.resize(2, 2);
		prob.d.resize(0);

		Scalar th1 = th + pt.theta;
		Scalar L = pt.len;
		Scalar x = (Scalar)q(0) + L*std::cos(th1);
		Scalar y = (Scalar)q(1) + L*std::sin(th1);
		prob.x << x, y;
		prob.J << std::cos(th), -L*std::sin(th1),
				  std::sin(th),  L*std::cos(th1);
	}
    void update_state_as_point(const Vec2X& q) {
        prob.J.resize(2, 2);
        prob.d.resize(0);
        prob.x << (Scalar)q(0), (Scalar)q(1);
        prob.J << 1, 0,
                  0, 1;
    }

	void ob_static_changed() {
		prob.g.clear();
		ob_static().add_to(prob.g);
	}
	Result& compute(const Vec2X& u_ref) {
		refresh_constraints();
		prob.a = u_ref;
		result = solve(prob);
		if (!result.good()) {
			printf("NSAVOID FAILED[%d] u (%.4f, %.4f), ulim (%.4f, %.4f), nCI,nMI,nME=%d,%d,%d, nRelax=%d\n",
				   result.ret_qp.state, u_ref(0), u_ref(1), prob.miu(0), prob.miu(1), result.nCI, result.nMI, result.nME, result.dim_relax);
		}
		return result;
	}
	ConsInfo min_dis(Vec2X q) {
		ConsInfo best = ob_of(0).min_dis(q);
		for (int i = 1; i < OBGroup_Num; ++i) {
			ConsInfo temp = ob_of(i).min_dis(q);
			if (temp.valid && (!best.valid || best.value < temp.value)) {
				best = temp;
			}
		}
		return best;
	}
	std::vector<ConsInfo> get_valid(Vec2X q) {
		std::vector<ConsInfo> r;
		for (int i = 0; i < OBGroup_Num; ++i) {
			std::vector<ConsInfo> temp = ob_of(i).get_valid(q);
			r.insert(r.end(), temp.begin(), temp.end());
		}
		return r;
	}

	ObManager& ob_static() {
		return ob_list[OBGroup_Static];
	}
	ObManager& ob_agent() {
		return ob_list[OBGroup_DynAgent];
	}
	ObManager& ob_env() {
		return ob_list[OBGroup_DynEnv];
	}
	ObManager& ob_target() {
		return ob_list[OBGroup_Target];
	}
	ObManager& ob_sensor() {
		return ob_list[OBGroup_Sensor];
	}
	ObManager& ob_of(int ob_class) {
		return ob_list[ob_class];
	}
	const ObManager& cob_of(int ob_class) const {
		//return ob_list.at(ob_class);
		return ob_list[ob_class];
	}
	static std::string ob_name(int ob_class) {
		switch (ob_class) {
		case OBGroup_Static:   return "obStatic";
		case OBGroup_DynAgent: return "obAgent";
		case OBGroup_DynEnv:   return "obEnvir";
		case OBGroup_Target:   return "obTarget";
		case OBGroup_Sensor:   return "obSensor";
		default: return "obUnknown";
		}
	}

private:
	void refresh_constraints() {
		size_t n_static = ob_static().size();
		if (n_static < prob.g.size()) {
			// Well the static obstacle info must be invalid
			// FIX ME: ISSUE A WARNING HERE
			ob_static_changed();
		}
		prob.g.resize(n_static);	// keep the first n_static obstacles
		for (size_t i = 0; i < OBGroup_Num; ++i) {
			if (i != OBGroup_Static) {
				ob_list[i].add_to(prob.g);
			}
		}
	}
public:
	UnicyclePoint pt;
	Problem prob;
	Solver  solve;
	Result  result;
	//std::vector<ObManager> ob_list;
	ObManager ob_list[OBGroup_Num];
};

/*
template<class Scalar>
void explain(const AlgoNSAvoid<Scalar>& prob, const NSResult<Scalar>& res, int brief = INFO_MSG, int detail = INFO_DEBUG) {
	typedef impl::util<Scalar> util;
	typedef typename util::Vec2X Vec2X;
	typedef typename util::Mat2X Mat2X;
	typedef typename util::VecX VecX;
	typedef AlgoNSAvoid<Scalar>   AlgoAvoid;
	typedef NSConstraint<Scalar>  Costraint;
	typedef NSProblem<Scalar>     Problem;
	typedef NSResult<Scalar>      Result;
	typedef typename Result::Info ConsInfo;
	typedef typename Result::QPResult     QPResult;
	typedef typename AlgoAvoid::ObManager ObManager;

	const std::vector<ConsInfo>& info = res.info;
	const QPResult& qp = res.ret_qp;

	int index_level[OBGroup_Num];
	index_level[0] = prob.cob_of(0).size();
	for (int i = 1; i < OBGroup_Num; ++i) {
		index_level[i] = index_level[i - 1] + prob.cob_of(i).size();
	}

	int level;
	level = brief;
	loginfo(level, "--- Brief Status ---");
	loginfo(level, "state    : {}", (res.good()? "Good" : "Error"));
	loginfo(level, "best_u   : ({:.2f}, {:.2f})", res.best_x()(0), res.best_x()(1));
	if (res.dim_relax > 0) {
		std::string delta = fmt::format("{:.2f}", qp.x(res.dim_u));
		for (int i = 1; i < res.dim_relax; ++i){
			delta += fmt::format(", {:.2f}", qp.x(res.dim_u + i));
		}
		loginfo(level, "delta    : ({})", delta);
	}
	loginfo(level, "cost     : {:.2f}", res.cost());
	loginfo(level, "n_active : {}", res.n_active());

	level = detail;
	loginfo(level, "--- Detail Status ---");
	if (!res.good()) {
		loginfo(level, "error state is {}", qp.StateInfo());
	}
	else{
		int nNEQ = res.nCI + res.nMI;
		loginfo(level, "enabled : nEQ {}, nNEQ {}({}+{})", res.nME, res.nMI + res.nCI, res.nCI, res.nMI);
		std::string active;
		for (int i = 0; i < qp.n_active; ++i) {
			active += fmt::format("{}({:.2f}) ", res.ret_qp.active_set(i), res.ret_qp.lagrange(i));
		}
		loginfo(level, "acitve index: {}", active);

		for (size_t i = 0; i < info.size(); ++i) {
			const ConsInfo& one = info.at(i);
			std::string sFrom, sMinMaxIndex, sRelax;
			for (int j = 0; j < OBGroup_Num; ++j) {
				if (one.index_from < index_level[j]) {
					sFrom = AlgoAvoid::ob_name(j);
					break;
				}
			}
			if (one.min_index >= 0)
				sMinMaxIndex += fmt::format("min({})", one.min_index);
			if (one.max_index >= 0)
				sMinMaxIndex += fmt::format("max({})", one.max_index);

			if (one.penalty_index >= 0) {
				int nk = one.penalty_index + res.dim_u;
				sRelax = fmt::format(", delta is {:.2f} idx {}({}-th), penalty {:.2f}, ", qp.x(nk), nk, one.penalty_index, one.penalty);
			}
			loginfo(level, "#{}, {}({}), enabled {}{}", i, sFrom, one.index_from, sMinMaxIndex, sRelax);

			if (one.min_index >= 0) {
				for (int k = 0; k < qp.n_active; ++k) {
					if (qp.active_set(k) == one.min_index) {
						// This one is active
						loginfo(level, "#{},    ACTIVE({}) <= {:.2f}, lam = {:.2f}", i, one.min_index, one.vlim.vmin, qp.lagrange(k));
						break;
					}
				}
			}
			if (one.max_index >= 0) {
				for (int k = 0; k < qp.n_active; ++k) {
					if (qp.active_set(k) == one.max_index) {
						// This one is active
						loginfo(level, "#{},    ACTIVE({}) >= {:.2f}, lam = {:.2f}", i, one.max_index, one.vlim.vmax, qp.lagrange(k));
						break;
					}
				}
			}
		}

		//for (size_t i = 0; i < )
	}
}
*/

}
