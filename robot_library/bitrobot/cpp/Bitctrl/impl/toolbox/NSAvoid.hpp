#pragma once

#include "EigenQP.hpp"
#include "NSAvoidUtil.hpp"
//#include "LoggingUtil.hpp"
#include <vector>

namespace NSAvoid {

enum {
	NSBound_IN = 0,
	NSBound_GE = 1,
	NSBound_LE = 2,
	NSBound_EQ = 3
};

// Since we don't have C++11, we have to do something ugly...
template<class Scalar>
struct NSBound {
	static Scalar inf() { return std::numeric_limits<Scalar>::infinity(); }

	NSBound() : vmin(-inf()), vmax(inf()) {}
	NSBound(Scalar _min, Scalar _max) : vmin(_min), vmax(_max) {}
	bool min_valid() const { return vmin > -inf(); }
	bool max_valid() const { return vmax < inf(); }

	Scalar vmin, vmax;
};


/*
* An NSProblem is to solve the following problem.
* - Let x be the state of the system and u be its input
* - Let g_k be the k-th constraint function asking g_k(x) <= 0
* - Let v_k be the approaching velocity limitation function for g_k
* - Let there realtionships be
* ---  dot(x) = J u + d    (J, d can be obtained by linearizing a dynamic system)
* ---     y_k = g_k(x),    k = 1..N
* ---     v_k = v_k(y_k),  k = 1..N
* - (It's required that y_k \in R, not R^m)
* - Let u0 be a reference value of u, and Q be a positive definite weighting matrix
* - The optimization problem is defined as
* ---   min     0.5*|u - a|^2_Q
* ---   s.t. d/dt y_k >= v_k,  k = 1..N
* ---             ME*u = me
* ---             MI*u <= mi
* - In particular, the constraints are for obstacles avoidance,
* - that is, g_k computes the distance between the current state x and obstacle k
*/

template<class Scalar>
class NSConstraint{
public:
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatX;
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecX;
	typedef NSBound<Scalar> Bound;

	struct ConsInfo {
		ConsInfo(bool valid = false) : valid(valid), inside(false), grad_dt(0), penalty(0) {}
		bool relaxed() const { return penalty > 0; }

		bool   valid;	// If it's valid
		bool   inside;	// If it's inside the forbidden region
		Scalar value;   // g(x)
		VecX   grad;    // Dg(x)
		Scalar grad_dt; // Dg(t), thus Dg/dt = Dg(x)\dot{x} + Dg(t)

		// return the velocity limit, with y = g(x)
		// we have vmin(y) <= Dg(x) dx/dt + err <= vmax(y)
		Bound  vlim;

		// return the penalty for err
		// i.e the cost is 1/2*penalty()*|err|^2
		//virtual Scalar penalty() const = 0;
		Scalar penalty;
	};

	// virtual deconstructor
	virtual ~NSConstraint() {}
	virtual ConsInfo compute(const VecX& x) const = 0;
};

template<class Scalar>
class NSProblem{
public:
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatX;
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecX;
	typedef NSConstraint<Scalar> Constraint;
	typedef typename Constraint::ConsInfo ConsInfo;

	NSProblem(int dimX = 0, int dimU = 0) {
		reset(dimX, dimU);
	}
	void reset(int dimX, int dimU) {
		x.resize(dimX); x.setZero();
		a.resize(dimU); a.setZero();
		w.resize(dimU); w.setOnes();
		J.resize(dimX, dimU);   J.setZero();
		d.resize(0);
		//d.resize(dimX); d.setZero();
		g.clear();
		MEu.resize(dimU, 0);        meu.resize(0);
		MIu.resize(dimU, 0);        miu.resize(0);
		MEx.resize(dimX, 0);        mex.resize(0);
		MIx.resize(dimX, 0);        mix.resize(0);
	}

	bool check() const {
		size_t dimX = x.size();
		size_t dimU = a.size();
		std::vector<bool> check_list = {
			(w.size() == dimX),
			(a.size() == dimU),
			(J.cols() == dimU && J.rows() == dimX),
			//(d.size() == dimX),
			(MEu.rows() == dimU && MEu.cols() == meu),
			(MIu.rows() == dimU && MIu.cols() == miu),
			(MEx.rows() == dimX && MEx.cols() == mex),
			(MIx.rows() == dimX && MIx.cols() == mix)
		};
		std::vector<bool>::iterator it = check_list.begin();
		for (; it != check_list.end(); ++it) {
			if (*it != true) return false;
		}
		return true;
	}

	VecX x;     // Current state of the system
	VecX a;     // Reference control signal at this time
	VecX w;     // Metric space weighting of u

	// Dynamic of the system
	// --  dot(x) = Ju + d
	MatX J;
	VecX d;     // can be of size 0

	// Dynamic Constraints for x(i.e. dot(x))
	// --  g(x) >= 0
	std::vector<Constraint*> g;     // can be of size 0

	// Constant constraints for x(i.e. dot(x)), u
	// These are always valid and enabled, and they cannot be relaxed
	// They're of the form:
	//     MIx^T *(Ju+d) <= mix
	//          Miu^T *u <= miu
	//           MEx^T *x = mex
	//           MEu^T *u = meu
	MatX MEu, MIu, MEx, MIx;    // can be of size 0
	VecX meu, miu, mex, mix;    // can be of size 0
};

template<class Scalar>
struct NSResult{
	typedef typename EigenQuad::QPResult<Scalar> QPResult;
	typedef typename impl::util<Scalar>::VecX VecX;

	struct Info
		: public NSConstraint<Scalar>::ConsInfo
	{
		Info() : penalty_index(-1), min_index(-1), max_index(-1), index_from(-1) {}
		// This constraint is enabled
		// if it.min_valid(), then the min_constraint is added as the min_index-th col in qp.CI
		// if it.max_valid(), then the max_constraint is added as the max_index-th col in qp.CI
		// if it's relaxed, then the relax variable is the penalty_index-th var in qp.x
		int penalty_index;
		int min_index;  //
		int max_index;  //
		int index_from;
	};

	bool good() const { return ret_qp.state == EigenQuad::QPErr_Good; }
	const VecX& best_x() const { return ret_qp.x; }
	Scalar cost() const { return ret_qp.min_value; }
	int n_active() const { return ret_qp.n_active; }
	int info_index(int index_qp) const {
		return index_qp - nME;
	}

	QPResult ret_qp;
	std::vector<Info> info;
	int dim_u, dim_relax;
	int nCI, nMI, nME;
};

template<class Scalar>
class NSSolver{
public:
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatX;
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecX;
	typedef typename Eigen::VectorXi VecXi;
	typedef typename EigenQuad::QPProblem<Scalar> QPProblem;
	typedef typename EigenQuad::QPSolver<Scalar>  QPSolver;
	typedef typename EigenQuad::QPResult<Scalar>  QPResult;
	typedef NSResult<Scalar>  Result;
	typedef NSProblem<Scalar> Problem;
	typedef NSConstraint<Scalar> Constraint;
	typedef NSBound<Scalar> Bound;

	Result operator ()(const Problem& p) const {
		Result ret;
		QPProblem qp;
		build_qp(p, qp, ret);
		QPSolver solve;
		ret.ret_qp = solve(qp);
		return ret;
	}

protected:
	void build_qp(const Problem& p, QPProblem& qp, Result& ret) const {
		typedef typename Result::Info RetInfo;
		typedef typename Constraint::ConsInfo ConsInfo;
		const VecX& x = p.x;

		// Extract info from valid/enabled obstacles
		std::vector<RetInfo>& info = ret.info;
		size_t nc = 0;
		size_t nrelax = 0;
		for (size_t k = 0; k < p.g.size(); ++k) {
			RetInfo temp;
			ConsInfo& temp_temp = temp;
			temp_temp = p.g[k]->compute(x);
			if (temp.valid) {
				temp.index_from = k;
				if (temp.vlim.min_valid()) temp.min_index = nc++;
				if (temp.vlim.max_valid()) temp.max_index = nc++;
				if (temp.penalty > 0) temp.penalty_index = nrelax++;
				info.push_back(temp);
			}
		}
		VecX relax_penalty(nrelax);
		relax_penalty.setOnes();
		for (size_t k = 0; k < info.size(); ++k) {
			RetInfo& one = info[k];
			if (one.penalty_index >= 0) {
				relax_penalty(one.penalty_index) = one.penalty;
			}
		}

		// Build CI, ci0 for dynamic constraints
		// Add inequality constraints about dot(y)
		size_t dimU = p.a.size();
		size_t dimU_ex = dimU + nrelax;
		size_t num_mix = p.MIx.cols();
		size_t num_miu = p.MIu.cols();
		size_t num_mic = num_mix + num_miu;
		qp.CI.resize(dimU_ex, nc + num_mic);
		qp.ci0.resize(nc + num_mic);
		qp.CI.setZero();
		qp.ci0.setZero();

		size_t dimX = p.x.size();
		const MatX& J = p.J;
		const VecX zero_d = VecX::Zero(dimX);
		const VecX& d = (p.d.size() > 0) ? p.d : zero_d;

		for (size_t i = 0; i < info.size(); ++i) {
			// From vmin <= Dg(Ju+d) + Dg_t + w <= vmax
			// to Dg(Ju) + w + Dg_dt - vmin >= 0
			//   -Dg(Ju) - w - Dg_dt + vmax >= 0  (i.e. CI x + ci0 >= 0)
			RetInfo& one = info[i];
			Scalar Dg_dt = one.grad.dot(d) + one.grad_dt;
			int kc, kpen = one.penalty_index;
			if (one.min_index >= 0) {
				kc = one.min_index;
				qp.CI.col(kc).head(dimU) = J.transpose() * one.grad;
				qp.ci0(kc) = Dg_dt - one.vlim.vmin;
				if (kpen >= 0)
					qp.CI.col(kc)(dimU + kpen) = 1;
			}
			if (one.max_index >= 0) {
				kc = one.max_index;
				qp.CI.col(kc).head(dimU) = -J.transpose() * one.grad;
				qp.ci0(kc) = -Dg_dt + one.vlim.vmax;
				if (kpen >= 0)
					qp.CI.col(kc)(dimU + kpen) = 1;
				++kc;
			}
		}

		// Add constant inequality constraints
		qp.CI.rightCols(num_mic).leftCols(num_mix).topRows(dimU) = -J.transpose()*p.MIx;
		qp.CI.rightCols(num_mic).rightCols(num_miu).topRows(dimU) = -p.MIu;
		qp.ci0.tail(num_mic).head(num_mix) = p.mix - p.MIx.transpose()*d;
		qp.ci0.tail(num_mic).tail(num_miu) = p.miu;

		// Add constant equality constraints
		size_t num_mex = p.MEx.cols();
		size_t num_meu = p.MEu.cols();
		size_t num_mec = num_mex + num_meu;
		qp.CE.resize(dimU_ex, num_mec);
		qp.ce0.resize(num_mec);
		qp.CE.leftCols(num_mex).topRows(dimU) = J.transpose()*p.MEx;
		qp.CE.rightCols(num_meu).topRows(dimU) = p.MEu;
		qp.ce0.head(num_mex) = p.MEx.transpose()*d - p.mex;
		qp.ce0.tail(num_meu) = -p.meu;

		// Set up cost metric and reference point
		qp.G.resize(dimU_ex, dimU_ex);
		qp.G.setZero();
		qp.G.topLeftCorner(dimU, dimU).diagonal() = p.w;
		qp.G.bottomRightCorner(nrelax, nrelax).diagonal() = relax_penalty;

		qp.g0.resize(dimU_ex);
		qp.g0.setZero();
		qp.g0.head(dimU) = -qp.G.topLeftCorner(dimU, dimU) * p.a;

		ret.dim_u = (int)dimU;
		ret.dim_relax = (int)nrelax;
		ret.nCI = (int)nc;
		ret.nME = (int)num_mec;
		ret.nMI = (int)num_mic;
	}
};




}
