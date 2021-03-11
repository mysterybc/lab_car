#pragma once
#include "BIT.h"
#include "helper.hpp"
#include "Geotools.hpp"
#include <algorithm>
#include <functional>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include "fmt/format.h"

#ifdef __GNUC__
#if __GNUC__ < 4
    #ifndef nullptr
    #define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
    #endif
#elif __GNUC__ == 4 && __GNUC_MINOR__ < 6
    #ifndef nullptr
    #define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
    #endif
#endif
#endif

namespace impl{
	class AlgoTracking{
	public:
		struct Info {
			Info() {
				valid = true;
			}
			bool   valid;
			pt2D   tar;
			pt2D   tarV;	// Feedforward
			double err;
			bool   delayed;
		};
		AlgoTracking(){
			pline        = nullptr;
			target_point = nullptr;
			target_uint = 100.0f;
			target_speed = 1.0f;
			szDZoneENormal = 0.0f;
			KTagent = 1.0f;
			KNorml  = 1.0f;
			lineShift = pt2D();
			pathFollow = false;
			pathFollowEx = false;
			t_tracking = 0.0f;
			t_lasttime = 0.0f;
			errThresh = 100.0f;
		}
		void reset(real_t t0);
		real_t compute_target(real_t t, pt2D me, real_t t_sacling);
		pt2D operator()(real_t t, pt2D me, real_t t_scaling = 1.0f);
		Info compute_info() const { return last_info; }

		// Parameters
		spline2D* pline;		// = nullptr;
		pt2D*  target_point;	// = nullptr;

		real_t target_uint;// = 100.0f;
		real_t target_speed;// = 1.0f;
		real_t szDZoneENormal;// = 0.0f;
		real_t KTagent;// = 1.0f;
		real_t KNorml;//  = 1.0f;
		pt2D   lineShift;// = pt2D{ 0, 0 };
		bool   pathFollow;
		bool   pathFollowEx;
		real_t t_tracking;
		real_t t_lasttime;
		real_t errThresh;
	private:
		Info last_info;
	};

	class AlgoConsensus{
	public:
		AlgoConsensus(){
			K = 1.0f;
			dq = pt2D();//{ 0, 0 };
		}
		pt2D operator ()(const pt2D& me, const pt2D& center) const{
			pt2D err = center + dq - me;
			return K * err;
		}

		real_t K; // = 1.0f;
		pt2D   dq; // = pt2D{ 0, 0 };
	};

	class AlgoHeadingAlign{
	public:
		AlgoHeadingAlign(){
			K = 1.0f;
			wmax = 120;
		}
		real_t operator()(real_t thMe, real_t thTarget) const{
			real_t w = K * anglediff(thTarget, thMe);
			return wmax > 0 ? saturate(w, deg2rad(wmax)) : w;
		}
		real_t operator()(real_t thMe, const pt2D& vec) const{
			return this->operator()(thMe, (real_t)std::atan2(vec.y, vec.x));
		}

		real_t K;// = 1.0f;
		real_t wmax;// = 120;	// in degree
	};


	struct HumanField{
		HumanField() :x(0), y(0), ratio(0){}
		HumanField(const real_t& x, const real_t& y, const real_t& r) :x(x), y(y), ratio(r){}
		pt2D pt() const { return pt2D(x, y); }
		real_t x, y, ratio;
	};
	inline HumanField operator + (const HumanField& a, const HumanField& b){
		return HumanField( a.x + b.x, a.y + b.y, a.ratio + b.ratio );
	}
	inline HumanField operator - (const HumanField& a, const HumanField& b){
		return HumanField( a.x - b.x, a.y - b.y, a.ratio - b.ratio );
	}
	inline HumanField operator * (real_t k, const HumanField& a){
		return HumanField( k*a.x, k*a.y, k*a.ratio);
	}
	inline HumanField operator / (const HumanField& a, real_t k){
		return HumanField( a.x / k, a.y / k, a.ratio / k );
	}
	struct HumanInput{
		real_t x, y, t;
	};
	struct DirectTeleData {
		uint loopCounter;
		real_t v, w;
	};
	struct DirectTeleVxyData {
		uint loopCounter;
		real_t vx, vy;
	};

	class AlgoHumanField{
	public:
		typedef wValid<HumanInput> wHumanInput;
		AlgoHumanField(){
			pHfNet = nullptr;
			Kc = 1.0f;
			Kt = 1.0f;
			decRatioPS = 0.8f;	// per second
			decScalePS = 0.8f;	// per second
			tHumanMax  = 2.0f;	// seconds
			normalize  = false;   // whether normalize the input before blending
			vmax = 100;
		}

		void reset(){
			if (pHfNet) pHfNet->clear();
			hfMe = HumanField();
		}

		// Return the blended inputs
		pt2D operator() (pt2D uLocal)const{
			double rHuman = hfMe.ratio;
			pt2D   uHuman = hfMe.pt();

			if (normalize){
				uHuman = impl::normalize(uHuman) * vmax;
			}

			return uHuman * rHuman + (1 - rHuman) * uLocal;
		}
		HumanField update(real_t dt, wHumanInput& hiLast);

		// These are states
		std::map<int, HumanField>* pHfNet;// = nullptr;
		HumanField hfMe;

		// Parameters
		real_t Kc;// = 1.0f;
		real_t Kt;// = 1.0f;
		real_t decRatioPS;// = 0.8f;	// per second
		real_t decScalePS;// = 0.8f;	// per second
		real_t tHumanMax;//  = 2.0f;	// seconds
		bool   normalize;//  = false;   // whether normalize the input before blending
		real_t vmax;	    // = 100;
	};

	struct DetectResult{
		double acc_err;
		double t0;
		double errbound;
	};

	class AlgoUnicycleMode{
	public:
		StateInfo operator()(real_t dt, const StateInfo& now, const ControlInfo& uw) const{
			StateInfo next = now;
			real_t th = deg2rad(now.heading);
			next.x += (real_t)(uw.v * std::cos(th) * dt);
			next.y += (real_t)(uw.v * std::sin(th) * dt);
			next.heading += (real_t)(uw.w * dt);
			while (next.heading >= 180) next.heading -= 360;
			while (next.heading < -180) next.heading += 360;
			next.v = uw.v;
			next.w = uw.w;
			return next;
		}
	};

	template<class StateType>
	class AlgoFaultDetect{
	public:
		AlgoFaultDetect(){
			bound_type = bound_acc;
			detect_bound = 10;	// on integral
			min_time  = 1;		// in seconds, issuing detection after reach bound + min_time
			err_decay = 0.8f;	// per second
			reset();
		}
		
		bool operator() (real_t dt, const StateType& observe, const StateType& predict){
			double err = DistanceFunction(observe, predict);
			real_t decay = std::pow(err_decay, dt);

			acc_err  = acc_err*decay + err;
			acc_time = acc_time*decay + dt;
			t_global += dt;

			if (bound_reached()){
                detected = true;
                return mintime_reached();
			}
			else{
                t_detect = t_global;
				detected = false;
				return false;
			}
		}

		bool bound_reached() const{
			if (bound_type == bound_acc){
				return acc_err > detect_bound;
			}
			else if (bound_type == bound_average){
				return acc_time > 0 && (acc_err / acc_time > detect_bound);
			}
			return false;
		}
        bool mintime_reached() const{
            return t_global - t_detect >= min_time;
        }

		void reset(){
			acc_err = 0;
			acc_time = 0;
			t_global = 0;
			t_detect = 0;
			detected = false;
		}

		// States
		double acc_err;		// Integration on err, wtih decay
		double acc_time;	// Integration on dt,  with decay
		double t_global;	// Integration on dt, no decay
		double t_detect;	// t_global when first detected
		bool   detected;	// if detected

		// Parameters
		enum{
			bound_acc = 1, bound_average = 2
		};
		int    bound_type;// = bound_acc;
		double detect_bound;// = 10;	// on integral
		real_t min_time;//  = 1;		// in seconds, issuing detection after reach bound + min_time
		real_t err_decay;// = 0.8f;	// per second

		typedef double (*disfun_t)(const StateType&, const StateType&);
		disfun_t DistanceFunction;
	};

	class AlgoPFAvoid{
	public:
		AlgoPFAvoid(){
			unit = 100.0f;
			k = c = kv = 1.0f;
			esp = 0.01f;
			react_range = 2.0f;
		}
		pt2D operator()(const pt2D& my_xy, const pt2D& my_vxy, const pt2D& ob_xy, const pt2D& ob_vxy) const{
			pt2D e_xy = (my_xy - ob_xy) / unit;
			pt2D e_vxy = (my_vxy - ob_vxy) / unit;

			double dis = norm2(e_xy);
			if (dis < react_range || react_range < 0){
				double phi = -1 / (dis*dis + 0.001);
				double h = 1 / sqrt(1 + esp*dis*dis);
				double kk = k + kv*norm2(my_vxy) / unit;
				pt2D acc = - (kk * phi*h*e_xy + c * e_vxy);
				return acc * unit;
			}
			return pt2D();
		}
		real_t unit;
		real_t k, c, esp, kv;
		real_t react_range;
	};


	// First order potential field based 
	// Obstacle avoidance
	class AlgoPFAvoidPower{
	public:
		AlgoPFAvoidPower(){
			unit = 100;
			vmax = 2.0f;
			react_range = 1.0f;
			power = 2.0f;
		}

		pt2D operator()(const pt2D& my_xy, const pt2D& ob_xy, real_t ob_radius) const{
			pt2D e_xy = (my_xy - ob_xy) / unit;
			double dis = std::abs(norm2(e_xy) - ob_radius / unit);

			if (dis < 0.01 * react_range){
				// This is some illness condition.
				e_xy = pt2D(rand() / RAND_MAX - 0.5, rand() / RAND_MAX - 0.5);
				e_xy = normalize(e_xy);
				return vmax * e_xy * unit;
			}
			
			if (dis > react_range) return pt2D();
			return normalize(e_xy) * f((real_t)dis) * unit;
		}

		pt2D operator() (const pt2D& my_xy, const std::vector<pt2D>& ob_xy, const std::vector<real_t>& ob_radius, pt2D acc = pt2D()) {
			size_t n = ob_xy.size() > ob_radius.size() ? ob_radius.size() : ob_xy.size();
			for (size_t i = 0; i < n; ++i){
				acc += this->operator()(my_xy, ob_xy[i], ob_radius[i]);
			}
			return saturate(acc, vmax);
		}

		// r = 0.5, power = k:  f = (1 - 1.0 / 2^k) vmax
		// Default: power = 2,  f(0.5range) = 0.75*vmax
		real_t f(real_t dis) const{
			real_t r = dis / react_range;
			return vmax*(1 - std::pow(r, power));
		}

		// Parameters
		real_t unit;
		real_t vmax;
		real_t react_range;
		real_t power;
	};

	class AlgoPFAttrack{
	public:
		AlgoPFAttrack(){
			k = c = 1.0f;
		}
		pt2D operator()(const pt2D& my_xy, const pt2D& my_vxy, const pt2D& ob_xy, const pt2D& ob_vxy) const{
			pt2D e_xy = (my_xy - ob_xy);
			pt2D e_vxy = (my_vxy - ob_vxy);
			//printf("[FXXK] track: e_xy=(%.2f, %.2f), e_vxy=(%.2f, %.2f)\n", e_xy.x, e_xy.y, e_vxy.x, e_vxy.y);
			return -(k*e_xy + c*e_vxy);
		}
		real_t k, c;
	};
	class AlgoPFAvoidAttrack{
	public:
		AlgoPFAvoidAttrack(){
			unit = 100.0f;
			k = c = kv = 1.0f;
			esp = 0.01f;
			react_range = 2.0f;
			desired_dis = 1.0f;
		}
		pt2D operator()(const pt2D& my_xy, const pt2D& my_vxy, const pt2D& ob_xy, const pt2D& ob_vxy) const{
			pt2D e_xy = (my_xy - ob_xy) / unit;
			pt2D e_vxy = (my_vxy - ob_vxy) / unit;

			double dis = norm2(e_xy);
			if (dis < react_range || react_range < 0){
				double phi = (dis - desired_dis) / (dis*dis + 0.001);
				double h = 1 / sqrt(1 + esp*dis*dis);
				double kk = k + kv*norm2(my_vxy) / unit;
				pt2D acc = -(kk * phi*h*e_xy + c * e_vxy);
				return acc * unit;
			}
			return pt2D();
		}
		real_t unit;
		real_t k, c, esp, kv;
		real_t react_range, desired_dis;
	};

	class AlgoSimpleTrack{
	public:
		AlgoSimpleTrack(){
			K    = 1.0f;
			vmax = 1.0f;
			unit = 100;
		}
		pt2D operator()(const pt2D& me, const pt2D& target)const{
			pt2D vel = K*(target - me);
			//printf("[FXXK] vel=(%.2f, %.2f), vmax = %.2f, unit = %.2f, K=%.2f\n", vel.x, vel.y, vmax, unit, K);
			if (vmax > 0)
				vel = saturate(vel, vmax * unit);
			return vel;
		}
		real_t unit;
		real_t K;
		real_t vmax;
	};

	class AlgoTracer{
	public:
		AlgoTracer(){
			unit = 100;
			acc_max = 1.0f;
			vel_dzone = 0.f;
			Khmn = 1.0f;
			Kneb = 1.0f;
			Kobs = 1.0f;
			human_shift = pt2D();
            print_debug = false;
            print_every  = 10;
            print_counter = 0;
		}

		pt2D operator() (
			double dt,  pt2D human,
			size_t index,
			const std::vector<pt2D>& xy, 
			const std::vector<pt2D>& vxy, 
			const std::vector<pt2D>& ob_xy)
		{
			const pt2D& my_xy = xy[index];
			const pt2D& my_vxy = vxy[index];
			lastMe = my_xy;
			lastMe.setValid(true);
            bool print_now = print_debug && ((print_counter % print_every) == 0);

			// Update Leader
			if (!leader){
				leader = my_xy;
				leader.setValid(true);
			}
			leader_vxy = leader_update(leader, human + human_shift*unit);
			leader += leader_vxy * dt;

			// Track Leader
			pt2D track_led = pt2D(); // { 0, 0 };
			track_led = pf_human(my_xy, my_vxy, leader, leader_vxy);

			// Track and Avoid neighbours
			pt2D track_neb = pt2D();// { 0, 0 };
			for (size_t i = 0; i < xy.size(); ++i){
				if (i != index)
				track_neb += pf_agent(my_xy, my_vxy, xy[i], vxy[i]);
			}
			
			// Avoid Obstacles
			pt2D avoid_obs = pt2D();
			pt2D ob_vel = pt2D();
			for (size_t i = 0; i < ob_xy.size(); ++i){
				avoid_obs += pf_ob(my_xy, my_vxy, ob_xy[i], ob_vel);
			}

			// Compute Final Results with Acc saturation and Vel dzone
			pt2D acc = Khmn*track_led + Kneb*track_neb + Kobs*avoid_obs;
            if (print_now){
                printf("[FXXK] acc=(%.2f, %.2f), Khmn=%.2f, eacc=%.2f, vdzone=%.3f, acc_max=%.2f\n",
                    acc.x, acc.y, Khmn, impl::norm2(acc - Khmn*track_led), vel_dzone, acc_max * unit);

            }
            
			acc = saturate(acc, acc_max * unit);

			pt2D vel = my_vxy + acc*dt;
            if (print_now){
                printf("[FXXK] final: v_old=(%.2f, %.2f), acc=(%.2f, %.2f), v_new=(%.2f, %.2f)\n",
                    my_vxy.x, my_vxy.y, acc.x, acc.y, vel.x, vel.y);
            }
			//vel = dead_zone(vel, vel_dzone*unit);
            print_counter++;
			return vel;
		}

		// Return if I've reached my target point
		//bool me_near() const;

		// Return if everyone reached target points
		//bool all_near() const;

		void reset(){
			leader = pt2D(0, 0);
			leader_vxy = pt2D(0, 0);
			leader.setValid(false);
			leader_vxy.setValid(false);
            print_counter = 0;
		}

		// States
		wValid<pt2D> leader, leader_vxy;

		// Parameter
		real_t unit;// = 100;
		real_t acc_max;// = 1.0f;
		real_t vel_dzone;// = 0.1f;
		pt2D   human_shift;// = pt2D{ 0, 0 };
		
		// Parameters on components
		real_t Kobs, Kneb, Khmn;

		// Components
		AlgoSimpleTrack    leader_update;
		AlgoPFAvoid		   pf_ob;
		AlgoPFAttrack	   pf_human;
		AlgoPFAvoidAttrack pf_agent;

        // Temp Debug
        bool print_debug;
        int print_every;
        int print_counter;
	private:
		// Temp States
		wValid<pt2D> lastMe;
	};

	class AlgoVxyVW {
	public:
		AlgoVxyVW(){
			Ltan = 1;
			Lnor = 0;
		}
		pt2D operator()(const pt2D& vxy, double th) const{
			return pt2D();
		}

		real_t Ltan, Lnor;
	};

	struct LatencyStat {
		LatencyStat() :
			max(-1), min(std::numeric_limits<short>::infinity()), 
			latest(-1), sum(0), num(0)
		{}
		void reset(){
			*this = LatencyStat();
		}
		void update(short val, float decay = 1.0f){
			latest = val;
			num = decay*num + 1;
			sum = decay*sum + val;
			sum_s2 = decay*sum_s2 + val*val;
			if (max < val) max = val;
			if (min > val) min = val;
		}
		float average() const {
			return num > 0 ? float(sum / num) : -1;
		}
		float variance() const {
			float mean = average();
			return num > 0 ? float(sum_s2 / num - mean*mean) : -1;
		}
		short  max, min, latest;
		double sum, sum_s2;
		double num;
	};


	class AlgoLatencyTest {
	public:
		AlgoLatencyTest()
			: tick_max(30000), dump_every(-1), myID(0),
			_master(false), _slave(false)
		{}

		void setID(int ID, const std::vector<int>& IDall) {
			return setID(ID, IDall, IDall);
		}
		void setID(int ID, const std::vector<int>& masterID, const std::vector<int>& testID) {
			if (myID == ID && 
				masterID.size() == id_master.size() && testID.size() == id_all.size() &&
				std::equal(masterID.begin(), masterID.end(), id_master.begin()) &&
				std::equal(testID.begin(), testID.end(), id_all.begin()))
			{
				return;
			}
			myID = ID; 	id_all = testID;  id_master = masterID;
			_master = std::find(id_master.begin(), id_master.end(), myID) != id_master.end();
			_slave  = std::find(id_all.begin(), id_all.end(), myID) != id_all.end();
			myMasterIndex = std::find(id_master.begin(), id_master.end(), myID) - id_master.begin();
			myAgentIndex  = std::find(id_all.begin(), id_all.end(), myID) - id_all.begin();
			reset();
		}
		const std::vector<int>& IDList() const {
			return id_all;
		}
		bool setDumpPath(const std::string& log_path) {
#ifdef ENABLE_LATENCYTEST_LOGGING
			if (is_master()){
				std::string f = log_path + fmt::format("LatencyTestLog-{}.txt", myID);
				if (dump.is_open()) dump.close();
				dump.open(f.c_str(), std::ios_base::app);
				return dump.is_open() && dump;
			}
#endif
			return true;
		}
		void setStatWindow(int win_size = 0) {
			if (win_size == 0) decay = 1.0f;
			else decay = (float)(1 - 1.0 / win_size);
		}

		bool valid() const     { return _master || _slave; }
		bool is_master() const { return _master; }
		bool is_slave() const  { return _slave; }
		size_t master_num() const { return id_master.size(); }
		size_t agent_num() const  { return id_all.size(); }

		bool reset() {
			record.clear();
			if (is_master()){
				curr_tick = 1;
				recv.resize(agent_num());	// number of agents that's reporting to me
				std::fill(recv.begin(), recv.end(), -1);

				stat.resize(agent_num());
				std::fill(stat.begin(), stat.end(), LatencyStat());

				if (dump) {
					dump << "------------Reset----------------\n";
				}
			}
			if (is_slave()) {
				recv_master.resize(master_num());
				std::fill(recv_master.begin(), recv_master.end(), -1);
			}
			return valid();
		}
		short currTick() const {
			return curr_tick;
		}
		size_t onSendMessage(short* dest) const {
			if (dest == nullptr) return valid() ? master_num() : 0;
			if (valid()) {
				for (size_t i = 0; i < master_num(); ++i) {
					dest[i] = (id_master[i] == myID) ? curr_tick :
						(is_slave()) ? recv_master[i] : -1;
				}
			}
			return valid() ? master_num() : 0;
		}
		void onRecvMessage(int srcID, short* msg) {
			// The input msg is of length master_num
			if (valid() && srcID != myID) {
				size_t recvMasterIndex = std::find(id_master.begin(), id_master.end(), srcID) - id_master.begin();
				size_t recvAgentIndex = std::find(id_all.begin(), id_all.end(), srcID) - id_all.begin();
				bool recvIsMaster = recvMasterIndex < id_master.size();
				bool recvIsAgent = recvAgentIndex < id_all.size();

				if (is_slave() && recvIsMaster) {
					// Update master info
					recv_master[recvMasterIndex] = msg[recvMasterIndex];
				}
				if (is_master() && recvIsAgent) {
					// Update agent info
					recv[recvAgentIndex] = msg[myMasterIndex];
				}
			}
		}
		void onNextStep() {
			if (is_master()) {
				recv[myMasterIndex] = curr_tick;
				record.push_back(recv);
				for (size_t i = 0; i < agent_num(); ++i){
					if (i != myAgentIndex) {
						short latency = (recv[i] == -1) ? stat[i].latest + 1 :
							(curr_tick >= recv[i]) ? curr_tick - recv[i] : curr_tick + (tick_max - recv[i]);
						stat[i].update(latency, decay);
					}
				}
#ifdef ENABLE_LATENCYTEST_LOGGING
				if (dump_every > 0 && (curr_tick % dump_every == 0)) {
					onDump();
				}
#endif
				curr_tick++;
				if (curr_tick > tick_max) {
					// The tick range is [1, tick_max]
					curr_tick = 1;
				}
				std::fill(recv.begin(), recv.end(), -1);
			}
		}
		
		std::vector<LatencyStat> stat;
		short tick_max;
		int dump_every;
	private:
		int myID;
		std::vector<int> id_master, id_all;
		bool   _master, _slave;
		size_t myMasterIndex, myAgentIndex;
		float decay;

		short curr_tick;
		std::vector<short> recv, recv_master;
		std::vector<std::vector<short> > record;
		std::ofstream dump;

		void onDump() {
			if (!is_master()) 
				return;
			if (!dump){
				if (dump.rdstate() & std::ios_base::eofbit){
					std::cout << "Dump is EOF" << std::endl;
				}
				if (dump.rdstate() & std::ios_base::failbit){
					std::cout << "Dump is FAIL" << std::endl;
				}
				if (dump.rdstate() & std::ios_base::badbit){
					std::cout << "Dump is BAD" << std::endl;
				}
			}
			//dump << "-----------------------------" << std::endl;
			for (size_t i = 0; i < record.size(); ++i) {
				const std::vector<short>& one = record[i];
				for (size_t j = 0; j < agent_num(); ++j){
					if (j == myAgentIndex){
						dump << "**" << (int)(one[j]) << "**\t";
					}
					else{
						dump << (int)(one[j]) << "\t";
					}
				}
				dump << std::endl;
			}
			dump.flush();
			record.clear();
		}
	};

	class AlgoSlowDown {
	public:
		AlgoSlowDown() : unit(100.0f) {}

		pt2D operator ()(double err, const pt2D& vxy, const pt2D& v_tan) {
			err /= unit;
			lastLamTan = lastLamNor = 1.0f;
			for (size_t i = 0; i < errLevel.size(); ++i) {
				if (err < errLevel[i]) {
					lastLamTan *= lamTan[i];
					lastLamNor *= lamNor[i];
				}
			}

			pt2D v_nor = impl::rot90(v_tan);
			real_t vTan = (real_t)impl::dot(v_tan, vxy);
			real_t vNor = (real_t)impl::dot(v_nor, vxy);
			return lastLamTan * vTan * v_tan + lastLamNor * vNor * v_nor;
		}

		real_t unit;
		real_t lastLamTan, lastLamNor;
		std::vector<real_t> errLevel;
		std::vector<real_t>  lamTan, lamNor;
	};

	class ConditionChecker {
	public:
		enum {
			Count_Good,
			Count_ContinousGood
		};

		ConditionChecker() : type(Count_Good), counter(0), limit(10) {};
		void reset() {
			counter = 0;
		}
		
		bool operator () (bool cond) {
			if (cond && counter < limit) counter++;
			if (!cond && type == Count_ContinousGood)
				counter = 0;
			return counter >= limit;
		}
		bool state() const {
			return counter >= limit;
		}

		int type;
		int counter;
		int limit;
	};


	class AlgoSignalConsensus{
	public:
		AlgoSignalConsensus() {}

		void reset() {
			_state = 0;
			checker.reset();
		}
		bool check(bool cond) { 
			if (checker(cond)) {
				_state++;
			}
			return consensus_reached();
		}
		bool consensus_reached() const {
			return state() == limit;
		}
		int state() const { 
			return _state; 
		}

		int limit;
		ConditionChecker checker;
	private:
		int _state;
	};


	class AlgoGroupFormation {
	public:
		struct OneAgent {
			int ID;
			pt2D  q, dq, z, hg;
			int   n;
			float r;
		};
		struct Output {
			pt2D u, z, hg;
			int  n, r;
		};
	};


} // namespace impl
