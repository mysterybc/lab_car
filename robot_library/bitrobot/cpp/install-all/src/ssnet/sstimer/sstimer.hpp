#pragma once
#include <vector>
#include <cstdio>

// High Precision Timer (ms level)
class sstimer {
public:
	static void sleep_ms(int ms);
	
	sstimer();
	~sstimer();
	void start();
	double elapsed();
	double restart() {
		double t = elapsed();
		start();
		return t;
	}
	
private:
	void* m_data;
};

class sstimer_scope {
public:
	struct tick_point {
		const char* name;
		double point_thresh;
		double time_pass;
	};
	sstimer_scope(double thresh = -1) {
		m_points.reserve(10);
		scope_thresh = thresh;
		m_timer.start();
	}
	
	~sstimer_scope();
	double tick(const char* name, double thresh = -1);
	sstimer& timer() { return m_timer; }
	
private:
	double scope_thresh;
	std::vector<tick_point> m_points;
	sstimer m_timer;
};

inline sstimer_scope::~sstimer_scope() {
	double tnow = m_timer.elapsed();
	if (tnow > scope_thresh) {
		printf("Scope Time %.1fms [%.1fms]\n", tnow, scope_thresh);
		int npoint = (int)m_points.size();
		for (int i=0;i<npoint; ++i) {
			double dt_thresh = m_points[i].point_thresh;
			double dt = m_points[i].time_pass;
			if (i > 0) dt -= m_points[i-1].time_pass;
			if (dt > dt_thresh) {
				printf("--P%d: %.1fms[%.1fms], %s\n", 
					i+1, dt, dt_thresh, m_points[i].name);
			}
		}
	}
}

inline double sstimer_scope::tick(const char* name, double thresh) {
	tick_point one;
	one.name = name;
	one.point_thresh = thresh;
	one.time_pass = m_timer.elapsed();
	m_points.push_back(one);
	return one.time_pass;
}

namespace sstimer_test {
	int stress_test_ms();
}