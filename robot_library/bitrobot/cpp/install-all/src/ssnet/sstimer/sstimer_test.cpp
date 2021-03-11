#include "sstimer.hpp"

namespace sstimer_test {
	typedef double (*workload_t)(int, double);
	typedef double (*recursive_t)(int, int, int, workload_t);

	double basic_fun(int nLoop, double r) {
		double ret = 1;
		for (int i = 1; i <= nLoop; ++i) ret += 1.0 / (double(i)+r);
		return ret;
	}
	double recurr_0(int n1, int n2, int nInner, workload_t fun) {
		double ret = 1;
		for (int i = 1; i <= n1; ++i) {
			for (int j = 1; j < n2; ++j) {
				ret += fun(nInner, double(i + j));
			}
		}
		return ret;
	}
	double recurr_1(int n1, int n2, int nInner, workload_t fun) {
		double ret = 1;
		for (int i = 1; i <= n1; ++i) {
			for (int j = 1; j < n2; ++j) {
				ret += recurr_0(1, 100, nInner, fun);
			}
		}
		return ret;
	}
	double recurr_2(int n1, int n2, int nInner, workload_t fun) {
		double ret = 1;
		for (int i = 1; i <= n1; ++i) {
			for (int j = 1; j < n2; ++j) {
				ret += recurr_0(1, 1000, nInner, fun);
			}
		}
		return ret;
	}

	struct WorkLoadBase {
		WorkLoadBase() { n1 = 1; n2 = 1000; nFun = 1000; }
		double operator()() {
			sstimer t;
			rec(n1, n2, nFun, fun);
			return t.elapsed();
		}
		double test_n(int n) {
			double r = 0;
			for (int i = 0; i < n; ++i) r += (*this)();
			return r / n;
		}

		workload_t  fun;
		recursive_t rec;
		int n1, n2;
		int nFun;
		double dt;
	};

	double test_milliseconds(double td, WorkLoadBase fun) {
		printf("Testing %.1f ms\n", td);
		int n = (int)(fun.n2 * td/fun.dt);
		if (n <= 0) {
			printf("Cannot test this workload for the given Td\n");
			return -1;
		}
		
		fun.n2 = n;
		double tm = fun();
		double err = (tm - td) / td;
		printf("--- Testing: Expect %.2fms, Measured %.2fms, err %.1f%%\n", td, tm, err*100);
		return err >= 0 ? err : -err;
	}
	double test_milliseconds_batch(int nTest, double td, WorkLoadBase fun) {
		double err = 0, eMax = 0;
		for (int i = 0; i < nTest; ++i) {
			double ee = test_milliseconds(td, fun);
			eMax = eMax > ee ? eMax : ee;
			err += ee;
		}
		err = (err-eMax) / ((double)nTest - 1);
		printf("[Result] On Average of [%d] Tests: Error = %.2f%%\n", nTest, err * 100);
		return err;
	}

	WorkLoadBase get_base_workload(workload_t fun, recursive_t rec) {
		WorkLoadBase workload;
		workload.n1 = 1;
		workload.n2 = 10000;
		workload.nFun = 10;
		workload.fun = fun;
		workload.rec = rec;
		
		while (workload.n2 < 100000000 && workload.n2 > 0) {
			WorkLoadBase w = workload;
			double ms = w();
			if (ms < 1) workload.n2 *= 1000;
			else if (ms < 100) workload.n2 *= 10;
			else if (ms < 1000) workload.n2 *= int(2000/ms);
			else {
				workload.n2 = w.n2;
				workload.dt = workload.test_n(10);
				return workload;
			}
			printf("Finding n2: n2 = %d, last dt = %.2lf\n", workload.n2, ms);
		}

		workload.n2 = 100000000;
		while (workload.n1 < 1000000 && workload.n1 > 0) {
			WorkLoadBase w = workload;
			double ms = w();
			if (ms < 1) workload.n1 *= 1000;
			else if (ms < 100)  workload.n1 *= 10;
			else if (ms < 1000) workload.n1 *= int(2000/ms);
			else {
				workload.n1 = w.n1;
				workload.dt = workload.test_n(10);
				return workload;
			}
			printf("Finding n1: n1 = %d, last dt = %.2f\n", workload.n2, ms);
		}

		workload.n1 = -1;
		return workload;
	}

	int stress_test_ms() {
		recursive_t recList[] = { recurr_0 , recurr_1, recurr_2 };
		WorkLoadBase workload;
		int wIndex = 0;
		for (wIndex = 0; wIndex < 3; ++wIndex) {
			printf("Testing Workload Function %d\n", wIndex);
			workload = get_base_workload(basic_fun, recList[wIndex]);
			if (workload.n1 > 0)
				break;
		}
		if (workload.n1 < 0) {
			printf("Failed to determine the test workload\n");
			return -1;
		}
		printf("Test Work Load: FuncIndex=%d, n1=%d, n2=%d, nFun=%d, dt=%.2fms\n", 
			wIndex, workload.n1, workload.n2, workload.nFun, workload.dt);

		int nTest = 10;
		double err = 0, tmp;
		double td[] = {1, 2, 5, 10, 50, 100, 1000};
		for (int i = 0; i < 7; ++i) {
			tmp = test_milliseconds_batch(nTest, td[i], workload);
			err = tmp > err ? tmp : err;
		}
		printf("\n------- Test Summary -------\n");
		printf("Maximum Workload Error: %.2f%%\n\n", err * 100);
		return err < 0.01 ? 1 : 0;
	}
}
