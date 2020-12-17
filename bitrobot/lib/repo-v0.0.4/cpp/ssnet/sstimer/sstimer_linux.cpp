#ifndef _WIN32
#include "sstimer.hpp"
#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>


// Static memeber functions
void sstimer::sleep_ms(int ms) {
	while (ms > 500) {
		usleep(500000);
		ms -= 500;
	}
	usleep(ms * 1000);
}

struct impl_sstimer {
	struct timespec t_start;
	struct timespec t_end;
};

// Memeber functions
sstimer::sstimer() {
	struct impl_sstimer* data = new impl_sstimer;
	m_data = data;
	start();
}
sstimer::~sstimer() {
	struct impl_sstimer* data = (struct impl_sstimer*)m_data;
	delete data;
}
void sstimer::start() {
	struct impl_sstimer* data = (struct impl_sstimer*)m_data;
	int ret = clock_gettime(CLOCK_REALTIME, &data->t_start);
	if (ret == -1) {
		//int err = errno;
		perror("[sstimer] clock_gettime()");
	}
}

double sstimer::elapsed() {
	struct impl_sstimer* data = (struct impl_sstimer*)m_data;
	int ret = clock_gettime(CLOCK_REALTIME, &data->t_end);
	if (ret == -1) {
		perror("[sstimer] clock_gettime()");
		return -1;
	}
	double sec = (double)(data->t_end.tv_sec - data->t_start.tv_sec);
	double nano = (double)(data->t_end.tv_nsec - data->t_start.tv_nsec);
	double dms = sec*1000 + nano/1000000;
	return dms;
}



#endif
