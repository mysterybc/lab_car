#ifdef _WIN32
#include "sstimer.hpp"

#define NOMINMAX
#include <Windows.h>


struct impl_timer {
	LARGE_INTEGER m_freq;
	LARGE_INTEGER m_start;
	LARGE_INTEGER m_stop;
};

// Static memeber functions
void sstimer::sleep_ms(int ms) {
	Sleep(ms);
}

// Memeber functions
sstimer::sstimer() {
	impl_timer* m_timer = new impl_timer;
	QueryPerformanceFrequency(&m_timer->m_freq);
	QueryPerformanceCounter(&m_timer->m_start);
	
	m_data = m_timer;
}

sstimer::~sstimer() {
	impl_timer* m_timer = (impl_timer*)m_data;
	delete m_timer;
}
void sstimer::start() {
	impl_timer* m_timer = (impl_timer*)m_data;
	QueryPerformanceCounter(&m_timer->m_start);
}

double sstimer::elapsed() {
	impl_timer* m_timer = (impl_timer*)m_data;
	QueryPerformanceCounter(&m_timer->m_stop);
	
	long long nano = m_timer->m_stop.QuadPart - m_timer->m_start.QuadPart;
	nano *= 1000000;

	double ms = double(nano / m_timer->m_freq.QuadPart);
	return ms / 1000;
}


#endif