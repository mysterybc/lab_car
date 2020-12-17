#include "utility.hpp"


#ifdef _WIN32
#include <Windows.h>
void my_sleepms(int ms) {
	Sleep(ms);
}
#else
#include <unistd.h>
void my_sleepms(int ms) {
	while (ms >= 500) {
		usleep(500*1000);
		ms -= 500;
	}
	usleep(ms * 1000);
}
#endif
