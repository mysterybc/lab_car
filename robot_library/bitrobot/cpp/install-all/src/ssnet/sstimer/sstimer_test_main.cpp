#ifdef BUILD_SSTIMER_TEST
#include "sstimer.hpp"

int main() {
	sstimer_test::stress_test_ms();
	return 0;
}
#endif