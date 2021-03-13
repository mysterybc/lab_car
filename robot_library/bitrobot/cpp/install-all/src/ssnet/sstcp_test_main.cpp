#ifdef BUILD_SSTCP_TEST
#include "sstcp.hpp"

int main() {
	sstcp_initialization();
	//sstcp_test::test_hello();
	sstcp_test::test_send_recv_multi_long();
	
	sstcp_cleanup();
	return 0;
}
#endif