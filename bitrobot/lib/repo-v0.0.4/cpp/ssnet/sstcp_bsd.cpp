#ifndef _WIN32
#include "sstcp.hpp"
#include "sstcp_api.hpp"


/*
 * Code Reference:
 * QNX: http://www.qnx.com/developers/docs/6.5.0/index.jsp?topic=%2Fcom.qnx.doc.neutrino_sys_arch%2Ftcpip.html
 * Linux: https://linux.die.net/man/7/socket
*/

sockaddr_in make_addr(const NetworkAddress& netaddr) {
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = netaddr.m_ip;
	addr.sin_port = netaddr.m_port;
	return addr;
}

NetworkAddress::NetworkAddress(const char* ip, int port) {
	int ret = inet_pton(AF_INET, ip, &m_ip);
	if (ret != 1) {
		int err = errno;
		printf("Error at inet_pton(): %d, %s\n", err, strerror(err));
		m_ip = 0;
	}
	m_port = htons(port);
}
std::string NetworkAddress::get_ip_string() const {
	char buf[24] = { 0 };
	sockaddr_in addr = make_addr(*this);
	inet_ntop(AF_INET, &addr.sin_addr, buf, 24);
	return std::string(buf);
}
std::string NetworkAddress::get_addr_string() const {
	char buf[32] = { 0 };
	std::string ipstr = get_ip_string();
	snprintf(buf, 32, "%s:%d", ipstr.c_str(), get_port());
	return std::string(buf);
}
int NetworkAddress::get_port() const {
	return ntohs(m_port);
}

bool sstcp_initialization() { return true; }
void sstcp_cleanup() {}



#endif