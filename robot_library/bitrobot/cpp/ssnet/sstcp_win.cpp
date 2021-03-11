#ifdef _WIN32
#include "sstcp.hpp"
#include "sstcp_api.hpp"


/*
 * Code Reference:
 * https://docs.microsoft.com/en-us/windows/win32/winsock/
*/
const char* error_msg(DWORD err) {
	switch (err) {
	case WSANOTINITIALISED: return "A successful WSAStartup call must occur before using this function.";
	case WSAEAFNOSUPPORT: return "Family parameter specified was not AF_INET or AF_INET6";
	case WSAEFAULT: return "pszAddrString or pAddrBuf parameters are NULL or invalid";
	case WSAENETDOWN: return "The network subsystem has failed.";
	case WSAENOTCONN: return "The socket is not connected.";
	case WSAEWOULDBLOCK: return "The socket is nonblocking";
	case WSAEMSGSIZE: return "The message was too large to fit into the specified buffer and was truncated.";
	case WSAEINVAL: return "The socket has not been bound with bind";
	case WSAETIMEDOUT: return "The connection has been dropped because of a network failure or because the peer system failed to respond.";
	default:
		return "Unkown Error";
	}
}


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
		printf("Error at inet_pton(): %d, %s\n", ret, error_msg(WSAGetLastError()));
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
#ifndef _MAX_ITOSTR_BASE10_COUNT
#define _MAX_ITOSTR_BASE10_COUNT 20
#endif

std::string NetworkAddress::get_addr_string() const {
	char buf[_MAX_ITOSTR_BASE10_COUNT + 1] = { 0 };
	buf[0] = ':';
	_itoa_s(get_port(), (char*)buf + 1, _MAX_ITOSTR_BASE10_COUNT, 10);
	return get_ip_string() + std::string(buf);
}
int NetworkAddress::get_port() const {
	return ntohs(m_port);
}



WSADATA* p_wsaData = nullptr;

bool sstcp_initialization() {
	if (p_wsaData == nullptr) {
		p_wsaData = new WSADATA;
	
		int iResult
			= WSAStartup(MAKEWORD(2,2), p_wsaData);
		if (iResult != 0) {
			printf("WSAStartup failed: %d\n", iResult);
			return false;
		}
		return true;
	}
	return true;
}
void sstcp_cleanup() {
	if (p_wsaData) {
		WSACleanup();
		delete p_wsaData;
	}
}



#endif