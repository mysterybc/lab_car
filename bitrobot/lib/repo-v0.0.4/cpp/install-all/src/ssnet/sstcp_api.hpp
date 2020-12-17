#pragma once
#include "sstcp.hpp"

#ifdef _WIN32

#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")


sockaddr_in make_addr(const NetworkAddress& netaddr);
const char* error_msg(DWORD err);

// Some api wrapper
class SocketAPI {
public:
	SocketAPI(SOCKET sock = INVALID_SOCKET): sock(sock) {}
	
	static SOCKET new_tcp_socket() {
		SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sock == INVALID_SOCKET) {
			printf("Error at socket(): %ld\n", WSAGetLastError());
			return INVALID_SOCKET;
		}
		return sock;
	}
	
	int set_recv_timeout(DWORD ms) {
		return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)(&ms), sizeof(ms));
	}
	int set_recv_buffersize(int size) {
		return setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char*)(&size), sizeof(size));
	}
	int set_send_timeout(DWORD ms) {
		return setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)(&ms), sizeof(ms));
	}
	int set_send_buffersize(int size) {
		return setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char*)(&size), sizeof(size));
	}
	int set_dont_linger() {
		LINGER option;
		option.l_onoff = 0;
		option.l_linger = 0;
		return setsockopt(sock, SOL_SOCKET, SO_LINGER, (char*)(&option), sizeof(option));
	}
	int set_nonblock() {
		u_long iMode = 1;
		int iResult = ioctlsocket(sock, FIONBIO, &iMode);
		if (iResult != NO_ERROR) {
			printf("ioctlsocket failed with error: %ld\n", iResult);
		}
		return iResult;
	}
	
	int connect(const NetworkAddress& netaddr) {
		sockaddr_in addr = make_addr(netaddr);
		int iResult = ::connect(sock, (SOCKADDR*)&addr, sizeof(addr));
		if (iResult == SOCKET_ERROR) {
			int err = WSAGetLastError();
			if (err != WSAEWOULDBLOCK)
				wprintf(L"connect function failed with error: %ld\n", WSAGetLastError());
		}
		return iResult;
	}

	int bind(const NetworkAddress& netaddr) {
		sockaddr_in addr = make_addr(netaddr);
		int iResult = ::bind(sock, (SOCKADDR*)&addr, sizeof(addr));
		if (iResult == SOCKET_ERROR) {
			wprintf(L"bind failed with error %u\n", WSAGetLastError());
		}
		return iResult;
	}
	int listen(int maximum_pending = 5) {
		int iResult = ::listen(sock, maximum_pending);
		if (iResult == SOCKET_ERROR) {
			wprintf(L"Error at listen() : %d\n", WSAGetLastError());
		}
		return iResult;
	}
	
	bool is_connected() {
		fd_set one; FD_ZERO(&one); FD_SET(sock, &one);
		timeval tm; tm.tv_sec = 0; tm.tv_usec = 0;
		return select(1, nullptr, &one, nullptr, &tm) == 1;
	}
	bool is_listening() {
		int val;
		socklen_t len = sizeof(val);
		if (getsockopt(sock, SOL_SOCKET, SO_ACCEPTCONN, (char*)&val, &len) == -1)
			return false; // not a socket
		else if (val)
			return true;  // is a listening socket
		else
			return false; // is a non-listening socket
	}

	int close() {
		int iResult = closesocket(sock);
		if (iResult == SOCKET_ERROR) {
			int err = WSAGetLastError();
			printf("close socket failed with error %d, %s\n", err, error_msg(err));
		}
		sock = INVALID_SOCKET;
		return iResult;
	}
	int recv(char* buffer, int buffer_len) {
		int iResult = ::recv(sock, buffer, buffer_len, 0);
		if (iResult == SOCKET_ERROR) {
			int err = WSAGetLastError();
			if (err != WSAEWOULDBLOCK)
				printf("recv failed with error %d, %s\n", err, error_msg(err));
		}
		return iResult;
	}
	SOCKET accept(NetworkAddress* addr) {
		sockaddr_in client_addr;
		SOCKET client;
		int length = sizeof(client_addr);
		client = ::accept(sock, (SOCKADDR*)(&client_addr), &length);
		if (addr) {
			addr->m_ip = client_addr.sin_addr.s_addr;
			addr->m_port = client_addr.sin_port;
		}

		if (client == INVALID_SOCKET) {
			int err = WSAGetLastError();
			if (err != WSAEWOULDBLOCK)
				printf("accept failed with error %d, %s\n", err, error_msg(err));
		}
		return client;
	}
	int send(char* data, int n_bytes) {
		int iResult = ::send(sock, data, n_bytes, 0);
		if (iResult == SOCKET_ERROR) {
			int err = WSAGetLastError();
			printf("send failed with error %d, %s\n", err, error_msg(err));
		}
		return iResult;
	}

	SOCKET sock;
};

#else 
#include <sys/types.h> 
#include <sys/socket.h>
#ifndef __QNX__
#include <sys/unistd.h>
#include <sys/fcntl.h>
#else
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
#define nullptr NULL
#endif
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <errno.h>
#include <cstdio>
#include <cstring>

sockaddr_in make_addr(const NetworkAddress& netaddr);

typedef int SOCKET;
typedef struct sockaddr SOCKADDR;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1

// Some api wrapper
class SocketAPI {
public:
	SocketAPI(SOCKET sock = -1): sock(sock) {}
	
	static SOCKET new_tcp_socket() {
		// For SOCK_STREAM and SOCK_DGRAM, the protocol part can be left as 0
		SOCKET sock = socket(PF_INET, SOCK_STREAM, 0);
		if (sock == -1) {
			printf("Error at socket(): %d, %s\n", errno, strerror(errno));
			return -1;
		}
		return sock;
	}
	
	int set_recv_timeout(int ms) {
		return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)(&ms), sizeof(ms));
	}
	int set_recv_buffersize(int size) {
		return setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char*)(&size), sizeof(size));
	}
	int set_send_timeout(int ms) {
		return setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)(&ms), sizeof(ms));
	}
	int set_send_buffersize(int size) {
		return setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char*)(&size), sizeof(size));
	}
	int set_dont_linger() {
		struct linger option;
		option.l_onoff = 0;
		option.l_linger = 0;
		return setsockopt(sock, SOL_SOCKET, SO_LINGER, (char*)(&option), sizeof(option));
	}
	int set_nonblock() {
		//u_long iMode = 1;
		int iResult = fcntl(sock, F_SETFL, O_NONBLOCK);
		if (iResult != 0) {
			printf("Error fcntl() nonblock failed with error: %d, %s\n", errno, strerror(errno));
		}
		return iResult;
	}

	int connect(const NetworkAddress& netaddr) {
		sockaddr_in addr = make_addr(netaddr);
		int iResult = ::connect(sock, (SOCKADDR*)&addr, sizeof(addr));
		if (iResult == -1) {
			int err = errno;
			if (err != EAGAIN && err != EWOULDBLOCK && err != EINPROGRESS)
				printf("connect function failed with error: %d, %s\n", errno, strerror(errno));
		}
		return iResult;
	}

	int bind(const NetworkAddress& netaddr) {
		sockaddr_in addr = make_addr(netaddr);
		int iResult = ::bind(sock, (SOCKADDR*)&addr, sizeof(addr));
		if (iResult == -1) {
			printf("bind failed with error %u\n", errno);
		}
		return iResult;
	}
	int listen(int maximum_pending = 5) {
		int iResult = ::listen(sock, maximum_pending);
		if (iResult == -1) {
			printf("Error at listen() : %d\n", errno);
		}
		return iResult;
	}
	bool is_connected() {
		fd_set one;  FD_ZERO(&one); FD_SET(sock, &one);
		struct timeval tm; tm.tv_sec = 0; tm.tv_usec = 0;
		return select(1 + 1, nullptr, &one, nullptr, &tm) == 0;
	}
	bool is_listening() {
		int val;
		socklen_t len = sizeof(val);
		if (getsockopt(sock, SOL_SOCKET, SO_ACCEPTCONN, &val, &len) == -1)
			return false; // not a socket
		else if (val)
			return true;  // is a listening socket
		else
			return false; // is a non-listening socket
	}
	int close() {
		int iResult = ::close(sock);
		if (iResult == -1) {
			printf("close socket failed with error %d\n", errno);
		}
		sock = -1;
		return iResult;
	}
	int recv(char* buffer, int buffer_len) {
		int iResult = ::recv(sock, buffer, buffer_len, 0);
		if (iResult == -1) {
			int err = errno;
			if (err != EAGAIN && err != EWOULDBLOCK && err != EINPROGRESS)
				printf("recv failed with error %d, %s\n", err, strerror(err));
		}
		return iResult;
	}
	SOCKET accept(NetworkAddress* addr) {
		sockaddr_in client_addr;
		SOCKET client;
		unsigned int length = sizeof(client_addr);
		client = ::accept(sock, (SOCKADDR*)(&client_addr), &length);
		if (addr) {
			addr->m_ip = client_addr.sin_addr.s_addr;
			addr->m_port = client_addr.sin_port;
		}

		if (client == -1) {
			int err = errno;
			if (err != EAGAIN && err != EWOULDBLOCK && err != EINPROGRESS)
				printf("accept failed with error %d, %s\n", err, strerror(err));
		}
		return client;
	}
	int send(char* data, int n_bytes) {
		// Note: Use MSG_NOSIGNAL, otherwise the program may be terminated for an unhandled signal
		// Note2: This seems not working in QNX??
		int iResult = ::send(sock, data, n_bytes, MSG_NOSIGNAL);
		if (iResult == -1) {
			int err = errno;
			printf("send failed with error %d, %s\n", err, strerror(err));
		}
		return iResult;
	}

	SOCKET sock;
};

#endif


