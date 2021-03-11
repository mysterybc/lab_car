#include "sstcp.hpp"
#include "sstcp_api.hpp"
#include <deque>

struct impl_clientinfo {
	impl_clientinfo() {}
	impl_clientinfo(SOCKET soc, const NetworkAddress& addr, size_t bufsize) 
		: s(soc), addr(addr), buffer((int)bufsize) {}
	SocketAPI s;
	NetworkAddress addr;
	
	RecvBuffer buffer;
};

struct impl_tcpclient {
	SocketAPI s;
	std::string buffer;
};
struct impl_tcpserver {
	SocketAPI s;
	std::vector<impl_clientinfo> c;
};

int recv_and_push(SocketAPI* m_sock, RecvBuffer& buffer) {
	int nfree = buffer.auto_shuffle();
	int nread = m_sock->recv(buffer.end(), nfree);
	if (nread > 0) 
		buffer.push(nread);
	return nread;
}

int SocketRoutine::recv_pending_data(SocketAPI* m_sock, RecvBuffer& buffer) {
	int total = 0;
	int length = recv_and_push(m_sock, buffer);
	int count_recv = 0, count_handle = 0, max_count = 50000;
	while (length > 0 && count_recv++ < max_count) {
		total += length;
		while (handle_data(buffer) && count_handle++ < max_count) {}
		length = recv_and_push(m_sock, buffer);
	}
	if (count_handle >= max_count || count_recv >= max_count) {
		printf("Warning: Infinity loops A. nHandle = %d, nRecv = %d\n", 
			count_handle, count_recv);
		buffer.clear();
		return total;
	}
	if (buffer.length_free() == 0) {
		printf("Warning: RecvBuffer is full.\n");
	}
	return total;
}

int SocketRoutine::init_nonblocking_tcp(SocketAPI* m_sock) {
	m_sock->sock = SocketAPI::new_tcp_socket();
	return config_nonblocking_tcp(m_sock);
}
int SocketRoutine::config_nonblocking_tcp(SocketAPI* m_sock) {
	m_sock->set_recv_timeout(0);
	m_sock->set_send_timeout(0);
	m_sock->set_nonblock();
	//m_sock->set_dont_linger();
	return 0;
}

int SocketRoutine::recv_pending_then_reply(SocketAPI* m_sock, RecvBuffer& bufrcv, SendBuffer& bufsend) {
	int total = 0;
	int nread = recv_and_push(m_sock, bufrcv);
	int count_recv = 0, count_handle = 0, max_count = 50000;
	while (nread > 0 && count_recv++ < max_count) {
		total += nread;
		bufsend.clear();
		while (handle_data_with_reply(bufrcv, bufsend) && count_handle++ < max_count) {
			if (bufsend.length() > 0) {
				if (m_sock->send(bufsend.begin(), bufsend.length()) != bufsend.length()) {
					printf("Warning: Send Length Mismatch\n");
				}
				bufsend.clear();
			}
		}
		if (bufsend.length() > 0) {
			if (m_sock->send(bufsend.begin(), bufsend.length()) != bufsend.length()) {
				printf("Warning: Send Length Mismatch\n");
			}
			bufsend.clear();
		}

		nread = recv_and_push(m_sock, bufrcv);
	}
	if (count_handle >= max_count || count_recv >= max_count) {
		printf("Warning: Infinity loops B. nHandle = %d, nRecv = %d\n",
			count_handle, count_recv);
		bufrcv.clear();
		bufsend.clear();
		return total;
	}

	if (bufrcv.length_free() == 0) {
		printf("Warning: RecvBuffer is full.\n");
	}
	return total;
}

// -------------------
// ---- TCPClient ----
// -------------------

TCPClient::TCPClient(std::size_t buffer_size): m_data(nullptr), buffer((int)buffer_size) {
	//std::copy_n()
	impl_tcpclient* m_sock = new impl_tcpclient();
	m_data = m_sock;

	init_nonblocking_tcp(&m_sock->s);
}
TCPClient::~TCPClient() {
	if (m_data != nullptr) {
		impl_tcpclient* m_sock = (impl_tcpclient*)m_data;
		m_sock->s.close();
		delete m_sock;
	}
}
int TCPClient::setsocketopt_sndbuf(int size) {
	impl_tcpclient* m_sock = (impl_tcpclient*)m_data;
	return m_sock->s.set_send_buffersize(size);
}
int TCPClient::setsocketopt_rcvbuf(int size) {
	impl_tcpclient* m_sock = (impl_tcpclient*)m_data;
	return m_sock->s.set_recv_buffersize(size);
}

bool TCPClient::connect(const NetworkAddress& server_addr) {
	impl_tcpclient* m_sock = (impl_tcpclient*)m_data;
	int ret = m_sock->s.connect(server_addr);
	return ret != SOCKET_ERROR;
}
bool TCPClient::is_connected() {
	impl_tcpclient* m_sock = (impl_tcpclient*)m_data;
	return m_sock->s.is_connected();
}


int TCPClient::send(const char* data, std::size_t size) {
	impl_tcpclient* m_sock = (impl_tcpclient*)m_data;
	return m_sock->s.send((char*)data, (int)size);
}

int TCPClient::recv_pending() {
	impl_tcpclient* m_sock = (impl_tcpclient*)m_data;
	return recv_pending_data(&m_sock->s, buffer);
}

// -------------------
// ---- TCPServer ----
// -------------------

TCPServer::TCPServer(std::size_t bufsz_rcv, std::size_t bufsz_snd) 
	: m_data(nullptr), bufsz_snd(bufsz_snd), bufsz_rcv(bufsz_rcv), buffer_send(bufsz_snd)
{
	sstcp_initialization();
	impl_tcpserver* m_sock = new impl_tcpserver();
	m_data = m_sock;

	init_nonblocking_tcp(&m_sock->s);
	m_sock->c.clear();
	m_sock->c.reserve(5);	// reserve for 5 clients
}
TCPServer::~TCPServer() {
	if (m_data != nullptr) {
		impl_tcpserver* m_sock = (impl_tcpserver*)m_data;
		m_sock->s.close();
		std::size_t nc = m_sock->c.size();
		for (std::size_t i = 0; i < nc; ++i) {
			m_sock->c[i].s.close();
		}
		delete m_sock;
		m_data = nullptr;
	}
}

int TCPServer::setsocketopt_sndbuf(int size) {
	impl_tcpserver* m_sock = (impl_tcpserver*)m_data;
	int ret = m_sock->s.set_send_buffersize(size);
	if (ret != SOCKET_ERROR) {
		buffer_send = SendBuffer(size);
	}
	return ret;
}
int TCPServer::setsocketopt_rcvbuf(int size) {
	impl_tcpserver* m_sock = (impl_tcpserver*)m_data;
	return m_sock->s.set_recv_buffersize(size);
}

int TCPServer::bind_and_listen(const NetworkAddress& server_addr) {
	impl_tcpserver* m_sock = (impl_tcpserver*)m_data;
	if (m_sock->s.sock != INVALID_SOCKET) {
		int ret = m_sock->s.bind(server_addr);
		if (ret == 0) {
			return m_sock->s.listen();
		}
		return ret;
	}
	return -1;
}
bool TCPServer::is_binded() {
	impl_tcpserver* m_sock = (impl_tcpserver*)m_data;
	return m_sock->s.is_listening();
}

int TCPServer::sendtoall(char* data, std::size_t size) {
	impl_tcpserver* m_sock = (impl_tcpserver*)m_data;
	std::vector<impl_clientinfo>::iterator ss = m_sock->c.begin();
	int nsend = 0;
	while (ss != m_sock->c.end()) {
		int ret = ss->s.send(data, (int)size);
		if (ret != (int)size) {
			ss = m_sock->c.erase(ss);
		}
		else ++ss;
		nsend += ret;
	}
	return nsend;
}

int TCPServer::accept_and_recv() {
	impl_tcpserver* m_sock = (impl_tcpserver*)m_data;

	// Checking new connections
	NetworkAddress addr;
	SOCKET client = m_sock->s.accept(&addr);
	while (client != INVALID_SOCKET) {
		// Initialize Server-Client Connection 
		SocketAPI ss(client);
		config_nonblocking_tcp(&ss);
		m_sock->c.push_back(impl_clientinfo(client, addr, bufsz_rcv));

		// Call on_accept handlers
		buffer_send.clear();
		handle_accept_with_reply(addr, buffer_send);
		if (buffer_send.length() > 0) {
			SocketAPI(client).send(buffer_send.begin(), buffer_send.length());
			buffer_send.clear();
		}

		// Check for others
		addr = NetworkAddress();
		client = m_sock->s.accept(&addr);
	}

	// Receive Pending Data
	std::size_t nc = m_sock->c.size();
	int total = 0;
	for (std::size_t i = 0; i < nc; ++i) {
		total += recv_pending_then_reply(&m_sock->c[i].s, m_sock->c[i].buffer, buffer_send);
	}
	return total;
}

int TCPServer::client_num() {
	impl_tcpserver* m_sock = (impl_tcpserver*)m_data;
	return (int)m_sock->c.size();
}

int TCPServer::remove_unreachable() {
	impl_tcpserver* m_sock = (impl_tcpserver*)m_data;
	
	std::vector<impl_clientinfo>::iterator iter = m_sock->c.begin();
	while (iter != m_sock->c.end()) {
		if (!iter->s.is_connected()) {
			iter->s.set_dont_linger();
			iter->s.close();
			iter = m_sock->c.erase(iter);
		}
		else ++iter;
	}
	return client_num();
}
