#pragma once
#include "compat.hpp"
#include <vector>
#include <string>
#include <cstring>
#include <map>
#include <algorithm>


// Used by windows sockets mainly
bool sstcp_initialization();
void sstcp_cleanup();

// The Address Tranformation Routine
class NetworkAddress {
public:
	NetworkAddress() : m_ip(0), m_port(0) {}
	NetworkAddress(const char* ip, int port);
	std::string get_ip_string() const;
	std::string get_addr_string() const;
	int get_port() const;

	unsigned long  m_ip;
	unsigned short m_port;
};

class RecvBuffer {
public:
	// Should enforce the buffer is '0' terminated, 
	// when using std string as the storage, this is done automatically
	RecvBuffer(int max_size = 1000)
		: curr_pos(0), curr_end(0), buffer(max_size, '\0') {} 

	// Length
	int length() const { return curr_end - curr_pos; }
	int length_free() const { return (int)buffer.size() - curr_end; }
	int capacity() const { return (int)buffer.size(); }
	
	// Pointers
	char* begin() { return (char*)buffer.data() + curr_pos; }
	const char* begin() const { return (const char*)buffer.data() + curr_pos; }
	char* end()   { return (char*)buffer.data() + curr_end; }
	char* raw_begin() { return (char*)buffer.data(); }
	char* raw_end() { return (char*)buffer.data() + buffer.size(); }

	// Move Data Inside The Vector
	int auto_shuffle() {
		if (length_free() < capacity() / 3) {
			return shuffle();
		}
		return length_free();
	}
	int shuffle() {
		if (curr_pos != 0) {
			int len = length();
			std::copy(begin(), end(), raw_begin());
			curr_pos = 0;
			curr_end = len;
		}
		return length_free();
	}
	
	// Manipulate the index
    int push(const char* data, int data_length) {
        int n_push = std::min(length_free(), data_length);

		std::copy(data, data + n_push, end());
		curr_end += n_push;
		return n_push;
	}
	int push_cstr(const char* str) { return push(str, (int)std::strlen(str)); }
	int push(int num) {
		int n_push = std::min(length_free(), num);

		curr_end += n_push;
		return n_push;
	}
	int eat(int num) {
		int n_eat = std::min(num, length());

		curr_pos += n_eat;
		if (curr_pos == curr_end) shuffle();
		return n_eat;
	}
	void clear() { eat(length()); curr_pos = 0; curr_end = 0; }
	void clear(char c) {
		clear();
		std::fill_n(begin(), length_free(), c);
	}


private:
	int curr_pos;
	int curr_end;
	std::string buffer;
};

typedef RecvBuffer SendBuffer;



class SocketAPI;
class SocketRoutine {
protected:
	// Return if need call this function again
	virtual bool handle_data(RecvBuffer& data) { data.clear(); return false; }

	// Return if need call this function again
	virtual bool handle_data_with_reply(RecvBuffer& data, SendBuffer& reply) { data.clear(); return false; }
	virtual void handle_accept_with_reply(const NetworkAddress& addr, SendBuffer& buffer) {}

protected:
	int init_nonblocking_tcp(SocketAPI* sock_data);
	int config_nonblocking_tcp(SocketAPI* sock_data);
	int recv_pending_data(SocketAPI* sock_data, RecvBuffer& buffer);
	int recv_pending_then_reply(SocketAPI* sock_data, RecvBuffer& bufrcv, SendBuffer& bufsend);
};


class TCPClient : public SocketRoutine {
public:
	// RecvBuffer size
	TCPClient(std::size_t buffer_size = 4096);
	virtual ~TCPClient();
	
	int setsocketopt_sndbuf(int size);
	int setsocketopt_rcvbuf(int size);

	// Note, return true does not mean it is connected
	bool connect(const NetworkAddress& server_addr);
	bool is_connected();

	int recv_pending();
	int send(const char* data, std::size_t size);

protected:
	virtual bool handle_data(RecvBuffer& data) { data.clear(); return false; }

private:
	// Shadowed constructor and assignment operator
	// -- You cannot copy the object. Otherwise, 
	// -- two objects will share the same socket.
	TCPClient(const TCPClient& other) : m_data(nullptr), buffer(1024) {}
	TCPClient& operator = (const TCPClient& other) { return *this; }
	
	void* m_data;
	RecvBuffer buffer;
};


class TCPServer : public SocketRoutine {
public:
	// RecvBuffer size for each connected client
	TCPServer(std::size_t bufsz_rcv = 4096, std::size_t bufsz_snd = 1024);
	virtual ~TCPServer();

	int setsocketopt_sndbuf(int size);
	int setsocketopt_rcvbuf(int size);

	bool is_binded();
	int bind_and_listen(const NetworkAddress& server_addr);
	int accept_and_recv();
	int sendtoall(char* data, std::size_t size);
	int client_num();
	int remove_unreachable(); // Remove clients that cannot be sent, return the client_num

protected:
	virtual void handle_accept_with_reply(const NetworkAddress& addr, SendBuffer& buffer) {}
	virtual bool handle_data_with_reply(RecvBuffer& data, SendBuffer& reply) { data.clear(); return false; }

private:
	// Shadowed constructor and assignment operator
	// -- You cannot copy the object. Otherwise, 
	// -- two objects will share the same socket.
	TCPServer(const TCPServer& other) {}
	TCPServer& operator = (const TCPServer& other) { return *this; }

	void*  m_data;
	std::size_t bufsz_snd;
	std::size_t bufsz_rcv;
	SendBuffer buffer_send;
};




// Testing Applications
namespace sstcp_test {
	
	// Setting up a simple Hello
	int test_hello();
	int test_send_recv_multi_long();
	
	int test_long_msg();
	
	int test_multiple_client();
	
	int test_reconnection();
}











