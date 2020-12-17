#include "sstcp.hpp"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include "mersenne/mersenne.h"

namespace sstcp_test {
#ifdef _WIN32
#define _NOMINMAX
#include <Windows.h>
	void my_sleepms(int ms) {
		Sleep(ms);
	}
#else
#include <unistd.h>
	void my_sleepms(int ms) {
		while (ms >= 500) {
			usleep(500 * 1000);
			ms -= 500;
		}
		usleep(ms * 1000);
	}
#endif
#ifdef min
#undef min
#endif

	void print_string(const char* data, std::size_t size) {
		for (std::size_t i = 0; i < size; ++i) putchar(data[i]);
	}
	class Server : public TCPServer {
	protected:
		void handle_accept_with_reply(const NetworkAddress& addr, SendBuffer& buffer) {
			std::string addr_string = addr.get_addr_string();
			printf("[Server] Get Connection From %s\n", addr_string.c_str());
			buffer.push_cstr("Hello world");
		}
		bool handle_data_with_reply(RecvBuffer& buffer, SendBuffer& reply) {
			int nsize = buffer.length();
			printf("[Server] Get Data [%d]: ", buffer.length());
			print_string(buffer.begin(), std::min(nsize, 10));
			putchar('\n');

			reply.push_cstr("I Get: ");
			reply.push(buffer.begin(), std::min(nsize, 10));
			printf("-- Replying with size %d\n", reply.length());

			buffer.clear();
			return false;	// Dont need to call this again
		}
	};
	class Client : public TCPClient {
	protected:
		bool handle_data(RecvBuffer& data) {
			int len = data.length();
			printf("[Client] Rcv [%d]: ", len);
			print_string(data.begin(), std::min(len, 10));
			putchar('\n');
			data.clear();
			return false;  // Dont need to call this again
		}
	};

	struct DataMatch {
		DataMatch() {}
		DataMatch(char* target, int length)
			:curr(0), target(target), length(length), failed(false) {}

		bool match(const char* p, int len) {
			if (curr + len > length) 
				return false;
			for (int i = 0; i < len; ++i) {
				if (target[curr + i] != p[i]) return false;
			}
			return true;
		}
		bool good() const { return !failed && curr == length; }

		int   curr;
		char* target;
		int   length;
		bool  failed;
	};

	class VeryLongServer: public TCPServer {
	protected:
		void handle_accept_with_reply(const NetworkAddress& addr, SendBuffer& reply) {}
		bool handle_data_with_reply(RecvBuffer& buffer, SendBuffer& reply) {
			//int nreply = 0;
			if (buffer.length() > 50) {
				int match_len = std::min(buffer.length(), reply.length_free());
				bool m1 = s1.match(buffer.begin(), match_len);
				bool m2 = s2.match(buffer.begin(), match_len);
				
				if (m1 && m2) {
					// Do nothing, wait for more messages
				}
				else if (m1) {
					s1.curr += match_len;
					int len0 = reply.length();
					if (reply.push(buffer.begin(), match_len) != match_len) {
						puts("Error: Push Reply Failed A");
					}
					int len1 = reply.length();
					if (len1 - len0 != match_len) {
						puts("Error: Buffer Push Error A");
					}

					printf("[Server] Sending A %d: ", match_len);
					print_string(buffer.begin(), std::min(10, buffer.length()));
					printf("\n");

					buffer.eat(match_len);
					return buffer.length() > 0;
				}
				else if (m2) {
					s2.curr += match_len;
					int len0 = reply.length();
					if (reply.push(buffer.begin(), match_len) != match_len) {
						puts("Error: Push Reply Failed B");
					}
					int len1 = reply.length();
					if (len1 - len0 != match_len) {
						puts("Error: Buffer Push Error B");
					}

					printf("[Server] Sending B %d: ", match_len);
					print_string(buffer.begin(), std::min(10, buffer.length()));
					printf("\n");

					buffer.eat(match_len);
					return buffer.length() > 0;
				}
			}
			return false;
		}
	public:
		DataMatch s1, s2;
	};
	class VeryLongClient : public TCPClient {
	protected:
		VeryLongClient() { nsend = 0; }
		bool handle_data(RecvBuffer& buffer) {
			int len = buffer.length();
			if (len > 50 && !s.failed) {
				if (s.match(buffer.begin(), buffer.length())) {
					s.curr += buffer.length();
				}
				else {
					s.failed = true;
				}
			}
			printf("[%s] Get Data %d -- %s\n", 
				name.c_str(), buffer.length(), s.failed ? "Failed" : "Good");
			printf("-->");
			print_string(buffer.begin(), std::min(10, buffer.length()));
			printf("\n");

			if (len > 50) {
				buffer.clear();
			}
			return false;
		}

	public:
		VeryLongClient(const std::string& name) : name(name) {}
		bool send_verylong() {
			int randint = 97;
			while (nsend < s.length) {
				int len = this->send((char*)s.target + nsend, std::min(s.length - nsend, randint));
				if (len > 0) nsend += len;
				else {
					printf("[%s] Send Failed()\n", name.c_str());
					return false;
				}
			}
			//printf("[Client] Sending Acc %d\n", nsend);
			return true;
		}

		int nsend;
		DataMatch s;
		std::string name;
	};
	
	void gen_rand_str(MersenneTwister& rnd, char* s, int length) {
		for (int i = 0; i < length; ++i) {
			char c = rnd() % 26 + 'a';
			s[i] = c;
		}
	}

	int test_send_recv_multi_long() {
		char str1[5001];
		char str2[3001];
		MersenneTwister rnd(7788);
		gen_rand_str(rnd, (char*)str1, 5000);
		gen_rand_str(rnd, (char*)str2, 3000);
		str1[5000] = 0;
		str2[3000] = 0;

		int tcp_bufsize = 1000;

		NetworkAddress addr_server("127.0.0.1", 18133);
		VeryLongServer server;
		server.setsocketopt_rcvbuf(tcp_bufsize);
		server.setsocketopt_sndbuf(tcp_bufsize);

		server.s1 = DataMatch((char*)str1, 5000);
		server.s2 = DataMatch((char*)str2, 3000);
		server.bind_and_listen(addr_server);

		VeryLongClient c1("ClientA"), c2("ClientB");
		c1.setsocketopt_rcvbuf(tcp_bufsize);
		c1.setsocketopt_sndbuf(tcp_bufsize);
		c2.setsocketopt_rcvbuf(tcp_bufsize);
		c2.setsocketopt_sndbuf(tcp_bufsize);

		c1.s = DataMatch((char*)str1, 5000);
		c2.s = DataMatch((char*)str2, 3000);
		c1.connect(addr_server);
		c2.connect(addr_server);

		server.accept_and_recv();
		
		for (int i = 0; i < int(5000 / tcp_bufsize+0.5); ++i) {
			c1.recv_pending();
			c2.recv_pending();
			c1.send_verylong();
			c2.send_verylong();

			server.accept_and_recv();
		}
		c1.recv_pending();
		c2.recv_pending();
		
		printf("Status: s1: %d/%d\n", server.s1.curr, server.s1.length);
		printf("Status: s2: %d/%d\n", server.s2.curr, server.s2.length);
		printf("Status: c1: %d/%d\n", c1.s.curr, c1.s.length);
		printf("Status: c2: %d/%d\n", c2.s.curr, c2.s.length);
		bool passed = server.s1.good() && server.s2.good();
		passed = passed && c1.s.good() && c2.s.good();
		if (passed) puts("--- Test Passed ---");
		else {
			printf("--- Test Failed ---\n");
			printf("VeryLong Data is saved to str1.txt and str2.txt\n");
			FILE* f1 = fopen("str1.txt", "w");
			fprintf(f1, "%s", str1);
			fclose(f1);
			FILE* f2 = fopen("str2.txt", "w");
			fprintf(f2, "%s", str2);
			fclose(f2);
		}
		return 0;
	}

	int test_hello() {
		puts("---- server bind and listen ----");
		NetworkAddress addr_server("127.0.0.1", 18133);
		Server server;
		server.bind_and_listen(addr_server);
		my_sleepms(100);

		puts("\n---- clients connect ----");
		Client c1, c2;
		c1.connect(addr_server);
		c2.connect(addr_server);
		my_sleepms(100);
		
		//puts("\n---- server accept and recv ----");
		server.accept_and_recv();
		printf("Client Num: %d\n", server.client_num());
		
		puts("\n---- client send message ----");
		const char* verylong = "asfasdfasvasdfwefvcvwrfwfadvadavsfwefwefwefwfweffweffwfwfasvasdwefef";
		int n = std::strlen(verylong);
		int cnt = 0;
		while (cnt < 1000) {
			int n1 = c1.send(verylong, n);
			int n2 = c2.send(verylong, n);
			if (n1 != n) {
				printf("c1 sent failure: %d != %d\n", n1, n); break;
			}
			if (n2 != n) {
				printf("c2 sent failure: %d != %d\n", n2, n);
			}

			cnt += n;
			//if (cnt % 10000 == 0) 
			printf("Sended to server: %d bytes.\n", cnt);
		}
		my_sleepms(100);
		

		puts("\n---- server accept_and_recv ----");
		server.accept_and_recv();
		my_sleepms(100);

		puts("\n---- client recv message ----");
		c1.recv_pending();
		c2.recv_pending();
		my_sleepms(100);

		puts("\n---- server accept_and_recv ----");
		server.accept_and_recv();
		my_sleepms(100);

		puts("\n---- client recv_pending ----");
		c1.recv_pending();
		c2.recv_pending();
		my_sleepms(100);

		printf("Done.\n");
		return 0;
	}

	int test_long_msg() {
		return 0;
	}

	int test_multiple_client() {
		return 0;
	}

	int test_reconnection() {
		return 0;
	}
}
