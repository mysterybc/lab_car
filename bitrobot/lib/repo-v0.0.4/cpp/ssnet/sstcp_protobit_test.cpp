#ifdef BUILD_SSTCP_PROTOBIT_TEST
#include "sstcp_protobit.hpp"
#include "sstimer/sstimer.hpp"
#include <iostream>

struct Handler: public protobit::DataHandler {
	Handler(const char* name = ""): line_prefix(name) {}
	void onPack(const PackBase* pack, SendBuffer& reply) {
		pack->print_data(stdout, line_prefix);
	}
	
	const char* line_prefix;
};

int main_server() {
	protobit::BITServer server;
	Handler serverhandler("server");
	server.onData = &serverhandler;

	protobit::STAT stat;
	stat.id = 1;
	stat.x = 100;
	stat.y = 900;
	stat.th = 180;
	stat.v = 30;
	stat.w = 40;
	stat.progress = 13;
	stat.type = 0;
	stat.status = 1;
	char buf[1024];
	int len = stat.write(buf, 1024);

	server.bind_and_listen(NetworkAddress("127.0.0.1", 11321));
	for (int i = 0;;++i) {
		server.accept_and_recv();
		if (i % 50 == 0) {
			server.sendtoall((char*)buf, len);
		}
		sstimer::sleep_ms(100);
	}

	sstcp_cleanup();
	return 0;
}

int main_client(const char* client_name) {
	protobit::BITClient client;
	Handler clienthandler(client_name);
	client.onData = &clienthandler;
	client.connect(NetworkAddress("127.0.0.1", 11321));
	
	while (!client.is_connected()) {
		sstimer::sleep_ms(200);
		client.connect(NetworkAddress("127.0.0.1", 11321));
	}
	printf("Connected\n");

	char buffer[5001];
	while (true) {
		if (std::cin.getline((char*)buffer, 5000, '\n')) {
			int nget = (int)std::cin.gcount();
			buffer[nget - 1] = '\n';
			buffer[nget] = 0;
			client.send((char*)buffer, nget);
			sstimer::sleep_ms(200);

			client.recv_pending();
		}
		client.recv_pending();
		sstimer::sleep_ms(100);
	}

	sstcp_cleanup();
	return  0;
}

int main(int argc, char* argv[]) {
	sstcp_initialization();
	if (argc == 1) return main_server();
	return main_client(argv[1]);
}









#endif