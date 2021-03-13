#pragma once
#include "compat.hpp"
#include "proto/proto_bit.hpp"
#include "sstcp.hpp"

namespace protobit {

struct DataHandler {
	virtual void onPack(PackBase* pack, SendBuffer& reply);
	virtual void onSTAT(STAT& data, SendBuffer& reply) {}
	virtual void onCOMD(COMD& data, SendBuffer& reply) {}
	virtual void onPATH(PATH& data, SendBuffer& reply) {}
	virtual void onTURN(TURN& data, SendBuffer& reply) {}
	virtual void onOBST(OBST& data, SendBuffer& reply) {}
	virtual void onTELE(TELE& data, SendBuffer& reply) {}
	virtual void onTALG(TALG& data, SendBuffer& reply) {}
	virtual void onTDEG(TDEG& data, SendBuffer& reply) {}
	virtual void onFORM(FORM& data, SendBuffer& reply) {}
	virtual void onEIGT(EIGT& data, SendBuffer& reply) {}
	virtual void onALGO(ALGO& data, SendBuffer& reply) {}
	virtual void onPLAN(PLAN& data, SendBuffer& reply) {}
	virtual void onMPLN(MPLN& data, SendBuffer& reply) {}
};

struct PackList {
	PackList() {
		for (int i=PackBegin; i<PackEnd; ++i) {
			pack.push_back(new_pack(i));
		}
	}
	~PackList() {
		for (unsigned i=0;i<pack.size();++i) {
			if (pack[i]) { delete pack[i]; pack[i] = nullptr; }
		}
	}
	
	std::vector<PackBase*> pack;
}; 

struct BITServerClient {
	BITServerClient(): onData(nullptr), log_file(nullptr) {}
	
	DataHandler* onData;
	FILE* log_file;
	PackList parser;

protected:
	bool impl_handle_data(RecvBuffer& data, SendBuffer& reply_buffer);
};


class BITServer: public TCPServer, public BITServerClient {
public:
	~BITServer() {}
	
protected:
	void handle_accept_with_reply(const NetworkAddress& addr, SendBuffer& buffer) {}
	bool handle_data_with_reply(RecvBuffer& data, SendBuffer& reply) {
		return impl_handle_data(data, reply);
	}
};


class BITClient: public TCPClient, public BITServerClient {
public:
	~BITClient() {}

protected:
	bool handle_data(RecvBuffer& data) {
		static SendBuffer unused(0);
		return impl_handle_data(data, unused);
	}
};

	
	
} // namespace protobit