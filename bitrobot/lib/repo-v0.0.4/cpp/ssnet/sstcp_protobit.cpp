#include "sstcp_protobit.hpp"
#include "proto/proto_util.hpp"
#include <cstdio>

namespace protobit {


bool BITServerClient::impl_handle_data(RecvBuffer& data, SendBuffer& reply_buffer) {
	// Find pack begin
	strproto::ReadUtil frombuf(data.begin(), data.length());
	frombuf.read_until('#');
	if (!frombuf.all_good) {
		// Fail to find pack start, wait for more data
		data.clear();
		return false;
	}
	int n1 = frombuf.buffer.beg - data.begin();
	
	frombuf.read_until('\n');
	if (!frombuf.all_good) {
		// Fail to find pack end, waiting for more data
		return false;
	}
	int n2 = frombuf.buffer.beg - data.begin();
	data.eat(n1);
	int length = n2 - n1 + 1;
	
	int npacks = (int)parser.pack.size();
	for (int i=0;i<npacks;++i) {
		PackBase* pack = parser.pack[i];
		int nread = pack->read(data.begin(), length);
		if (nread == length) {
			// This is it
			if (onData) onData->onPack(pack, reply_buffer);
			data.eat(nread);
			
			// call this function again if the remaining buffer is not empty
			return true;
		}
		else {
			int error = pack->error();
			if (error == PackBase::PackHeadError) {
				// Do nothing, this is normal
			}
			else {
				// This Package is either damaged
				if (log_file) {
					if (nread > 0)
						fprintf(log_file, "Error, nread > 0 and nread != length\n");
					else
						fprintf(log_file, "Error, get invalid packet\n");
				}
				data.eat(length);

				// call this function again if the remaining buffer is not empty
				return true;
			}
		}
	}
	
	// Unrecognized pack
	if (log_file) {
		fprintf(log_file, "Error, Unkown Packet, discared.\n");
	}
	data.eat(length);
	return true;
}

void DataHandler::onPack(PackBase* pack, SendBuffer& reply) {
	if (!pack) return;
#define __impl_handler_caseline(name) case Pack##name: return on##name(*((name*)pack), reply)
	int packID = pack->packID();
	switch (packID) {
		__impl_handler_caseline(STAT);
		__impl_handler_caseline(COMD);
		__impl_handler_caseline(PATH);
		__impl_handler_caseline(TURN);
		__impl_handler_caseline(OBST);
		__impl_handler_caseline(TELE);
		__impl_handler_caseline(TALG);
		__impl_handler_caseline(TDEG);
		__impl_handler_caseline(FORM);
		__impl_handler_caseline(EIGT);
		__impl_handler_caseline(ALGO);
		__impl_handler_caseline(PLAN);
		__impl_handler_caseline(MPLN);
#undef __impl_handler_caseline
	default: return;
	}
}

	
} // namespace protobit
