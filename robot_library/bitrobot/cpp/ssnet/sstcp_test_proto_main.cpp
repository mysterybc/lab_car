#ifdef BUILD_SSTCP_TEST_PROTO
#include "sstcp_proto.hpp"
#include "sstimer/sstimer.hpp"
#include <string>

namespace sstcp_proto_test {
	struct SerPar {
		SerializerBase* w;
		ParserBase* r;
	};
	struct PackData {
		PackData() : p_data(nullptr) {}
		PackageInfo info;
		void* p_data;
	};

	struct DataBase {
	public:
		// Purpose: The functional interface to manipulate the any data contained in PackData
		virtual int type_enum() const = 0;
		virtual bool pack_to(SerPar tobuf, WriteBuffer& buffer, PackData& pack) = 0;
		virtual bool pack_random(SerPar tobuf, WriteBuffer& buffer, PackData& pack) = 0; // Random init pack, then pack_to
		virtual bool unpack_and_check(SerPar tobuf, ReadBuffer& buffer, const PackData& stored) = 0;
		virtual void delete_data(PackData& pack) = 0;
	};

	template<class DataType> struct DataTypeEnum { enum { type = PackageInfo::PackUnknown }; };
#define fast_type_enum(name) template<> struct DataTypeEnum< name > { enum {type = PackageInfo::Pack##name }; }
	fast_type_enum(RobotCommand);
	fast_type_enum(RobotConfigMap);
	fast_type_enum(RobotConfigPath);
	fast_type_enum(RobotConfigState);
	fast_type_enum(RobotTeleMsg);
	fast_type_enum(RobotStateLog);
#undef fast_type_enum

	template<class DataType>
	struct OneData : public DataBase {
		// Purpose: The functional interface to manipulate the PackageData contained in PackData
		//          Implementation relies on the generic interface provided by 
		//          - SerializerBase, ParserBase, msgparser_test::random_data, DataTypeEnum
	public:
		enum { type = DataTypeEnum<DataType>::type };
		int type_enum() const { return type; }

		void delete_data(PackData& pack) {
			DataType* data = (DataType*)pack.p_data;
			delete data;
			pack.p_data = nullptr;
		}
		
		bool pack_to(SerPar tobuf, WriteBuffer& buffer, PackData& pack) {
			const DataType* data = (const DataType*)pack.p_data;
			int nWrite = tobuf.w->write(buffer, pack.info, *data);
			if (nWrite > 0) buffer.eat(nWrite);
			return nWrite > 0;
		}
		bool pack_random(SerPar tobuf, WriteBuffer& buffer, PackData& pack) {
			DataType* data = new DataType;
			pack.p_data = (void*)data;  // TODO: There is an memory leak ... PackData are copied arbitarily and not deleted
			msgparser_test::random_data(pack.info);
			msgparser_test::random_data(*data);

			return pack_to(tobuf, buffer, pack);
		}
		bool unpack_and_check(SerPar tobuf, ReadBuffer& buffer, const PackData& stored) {
			DataType data;
			PackageInfo info;
			int nRead = tobuf.r->read(buffer, info, data);
			if (nRead > 0) {
				return info == stored.info
					&& data == *((const DataType*)stored.p_data);
			}
			return false;
		}
	};
	OneData<RobotCommand> mRobotCommand;
	OneData<RobotConfigMap> mRobotConfigMap;
	OneData<RobotConfigPath> mRobotConfigPath;
	OneData<RobotConfigState> mRobotConfigState;
	OneData<RobotTeleMsg> mRobotTeleMsg;
	OneData<RobotStateLog> mRobotStateLog;


	struct OneItem {
		template<class DataType>
		bool get(PackageInfo& info, DataType& value) {
			if (fun && fun->type_enum() == DataTypeEnum<DataType>::type) {
				info = pack.info;
				value = *((DataType*)pack.p_data);
				return true;
			}
			return false;
		}
		bool write_to(SerializerBase* ser, WriteBuffer& buffer) {
			if (fun && pack.p_data) {
				SerPar tobuf; tobuf.w = ser;
				return fun->pack_to(tobuf, buffer, pack);
			}
			return false;
		}

		int choice_index;
		DataBase* fun;
		PackData  pack;
	};


	struct TestData {
		// Purpose: Generate a list of package (and its PackageInfo) whose wire size <= szSend
		//          Write the data into buf_send, which could then be sent (in different parts) to someone else
		TestData(int szSend = 10000)
			: buf_send(szSend, '\0') {}

		std::vector<DataBase*> get_choice_list() {
			std::vector<DataBase*> choice_list;
			choice_list.push_back(&mRobotCommand);
			choice_list.push_back(&mRobotConfigMap);
			choice_list.push_back(&mRobotConfigPath);
			choice_list.push_back(&mRobotConfigState);
			choice_list.push_back(&mRobotTeleMsg);
			choice_list.push_back(&mRobotStateLog);
			return choice_list;
		}

		int prepare_datalist(SerPar tobuf, std::vector<OneItem>& item_send, WriteBuffer bufw) {
			std::vector<DataBase*> choice_list = get_choice_list();
			item_send.clear();

			int nChoice = (int)choice_list.size();
			WriteBuffer buf_backup = bufw;
			while (true) {
				double r = msgparser_test::rand_u32();
				int k = int((double)nChoice * (r / UINT16_MAX) + 0.5);
				if (k < 0) k = 0;
				if (k >= nChoice) k = nChoice - 1;

				OneItem item;
				item.choice_index = k;
				item.fun = choice_list[k];
				if (choice_list[k]->pack_random(tobuf, bufw, item.pack)) {
					item_send.push_back(item);
					//printf("Packed %d/%d\n", buf_backup.size - bufw.size, buf_backup.size);
				}
				else break;
			}
			return buf_backup.size - bufw.size;
		}

		void prepare_data(SerPar tobuf, std::uint32_t seed) {
			msgparser_test::set_random_seed(seed);
			size_t n = buf_send.size();
			buf_send = std::string(n, 0);

			WriteBuffer bufw((char*)buf_send.data(), n);
			int nWrite = prepare_datalist(tobuf, item_list, bufw);
			buf_send.resize(nWrite);
		}


		std::string buf_send;
		std::vector<OneItem> item_list;
	};

	class TestHandler : public DataHandler {
		// Purpose: when a new data comes, check if it matches the data contained in to_match
		//          if it matches, send the reply contained in to_reply
	public:
		TestHandler() 
			: match_failed(false), curr_match(0), curr_reply(0), 
			  to_match(nullptr), to_reply(nullptr), serializer(nullptr) {}

		template<class DataType>
		void metaTest(const PackageInfo& info, const DataType& data, SendBuffer& reply) {
			PackageInfo info2; DataType data2;
			if (to_match && !match_failed) {
				if (curr_match >= (int)to_match->item_list.size()) {
					match_failed = true;
					return;
				}
				if (to_match->item_list[curr_match].get(info2, data2)) {
					if (info2 == info && data2 == data) {
						++curr_match;
						if (serializer && to_reply) {
							if (curr_reply >= (int)to_reply->item_list.size()) {
								return;
							}
							WriteBuffer buf(reply.end(), reply.length_free());
							if (to_reply->item_list[curr_reply].write_to(serializer, buf)) {
								int nwrite = buf.buffer - reply.end();
								reply.push(nwrite);
							}
							++curr_reply;
						}

						// Return now, it means the match is good
						return;
					}
					// puts("Data Value Mismatch");
				}
				// puts("Data Type Mismatch");
				// Match Failed
				match_failed = true;
			}
			// Match Already Failed
			// Do nothing
		}

		void onRobotStateLog(const PackageInfo& info, const RobotStateLog& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotCommand(const PackageInfo& info, const RobotCommand& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotTeleMsg(const PackageInfo& info, const RobotTeleMsg& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotConfigState(const PackageInfo& info, const RobotConfigState& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotConfigPath(const PackageInfo& info, const RobotConfigPath& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotConfigMap(const PackageInfo& info, const RobotConfigMap& data, SendBuffer& reply) { metaTest(info, data, reply); }

		TestHandler& set_match(TestData* data) { to_match = data; return *this; }
		TestHandler& set_send(TestData* data, SerializerBase* with) { to_reply = data; serializer = with;return *this; }

		bool all_matched() const { return to_match && curr_match == (int)to_match->item_list.size(); }
		bool all_sent() const { return to_reply && curr_reply == (int)to_reply->item_list.size(); }

		bool match_failed;
		int curr_match;
		int curr_reply;
		TestData* to_match;
		TestData* to_reply;
		SerializerBase* serializer;
	};
	class MultiTestHandler : public DataHandler {
	public:
		MultiTestHandler() : failed(false) {}
		template<class DataType>
		void metaTest(const PackageInfo& info, const DataType& data, SendBuffer& reply) {
			if (!failed) {
				for (size_t i = 0; i < subtest.size(); ++i) {
					TestHandler& one = subtest[i];
					one.metaTest(info, data, reply);
					if (one.match_failed) {
						// Its ok if one is failed
						one.match_failed = false;
					}
					else {
						// Find one, good
						return;
					}
				}
				// Now, no one can read this message...
				failed = true;
			}
		}

		void onRobotStateLog(const PackageInfo& info, const RobotStateLog& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotCommand(const PackageInfo& info, const RobotCommand& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotTeleMsg(const PackageInfo& info, const RobotTeleMsg& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotConfigState(const PackageInfo& info, const RobotConfigState& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotConfigPath(const PackageInfo& info, const RobotConfigPath& data, SendBuffer& reply) { metaTest(info, data, reply); }
		void onRobotConfigMap(const PackageInfo& info, const RobotConfigMap& data, SendBuffer& reply) { metaTest(info, data, reply); }

		bool failed;
		bool all_matched() const {
			for (size_t i = 0; i < subtest.size(); ++i) if (!subtest[i].all_matched()) return false;
			return true;
		}
		std::vector<TestHandler> subtest;
	};

	struct MainTest {
		struct TestConfig {
			TestConfig()
				: server_ip("127.0.0.1"), port_server(12313), 
				  is_server(true), is_c1(true), is_c2(true) {}
			const char* server_ip;
			int port_server;
			bool is_server;
			bool is_c1;
			bool is_c2;
		};
		
		int run(TestConfig config);
	};

	int MainTest::run(MainTest::TestConfig config) {
		// Generate Data
		SerPar tobuf;
		tobuf.w = &get_seralizer_binary();
		tobuf.r = &get_parser_binary();
		TestData ds(30000), d1(50000), d2(40000);
		ds.prepare_data(tobuf, 1234);
		d1.prepare_data(tobuf, 2341);
		d2.prepare_data(tobuf, 3412);

		// Config Test Handlers
		TestHandler hs1, hs2, h1, h2;
		hs1.set_match(&d1).set_send(&ds, tobuf.w);
		hs2.set_match(&d2).set_send(&ds, tobuf.w);
		h1.set_match(&ds).set_send(nullptr, nullptr);
		h2.set_match(&ds).set_send(nullptr, nullptr);

		MultiTestHandler hs;
		hs.subtest.push_back(hs1);
		hs.subtest.push_back(hs2);

		// Setup server clients
		ProtoServer s;
		ProtoClient c1, c2;
		s.set_proto_binary();
		s.set_handler(&hs);
		c1.set_proto_binary();
		c1.set_handler(&h1);
		c2.set_proto_binary();
		c2.set_handler(&h2);

		// Config Buffer Size
		// For client, ensure they can send all data at once
		// But server has to recv them in multiple steps (Test multi-recv and package split into different parts)
		c1.setsocketopt_sndbuf((int)d1.buf_send.size());
		c2.setsocketopt_sndbuf((int)d2.buf_send.size());
		s.setsocketopt_sndbuf((int)d1.buf_send.size() / 10);
		s.setsocketopt_rcvbuf(std::max((int)d1.buf_send.size() / 50, 50)); 


		// Estabilsh connections
		NetworkAddress addr_server(config.server_ip, config.port_server);
		if (config.is_server)
			s.bind_and_listen(addr_server);

		if (config.is_c1) {
			c1.connect(addr_server);
		}

		if (config.is_c2) {
			c2.connect(addr_server);
		}

		puts("Waiting Clients");
		if (config.is_server) {
			while (s.client_num() != 2) {
				s.accept_and_recv();
				sstimer::sleep_ms(100);
			}
		}

		puts("Connecting Server");
		if (config.is_c1) {
			while (!c1.is_connected()) { c1.connect(addr_server); sstimer::sleep_ms(10); }
		}

		if (config.is_c2) {
			while (!c2.is_connected()) { c2.connect(addr_server); sstimer::sleep_ms(10); }
		}

		// Now, the main loop
		bool failed = false;
		bool all_complete = true;

		int nRound = (int)ds.item_list.size();
		ReadBuffer bc1(d1.buf_send.data(), d1.buf_send.size());
		ReadBuffer bc2(d2.buf_send.data(), d2.buf_send.size());

		puts("Test Begin");
		for (int i = 0; i < nRound * 10; ++i) {
			if (config.is_c1) {
				size_t sz0 = d1.buf_send.size() / nRound + 1;
				size_t sz1 = std::min(sz0, bc1.size);
				int nsend = c1.send(bc1.buffer, sz1);
				bc1.eat(nsend);
				if (bc1.size == 0) {
					puts("C1 all sent");
				}
			}
			if (config.is_c2) {
				size_t sz0 = d2.buf_send.size() / nRound + 1;
				size_t sz1 = std::min(sz0, bc2.size);
				int nsend = c2.send(bc2.buffer, sz1);
				bc2.eat(nsend);
				if (bc2.size == 0) {
					puts("C2 all sent");
				}
			}

			failed = false;
			all_complete = true;
			printf("--- Loop %d ---\n", i + 1);
			if (config.is_server) {
				s.accept_and_recv();
				printf("[s1] %d/%d\n", hs.subtest[0].curr_match, (int)hs.subtest[0].to_match->item_list.size());
				printf("[s2] %d/%d\n", hs.subtest[1].curr_match, (int)hs.subtest[1].to_match->item_list.size());
				if (!hs.all_matched()) all_complete = false;
				if (hs.failed) { failed = true; puts("[s] failed"); }
			}
			if (config.is_c1) {
				c1.recv_pending();
				printf("[c1] %d/%d\n", h1.curr_match, (int)h1.to_match->item_list.size());
				if (!h1.all_matched()) all_complete = false;
				if (h1.match_failed) { failed = true; puts("[c1] failed"); }
			}
			if (config.is_c2) {
				c2.recv_pending();
				printf("[c2] %d/%d\n", h2.curr_match, (int)h2.to_match->item_list.size());
				if (!h2.all_matched()) all_complete = false;
				if (h2.match_failed) { failed = true; puts("[c2] failed"); }
			}
			if (all_complete || failed) break;
		}
		if (all_complete) puts("\n--- Test Passed ---\n");

		bool all_sent = false;
		while (!all_sent) {
			all_sent = true;
			if (config.is_c1 && bc1.size > 0) {
				all_sent = false;
				int nsend = c1.send(bc1.buffer, bc1.size);
				bc1.eat(nsend);
				if (bc1.size == 0) {
					puts("C1 all sent");
				}
			}
			if (config.is_c2 && bc2.size > 0) {
				all_sent = false;
				int nsend = c2.send(bc2.buffer, bc2.size);
				bc2.eat(nsend);
				if (bc2.size == 0) {
					puts("C2 all sent");
				}
			}
		}
		
		sstimer::sleep_ms(100);

		return all_complete ? 0 : -1;
	}

} // namespace sstcp_proto_test


int main(int argc, char* argv[]) {
	sstcp_initialization();
	sstcp_proto_test::MainTest test;
	sstcp_proto_test::MainTest::TestConfig config;
	
	if (argc >= 2) {
		// Read role str
		std::string role(argv[1]);
		config.is_server = role.find('s') != std::string::npos;
		config.is_c1 = role.find('1') != std::string::npos;
		config.is_c2 = role.find('2') != std::string::npos;

		if (argc == 3) {
			config.server_ip = argv[2];
		}
	}
	config.port_server = 1778;

	int ret = test.run(config);
	sstcp_cleanup();
	return ret;
}
#endif