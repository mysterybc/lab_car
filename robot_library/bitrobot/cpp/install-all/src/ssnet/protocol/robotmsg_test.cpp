#include "msgparser.hpp"
#include <cstdio>

namespace msgparser_test {
	struct TestingSuite {
		SerializerBase* ser;
		ParserBase* par;
		const char* data_name;
		void perror(const char* test_name, const char* err_name) {
			printf("Error at Test %s, Data %s: %s\n", test_name, data_name, err_name);
		}
		void ppass(const char* test_name) {
			printf("--- Passed Test %s, Data = %s ---\n", test_name, data_name);
		}
	};
	
	
	template<class T>
	bool test_basic(TestingSuite suit, T& data, PackageInfo& info) {
		char buffer[5000];
		int buffer_max = 5000, buffer_pad = 50;
		char pad_char = '+';
		for (int i = 0; i < buffer_max; ++i)
			buffer[i] = pad_char;

		WriteBuffer bufw((char*)buffer + buffer_pad, buffer_max - 2*buffer_pad);
		
		int nWrite = suit.ser->write(bufw, info, data);
		if (nWrite <= 0) {
			suit.perror("Basic", "Write Failed"); return false;
		}
		
		for (int i=0;i<buffer_pad;++i) {
			if (buffer[i] != pad_char) {
				suit.perror("Basic", "Write Overflow (Forward)"); return false;
			}
		}
		for (int i=buffer_pad+nWrite; i<buffer_max;++i) {
			if (buffer[i] != pad_char) {
				suit.perror("Basic", "Write Overflow (Backward)"); return false;
			}
		}
		
		PackageInfo info2;
		T data2;
		ReadBuffer bufr((char*)buffer + buffer_pad, nWrite);
		int nRead = suit.par->read(bufr, info2, data2);
		if (nRead <= 0) {
			suit.perror("Basic", "Read Error"); 
			return false;
		}
		if (info != info2) { 
			suit.perror("Basic", "InfoMismatch"); return false;
		}
		if (data != data2) { 
			suit.perror("Basic", "DataMismatch"); return false;
		}
		
		ReadBuffer bufr2((char*)buffer + buffer_pad, nWrite-1);
		nRead = suit.par->read(bufr2, info2, data2);
		if (nRead > 0) {
			suit.perror("Basic", "Read Overflow"); return false;
		}
		
		return true;
	}
	
	template<class T>
	bool test_parse(TestingSuite suit, T& data, PackageInfo& info) {
		char buffer[8000];
		int buffer_max = 8000, buffer_pad = 100;
		char pad_char = '+';
		for (int i = 0; i < buffer_max; ++i)
			buffer[i] = pad_char;
		
		WriteBuffer bufr((char*)buffer+buffer_pad, buffer_max);
		int nWrite = suit.ser->write(bufr, info, data);
		if (nWrite <= 0) {
			suit.perror("Parse", "Write Error");
			return false;
		}
		bufr.eat((std::size_t)nWrite);
		nWrite = suit.ser->write(bufr, info, data);
		if (nWrite <= 0) {
			suit.perror("Parse", "Write Error (Second)");
			return false;
		}

		{
			ReadBuffer buf1((char*)buffer, buffer_max);
			//int k = suit.par->find_packheader(buf1);
			PackageInfo tmp;
			int k = suit.par->find_packet(buf1, tmp);
			if (k != buffer_pad) {
				suit.perror("Parse", "Placeholder Error");
				return false;
			}
			buf1.eat((std::size_t)k);

			T data2; PackageInfo info2;
			int nRead = suit.par->read(buf1, info2, data2);
			if (nRead <= 0) {
				suit.perror("Parse", "Read Error");
				return false;
			}
			if (info2 != info) {
				suit.perror("Parse", "InfoMismatch");
				return false;
			}
			if (data2 != data) {
				suit.perror("Parse", "DataMismatch");
				return false;
			}

			
			buf1.eat(1);
			k = suit.par->find_packet(buf1, tmp);
			if (k != nRead - 1) {
				suit.perror("Parse", "Placeholder Error (Second Part)");
				return false;
			}
			buf1.eat(k);
			

			nRead = suit.par->read(buf1, info2, data2);
			if (nRead <= 0) {
				suit.perror("Parse", "Read Error (Second Part)");
				return false;
			}
			if (info2 != info) {
				suit.perror("Parse", "InfoMismatch (Second Part)");
				return false;
			}
			if (data2 != data) {
				suit.perror("Parse", "DataMismatch (Second Part)");
				return false;
			}
		}
		return true;
	}

	
	template<class T>
	bool test_batch_one(TestingSuite suit, int nTest) {
		for (int i=0;i<nTest; ++i) {
			PackageInfo info;
			T data;
			random_data(info);
			random_data(data);
			if (!test_basic(suit, data, info)) return false;
		}
		suit.ppass("Basic");

		for (int i = 0; i < nTest; ++i) {
			PackageInfo info;
			T data;
			random_data(info);
			random_data(data);
			if (!test_parse(suit, data, info)) return false;
		}
		suit.ppass("Parse");
		return true;
	}
	
	int test_all(TestingSuite suit, int nTest) {
		suit.data_name = "RobotStateLog";
		if (!test_batch_one<RobotStateLog>(suit, nTest)) return -1;
		
		suit.data_name = "RobotCommand";
		if (!test_batch_one<RobotCommand>(suit, nTest)) return -1;
		
		suit.data_name = "RobotTeleMsg";
		if (!test_batch_one<RobotTeleMsg>(suit, nTest)) return -1;
		
		suit.data_name = "RobotConfigState";
		if (!test_batch_one<RobotConfigState>(suit, nTest)) return -1;
		
		suit.data_name = "RobotConfigPath";
		if (!test_batch_one<RobotConfigPath>(suit, nTest)) return -1;
		
		suit.data_name = "RobotConfigMap";
		if (!test_batch_one<RobotConfigMap>(suit, nTest)) return -1;
		return 0;
	}
	
	int test_binary_method() {
		TestingSuite suit;
		suit.ser = &get_seralizer_binary();
		suit.par = &get_parser_binary();
		return test_all(suit, 10);
	}
	
	
	int test_json_method() { 
		//TestingSuite suit;
		//suit.ser = &get_seralizer_json();
		//suit.par = &get_parser_json();
		//return test_all(suit, 10);
		return 0;
	}
	
}