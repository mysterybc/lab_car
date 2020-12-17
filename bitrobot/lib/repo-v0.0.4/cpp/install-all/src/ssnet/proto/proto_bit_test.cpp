#ifdef BUILD_PROTO_BIT_TEST
#include "proto_bit.hpp"
#include <iostream>
#include <string>
#include <vector>

using namespace std;

int main() {
	vector<PackBase*> packs;
	for (int i = protobit::PackBegin; i < protobit::PackEnd; ++i) {
		packs.push_back(protobit::new_pack(i));
	}
	
	char buffer[10001];
	memset((char*)buffer, 0, 10001);
	while (cin.getline((char*)buffer, 10000, '\n')) {
		if (strcmp(buffer, "exit") == 0) return 0;
		
		int nread = (int)cin.gcount();
		buffer[nread - 1] = '\n';
		buffer[nread] = '\0';
		for (unsigned i=0;i<packs.size();++i) {
			if (packs[i]->read(buffer, nread) != -1) {
				printf("Input is type %s\n", packs[i]->pack_name());
				packs[i]->print_data(stdout, "");
				
				if (packs[i]->valid()) {
					// Try Encode it Myself	
					memset((char*)buffer, 0, 10001);
					int nwrite = packs[i]->write((char*)buffer, nread);
					if (nwrite == -1) {
						printf("Failed to encode data myself\n");
					}
					else {
						buffer[nwrite] = 0;
						printf("My Encoded Result is:\n%s", buffer);
						
						// Try Decode What I Encode
						PackBase* tmp = protobit::new_pack(packs[i]->packID());
						if (tmp->read((char*)buffer, nwrite) != -1) {
							if (*tmp == *packs[i]) {
								printf("-- Data Matched --\n");
							}
							else {
								printf("Decode Error, Data Mismatched\n");
								printf("What I Decoded is as follows\n");
								tmp->print_data(stdout, "");
							}
						}
						else {
							printf("Failed to decode what I encoded\n");
						}
						puts("");
						delete tmp;
					}
				}
			}
			else {
				int err = packs[i]->error();
				if (err == PackBase::PackHeadError) {
					// This is the normal case
				}
				if (err == PackBase::PackEndError) {
					printf("Error: PackEndError at parsing %s\n", packs[i]->pack_name());
				}
				if (err == PackBase::PackIncomplete) {
					printf("Error:PackIncomplete at parsing %s\n", packs[i]->pack_name());
				}
				if (err == PackBase::PackDataError) {
					printf("Error: PackDataError at parsing %s\n", packs[i]->pack_name());
				}
			}
		}
	}
	
	return 0;
}




#endif