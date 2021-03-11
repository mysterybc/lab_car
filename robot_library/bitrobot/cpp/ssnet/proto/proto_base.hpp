#pragma once
#include <cstdio>
#include "../compat.hpp"

class PackBase {
public:
	enum {
		NoError = 0,
		PackHeadError,
		PackEndError, 
		PackIncomplete,
		PackDataError,
		BufferTooSmall,
		DataInvalid
	};
	PackBase(): last_error(NoError) {}
	virtual ~PackBase() {}
	
	// Help user to identify the 
	virtual int packID() const  = 0;
	virtual const char* pack_name() const = 0;
	virtual void print_data(std::FILE* out, const char* line_prefix = "") const {}
	virtual void reset() = 0;

	// Return if the data of this package is valid
	virtual bool valid() const = 0;
	virtual bool operator == (const PackBase& other) const = 0;
	bool operator != (const PackBase& other) const {
		return !((*this) == other);
	}
	
	// Return number of bytes written
	// on failure, write nothing, return -1, set last_error
	virtual int write(char* buffer, int size) = 0;
	
	// Return number of bytes read and store inside this pack
	// on failure, return -1, set last_error
	virtual int read(const char* buffer, int size) = 0;
	
	// The last read/write error occured
	int error() const { return last_error; }
	
	// Printing the information to an output file
	
	
protected:
	int last_error;
};






