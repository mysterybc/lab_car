#include "proto_util.hpp"


namespace strproto {
	
int read(ReadView& buffer, int& ret) {
	if (buffer.empty()) return EmptyBuffer;
	
	const char* p = buffer.beg;
	const char* end = buffer.end;
	bool neg = false;
	if (*p == '-') {
		neg = true;
		++p;
	}
	else if (*p == '+') {
		neg = false;
		++p;
	}
	
	if (p == end) {
		return ReachEnd;
	}
	
	// Ensure the first char is a digit
	int c = *p - '0';
	if (c < 0 || c >= 10) {
		return Mismatch;
	}
	
	// Read the rest of the data, until a mismatch
	ret = -c; ++p;
	for (;p != end; ++p) {
		c = *p - '0';
		if (c >= 0 && c < 10) {
			ret *= 10;
			ret -= c;
		}
		else {
			// Only now count as a success reach
			buffer.beg = p;
			if (!neg) ret = -ret;
			return Good;
		}
	}
	
	// A partial result
	if (!neg) ret = -ret;
	return ReachEnd;
}

int read_match(ReadView& buffer, char c) {
	if (buffer.empty()) return EmptyBuffer;
	
	if (*buffer.beg == c) {
		++buffer.beg; 
		return Good;
	}
	return Mismatch;
}

int read_match(ReadView& buffer, const char* c, int size) {
	const char* p = buffer.beg;
	const char* end = buffer.end;
	for (; p < end && size > 0; ++p, ++c, --size) {
		if (*p != *c) {
			return Mismatch;
		}
	}
	if (size == 0) { 
		buffer.beg = p; return Good; 
	}
	return Mismatch;
}

int read_until(ReadView& buffer, char c) {
	const char* p = buffer.beg;
	const char* end = buffer.end;
	if (p == end) return ReachEnd;
	if (*p == c) return Good; 
	
	++p;
	for (; p < end; ++p) {
		if (*p == c) {
			buffer.beg = p;
			return Good;
		}
	}
	return ReachEnd;
}



/* ********** on int to char **********
 * max 32 bit int "-2147483648"
 *     length 11 (12 if counting \0)
 * for 64 bit int "-18446744073709551615" 
 *     len = 21 (22 if counting \0)
 **************************************/
int write(WriteView& buffer, int val) {
	if (val == 0) return write(buffer, '0');
	
	char temp[24];  // More than eough
	char* end = (char*)temp + 23;
	*end = '\0';
	
	bool neg = true;
	if (val > 0) {
		neg = false;
		val = -val; // int has more negative numbers than positive numbers
	}
	
	char* beg = end;
	while (val != 0) {
		--beg;             // ensure that when break out while temp[k] is the first digit
		int c = val % 10;  // since val == (val / 10) * 10 + (val % 10)
		                   // when val < 0, we have c = val % 10 < 0
		*beg = '0' - c;    // since c < 0, '0' - c is its char
		val /= 10;
	}
	if (neg) {
		--beg;
		*beg = '-';
	}
	return write(buffer, beg, end - beg);
}

int write(WriteView& buffer, char c) {
	if (buffer.size() > 0) {
		*buffer.beg = c;
		++buffer.beg;
		return Good;
	}
	return NotEnoughSpace;
}

int write(WriteView& buffer, const char* c, int size) {
	if (buffer.size() >= size) {
		for (int i=0;i<size;++i) {
			buffer.beg[i] = c[i];
		}
		buffer.beg += size;
		return Good;
	}
	return NotEnoughSpace;
}

} // namespace strproto