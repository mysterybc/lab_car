#pragma once
#include "../compat.hpp"
#include <vector>


template<int lb, int ub>
struct RangeInt {
	enum { LowB = lb, UpB = ub };
	
	RangeInt(int v = 0): value(v) {}
	bool valid() const { return value >= lb && value <= ub; }
	int& operator() () { return value; }
	const int& operator() () const { return value; }
	operator int() const { return value; }
	
	bool operator == (const RangeInt& it) const { return value == it.value; }
	bool operator != (const RangeInt& it) const { return value != it.value; }
	bool operator > (const RangeInt& it)  const { return value > it.value;  }
	bool operator >= (const RangeInt& it) const { return value >= it.value; }
	bool operator < (const RangeInt& it)  const { return value < it.value;  }
	bool operator <= (const RangeInt& it) const { return value <= it.value; }
	
	int value;
};


template<class T>
struct StrView {
	typedef T* PointerT;
	
	StrView(PointerT beg = nullptr, PointerT end = nullptr) 
		: beg(beg), end(end) {}
	StrView(PointerT beg, int size) 
		: beg(beg), end(beg + size) {}
	
	T& operator [] (int k) { return beg[k]; }
	const T& operator [] (int k) const { return beg[k]; }
	int size() const { return end - beg; }
	bool empty() const { return beg == end; }
	
	PointerT beg;
	PointerT end;
};
typedef StrView<char> WriteView;
typedef StrView<const char> ReadView;


namespace strproto {

enum ErrorCode {
	Good = 0,
	ReachEnd,	    // read error
	Mismatch,		// read error
	EmptyBuffer,	// read error
	NotEnoughSpace  // write error
};
int read(ReadView& buffer, int& ret);
int read_match(ReadView& buffer, char c);
int read_match(ReadView& buffer, const char* c, int size);
int read_until(ReadView& buffer, char c);

int write(WriteView& buffer, int val);
int write(WriteView& buffer, char c);
int write(WriteView& buffer, const char* c, int size);


struct ReadUtil {
	ReadUtil(const char* data, int size): buffer(data, size) { reset(); }
	ReadUtil(ReadView buffer): buffer(buffer) { reset();}
	void reset() { n_success = 0; all_good = true; last_error = Good; }
	operator bool () const { return all_good; }
	bool operator !() const { return !all_good; }
	
	ReadUtil& read(int& ret) {
		if (all_good) {
			last_error = strproto::read(buffer, ret);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	ReadUtil& match(char c) {
		if (all_good) {
			last_error = strproto::read_match(buffer, c);
			all_good = last_error == Good;
		}
		return *this;
	}
	ReadUtil& match(const char* c, int size) {
		if (all_good) {
			last_error = strproto::read_match(buffer, c, size);
			all_good = last_error == Good;
		}
		return *this;
	}
	ReadUtil& read_until(char c) {
		if (all_good) {
			last_error = strproto::read_until(buffer, c);
			all_good = last_error == Good;
		}
		return *this;
	}
	
	// Comma Sep Int Utils
	ReadUtil& read_ints(int& a, int& b, char sep) {
		return read(a).match(sep).read(b);
	}
	ReadUtil& read_ints(int& a, int& b, int& c, char sep) {
		return read_ints(a, b, sep).match(sep).read(c);
	}
	ReadUtil& read_ints(int& a, int& b, int& c, int& d, char sep) {
		return read_ints(a, b, c, sep).match(sep).read(d);
	}
	ReadUtil& read_ints(int& a, int& b, int& c, int& d, int& e, char sep) {
		return read_ints(a, b, c, d, sep).match(sep).read(e);
	}
	ReadUtil& read_ints(int& a, int& b, int& c, int& d, int& e, int& f, char sep) {
		return read_ints(a, b, c, d, e, sep).match(sep).read(f);
	}
	
	ReadView buffer;
	bool all_good;
	int n_success;  // how many ints read, not counting matches
	int last_error;
};



struct WriteUtil {
	WriteUtil(char* data, int size): buffer(data, size) { reset(); }
	WriteUtil(WriteView buffer): buffer(buffer) { reset();}
	void reset() { n_success = 0; all_good = true; last_error = Good; }
	operator bool () const { return all_good; }
	bool operator !() const { return !all_good; }

	WriteUtil& write(int value) {
		if (all_good) {
			last_error = strproto::write(buffer, value);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	WriteUtil& write(char c) {
		if (all_good) {
			last_error = strproto::write(buffer, c);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	WriteUtil& write(const char* c, int size) {
		if (all_good) {
			last_error = strproto::write(buffer, c, size);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	
	// Comma Sep Int Utils
	// Well, we dont have 
	WriteUtil& write_ints(int a, int b, char sep) {
		return write(a).write(sep).write(b);
	}
	WriteUtil& write_ints(int a, int b, int c, char sep) {
		return write_ints(a, b, sep).write(sep).write(c);
	}
	WriteUtil& write_ints(int a, int b, int c, int d, char sep) {
		return write_ints(a, b, c, sep).write(sep).write(d);
	}
	WriteUtil& write_ints(int a, int b, int c, int d, int e, char sep) {
		return write_ints(a, b, c, d, sep).write(sep).write(e);
	}
	WriteUtil& write_ints(int a, int b, int c, int d, int e, int f, char sep) {
		return write_ints(a, b, c, d, e, sep).write(sep).write(f);
	}
	
	WriteView buffer;
	bool all_good;
	int n_success;
	int last_error;
};



} // namespace strproto

namespace binproto {

enum ErrorCode {
	Good = 0,
	ReachEnd,	    // read error
	Mismatch,		// read error
	EmptyBuffer,	// read error
	NotEnoughSpace  // write error
};

template<class BasicType>
int read(ReadView& buffer, BasicType& ret) {
	if (buffer.size() >= sizeof(BasicType)) {
		ret = *static_cast<const BasicType*>(buffer.beg);
		buffer.beg += sizeof(BasicType);
		return Good;
	}
	return buffer.size() > 0 ? ReachEnd : EmptyBuffer;
}

template<class BasicType>
int read_fixedsize(ReadView& buffer, BasicType* pOut, unsigned size) {
	unsigned len = size * sizeof(BasicType);
	if ((unsigned)buffer.size() >= len) {
		BasicType* pIn = static_cast<BasicType*>(buffer.beg);
		for (unsigned i = 0; i < size; ++i) {
			pOut[i] = pIn[i];
		}
		buffer.beg += len;
		return Good;
	}
	return buffer.size() > 0 ? ReachEnd : EmptyBuffer;
}

template<class BasicType>
int read_stdvec(ReadView& buffer, std::vector<BasicType> pOut) {
	if (buffer.size() >= sizeof(std::uint16_t)) {
		ReadView tmp(buffer.beg, buffer.end);
		std::uint16_t size;
		if (read(tmp, size) == Good) {
			unsigned len = size * sizeof(BasicType);
			if (tmp.size() >= len) {
				pOut.resize(size);
				read_fixedsize(tmp, &pOut[0], size);
				buffer.beg += len + sizeof(size);
				return Good;
			}
			return ReachEnd;
		}
	}
	return buffer.size() > 0 ? ReachEnd : EmptyBuffer;
}

template<class BasicType>
int read_match(ReadView& buffer, const BasicType& c) {
	if (buffer.size() >= sizeof(BasicType)) {
		BasicType tmp = *static_cast<const BasicType*>(buffer.beg);
		if (tmp == c) {
			buffer.beg += sizeof(BasicType);
			return Good;
		}
		return Mismatch;
	}
	return buffer.size() > 0 ? ReachEnd : EmptyBuffer;
}

template<class BasicType>
int read_until(ReadView& buffer, const BasicType& c) {
	int len = buffer.size() - sizeof(BasicType);
	for (int i = 0; i < len; ++i) {
		ReadView tmp(buffer.beg + i, buffer.end);
		if (read_match(tmp, c) == Good) {
			buffer.beg += i;
			return Good;
		}
	}
	return Mismatch;
}

template<class BasicType>
int write(WriteView& buffer, const BasicType& value) {
	if (buffer.size() >= sizeof(BasicType)) {
		*static_cast<BasicType*>(buffer.beg) = value;
		buffer.beg += sizeof(BasicType);
		return Good;
	}
	return NotEnoughSpace;
}

template<class BasicType>
int write_fixedsize(WriteView& buffer, const BasicType* pValue, unsigned size) {
	int len = sizeof(BasicType) * size;
	if (buffer.size() >= len) {
		BasicType* pOut = static_cast<BasicType*>(buffer.beg);
		for (unsigned i = 0; i < size; ++i) {
			pOut[i] = pValue[i];
		}
		buffer.beg += len;
		return Good;
	}
	return NotEnoughSpace;
}

template<class BasicType>
int write_stdvec(WriteView& buffer, const std::vector<BasicType>& value) {
	if (value.size() >= 65536) return NotEnoughSpace;
	std::uint16_t size = static_cast<std::uint16_t>(value.size());
	std::uint32_t len = size * sizeof(BasicType) + sizeof(size);
	if (buffer.size() >= len) {
		write(buffer, size);
		write_fixedsize(buffer, &value[0], size);
		return Good;
	}
	return NotEnoughSpace;
}


struct ReadUtil {
	ReadUtil(const char* data, int size) : buffer(data, size) { reset(); }
	ReadUtil(ReadView buffer) : buffer(buffer) { reset(); }
	void reset() { n_success = 0; all_good = true; last_error = Good; }
	operator bool() const { return all_good; }
	bool operator !() const { return !all_good; }

	template<class BasicType>
	ReadUtil& read(BasicType& ret) {
		if (all_good) {
			last_error = binproto::read(buffer, ret);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	template<class BasicType>
	ReadUtil& read_fixedsize(BasicType* pOut, unsigned size) {
		if (all_good) {
			last_error = binproto::read_fixedsize(buffer, pOut, size);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	template<class BasicType>
	ReadUtil& read_stdvec(std::vector<BasicType>& vec) {
		if (all_good) {
			last_error = binproto::read_stdvec(buffer, vec);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}

	template<class BasicType>
	ReadUtil& match(const BasicType& c) {
		if (all_good) {
			last_error = binproto::read_match(buffer, c);
			all_good = last_error == Good;
		}
		return *this;
	}

	template<class BasicType>
	ReadUtil& read_until(const BasicType& c) {
		if (all_good) {
			last_error = binproto::read_until(buffer, c);
			all_good = last_error == Good;
		}
		return *this;
	}

	ReadView buffer;
	bool all_good;
	int n_success;  // how many ints read, not counting matches
	int last_error;
};


struct WriteUtil {
	WriteUtil(char* data, int size) : buffer(data, size) { reset(); }
	WriteUtil(WriteView buffer) : buffer(buffer) { reset(); }
	void reset() { n_success = 0; all_good = true; last_error = Good; }
	operator bool() const { return all_good; }
	bool operator !() const { return !all_good; }

	template<class BasicType>
	WriteUtil& write(const BasicType& value) {
		if (all_good) {
			last_error = binproto::write(buffer, value);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	template<class BasicType>
	WriteUtil& write_fixedsize(const BasicType* pOut, unsigned size) {
		if (all_good) {
			last_error = binproto::write_fixedsize(buffer, pOut, size);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	template<class BasicType>
	WriteUtil& write_stdvec(const std::vector<BasicType>& out) {
		if (all_good) {
			last_error = binproto::write_stdvec(buffer, out);
			all_good = last_error == Good;
			if (all_good) ++n_success;
		}
		return *this;
	}
	
	WriteView buffer;
	bool all_good;
	int n_success;
	int last_error;
};

} // namespace binproto
