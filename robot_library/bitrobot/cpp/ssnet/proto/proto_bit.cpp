#include "proto_bit.hpp"
#include <cstdio>
#include <cstdlib>
#include <algorithm>

namespace protobit {
	
/**************************
  ProtoBIT Implementations 
 **************************/
/**************************
     PackCreation By ID
 **************************/
PackBase* new_pack(int packID) {
	switch (packID) {
	case PackSTAT: return new STAT();
	case PackCOMD: return new COMD();
	case PackPATH: return new PATH();
	case PackTURN: return new TURN();
	case PackOBST: return new OBST();
	case PackTELE: return new TELE();
	case PackTALG: return new TALG();
	case PackTDEG: return new TDEG();
	case PackFORM: return new FORM();
	case PackEIGT: return new EIGT();
	case PackALGO: return new ALGO();
	case PackPLAN: return new PLAN();
	case PackMPLN: return new MPLN();
	default: return nullptr;
	}
}

/**************************
     Utility Functions
 **************************/

#define __valid_check2(a, b) ( a.valid() && b.valid() )
#define __valid_check3(a, b, c) ( a.valid() && b.valid() && c.valid() )
#define __valid_check4(a, b, c, d) ( __valid_check3(a, b, c) && d.valid() )
#define __valid_check5(a, b, c, d, e) ( __valid_check4(a, b, c, d) && e.valid() )
#define __valid_check6(a, b, c, d, e, f) ( __valid_check5(a, b, c, d, e) && f.valid() )
#define __valid_check7(a, b, c, d, e, f, g) ( __valid_check6(a, b, c, d, e, f) && g.valid() )
#define __resize2(n, a, b) do { a.resize(n); b.resize(n);  } while(0);
#define __resize3(n, a, b, c) do { a.resize(n); b.resize(n); c.resize(n);  } while(0);
#define __resize4(n, a, b, c, d) do { a.resize(n); b.resize(n); c.resize(n); d.resize(n); } while(0);
#define __equal_it2(a, b) ( a == it.a && b == it.b )
#define __equal_it3(a, b, c) ( __equal_it2(a, b) && c == it.c )
#define __equal_it4(a, b, c, d) ( __equal_it3(a, b, c) && d == it.d )
#define __equal_it5(a, b, c, d, e) ( __equal_it4(a, b, c, d) && e == it.e )
#define __equal_it6(a, b, c, d, e, f) ( __equal_it5(a, b, c, d, e) && f == it.f )
#define __equal_it7(a, b, c, d, e, f, g) ( __equal_it6(a, b, c, d, e, f) && g == it.g )

int write_end_routine(strproto::WriteUtil& tobuf, char* buffer, int& last_error) {
	if (tobuf.all_good) {
		// This is how many bytes written to buffer
		return tobuf.buffer.beg - buffer;;
	}
	
	last_error = PackBase::BufferTooSmall;
	return -1;
}

int check_end_routine(strproto::ReadUtil& frombuf, const char* buffer, int& last_error) {
	if (!frombuf.all_good) {
		if (frombuf.last_error == strproto::Mismatch) {
			last_error = PackBase::PackDataError;
			return -1;
		}
		last_error = PackBase::PackIncomplete;
		return -1;
	}
		   
	frombuf.match('\n');
	if (!frombuf.all_good) {
		if (frombuf.last_error == strproto::Mismatch) {
			last_error = PackBase::PackEndError;
			return -1;
		}
		last_error = PackBase::PackIncomplete;
		return -1;
	}
	
	// This is how many bytes read from buffer
	return frombuf.buffer.beg - buffer;
}

/**************************
        Package STAT
 **************************/
bool STAT::valid() const {
	return __valid_check4(id, x, y, th)
	    && __valid_check2(v, w)
		&& __valid_check3(status, type, progress);
}

int STAT::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}

	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#STAT,", 6)
		 .write_ints(id(), x(), y(), th(), ',')
		 .write(',').write_ints(v(), w(), ',')
		 .write(',').write_ints(status(), type(), progress(), ',')
		 .write('\n');
	
	return write_end_routine(tobuf, buffer, last_error);
}
int STAT::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
	       .match("#STAT,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}

	frombuf.read_ints(id(), x(), y(), th(), ',')
		   .match(',').read_ints(v(), w(), ',')
		   .match(',').read_ints(status(), type(), progress(), ',');
	
	return check_end_routine(frombuf, buffer, last_error);
}
void STAT::print_data(std::FILE* out, const char* line_prefix) const  {
	const char* tmp = valid() ? "" : "[invalid]";
	fprintf(out, "%s%s%s, ID: %d\n", line_prefix, pack_name(), tmp, id());
	fprintf(out, "%s  x: %d, y: %d, th: %d, v: %d, w: %d\n", line_prefix, x(), y(), th(), v(), w());
	fprintf(out, "%s  status: %d, type: %d, progress: %d\n", line_prefix, status(), type(), progress());
}
bool STAT::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const STAT& it = *((const STAT*)&other);
		return __equal_it4(id, x, y, th) 
		    && __equal_it2(v, w)
			&& __equal_it3(status, type, progress);
	}
	return false;
}

/**************************
        Package COMD
 **************************/
bool COMD::valid() const {
	return __valid_check2(id, cmd);
}
int COMD::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#COMD,", 6)
	     .write_ints(id(), cmd(), ',')
		 .write('\n');
		 
	return write_end_routine(tobuf, buffer, last_error);
}
int COMD::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
	       .match("#COMD,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), cmd(), ',');
	return check_end_routine(frombuf, buffer, last_error);
}
void COMD::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	fprintf(out, "%s%s%s, ID: %d, cmd: %d\n", line_prefix, pack_name(), tmp, id(), cmd());
}
bool COMD::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const COMD& it = *((const COMD*)&other);
		return __equal_it2(id, cmd);
	}
	return false;
}

/**************************
        Package TURN
 **************************/
bool TURN::valid() const {
	return __valid_check3(id, xd, yd);
}
int TURN::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#TURN,", 6)
	     .write_ints(id(), xd(), yd(), ',')
		 .write('\n');
		 
	return write_end_routine(tobuf, buffer, last_error);
}
int TURN::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
	       .match("#TURN,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), xd(), yd(), ',');
	return check_end_routine(frombuf, buffer, last_error);
}
void TURN::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	fprintf(out, "%s%s%s, ID: %d, xd: %d, yd: %d\n", line_prefix, pack_name(), tmp, id(), xd(), yd());
}
bool TURN::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const TURN& it = *((const TURN*)&other);
		return __equal_it3(id, xd, yd);
	}
	return false;
}

/**************************
		Package TDEG
 **************************/
bool TDEG::valid() const {
	return __valid_check3(id, type, theta);
}
int TDEG::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#TDEG,", 6)
		.write_ints(id(), type(), theta(), ',')
		.write('\n');

	return write_end_routine(tobuf, buffer, last_error);
}
int TDEG::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
		.match("#TDEG,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), type(), theta(), ',');
	return check_end_routine(frombuf, buffer, last_error);
}
void TDEG::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	fprintf(out, "%s%s%s, ID: %d, type: %d, theta: %d\n", line_prefix, pack_name(), tmp, id(), type(), theta());
}
bool TDEG::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const TDEG& it = *((const TDEG*)&other);
		return __equal_it3(id, type, theta);
	}
	return false;
}

/**************************
		Package EIGT
 **************************/
bool EIGT::valid() const {
	if (__valid_check5(len, dir, v, r, n) && (int)ids.size() == n()) {
		for (int i = 0; i < n(); ++i) if (!ids[i].valid()) return false;
		return true;
	}
	return false;
}
int EIGT::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#EIGT,", 6)
		 .write_ints(len(), dir(), v(), r(), ',');
	tobuf.write(',').write(n());
	for (int i = 0; i < n(); ++i) {
		tobuf.write(',').write(ids[i]);
	}
	tobuf.write('\n');
	return write_end_routine(tobuf, buffer, last_error);
}
int EIGT::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
		   .match("#EIGT,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(len(), dir(), v(), r(), ',')
		   .match(',').read(n());
	if (!__valid_check5(len, dir, v, r, n)) 
		return -1;
	ids.resize(n());
	for (int i = 0; i < n(); ++i) {
		frombuf.match(',').read(ids[i]());
	}
	return check_end_routine(frombuf, buffer, last_error);
}
void EIGT::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	fprintf(out, "%s%s%s, len: %d, dir: %d, v: %d, r: %d, n: %d\n", line_prefix, pack_name(), tmp, len(), dir(), v(), r(), n());
	fprintf(out, "%s   ids:", line_prefix);
	for (unsigned i = 0; i < ids.size(); ++i) {
		fprintf(out, " %d", ids[i]());
	}
	fprintf(out, "\n");
}
bool EIGT::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const EIGT& it = *((const EIGT*)&other);
		return __equal_it4(len, dir, v, r) && __equal_it2(n, ids);
	}
	return false;
}



/**************************
        Package PATH
 **************************/
bool PATH::valid() const {
	if (__valid_check2(id, n)) {
		if (n() == (int)x.size() && n() == (int)y.size() && n() == (int)v.size()) {
			bool good = true;
			for (int i=0;i<n;++i) {
				good = good && __valid_check3(x[i], y[i], v[i]);
			}
			return good;
		}
	}
	return false;
}
int PATH::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#PATH,", 6)
	     .write_ints(id(), n(), ',');
	for (int i=0;i<n;++i) {
		tobuf.write(',').write_ints(x[i](), y[i](), v[i](), ',');
	}
	tobuf.write('\n');
	return write_end_routine(tobuf, buffer, last_error);
}
int PATH::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
	       .match("#PATH,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), n(), ',');
	if (!__valid_check2(id, n)) {
		return -1;
	}

	__resize3(n(), x, y, v);
	for (int i=0; i<n; ++i) {
		frombuf.match(',').read_ints(x[i](), y[i](), v[i](), ',');
	}
	return check_end_routine(frombuf, buffer, last_error);
}
void PATH::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	fprintf(out, "%s%s%s, ID: %d, n: %d\n", line_prefix, pack_name(), tmp, id(), n());
	
	for (unsigned k=0;k<(unsigned)n();++k) {
		fprintf(out, "%s  k=%d", line_prefix, k + 1);
		if (k < x.size()) fprintf(out, ", x: %d", x[k]());
		else fprintf(out, ", x: n/a");
		if (k < y.size()) fprintf(out, ", y: %d", y[k]());
		else fprintf(out, ", y: n/a");
		if (k < v.size()) fprintf(out, ", v: %d", v[k]());
		else fprintf(out, ", v: n/a");
		fprintf(out, "\n");
	}
}
bool PATH::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const PATH& it = *((const PATH*)&other);
		return __equal_it2(id, n)
		    && __equal_it3(x, y, v);
	}
	return false;
}

/**************************
		Package FORM
 **************************/
bool FORM::valid() const {
	if (__valid_check4(type, nrobot, elen, fmgroup)) {
		if (nrobot() != (int)robot.size()) return false;
		for (int i = 0; i < nrobot(); ++i) {
			if (!robot[i].valid()) return false;
		}
		if (type() == FORM::TestEight) {
			// Then we dont need path info
			return true;
		}
		if (type() == FORM::Basic) {
			if (npoint.valid() && npoint() == (int)x.size() && npoint() == (int)y.size() && npoint() == (int)v.size()) {
				for (int i = 0; i < npoint(); ++i) {
					if (!__valid_check3(x[i], y[i], v[i]))
						return false;
				}
				return true;
			}
		}
	}
	return false;
}
int FORM::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#FORM,", 6);
	tobuf.write_ints(type(), elen(), fmgroup(), ',');
	tobuf.write(',').write(nrobot());
	for (int i = 0; i < nrobot(); ++i) {
		tobuf.write(',').write(robot[i]);
	}
	if (type() == FORM::Basic) {
		tobuf.write(',').write(npoint());
		for (int i = 0; i < npoint(); ++i) {
			tobuf.write(',').write_ints(x[i], y[i], v[i], ',');
		}
	}
	tobuf.write('\n');
	return write_end_routine(tobuf, buffer, last_error);
}
int FORM::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
		.match("#FORM,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(type(), elen(), fmgroup(), ',');
	frombuf.match(',').read(nrobot());
	if (!__valid_check4(type, elen, fmgroup, nrobot)) {
		return -1;
	}
		
	robot.resize(nrobot());
	for (int i = 0; i < nrobot(); ++i) {
		frombuf.match(',').read(robot[i]());
	}
	if (type() == FORM::Basic) {
		frombuf.match(',').read(npoint());
		__resize3(npoint(), x, y, v);
		for (int i = 0; i < npoint(); ++i) {
			frombuf.match(',').read_ints(x[i](), y[i](), v[i](), ',');
		}
	}

	return check_end_routine(frombuf, buffer, last_error);
}
void FORM::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";

	fprintf(out, "%s%s%s, nrobot: %d, elen: %d, fmgroup: %d\n", line_prefix, pack_name(), tmp, nrobot(), elen(), fmgroup());
	fprintf(out, "%s  robot:", line_prefix);
	for (int i = 0; i < (int)robot.size(); ++i) {
		fprintf(out, " %d", robot[i]());
	}
	fprintf(out, "\n");
	if (type() == FORM::Basic) {
		fprintf(out, "%s npoint: %d\n", line_prefix, npoint());
		for (unsigned k = 0; k < (unsigned)npoint(); ++k) {
			fprintf(out, "%s  k=%d", line_prefix, k + 1);
			if (k < x.size()) fprintf(out, ", x: %d", x[k]());
			else fprintf(out, ", x: n/a");
			if (k < y.size()) fprintf(out, ", y: %d", y[k]());
			else fprintf(out, ", y: n/a");
			if (k < v.size()) fprintf(out, ", v: %d", v[k]());
			else fprintf(out, ", v: n/a");
			fprintf(out, "\n");
		}
	}
}
bool FORM::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const FORM& it = *((const FORM*)&other);
		bool good = __equal_it3(type, elen, fmgroup)
			     && __equal_it2(nrobot, robot);
		if (good) {
			if (type() == FORM::Basic) {
				return __equal_it4(npoint, x, y, v);
			}
			return true;
		}
	}
	return false;
}

/**************************
        Package OBST
 **************************/
bool OBST::valid() const {
	if (__valid_check3(id, type, n)) {
		if (type() == Circle) {
			if (n() == (int)x.size() && n() == (int)y.size() && n() == (int)r.size()) {
				bool good = true;
				for (int i=0;i<n;++i) {
					good = good && __valid_check3(x[i], y[i], r[i]);
				}
				return good;	
			}
		}
		else if (type() == Rect) {
			int n4 = 4*n;
			if (n4 == (int)x.size() && n4 == (int)y.size()) {
				bool good = true;
				for (int i=0;i<n4;++i) {
					good = good && __valid_check2(x[i], y[i]);
				}
				return good;	
			}
		}
	}
	return false;
}
int OBST::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#OBST,", 6)
	     .write_ints(id(), type(), n(), ',');
	if (type() == OBST::Circle) {
		for (int i=0;i<n;++i) {
			tobuf.write(',').write_ints(x[i](), y[i](), r[i](), ',');
		}
	}
	else if (type() == OBST::Rect) {
		for (int i=0;i<n;++i) {
			for (int j=0;j<4;++j) {
				int k = i*4+j;
				tobuf.write(',').write_ints(x[k](), y[k](), ',');
			}
		}
	}
	tobuf.write('\n');
	
	return write_end_routine(tobuf, buffer, last_error);
}
int OBST::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
	       .match("#OBST,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), type(), n(), ',');
	if (!__valid_check3(id, type, n)) {
		return -1;
	}

	if (type() == OBST::Circle) {
		__resize3(n(), x, y, r);
		for (int i=0; i<n; ++i) {
			frombuf.match(',').read_ints(x[i](), y[i](), r[i](), ',');
		}
	}
	else if (type() == OBST::Rect) {
		int n4 = n()*4;
		__resize2(n4, x, y);
		for (int i=0;i<n;++i) {
			for (int j=0;j<4;++j) {
				int k = i*4+j;
				frombuf.match(',').read_ints(x[k](), y[k](), ',');
			}
		}
	}
	else {
		last_error = PackBase::PackDataError;
		return -1;
	}
	return check_end_routine(frombuf, buffer, last_error);
}
void OBST::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	const char* tps = type() == OBST::Circle ? "Circle" :
	                  type() == OBST::Rect ? "Rect" :
					  "Unknown";
	fprintf(out, "%s%s%s, ID: %d, type: %s, n: %d\n", line_prefix, pack_name(), tmp, id(), tps, n());
	if (type() == OBST::Circle) {
		for (unsigned k=0;k<(unsigned)n();++k) {
			fprintf(out, "%s  k=%d", line_prefix, k + 1);
			if (k < x.size()) fprintf(out, ", x: %d", x[k]());
			else fprintf(out, ", x: n/a");
			if (k < y.size()) fprintf(out, ", y: %d", y[k]());
			else fprintf(out, ", y: n/a");
			if (k < r.size()) fprintf(out, ", r: %d", r[k]());
			else fprintf(out, ", r: n/a");
			fprintf(out, "\n");
		}
	}
	if (type() == OBST::Rect) {
		for (unsigned i=0;i<(unsigned)n();++i) {
			fprintf(out, "%s  Rect %d:\n", line_prefix, i+1);
			for (unsigned j=0;j<4;++j) {
				unsigned k = i*4+j;
				fprintf(out, "%s    k=%d", line_prefix, k + 1);
				if (k < x.size()) fprintf(out, ", x: %d", x[k]());
				else fprintf(out, ", x: n/a");
				if (k < y.size()) fprintf(out, ", y: %d", y[k]());
				else fprintf(out, ", y: n/a");
				fprintf(out, "\n");
			}			
		}
	}
}
bool OBST::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const OBST& it = *((const OBST*)&other);
		if (type() == OBST::Circle) {
			return __equal_it3(id, type, n)
				&& __equal_it3(x, y, r);
		}
		if (type() == OBST::Rect) {
			return __equal_it3(id, type, n)
				&& __equal_it2(x, y);
		}
	}
	return false;
}

/**************************
        Package TELE
 **************************/
bool TELE::valid() const {
	return __valid_check4(id, type, v, w);
}
int TELE::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#TELE,", 6)
	     .write_ints(id(), type(), v(), w(), ',')
		 .write('\n');
	return write_end_routine(tobuf, buffer, last_error);
}
int TELE::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
	       .match("#TELE,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), type(), v(), w(), ',');
	return check_end_routine(frombuf, buffer, last_error);
}
void TELE::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	fprintf(out, "%s%s%s, ID: %d, type: %d, v: %d, w: %d\n", line_prefix, pack_name(), tmp, id(), type(), v(), w());
}
bool TELE::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const TELE& it = *((const TELE*)&other);
		return __equal_it4(id, type, v, w);
	}
	return false;
}


/**************************
		Package TALG
 **************************/
bool TALG::valid() const {
	return __valid_check4(id, type, v1, v2);
}
int TALG::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#TALG,", 6)
		.write_ints(id(), type(), v1(), v2(), ',')
		.write('\n');
	return write_end_routine(tobuf, buffer, last_error);
}
int TALG::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
		.match("#TALG,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), type(), v1(), v2(), ',');

	return check_end_routine(frombuf, buffer, last_error);
}
void TALG::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	if (type() == ByVxy) {
		fprintf(out, "%s%s%s, ID: %d, type: %d, vx: %d, vy: %d\n", line_prefix, pack_name(), tmp, id(), type(), v1(), v2());
	}
	if (type() == ByVW) {
		fprintf(out, "%s%s%s, ID: %d, type: %d, v: %d, w: %d\n", line_prefix, pack_name(), tmp, id(), type(), v1(), v2());
	}
}
bool TALG::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const TALG& it = *((const TALG*)&other);
		return  __equal_it4(id, type, v1, v2);
	}
	return false;
}


/**************************
		Package ALGO
 **************************/
bool ALGO::valid() const {
	if (__valid_check2(id, subType)) {
		if (subType() == ALGO::shp) {
			return __valid_check4(nrobot, gpID, shpIndex, priority);
		}
		if (subType() == ALGO::info) {
			return __valid_check2(task, state);
		}
		return true;
	}
	return false;
}
int ALGO::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#ALGO,", 6)
		 .write_ints(id(), subType(), ',');
	if (subType() == ALGO::shp) {
		tobuf.write(',').write_ints(nrobot(), gpID(), shpIndex(), priority(), ',');
	}
	if (subType() == ALGO::info) {
		tobuf.write(',').write_ints(task(), state(), ',');
	}
	tobuf.write('\n');
	return write_end_routine(tobuf, buffer, last_error);
}

int ALGO::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
		   .match("#ALGO,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), subType(), ',');
	if (subType.valid()) {
		if (subType() == ALGO::shp) {
			frombuf.match(',').read_ints(nrobot(), gpID(), shpIndex(), priority(), ',');
		}
		if (subType() == ALGO::info) {
			frombuf.match(',').read_ints(task(), state(), ',');
		}
	}

	return check_end_routine(frombuf, buffer, last_error);
}
void ALGO::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	if (subType() == ALGO::shp) {
		fprintf(out, "%s%s%s, ID: %d, type: shp, nrobot: %d, gpID: %d, shpIndex: %d, priority: %d\n", 
			line_prefix, pack_name(), tmp, id(), 
			nrobot(), gpID(), shpIndex(), priority());
	}
	else if (subType() == ALGO::info) {
		fprintf(out, "%s%s%s, ID: %d, type: info, task: %d, status: %d\n",
			line_prefix, pack_name(), tmp, id(), task(), state());
	}
	else {
		fprintf(out, "%s%s%s, ID: %d, type: %d\n", line_prefix, pack_name(), tmp, id(), subType());
	}
}
bool ALGO::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const ALGO& it = *((const ALGO*)&other);
		if (__equal_it2(id, subType)) {
			if (subType() == ALGO::shp) {
				return __equal_it4(nrobot, gpID, shpIndex, priority);
			}
			if (subType() == ALGO::info) {
				return __equal_it2(task, state);
			}
		}
	}
	return false;
}

/**************************
		Package PLAN
 **************************/
bool PLAN::valid() const {
	if (__valid_check4(id, type, v, d)) {
		if (type() != Basic) return __valid_check2(xd, yd);
		return __valid_check4(xd, yd, x0, y0);
	}
	return false;
}
int PLAN::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#PLAN,", 6)
		.write_ints(id(), type(), v(), d(), xd(), yd(), ',');
	if (type() == Basic) {
		tobuf.write(',').write_ints(x0(), y0(), ',');
	}
	tobuf.write('\n');
	return write_end_routine(tobuf, buffer, last_error);
}
int PLAN::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
		.match("#PLAN,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), type(), v(), d(), xd(), yd(), ',');
	if (!type.valid()) return PackBase::PackDataError;
	if (type() == Basic) {
		frombuf.match(',').read_ints(x0(), y0(), ',');
	}

	return check_end_routine(frombuf, buffer, last_error);
}
void PLAN::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	if (type() == Basic) {
		fprintf(out, "%s%s%s, ID: %d, type: %d, v: %d, d: %d, qd: (%d, %d), q0: (%d, %d)\n",
			line_prefix, pack_name(), tmp, id(), type(), v(), d(), xd(), yd(), x0(), y0());
	}
	else {
		fprintf(out, "%s%s%s, ID: %d, type: %d, v: %d, d: %d, qd: (%d, %d), q0: self\n",
			line_prefix, pack_name(), tmp, id(), type(), v(), d(), xd(), yd());
	}
}
bool PLAN::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const PLAN& it = *((const PLAN*)&other);
		if (__equal_it4(id, type, v, d)) {
			if (type() != Basic) return __equal_it2(xd, yd);
			return __equal_it4(xd, yd, x0, y0);
		}
	}
	return false;
}


/**************************
		Package MPLN
 **************************/
bool MPLN::valid() const {
	if (__valid_check4(id, type, d, n)) {
		if ((int)x.size() == n() && (int)y.size() == n() && (int)v.size() == n()) {
			for (int i = 0; i < n(); ++i) {
				if (!__valid_check3(x[i], y[i], v[i])) return false;
			}
			return true;
		}
	}
	return false;
}
int MPLN::write(char* buffer, int size) {
	if (!valid()) {
		last_error = PackBase::DataInvalid;
		return -1;
	}
	strproto::WriteUtil tobuf(buffer, size);
	tobuf.write("#MPLN,", 6)
		.write_ints(id(), type(), d(), n(), ',');
	for (int i = 0; i < n(); ++i) {
		tobuf.write(',').write_ints(x[i](), y[i](), v[i](), ',');
	}
	tobuf.write('\n');
	return write_end_routine(tobuf, buffer, last_error);
}
int MPLN::read(const char* buffer, int size) {
	strproto::ReadUtil frombuf(buffer, size);
	frombuf.read_until('#')
		.match("#MPLN,", 6);
	if (!frombuf.all_good) {
		last_error = PackBase::PackHeadError;
		return -1;
	}
	frombuf.read_ints(id(), type(), d(), n(), ',');
	if (!n.valid()) return PackBase::PackDataError;

	__resize3(n(), x, y, v);
	for (int i = 0; i < n(); ++i) {
		frombuf.match(',').read_ints(x[i](), y[i](), v[i](), ',');
	}
	return check_end_routine(frombuf, buffer, last_error);
}
void MPLN::print_data(std::FILE* out, const char* line_prefix) const {
	const char* tmp = valid() ? "" : "[invalid]";
	fprintf(out, "%s%s%s, ID: %d, type: %d, d: %d, n: %d\n",
		line_prefix, pack_name(), tmp, id(), type(), d(), n());
	for (unsigned k = 0; k < (unsigned)n(); ++k) {
		fprintf(out, "%s  k=%d", line_prefix, k + 1);
		if (k < x.size()) fprintf(out, ", x: %d", x[k]());
		else fprintf(out, ", x: n/a");
		if (k < y.size()) fprintf(out, ", y: %d", y[k]());
		else fprintf(out, ", y: n/a");
		if (k < v.size()) fprintf(out, ", v: %d", v[k]());
		else fprintf(out, ", v: n/a");
		fprintf(out, "\n");
	}
}
bool MPLN::operator == (const PackBase& other) const {
	if (packID() == other.packID()) {
		const MPLN& it = *((const MPLN*)&other);
		return __equal_it7(id, type, d, n, x, y, v);
	}
	return false;
}


} // namespace protobit
