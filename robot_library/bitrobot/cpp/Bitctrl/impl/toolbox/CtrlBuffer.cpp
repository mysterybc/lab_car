#include <string>
#include <cstdio>
#include <cassert>
#include <algorithm>
#include "CtrlBuffer.hpp"



namespace conf {

Quantizer<float, std::uint8_t> CtrlPack::q_hfr = Quantizer<float, std::uint8_t>(0, 1);
Quantizer<float, std::uint16_t> CtrlPack::q_th  = Quantizer<float, std::uint16_t>(0, 360);

template<class T> T round_angle(const T& v) {
	return v > 360 ? round_angle(v - 360) :
		   v < 0 ? round_angle(v + 360) : v;
}

void CtrlPack::print_quanerr() {
#define __peMax(name) printf(#name " eMax = %.6f\n", name .eMax());
	__peMax(q_hfr);
	__peMax(q_th);
#undef __peMax
}

template<class CSumType>
CSumType compute_csum(const void* buf, size_t size) {
	CSumType sum = 0;
	for (size_t i = 0; i < size; ++i) {
		sum += (CSumType)(*((const std::uint8_t*)buf + i));
	}
	return sum;
}

//
// Constructor for CtrlPack
// - Auto reset on construction
 CtrlPack::CtrlPack() {
	reset();
}


//
// Reset Function
// - Make all options & values invalid
void CtrlPack::reset() {
	_conf = 0;
	_valid = 0;
	_valid = 0;
}


//
// Binary Option IsFaulty
//
CtrlPack& CtrlPack::withIsFaulty(const bool& with) {
	_valid |= (1 << IsFaulty_Bit);
	if (with)	_conf |= (1 << IsFaulty_Bit);
	else	_conf &= ~(1 << IsFaulty_Bit);
	return *this;
}
CtrlPack& CtrlPack::resetIsFaulty() {
	_valid &= ~(1 << IsFaulty_Bit);
	return *this;
}
bool CtrlPack::withIsFaulty() const {
	return (_conf & (1 << IsFaulty_Bit)) != 0;
}
bool CtrlPack::validIsFaulty() const {
	return (_valid & (1 << IsFaulty_Bit)) != 0;
}


//
// ValueConfig AgentID
//
CtrlPack& CtrlPack::setAgentID(const std::uint8_t& agentID) {
	__agentID = agentID;
	_valid |= (1 << AgentIDValue_Bit);
	return *this;
}
CtrlPack& CtrlPack::resetAgentID() {
	_valid &= ~(1 << AgentIDValue_Bit);
	return *this;
}
bool CtrlPack::validAgentID() const {
	return (_valid & (1 << AgentIDValue_Bit)) != 0;
}
bool CtrlPack::getAgentID(std::uint8_t& agentID) const {
	if (validAgentID()) {
		agentID = __agentID;
		return true;
	}
	return false;
}


//
// ValueConfig TaskState
//
CtrlPack& CtrlPack::setTaskState(const std::uint8_t& taskID, const std::uint8_t& task_state) {
	__taskID = taskID;
	__task_state = task_state;
	_valid |= (1 << TaskStateValue_Bit);
	return *this;
}
CtrlPack& CtrlPack::resetTaskState() {
	_valid &= ~(1 << TaskStateValue_Bit);
	return *this;
}
bool CtrlPack::validTaskState() const {
	return (_valid & (1 << TaskStateValue_Bit)) != 0;
}
bool CtrlPack::getTaskState(std::uint8_t& taskID, std::uint8_t& task_state) const {
	if (validTaskState()) {
		taskID = __taskID;
		task_state = __task_state;
		return true;
	}
	return false;
}


//
// ValueConfig HField
//
CtrlPack& CtrlPack::setHField(const std::uint8_t& hfr, const float& hfx, const float& hfy) {
	__hfr = hfr;
	__hfx = hfx;
	__hfy = hfy;
	_valid |= (1 << HFieldValue_Bit);
	return *this;
}
CtrlPack& CtrlPack::resetHField() {
	_valid &= ~(1 << HFieldValue_Bit);
	return *this;
}
bool CtrlPack::validHField() const {
	return (_valid & (1 << HFieldValue_Bit)) != 0;
}
bool CtrlPack::getHField(std::uint8_t& hfr, float& hfx, float& hfy) const {
	if (validHField()) {
		hfr = __hfr;
		hfx = __hfx;
		hfy = __hfy;
		return true;
	}
	return false;
}


//
// ValueConfig Latency
//
CtrlPack& CtrlPack::setLatency(const std::uint16_t& lat1, const std::uint16_t& lat2, const std::uint16_t& lat3, const std::uint16_t& lat4) {
	__lat1 = lat1;
	__lat2 = lat2;
	__lat3 = lat3;
	__lat4 = lat4;
	_valid |= (1 << LatencyValue_Bit);
	return *this;
}
CtrlPack& CtrlPack::resetLatency() {
	_valid &= ~(1 << LatencyValue_Bit);
	return *this;
}
bool CtrlPack::validLatency() const {
	return (_valid & (1 << LatencyValue_Bit)) != 0;
}
bool CtrlPack::getLatency(std::uint16_t& lat1, std::uint16_t& lat2, std::uint16_t& lat3, std::uint16_t& lat4) const {
	if (validLatency()) {
		lat1 = __lat1;
		lat2 = __lat2;
		lat3 = __lat3;
		lat4 = __lat4;
		return true;
	}
	return false;
}

//
// ValueConfig Shape
//
CtrlPack& CtrlPack::setShape(const std::uint8_t& num_agent, const std::uint8_t& shape_index, const std::uint8_t& fmgroup, const std::uint16_t& priority) {
	__num_agent = num_agent;
	__shape_index = shape_index;
	__fmgroup = fmgroup;
	__priority = priority;
	_valid |= (1 << ShapeValue_Bit);
	return *this;
}
CtrlPack& CtrlPack::resetShape() {
	_valid &= ~(1 << ShapeValue_Bit);
	return *this;
}
bool CtrlPack::validShape() const {
	return (_valid & (1 << ShapeValue_Bit)) != 0;
}
bool CtrlPack::getShape(std::uint8_t& num_agent, std::uint8_t& shape_index, std::uint8_t& fmgroup, std::uint16_t& priority) const {
	if (validShape()) {
		num_agent = __num_agent;
		shape_index = __shape_index;
		fmgroup = __fmgroup;
		priority = __priority;
		return true;
	}
	return false;
}


//
// ValueConfig GField
//
CtrlPack& CtrlPack::setGField(const std::uint8_t& ng, const std::uint8_t& rg, const float& hgx, const float& hgy, const float& zg) {
	__ng = ng;
	__rg = rg;
	__hgx = hgx;
	__hgy = hgy;
	__zg = zg;
	_valid |= (1 << GFieldValue_Bit);
	return *this;
}
CtrlPack& CtrlPack::resetGField() {
	_valid &= ~(1 << GFieldValue_Bit);
	return *this;
}
bool CtrlPack::validGField() const {
	return (_valid & (1 << GFieldValue_Bit)) != 0;
}
bool CtrlPack::getGField(std::uint8_t& ng, std::uint8_t& rg, float& hgx, float& hgy, float& zg) const {
	if (validGField()) {
		ng = __ng;
		rg = __rg;
		hgx = __hgx;
		hgy = __hgy;
		zg = __zg;
		return true;
	}
	return false;
}

//
// ValueConfig StateInfo
//
CtrlPack& CtrlPack::setStateInfo(const float& my_x, const float& my_y, const float& my_vx, const float& my_vy, const std::uint16_t& my_angle) {
	__my_x = my_x;
	__my_y = my_y;
	__my_vx = my_vx;
	__my_vy = my_vy;
	__my_angle = my_angle;
	_valid |= (1 << StateInfoValue_Bit);
	return *this;
}
CtrlPack& CtrlPack::resetStateInfo() {
	_valid &= ~(1 << StateInfoValue_Bit);
	return *this;
}
bool CtrlPack::validStateInfo() const {
	return (_valid & (1 << StateInfoValue_Bit)) != 0;
}
bool CtrlPack::getStateInfo(float& my_x, float& my_y, float& my_vx, float& my_vy, std::uint16_t& my_angle) const {
	if (validStateInfo()) {
		my_x = __my_x;
		my_y = __my_y;
		my_vx = __my_vx;
		my_vy = __my_vy;
		my_angle = __my_angle;
		return true;
	}
	return false;
}



//
// Interface function to read/write configs from raw buffer
//

// Compute required buffer size for current configurations
std::size_t CtrlPack::buffer_size() const {
	std::size_t size = 0;

	// Buffer for Binary Options
	size += sizeof(_conf) + sizeof(_valid);

	// Buffer for Value Config

	// Buffer for Value Option AgentID
	if (validAgentID()) {
		size += sizeof(__agentID);
	}

	// Buffer for Value Option TaskState
	if (validTaskState()) {
		size += sizeof(__taskID) + sizeof(__task_state);
	}

	// Buffer for Value Option HField
	if (validHField()) {
		size += sizeof(__hfr) + sizeof(__hfx) + sizeof(__hfy);
	}

	// Buffer for Value Option Latency
	if (validLatency()) {
		size += sizeof(__lat1) + sizeof(__lat2) + sizeof(__lat3) + sizeof(__lat4);
	}

	// Buffer for Value Option Shape
	if (validShape()) {
		size += sizeof(__num_agent) + sizeof(__shape_index) + sizeof(__fmgroup) + sizeof(__priority);
	}

	// Buffer for Value Option StateInfo
	if (validStateInfo()) {
		size += sizeof(__my_x) + sizeof(__my_y) + sizeof(__my_vx) + sizeof(__my_vy) + sizeof(__my_angle);
	}

	// Buffer for Value Option GField
	if (validGField()) {
		size += sizeof(__ng) + sizeof(__rg) + sizeof(__hgx) + sizeof(__hgy) + sizeof(__zg);
	}

	return size;
}


// Writing configurations to buffer
std::size_t CtrlPack::write_buffer(void* buffer, std::size_t max_size) const {
	// Check size valid
	std::size_t size_required = buffer_size();
	if (max_size < size_required) {
		return 0;
	}

	char* buf = (char*)buffer;

	// Buffer for Binary Options
	*((std::uint8_t*)buf) = _valid;
	buf += sizeof(_valid);
	*((std::uint8_t*)buf) = _conf;
	buf += sizeof(_conf);

	// Buffer for Value Config

	// Buffer for Value Option AgentID
	if (validAgentID()) {
		*((std::uint8_t*)buf) = __agentID;
		buf += sizeof(__agentID);
	}

	// Buffer for Value Option TaskState
	if (validTaskState()) {
		*((std::uint8_t*)buf) = __taskID;
		buf += sizeof(__taskID);
		*((std::uint8_t*)buf) = __task_state;
		buf += sizeof(__task_state);
	}

	// Buffer for Value Option HField
	if (validHField()) {
		*((std::uint8_t*)buf) = __hfr;
		buf += sizeof(__hfr);
		*((float*)buf) = __hfx;
		buf += sizeof(__hfx);
		*((float*)buf) = __hfy;
		buf += sizeof(__hfy);
	}

	// Buffer for Value Option Latency
	if (validLatency()) {
		*((std::uint16_t*)buf) = __lat1;
		buf += sizeof(__lat1);
		*((std::uint16_t*)buf) = __lat2;
		buf += sizeof(__lat2);
		*((std::uint16_t*)buf) = __lat3;
		buf += sizeof(__lat3);
		*((std::uint16_t*)buf) = __lat4;
		buf += sizeof(__lat4);
	}

	// Buffer for Value Option Shape
	if (validShape()) {
		*((std::uint8_t*)buf) = __num_agent;
		buf += sizeof(__num_agent);
		*((std::uint8_t*)buf) = __shape_index;
		buf += sizeof(__shape_index);
		*((std::uint8_t*)buf) = __fmgroup;
		buf += sizeof(__fmgroup);
		*((std::uint16_t*)buf) = __priority;
		buf += sizeof(__priority);
	}

	// Buffer for Value Option StateInfo
	if (validStateInfo()) {
		*((float*)buf) = __my_x;
		buf += sizeof(__my_x);
		*((float*)buf) = __my_y;
		buf += sizeof(__my_y);
		*((float*)buf) = __my_vx;
		buf += sizeof(__my_vx);
		*((float*)buf) = __my_vy;
		buf += sizeof(__my_vy);
		*((std::uint16_t*)buf) = __my_angle;
		buf += sizeof(__my_angle);
	}

	// Buffer for Value Option GField
	if (validGField()) {
		*((std::uint8_t*)buf) = __ng;
		buf += sizeof(__ng);
		*((std::uint8_t*)buf) = __rg;
		buf += sizeof(__rg);
		*((float*)buf) = __hgx;
		buf += sizeof(__hgx);
		*((float*)buf) = __hgy;
		buf += sizeof(__hgy);
		*((float*)buf) = __zg;
		buf += sizeof(__zg);
	}

	// Check size calculation
	std::size_t size_write = buf - (char*)buffer;
	assert(size_write == size_required);

	// Return bytes written
	return size_write;
}

// Reading configurations from buffer
bool CtrlPack::read_buffer(const void* buffer, std::size_t size) {
	char* beg = (char*)buffer;
	const char* end = beg + size;
	std::uint8_t tmp_conf;
	std::uint8_t tmp_vali;

	if (end >= beg + sizeof(tmp_vali)) {
		tmp_vali = *((std::uint8_t*)beg);
		beg += sizeof(tmp_vali);
	}
	else {
		return false;
	}
	if (end >= beg + sizeof(tmp_conf)) {
		tmp_conf = *((std::uint8_t*)beg);
		beg += sizeof(tmp_conf);
	}
	else {
		return false;
	}

	// If only both are valid
	_valid = tmp_vali;
	_conf = tmp_conf;

	// Load value options

	// Loading AgentID
	if ((tmp_vali & (1 << AgentIDValue_Bit)) != 0) {
		if (end >= beg + sizeof(__agentID)) {
			__agentID = *((std::uint8_t*)beg);
			beg += sizeof(__agentID);

			_valid |= (1 << AgentIDValue_Bit);
		}
		else {
			// Size-Type Mismatch
			return false;
		}
	}

	// Loading TaskState
	if ((tmp_vali & (1 << TaskStateValue_Bit)) != 0) {
		if (end >= beg + sizeof(__taskID) + sizeof(__task_state)) {
			__taskID = *((std::uint8_t*)beg);
			beg += sizeof(__taskID);
			__task_state = *((std::uint8_t*)beg);
			beg += sizeof(__task_state);

			_valid |= (1 << TaskStateValue_Bit);
		}
		else {
			// Size-Type Mismatch
			return false;
		}
	}

	// Loading HField
	if ((tmp_vali & (1 << HFieldValue_Bit)) != 0) {
		if (end >= beg + sizeof(__hfr) + sizeof(__hfx) + sizeof(__hfy)) {
			__hfr = *((std::uint8_t*)beg);
			beg += sizeof(__hfr);
			__hfx = *((float*)beg);
			beg += sizeof(__hfx);
			__hfy = *((float*)beg);
			beg += sizeof(__hfy);

			_valid |= (1 << HFieldValue_Bit);
		}
		else {
			// Size-Type Mismatch
			return false;
		}
	}

	// Loading Latency
	if ((tmp_vali & (1 << LatencyValue_Bit)) != 0) {
		if (end >= beg + sizeof(__lat1) + sizeof(__lat2) + sizeof(__lat3) + sizeof(__lat4)) {
			__lat1 = *((std::uint16_t*)beg);
			beg += sizeof(__lat1);
			__lat2 = *((std::uint16_t*)beg);
			beg += sizeof(__lat2);
			__lat3 = *((std::uint16_t*)beg);
			beg += sizeof(__lat3);
			__lat4 = *((std::uint16_t*)beg);
			beg += sizeof(__lat4);

			_valid |= (1 << LatencyValue_Bit);
		}
		else {
			// Size-Type Mismatch
			return false;
		}
	}

	// Loading Shape
	if ((tmp_vali & (1 << ShapeValue_Bit)) != 0) {
		if (end >= beg + sizeof(__num_agent) + sizeof(__shape_index) + sizeof(__fmgroup) + sizeof(__priority)) {
			__num_agent = *((std::uint8_t*)beg);
			beg += sizeof(__num_agent);
			__shape_index = *((std::uint8_t*)beg);
			beg += sizeof(__shape_index);
			__fmgroup = *((std::uint8_t*)beg);
			beg += sizeof(__fmgroup);
			__priority = *((std::uint16_t*)beg);
			beg += sizeof(__priority);

			_valid |= (1 << ShapeValue_Bit);
		}
		else {
			// Size-Type Mismatch
			return false;
		}
	}

	// Loading StateInfo
	if ((tmp_vali & (1 << StateInfoValue_Bit)) != 0) {
		if (end >= beg + sizeof(__my_x) + sizeof(__my_y) + sizeof(__my_vx) + sizeof(__my_vy) + sizeof(__my_angle)) {
			__my_x = *((float*)beg);
			beg += sizeof(__my_x);
			__my_y = *((float*)beg);
			beg += sizeof(__my_y);
			__my_vx = *((float*)beg);
			beg += sizeof(__my_vx);
			__my_vy = *((float*)beg);
			beg += sizeof(__my_vy);
			__my_angle = *((std::uint16_t*)beg);
			beg += sizeof(__my_angle);

			_valid |= (1 << StateInfoValue_Bit);
		}
		else {
			// Size-Type Mismatch
			return false;
		}
	}

	// Loading GField
	if ((tmp_vali & (1 << GFieldValue_Bit)) != 0) {
		if (end >= beg + sizeof(__ng) + sizeof(__rg) + sizeof(__hgx) + sizeof(__hgy) + sizeof(__zg)) {
			__ng = *((std::uint8_t*)beg);
			beg += sizeof(__ng);
			__rg = *((std::uint8_t*)beg);
			beg += sizeof(__rg);
			__hgx = *((float*)beg);
			beg += sizeof(__hgx);
			__hgy = *((float*)beg);
			beg += sizeof(__hgy);
			__zg = *((float*)beg);
			beg += sizeof(__zg);

			_valid |= (1 << GFieldValue_Bit);
		}
		else {
			// Size-Type Mismatch
			return false;
		}
	}

	// All good
	return true;
}

// Update stored values
// Return whether values have changed
bool CtrlPack::update(const CtrlPack& conf) {
	// Use this variable to check if value changed
	bool changed = false;

	// Update Options
	if (conf.validIsFaulty()) {
		withIsFaulty(conf.withIsFaulty());
		changed = true;
	}

	// Update Values
	if (conf.validAgentID()) {
		conf.getAgentID(__agentID);
		changed = true;
	}
	if (conf.validTaskState()) {
		conf.getTaskState(__taskID, __task_state);
		changed = true;
	}
	if (conf.validHField()) {
		conf.getHField(__hfr, __hfx, __hfy);
		changed = true;
	}
	if (conf.validLatency()) {
		conf.getLatency(__lat1, __lat2, __lat3, __lat4);
		changed = true;
	}
	if (conf.validShape()) {
		conf.getShape(__num_agent, __shape_index, __fmgroup, __priority);
		changed = true;
	}
	if (conf.validStateInfo()) {
		conf.getStateInfo(__my_x, __my_y, __my_vx, __my_vy, __my_angle);
		changed = true;
	}
	if (conf.validGField()) {
		conf.getGField(__ng, __rg, __hgx, __hgy, __zg);
		changed = true;
	}

	return changed;
}

// Test equal
// Return if this equals input
bool CtrlPack::operator ==(const CtrlPack& other) const {
	// Check me.validXX() <--> other.validXX()
	if (validIsFaulty() != other.validIsFaulty()) {
		return false;
	}
	if (validAgentID() != other.validAgentID()) {
		return false;
	}
	if (validTaskState() != other.validTaskState()) {
		return false;
	}
	if (validHField() != other.validHField()) {
		return false;
	}
	if (validLatency() != other.validLatency()) {
		return false;
	}
	if (validShape() != other.validShape()) {
		return false;
	}
	if (validStateInfo() != other.validStateInfo()) {
		return false;
	}
	if (validGField() != other.validGField()) {
		return false;
	}
	// Check difference
	CtrlPack mecopy = *this;
	return mecopy.update(other);
}

// -----------------------------
// ---- Manuual Coded Begin ----
// -----------------------------
std::size_t CtrlPack::write_buffer_all(void* buffer, std::size_t max_size, char prefill) {
	last_err = ERR_GOOD;
	if(max_size >= __buffer_size){
		std::fill_n((char*)buffer, __buffer_size, prefill);
		std::size_t size_max = __buffer_size - sizeof(CSumType);
        std::size_t size_write = write_buffer(buffer, size_max);
        if(size_write > 0 && size_write <= size_max){
			CSumType __csum = compute_csum<CSumType>(buffer, size_max);
			*((CSumType*)((std::uint8_t*)buffer + size_max)) = __csum;
            return __buffer_size;
        }else{
			last_err = ERR_OTHER;
            std::cout<<"write buffer:  something wrong with write buffer...."<<std::endl;
            return 0;
        }
    }else{
		last_err = ERR_BUF_SMALL;
        std::cout<<"write buffer:  input max_size needed more than 80...."<<std::endl;
        return 0;
    }
 }

std::size_t CtrlPack::read_buffer_all(const void* buffer, std::size_t size) {
	last_err = ERR_GOOD;
    if(size >=__buffer_size){
		std::size_t size_max = __buffer_size - sizeof(CSumType);
		CSumType __csum = compute_csum<CSumType>(buffer, size_max);
		CSumType _csum = *((std::uint8_t*)buffer + size_max);
		_last_csum_get = _csum;
		_last_csum_compute = __csum;

        //std::cout<<"check_sum"<<int(_csum)<<std::endl;
        if (__csum == _csum) {
			if (read_buffer(buffer, size_max)){
				return __buffer_size;
			}
			else {
				last_err = ERR_OTHER;
			}
        }else{
			last_err = ERR_CSUM;
            //printf("read buffer: check sum failed, get %x, computed %x\n", (unsigned int)(_csum), (unsigned int)(__csum));
        }
    }else{
		last_err = ERR_BUF_SMALL;
        //std::cout<<"read buffer:  something wrong with the input size....."<<std::endl;
    }
	return 0;
}

// -----------------------------
// ----- Manuual Coded End -----
// -----------------------------


} // namespace conf
