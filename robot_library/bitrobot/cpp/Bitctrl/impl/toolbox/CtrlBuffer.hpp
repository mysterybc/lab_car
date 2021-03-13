#pragma once
#include <string>
#include <iostream>

#ifdef __GNUC__
#if __GNUC__ < 4 || (__GNUC__ == 4 && __GNUC_MINOR__ < 6)
#include <limits>
namespace std {
	typedef unsigned char  uint8_t;
	typedef unsigned short uint16_t;
}
#else
#include <cstdint>
#endif
#else
#include <cstdint>
#endif



namespace conf {

template<class ValueType, class StoreType = std::uint8_t>
class Quantizer {
public:
	Quantizer(const ValueType& _vmin = 0, const ValueType& _vmax = 1)
		: _vmin(_vmin), _vmax(_vmax)
	{
		_vOverStore = (_vmax - _vmin) * 1.0 / std::numeric_limits<StoreType>::max();
	}
	double eMax() const { return _vOverStore / 2; }
	ValueType vmin() const { return _vmin; }
	ValueType vmax() const { return _vmax; }
	StoreType store(const ValueType& value) const {
		ValueType v = value > _vmax ? _vmax :
			value < _vmin ? _vmin : value;
		return static_cast<StoreType>((v - _vmin) / _vOverStore + 0.5);
	}
	ValueType retrive(const StoreType& value) const {
		return static_cast<ValueType>((value * _vOverStore) + _vmin);
	}
private:
	ValueType _vmin, _vmax;
	double _vOverStore;
};


//
// Declaration of Interface CtrlPack
//
class CtrlPack {
public:
	//
	// Enum definition of Bit Postions
	//
	enum {
		IsFaulty_Bit = 0,
		AgentIDValue_Bit = 1,
		TaskStateValue_Bit = 2,
		HFieldValue_Bit = 3,
		LatencyValue_Bit = 4,
		ShapeValue_Bit = 5,
		StateInfoValue_Bit = 6,
		GFieldValue_Bit = 7
	};

	//
	// Constructor for CtrlPack
	// - Auto reset on construction
	CtrlPack();

	//
	// Reset Function
	// - Make all options & values invalid
	void reset();

	//
	// Binary Option IsFaulty
	//
	CtrlPack& withIsFaulty(const bool& with);
	CtrlPack& resetIsFaulty();
	bool withIsFaulty() const;
	bool validIsFaulty() const;

	//
	// ValueConfig AgentID
	//
	CtrlPack& setAgentID(const std::uint8_t& agentID);
	CtrlPack& resetAgentID();
	bool validAgentID() const;
	bool getAgentID(std::uint8_t& agentID) const;

	//
	// ValueConfig TaskState
	//
	CtrlPack& setTaskState(const std::uint8_t& taskID, const std::uint8_t& task_state);
	CtrlPack& resetTaskState();
	bool validTaskState() const;
	bool getTaskState(std::uint8_t& taskID, std::uint8_t& task_state) const;

	//
	// ValueConfig HField
	//
	CtrlPack& setHField(const std::uint8_t& hfr, const float& hfx, const float& hfy);
	CtrlPack& resetHField();
	bool validHField() const;
	bool getHField(std::uint8_t& hfr, float& hfx, float& hfy) const;

	//
	// ValueConfig Latency
	//
	CtrlPack& setLatency(const std::uint16_t& lat1, const std::uint16_t& lat2, const std::uint16_t& lat3, const std::uint16_t& lat4);
	CtrlPack& resetLatency();
	bool validLatency() const;
	bool getLatency(std::uint16_t& lat1, std::uint16_t& lat2, std::uint16_t& lat3, std::uint16_t& lat4) const;

	//
	// ValueConfig Shape
	//
	CtrlPack& setShape(const std::uint8_t& num_agent, const std::uint8_t& shape_index, const std::uint8_t& fmgroup, const std::uint16_t& priority);
	CtrlPack& resetShape();
	bool validShape() const;
	bool getShape(std::uint8_t& num_agent, std::uint8_t& shape_index, std::uint8_t& fmgroup, std::uint16_t& priority) const;

	//
	// ValueConfig StateInfo
	//
	CtrlPack& setStateInfo(const float& my_x, const float& my_y, const float& my_vx, const float& my_vy, const std::uint16_t& my_angle);
	CtrlPack& resetStateInfo();
	bool validStateInfo() const;
	bool getStateInfo(float& my_x, float& my_y, float& my_vx, float& my_vy, std::uint16_t& my_angle) const;

	//
	// ValueConfig GField
	//
	CtrlPack& setGField(const std::uint8_t& ng, const std::uint8_t& rg, const float& hgx, const float& hgy, const float& zg);
	CtrlPack& resetGField();
	bool validGField() const;
	bool getGField(std::uint8_t& ng, std::uint8_t& rg, float& hgx, float& hgy, float& zg) const;

	//
	// Interface function to read/write configs from raw buffer
	//

	// Compute required buffer size for current configurations
	std::size_t buffer_size() const;

	// Writing configurations to buffer
	std::size_t write_buffer(void* buffer, std::size_t max_size) const;

	// Reading configurations from buffer
	bool read_buffer(const void* buffer, std::size_t size);

	// Update stored values
	// Return whether values have changed
	bool update(const CtrlPack& conf);

	// Test equal
	// Return if this equals input
	bool operator ==(const CtrlPack& other) const;


private:
	std::uint8_t _conf;
	std::uint8_t _valid;
	std::uint8_t __agentID;
	std::uint8_t __taskID;
	std::uint8_t __task_state;
	std::uint8_t __hfr;
	float __hfx;
	float __hfy;
	std::uint16_t __lat1;
	std::uint16_t __lat2;
	std::uint16_t __lat3;
	std::uint16_t __lat4;
	std::uint8_t __num_agent;
	std::uint8_t __shape_index;
	std::uint8_t __fmgroup;
	std::uint16_t __priority;
	float __my_x;
	float __my_y;
	float __my_vx;
	float __my_vy;
	std::uint16_t __my_angle;
	std::uint8_t __ng;
	std::uint8_t __rg;
	float __hgx;
	float __hgy;
	float __zg;

	// -----------------------------
	// ---- Manuual Coded Begin ----
	// -----------------------------
public:
	typedef std::uint8_t CSumType;
	static const std::size_t __buffer_size = 80;
	enum {
		ERR_GOOD = 0,
		ERR_BUF_SMALL,
		ERR_CSUM,
		ERR_OTHER
	};
	int lastError() const {
		return last_err;
	}
	//Reset all valid infomation
	CtrlPack& resetAll() {
		_valid = 0;
		_conf = 0;
		last_err = 0;
		return *this;
	}
	std::size_t buffer_size_all() const {
		return __buffer_size;
	}
	bool operator != (const CtrlPack& other) const {
		return !(this->operator==(other));
	}
	CtrlPack& setLatency(const std::uint16_t* lat) {
		return setLatency(lat[0], lat[1], lat[2], lat[3]);
	}
	bool getLatency(std::uint16_t* lat) const {
		return getLatency(lat[0], lat[1], lat[2], lat[3]);
	}

	CtrlPack& setHField(float r, float x, float y) {
		return setHField(q_hfr.store(r), x, y);
	}
	bool getHField(float& hfr, float& hfx, float& hfy) const {
		std::uint8_t tmp;
		if (getHField(tmp, hfx, hfy)) {
			hfr = q_hfr.retrive(tmp);
			return true;
		}
		return false;
	}
	CtrlPack& setStateInfo(const float& my_x, const float& my_y, const float& my_vx, const float& my_vy, const float& my_angle) {
		return setStateInfo(my_x, my_y, my_vx, my_vy, q_th.store(my_angle));
	}

	std::size_t write_buffer_all(void* buffer, std::size_t max_size, char prefill = '\0');
	std::size_t read_buffer_all(const void* buffer, std::size_t size);

public:
	int last_err;
	CSumType _last_csum_get;
	CSumType _last_csum_compute;

	static void print_quanerr();
	static Quantizer<float, std::uint8_t> q_hfr;
	static Quantizer<float, std::uint16_t> q_th;
	// -----------------------------
	// ----- Manuual Coded End -----
	// -----------------------------
};

} // namespace conf
