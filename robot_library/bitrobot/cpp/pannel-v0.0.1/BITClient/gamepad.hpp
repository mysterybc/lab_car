#pragma once

#include <vector>		// for vector
#include <cstdlib>		// for size_t
#include <utility>		// for pair
#include <string>		// for string
namespace gamepad{
	class Joystick{
	public:
		enum{
			btnY = 0, 	btnB = 1, 
			btnA = 2, 	btnX = 3,
			btnLB = 4,  btnRB = 5,
			btnLT = 6,  btnRT = 7,
			btnBack = 8, btnStart = 9
		};
		enum{
			axLS = 0, axRS = 1, axPannel = 2
		};

		inline Joystick();
		inline ~Joystick();
		
		inline bool X();
		inline bool Y();
		inline bool B();
		inline bool A();
		inline bool LB();
		inline bool LT();
		inline bool RB();
		inline bool RT();
		inline bool BtnStart();
		inline bool BtnBack();
		inline bool Btn(size_t index);	// 0, 1, 2, ...
		inline std::pair<float, float> AxisLS();
		inline std::pair<float, float> AxisRS();
		inline std::pair<float, float> AxisPannel();
		inline std::pair<float, float> Axis(size_t index); // 0, 1
		static inline std::string getBtnName(size_t index);
		static inline std::string getAxisName(size_t index);

		// Basic Connections
		inline bool connect(int js_number = 0);
		inline bool connected() const;
		inline int  read();	// -1: not connected, >=0: num of events readed

		// Get Readed Events
		// The list is of 0, 1
		const std::vector<int>& BottonChanged() const { return btn_changed; }
		const std::vector<int>& AxisChanged() const { return axis_changed; }
		
	private:
		void* data;	// platform dependent data
		
		void reset_changed(){
			std::fill(btn_changed.begin(), btn_changed.end(), 0);
			std::fill(axis_changed.begin(), axis_changed.end(), 0);
		}
		std::vector<int> btn_changed;
		std::vector<int> axis_changed;
	};


	bool Joystick::X(){ return Btn(Joystick::btnX); }
	bool Joystick::Y(){ return Btn(Joystick::btnY); }
	bool Joystick::B(){ return Btn(Joystick::btnB); }
	bool Joystick::A(){ return Btn(Joystick::btnA); }
	bool Joystick::LB(){ return Btn(Joystick::btnLB); }
	bool Joystick::LT(){ return Btn(Joystick::btnLT); }
	bool Joystick::RB(){ return Btn(Joystick::btnRB); }
	bool Joystick::RT(){ return Btn(Joystick::btnRT); }
	bool Joystick::BtnStart(){ return Btn(Joystick::btnStart); }
	bool Joystick::BtnBack(){ return Btn(Joystick::btnBack); }

	std::pair<float, float> Joystick::AxisLS()    { return Axis(Joystick::axLS); }
	std::pair<float, float> Joystick::AxisRS()    { return Axis(Joystick::axRS); }
	std::pair<float, float> Joystick::AxisPannel(){ return Axis(Joystick::axPannel); }

	std::string Joystick::getBtnName(size_t index) {
		switch (index){
#define mapping(num, val) case num : return #val
			mapping(Joystick::btnX, X);
			mapping(Joystick::btnY, Y);
			mapping(Joystick::btnB, B);
			mapping(Joystick::btnA, A);
			mapping(Joystick::btnLB, LB);
			mapping(Joystick::btnLT, LT);
			mapping(Joystick::btnRB, RB);
			mapping(Joystick::btnRT, RT);
			mapping(Joystick::btnStart, START);
			mapping(Joystick::btnBack, BACK);
#undef mapping
		}
		return "Unknown Button";
	}

	std::string Joystick::getAxisName(size_t index) {
		if (index == Joystick::axLS)	 return "LS";
		if (index == Joystick::axRS)	 return "RS";
		if (index == Joystick::axPannel) return "Pannel";
		return "Unknown Axis";
	}

}	// namespace gamepad

#ifdef _WIN32
#include <Windows.h>

namespace gamepad{

	namespace impl{
                inline int getJoyID(int id){
			if (id == 0) return JOYSTICKID1;
                        if (id == 1) return JOYSTICKID2;

                        // Only Two Joystick is supported !!
			return -1;
		}

		inline bool getJoyInfo(JOYINFOEX& joyinfo, int joyID = 0){
			joyID = getJoyID(joyID);

			MMRESULT  ret;
			joyinfo.dwFlags = JOY_RETURNALL;
			joyinfo.dwSize = sizeof(joyinfo);
			ret = joyGetPosEx(joyID, &joyinfo);
			return ret == JOYERR_NOERROR;
		}
		inline float dead_zone(float val, float dzone){
			if (val > dzone)       return (val - dzone) / (1 - dzone);
			else if (val < -dzone) return (val + dzone) / (1 - dzone);
			else return 0;
		}
		inline std::pair<float, float> getXY(const JOYINFOEX& info, float dzone = 0.05f){
			float x = 2 * float(info.dwXpos / 65535.0 - 0.5);
			float y = 2 * float(0.5 - info.dwYpos / 65535.0);
			return{ dead_zone(x, dzone), dead_zone(y, dzone) };
		}

		inline std::pair<float, float> getRZ(const JOYINFOEX& info, float dzone = 0.05f){
			float x = 2 * float(info.dwZpos / 65535.0 - 0.5);
			float y = 2 * float(0.5 - info.dwRpos / 65535.0);
			return{ dead_zone(x, dzone), dead_zone(y, dzone) };
		}

		inline std::pair<float, float> getUV(const JOYINFOEX& info, float dzone = 0.05f){
			float x = 2 * float(info.dwUpos / 65535.0 - 0.5);
			float y = 2 * float(0.5 - info.dwVpos / 65535.0);
			return{ dead_zone(x, dzone), dead_zone(y, dzone) };
		}

		inline bool axis_changed(const JOYINFOEX& p, const JOYINFOEX& q, size_t index){
#define fast_comp( item1, item2 ) return ( p. ## item1 != q. ## item1) || ( p. ## item2 != q. ## item2 )
			if (index == 0)	fast_comp(dwXpos, dwYpos);
			if (index == 1)	fast_comp(dwZpos, dwRpos);
			if (index == 2)	fast_comp(dwUpos, dwVpos);
			return false;
#undef fast_comp
		}
		inline bool check_button(const JOYINFOEX& info, size_t index){
			return (info.dwButtons & (1 << index) ) != 0;
		}
	}

	struct JoyData{
		bool connected;
		bool last_valid;
		JOYINFOEX infolast;
		
		char  btn[16];
		float axis[16];
		int   currJsID;
	};

	Joystick::Joystick(){
		JoyData* d = new JoyData;
		d->last_valid = false;
		d->connected  = false;
        d->currJsID   = 0;

		btn_changed.resize(16);
		axis_changed.resize(5);
		reset_changed();

		data = (void*)d;
	}
	Joystick::~Joystick(){
		delete (JoyData*)data;
	}

	bool Joystick::connect(int js_number){
		if (((JoyData*)data)->connected){
			// Reset Data
                        ((JoyData*)data)->connected = false;
                        ((JoyData*)data)->last_valid = false;
		}

		JOYINFOEX tmp;
		((JoyData*)data)->connected = impl::getJoyInfo(tmp, js_number);
		if (((JoyData*)data)->connected){
			((JoyData*)data)->currJsID = js_number;
			return true;
		}
		((JoyData*)data)->currJsID = -1;
		return false;
	}
	bool Joystick::connected() const{
		return ((JoyData*)data)->connected;
	}
	int  Joystick::read(){
		JOYINFOEX info;
		JoyData& joy = *((JoyData*)data);
		if (impl::getJoyInfo(info, joy.currJsID)){
			if (!joy.last_valid){
				joy.infolast = info;
				joy.last_valid = true;
			}
			else{
				size_t changed = 0;
				reset_changed();
				for (size_t i = 0; i < 16; ++i){
					btn_changed[i] = impl::check_button(info, i) != impl::check_button(joy.infolast, i);
					if (btn_changed[i]) changed++;
				}
				for (size_t i = 0; i < 4; ++i){
					axis_changed[i] = impl::axis_changed(info, joy.infolast, i);
					if (axis_changed[i]) changed++;
				}
				joy.infolast = info;
				return changed;
			}
			return 1;
		}
		else{
			return -1;
		}
	}

	bool Joystick::Btn(size_t index){
		return impl::check_button(((JoyData*)data)->infolast, index);
	}
	std::pair<float, float> Joystick::Axis(size_t index){
		if (index == 0) return impl::getXY(((JoyData*)data)->infolast);
		if (index == 1) return impl::getRZ(((JoyData*)data)->infolast);
		if (index == 2) return impl::getUV(((JoyData*)data)->infolast);
		return{ 0.f, 0.f };
	}


} // namespace  gamepad

#else
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <algorithm>


namespace gamepad{
	struct JoyData{
		int   fd;
		char  btn[24];
		float axis[8];
	};
	Joystick::Joystick(){
		JoyData* d = new JoyData;
		d->fd = -1;
		std::fill(d->btn, d->btn+24, 0);
		std::fill(d->axis, d->axis+8, 0);
		data = (void*)d;
		
		btn_changed.resize(24); 
		axis_changed.resize(8);
		reset_changed();
	}
	Joystick::~Joystick(){
		delete (JoyData*)(data);
	}
	bool Joystick::connect(int js_number){
		if (connected()){
			close(((JoyData*)data)->fd);

			// Reset data
			std::fill(((JoyData*)data)->btn, ((JoyData*)data)->btn+24, 0);
			std::fill(((JoyData*)data)->axis, ((JoyData*)data)->axis+8, 0);
		}

		std::string target = "/dev/input/js0";
		target.back() += js_number;
		((JoyData*)data)->fd = open(target.c_str(), O_RDONLY | O_NONBLOCK);
		return ((JoyData*)data)->fd != -1;
	}
	bool Joystick::connected() const{
		return ((JoyData*)data)->fd != -1;
	}
	int Joystick::read(){
		JoyData& joyinfo = *((JoyData*)data);
		if (joyinfo.fd == -1)
			return -1;
		
		js_event jslist[16];
		size_t num_read = 0;
		for (; num_read < 16; ++num_read){
			if (::read(joyinfo.fd, jslist+num_read, sizeof(struct js_event)) <= 0)
				break;
		}
		
		reset_changed();
		for (size_t i=0;i<num_read;++i){
			auto& js = jslist[i];
			switch (js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_AXIS:
					joyinfo.axis[js.number] = (float)(js.value / 32767.0f);
					if (js.number % 2 != 0)
						joyinfo.axis[js.number] = -joyinfo.axis[js.number];
					axis_changed[js.number / 2] = 1;
					break;
				case JS_EVENT_BUTTON:
					joyinfo.btn[js.number] = (char)(js.value);
					btn_changed[js.number] = 1;
					break;
			}
		}
		return (int)num_read;
	}
	
	bool Joystick::Btn(size_t index){ 
		return ((JoyData*)data)->btn[index] != 0; 
	}
	
	std::pair<float, float> Joystick::Axis(size_t index){
		float* axis = ((JoyData*)data)->axis;
		return { axis[index * 2], axis[index * 2 + 1] };
	}
}	// namespace gamepad
#endif
