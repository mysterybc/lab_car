#pragma once
#include <vector>
#ifdef __QNX__
#include <stdint.h>
#else
#include <cstdint>
#endif

struct RobotStateLog {
	std::uint32_t loop_counter;
	std::uint8_t task_motor;
	std::uint8_t task_aux;
	float x, y, heading;
	float uv, uw;
	float xd, yd;
	float task_progess;
};

struct RobotCommand {
	enum {
		StopAll,
		StopMotorTask,
		StopAuxTask,
		TaskStart,
		TaskStop,
		Pause,
		Resume
	};
	std::uint8_t command;
	std::uint8_t command_ex;
	float dataX;
	float dataY;
};

struct RobotTeleMsg {
	float v;
	float w;
};

struct RobotConfigState {
	float x;
	float y;
	float vx;
	float vy;
	float heading;
};

struct RobotConfigPath {
	enum {
		PathSingle, 
		PathMulti
	};
	std::uint8_t pathType;
	float uniform_velocity;
	std::vector<float> qx, qy;
	std::vector<float> vd, wd;
};

struct RobotConfigMap {
	enum {
		MapReset,
		MapAppend
	};
	std::uint8_t command;

	struct Point  { float x, y; };
	struct Circle { Point q; float r; };
	struct Line   { Point p, q; };
	typedef std::vector<Point> CPoly;

	std::vector<Circle> obPoint;
	std::vector<Line>   obLine;
	std::vector<CPoly>  obCPoly;
};


