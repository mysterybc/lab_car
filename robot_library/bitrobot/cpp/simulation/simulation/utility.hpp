#pragma once
#include <cmath>
#include <fstream>
#include <string>

#ifndef PI
#define PI 3.141592653589793
#endif

inline double rad2deg(double x) { return x / PI * 180; }
inline double deg2rad(double x) { return x / 180 * PI; }
inline double m2cm(double x)    { return x * 100; }
inline double cm2m(double x)    { return x / 100; }
inline double ms2sec(double x)  { return x / 1000; }
inline double sec2ms(double x)  { return x * 1000; }

//
// Here we need a sleep function that works everywhere

void my_sleepms(int ms);




inline bool checkFileExist(const std::string& name) {
	std::ifstream f(name.c_str());
	return f.good();
}
