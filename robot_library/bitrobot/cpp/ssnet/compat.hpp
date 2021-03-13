#pragma once

#ifdef __QNX__
#ifndef nullptr
#define nullptr NULL
#endif
#include <stdint.h>
#else
#include <cstdint>
#endif



#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif