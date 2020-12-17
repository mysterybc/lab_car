// //////////////////////////////////////////////////////////
// mersenne.h
// Copyright (c) 2014 Stephan Brumme. All rights reserved.
// see http://create.stephan-brumme.com/disclaimer.html
//

#pragma once

#if defined(__QNX__)
#include <stdint.h>
#else
#include <cstdint>
#endif

/// Mersenne twister pseudo-random number generator
/** algorithm invented by Makoto Matsumoto and Takuji Nishimura **/
class MersenneTwister
{
  /// state size
  enum   { SizeState = 624 };
  /// internal state
  std::uint32_t state[SizeState];
  /// offset of next state's word
  int      next;

public:
  /// generate initial internal state
  MersenneTwister(std::uint32_t seed = 5489);

  /// return a random 32 bit number
  std::uint32_t operator()();

private:
  /// create new state (based on old one)
  void twist();
};
