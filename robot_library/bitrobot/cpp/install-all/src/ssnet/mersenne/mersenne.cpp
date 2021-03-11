// //////////////////////////////////////////////////////////
// mersenne.cpp
// Copyright (c) 2014 Stephan Brumme. All rights reserved.
// see http://create.stephan-brumme.com/disclaimer.html
//

#include "mersenne.h"

/// generate initial internal state
MersenneTwister::MersenneTwister(std::uint32_t seed)
: next(0)
{
  state[0] = seed;
  for (int i = 1; i < SizeState; i++)
    state[i] = 1812433253UL * (state[i-1] ^ (state[i-1] >> 30)) + i;

  // let's twist'n'shout ...
  twist();
}


/// return a random 32 bit number
uint32_t MersenneTwister::operator()()
{
  // compute new state ?
  if (next >= SizeState)
    twist();

  // shuffle bits around
  uint32_t x = state[next++];
  x ^=  x >> 11;
  x ^= (x <<  7) & 0x9d2c5680;
  x ^= (x << 15) & 0xefc60000;
  x ^=  x >> 18;
  return x;
}


/// create new state (based on old one)
void MersenneTwister::twist()
{
  const int M = 397;
  const int FirstHalf = SizeState - M;

  // first 624-397=227 words
  int i;
  for (i = 0; i < FirstHalf; i++)
  {
    std::uint32_t bits = (state[i] & 0x80000000) | (state[i + 1] & 0x7fffffff);
    state[i] = state[i + M]         ^ (bits >> 1) ^ ((bits & 1) * 0x9908b0df);
  }
  // remaining words (except the very last one)
  for ( ; i < SizeState - 1; i++)
  {
    std::uint32_t bits = (state[i] & 0x80000000) | (state[i + 1] & 0x7fffffff);
    state[i] = state[i - FirstHalf] ^ (bits >> 1) ^ ((bits & 1) * 0x9908b0df);
  }

  // last word is computed pretty much the same way, but i + 1 must wrap around to 0
  std::uint32_t bits = (state[i] & 0x80000000) | (state[0] & 0x7fffffff);
  state[i] = state[M - 1] ^ (bits >> 1) ^ ((bits & 1) * 0x9908b0df);

  // word used for next random number
  next = 0;
}
