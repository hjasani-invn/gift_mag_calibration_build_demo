/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Random number generator
* \ingroup         PF
* \file            rand.cpp
* \author          M.Frolov
* \date            28.11.2014
*/

#include "rand.hpp"


using namespace pf_random;

rng32::rng32()
{
    next_rand = 0;
}


rng32::rng32(uint32_t seed)
{
    next_rand = seed;
}

void rng32::seed(uint32_t seed)
{
    next_rand = seed;
}

uint32_t rng32::rand()
{
    next_rand = next_rand * 1103515245UL + 12345;
    uint32_t r = ( uint32_t )( ( next_rand / 65536UL ) & pf_random::RNG32_MAX_RAND );
    return r;
}

const int rng32::N = 12;


lfsr113::lfsr113()
{
  z1=2;
  z2=8;
  z3=16;
  z4=128;
}

lfsr113::lfsr113( uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4 )
{
    z1 = s1;
    z2 = s2;
    z3 = s3;
    z4 = s4;
}

void lfsr113::seed(uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4 )
{
    z1 = s1;
    z2 = s2;
    z3 = s3;
    z4 = s4;
}


uint32_t lfsr113::rand()
{
  uint32_t b;
  uint32_t r;
  b = (((z1 << 6) ^ z1) >> 13);
  z1 = (((z1 & 4294967294UL) << 18) ^ b);
  b = (((z2 << 2) ^ z2) >> 27);
  z2 = (((z2 & 4294967288UL) << 2) ^ b);
  b = (((z3 << 13) ^ z3) >> 21);
  z3 = (((z3 & 4294967280UL) << 7) ^ b);
  b = (((z4 << 3) ^ z4) >> 12);
  z4 = (((z4 & 4294967168UL) << 13) ^ b);
  r = (z1 ^ z2 ^ z3 ^ z4)&pf_random::LFSR113_MAX_RAND;
  return r;
}

const int lfsr113::N = 12;
