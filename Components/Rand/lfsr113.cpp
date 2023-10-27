//
//  lfsr113.cpp
//  iOS_RT_Test
//
//  Created by Vladimir Pentyukhov on 23/10/2017.
//  Copyright © 2017 Vladimir Pentyukhov. All rights reserved.
//

#include "lfsr113.hpp"

using namespace pf_random;


lfsr113::lfsr113()
{
  z1=2;
  z2=8;
  z3=16;
  z4=128;

  set_max_bits();
}

lfsr113::lfsr113( uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4 )
{
    z1 = s1;
    z2 = s2;
    z3 = s3;
    z4 = s4;

    set_max_bits();
}

void lfsr113::seed()
{
    z1 = 2;
    z2 = 8;
    z3 = 16;
    z4 = 128;
}

void lfsr113::seed(uint32_t seed)
{
    z1 = seed;
    z2 = seed << 2;
    z3 = seed << 4;
    z4 = seed << 7;
}


void lfsr113::seed(uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4 )
{
    z1 = s1;
    z2 = s2;
    z3 = s3;
    z4 = s4;
}

void lfsr113::set_max_bits()
{
    uint32_t n = LFSR113_MAX_RAND;
    max_bits = 0;
    while (n > 0)
    {
        n >>= 1;
        max_bits++;
    }
}


uint32_t lfsr113::get_max_rand()
{
    return LFSR113_MAX_RAND;
}



uint32_t lfsr113::uniform_rand()
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
  r = (z1 ^ z2 ^ z3 ^ z4) & LFSR113_MAX_RAND;
  return r;
}

double  lfsr113::randf()
{
    double r = static_cast<double>(5.96046412226772e-008) * (uniform_rand() + 1UL);
    return r;
}



//const int lfsr113::N = 12;
