/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Random number generator
* \ingroup         PF
* \file            rng32.cpp
* \author          M.Frolov
* \date            28.11.2014
*/

#include "rng32.hpp"

#include <iostream>

using namespace pf_random;

rng32::rng32()
{
    next_rand = 0;
    set_max_bits();
}


rng32::rng32(uint32_t seed)
{
    next_rand = seed;
    set_max_bits();
}

void rng32::seed()
{
    next_rand = 0;
}

void rng32::seed(uint32_t seed)
{
    next_rand = seed;
}

void rng32::seed(uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4)
{
    next_rand = s1 + s2 + s3 + s4;
}

void rng32::set_max_bits()
{
    uint32_t n = RNG32_MAX_RAND;
    max_bits = 0;
    while (n > 0)
    {
        n >>= 1;
        max_bits++;
    }
}


uint32_t rng32::get_max_rand()
{
    return RNG32_MAX_RAND;
}


uint32_t rng32::uniform_rand()
{
    //  next_rand = next_rand * 1103515245UL + 12345;
    //uint32_t r = ( uint32_t )( ( next_rand / 65536UL ) & RNG32_MAX_RAND );
    next_rand = next_rand * 0x5DEECE66D + 0xB;
    uint32_t r = (uint32_t)(next_rand & RNG32_MAX_RAND);
    //uint32_t r = (uint32_t)((next_rand >> 32) & RNG32_MAX_RAND);

//    std::cout << "uniform_rand()   " << (uint64_t)this << "   " << (uint64_t)r << std::endl;


    return r;
}

double  rng32::randf()
{
    double r = static_cast<double>((uniform_rand() + 1UL)) / (RNG32_MAX_RAND + 2UL);
    return r;
}


//const int rng32::N = 12;

