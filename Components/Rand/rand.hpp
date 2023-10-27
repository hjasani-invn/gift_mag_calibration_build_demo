/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Random number generator declaration
* \ingroup         PF
* \file            rand.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef PARTICLE_FILTER_RAND_H
#define PARTICLE_FILTER_RAND_H

#if 0 //def _WIN32
#include "stdint.h"
#else
#include <stdint.h>
#endif
namespace pf_random
{
    static  const uint32_t RNG32_MAX_RAND = 0x7FFF;

    class rng32
    {
        public:
            rng32();
            explicit rng32( uint32_t seed );
            void seed( uint32_t seed );
            uint32_t rand();

            template<class T> T  randf()
            {
                T r = static_cast<T>( (rand() + 1UL ) )/ ( pf_random::RNG32_MAX_RAND + 2UL );
                return r;
            }

            template<class T> T  randnf()
            {
                T r = 0;
                for( int i = 0; i < rng32::N; i++ )
                {
                    r += rand();
                }

                T res = ( r * ( 1. / ( pf_random::RNG32_MAX_RAND ) ) - N / 2 );
                return res;
            }
        private:
            uint32_t next_rand;
            static  const int N;
    };


    static  const uint32_t LFSR113_MAX_RAND = 0x00FFFFFF; //(1<<24) -1
    class lfsr113
    {
        public:
            lfsr113();
            /*  NOTE: the seed MUST satisfy
            z1 > 1, z2 > 7, z3 > 15, and z4 > 127 */
            lfsr113( uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4 );
            void seed(uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4 );
            uint32_t rand();

            template<class T> T  randf()
            {
                T r = static_cast<T>(5.96046412226772e-008) * (rand() + 1UL );
                return r;
            }

            template<class T> T  randnf()
            {
                T r = 0;
                for( int i = 0; i < lfsr113::N; i++ )
                {
                    r += rand();
                }

                T res = ( r * ( 1. / ( pf_random::LFSR113_MAX_RAND ) ) - N / 2 );
                return res;
            }
        private:
            uint32_t z1, z2, z3, z4;
            static  const int N;
    };


}

#endif  //PARTICLE_FILTER_RAND_H
