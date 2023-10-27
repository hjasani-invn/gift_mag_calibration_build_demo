/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Random number generator declaration
* \ingroup         PF
* \file            rng32.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef RNG32_H
#define RNG32_H

#include "uniform_random.hpp"

namespace pf_random
{
    class rng32 : public uniform_random
    {
        public:
            rng32();
            explicit rng32( uint32_t seed );
            virtual void seed();
            virtual void seed(uint32_t seed);
            virtual void seed(uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4);

            virtual uint32_t get_max_rand();
            virtual uint32_t uniform_rand();

            virtual double  randf();

    protected:
        void set_max_bits();
    
    private:
            uint64_t next_rand;
            static  const uint32_t RNG32_MAX_RAND = 0x7FFFFFFF;

    };

}

#endif  //RNG32_H
