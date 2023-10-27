/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Random number generator declaration
* \ingroup         PF
* \file            uniform_random.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef UNIFORM_RANDOM_H
#define UNIFORM_RANDOM_H

#include <stdint.h>

namespace pf_random
{
    class uniform_random
    {
        public:
            uniform_random(){};
            virtual ~uniform_random(){};

            virtual void seed() = 0;
            virtual void seed(uint32_t seed) = 0;
            virtual void seed(uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4) = 0;

            uint32_t get_max_bits() { return max_bits; }
            virtual uint32_t get_max_rand() = 0;
            virtual uint32_t uniform_rand() = 0;

            virtual double  randf() = 0;

    protected:
        void set_max_bits();
        int max_bits;

    };

}

#endif  //UNIFORM_RANDOM_H
