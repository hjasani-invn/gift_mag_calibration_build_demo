/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Random number generator declaration
* \ingroup         PF
* \file            normal_random.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef NORMAL_RANDOM_H
#define NORMAL_RANDOM_H

#include <stdint.h>

#define USE_RANDOM_SEEDS 0

namespace pf_random
{
    class normal_random
    {
        public:
            normal_random(){};
            virtual ~normal_random(){};
            virtual double  randnf() = 0;
            virtual void  seed(uint32_t seed_value) {}; // it is not abstract becouse it optional implemented method
            virtual uint32_t get_seed() { return 0; }; // it is not abstract becouse it optional implemented method
    };
}

#endif  //NORMAL_RANDOM_H
