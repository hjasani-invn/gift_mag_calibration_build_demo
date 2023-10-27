/**
* \copyright       Copyright (C) TDK-Invensense, 2017
* \brief           Zigurat random number generator declaration
* \ingroup         PF
* \file            ziggurat_random.hpp
* \author          V.Penpukhov
* \date            28.06.2014
*/

#ifndef ZIGGURAT_RANDOM_H
#define ZIGGURAT_RANDOM_H

#include "uniform_random.hpp"
#include "normal_random.hpp"

#include <stdint.h>

namespace pf_random
{

    class ziggurat_random : public normal_random
    {
        public:
            ziggurat_random() : ziggurat_random(100) {};
            ziggurat_random(uint32_t seed_value);

            ~ziggurat_random() {};

            uint32_t get_seed();
            void seed(uint32_t seed_value);

            virtual double  randnf();

private:
    int32_t max_uni_bits;
    int32_t shift;
    uint32_t uniform_seed;
    bool use_random_seed;
    uint64_t next_rand_stat;

    static  const uint32_t RNG32_MAX_RAND = 0x7FFFFFFF;
    };
}

#endif  //ZIGGURAT_RANDOM_H
