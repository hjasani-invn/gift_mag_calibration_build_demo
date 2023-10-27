//
//  lfsr113.hpp
//  iOS_RT_Test
//
//  Created by Vladimir Pentyukhov on 23/10/2017.
//  Copyright © 2017 Vladimir Pentyukhov. All rights reserved.
//

#ifndef lfsr113_hpp
#define lfsr113_hpp

#include "uniform_random.hpp"

namespace pf_random
{
    class lfsr113 : public uniform_random
    {
    public:
        lfsr113();
        /*  NOTE: the seed MUST satisfy
        z1 > 1, z2 > 7, z3 > 15, and z4 > 127 */
        lfsr113(uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4);
        virtual void seed();
        virtual void seed(uint32_t seed);
        virtual void seed(uint32_t s1, uint32_t s2, uint32_t s3, uint32_t s4);

        virtual uint32_t get_max_rand();
        virtual uint32_t uniform_rand();

        virtual double  randf();

    protected:
        void set_max_bits();

    private:
        uint32_t z1, z2, z3, z4;
        static  const uint32_t LFSR113_MAX_RAND = 0x00FFFFFF; //(1<<24) -1
    };
}

#endif /* lfsr113_hpp */
