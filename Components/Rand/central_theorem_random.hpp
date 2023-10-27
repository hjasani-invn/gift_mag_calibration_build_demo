/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Random number generator declaration
* \ingroup         PF
* \file            central_theorem_random.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef CENTRAL_THEOREM_RANDOM_H
#define CENTRAL_THEOREM_RANDOM_H

#include "uniform_random.hpp"
#include "normal_random.hpp"

namespace pf_random
{

    class central_theorem_random : public normal_random
    {
        public:
            central_theorem_random(){};

            virtual ~central_theorem_random(){};

            virtual double  randnf()
            {
            	double r = 0;
                for( int i = 0; i < N; i++ )
                {
                    r += u_rand->uniform_rand();
                }

                double res = (r * (1. / ( u_rand->get_max_rand() )) - N / 2);
                return res;
            };

    private:
         static  const int N = 12;
         uniform_random *u_rand;

    };

}

#endif  //CENTRAL_THEOREM_RANDOM_H
