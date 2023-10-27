#ifndef CALCULATECOVERAGE_HPP
#define CALCULATECOVERAGE_HPP

#include "Fpbl.hpp"
#include "OccupancyOfPosition.hpp"

namespace FPVerificator
{
    double  calculate_coverage(
        std::vector<OccupancyOfPosition> poslist, // IRL positions with number of measuments
        int32_t min_meas_num  // minimal number of measuments 
        );
}
#endif //CALCULATECOVERAGE
