
#include "Fpbl.hpp"
#include "OccupancyOfPosition.hpp"

namespace FPVerificator
{
    double  calculate_coverage(std::vector<OccupancyOfPosition> poslist, int32_t min_meas_num)
    {
        double  coverage;
        int32_t counter = 0;

        for (auto it = poslist.begin(); it != poslist.end(); it++)
        {
            OccupancyOfPosition  pos = *it;
            if (pos.number_of_measurements >= min_meas_num)
                counter++;
        }

        coverage = ((double)counter / poslist.size()) * 100;
        return coverage;
    }
}
