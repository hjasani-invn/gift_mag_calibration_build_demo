#ifndef CREATEIRLPOSITIONS_HPP
#define CREATEIRLPOSITIONS_HPP

#include "Fpbl.hpp"
#include "OccupancyOfPosition.hpp"

namespace FPVerificator
{
    void create_irl_positions(
        // input
        Venue &venue, // venue struct with building parameters
        char* pBuffer, // buffer with IRL data
        uint32_t buffer_size, // buffer size 
        double interpolation_interval, // interval of interpolation for IRL positions
        // output
        std::vector<OccupancyOfPosition> &irlposlist // IRL positions with number of measuments
        );
}
#endif //CREATEIRLPOSITIONS_HPP
