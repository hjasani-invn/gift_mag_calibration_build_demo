#ifndef CALCULATENUMBEROFMAGMESUMENTS_HPP
#define CALCULATENUMBEROFMAGMESUMENTS_HPP


#include "Fpbl.hpp"
#include "OccupancyOfPosition.hpp"

namespace FPVerificator
{
    void calculate_number_of_mag_mesuments(
        // input
        Fpbl::MagneticGrid &magneticGrid, // grid with magnetic data
        double mag_grid_size,             // size of magnetic cell
        // input / output
        std::vector<OccupancyOfPosition> &irlposlist // IRL positions with number of measuments
        );
}
#endif //CALCULATENUMBEROFMAGMESUMENTS_HPP
