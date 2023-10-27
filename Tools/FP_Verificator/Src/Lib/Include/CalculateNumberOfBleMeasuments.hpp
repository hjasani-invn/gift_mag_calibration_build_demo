#ifndef CALCULATENUMBEROFBLEMESUMENTS_HPP
#define CALCULATENUMBEROFBLEMESUMENTS_HPP


#include "Fpbl.hpp"
#include "OccupancyOfPosition.hpp"

namespace FPVerificator
{
    void calculate_number_of_ble_mesuments(
        // input
        Fpbl::BleGrid &bleGrid, // grid with BLE data
        double ble_grid_size, // size of WiFi cell
        // input / output
        std::vector<OccupancyOfPosition> &irlposlist // IRL positions with number of measuments
        );
}
#endif //CALCULATENUMBEROFBLEMESUMENTS_HPP
