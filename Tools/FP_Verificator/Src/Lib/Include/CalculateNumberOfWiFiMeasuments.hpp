#ifndef CALCULATENUMBEROFWIFIMESUMENTS_HPP
#define CALCULATENUMBEROFWIFIMESUMENTS_HPP


#include "Fpbl.hpp"
#include "OccupancyOfPosition.hpp"

namespace FPVerificator
{
    void calculate_number_of_wifi_mesuments(
        // input
        Fpbl::WiFiGrid &wifiGrid, // grid with WiFi data
        double wifi_grid_size,    // size of WiFi cell
        // input / output
        std::vector<OccupancyOfPosition> &irlposlist // IRL positions with number of measuments
        );
}
#endif //CALCULATENUMBEROFWIFIMESUMENTS_HPP
