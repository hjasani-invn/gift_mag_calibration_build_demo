
#include <cmath>
#include <algorithm>
#include <string>

#include <iostream>
//#include <iomanip>
#include <fstream>
#include <sstream>

#include <stdint.h>

#include "CoordinateConverter.h"

#include "CalculateNumberOfWiFiMeasuments.hpp"
#include "OccupancyOfPosition.hpp"

#include "imm_mapobject.h"

namespace FPVerificator
{
    void calculate_number_of_wifi_mesuments(
        // input
        Fpbl::WiFiGrid &wifiGrid, // grid with WiFi data
        double wifi_grid_size,    // size of WiFi cell
        // input / output
        std::vector<OccupancyOfPosition> &irlpositions // IRL positions with number of measuments
        )
    {
        OccupancyOfPosition nearcell;
        std::vector<OccupancyOfPosition> nearcelllist;

        for (auto it = irlpositions.begin(); it != irlpositions.end(); it++)
        {
            OccupancyOfPosition  pos = *it;
            //std::cout << pos.X << "         " << pos.Y;
            //////////////////
            int32_t number = 0;
            double x = 0, y = 0;
            bool flag;
            double min_distance = wifi_grid_size * sqrt(2.0);
            double distance;

            nearcelllist.clear();

            for (uint32_t k = 0; k < wifiGrid.size(); k++)
            {
                flag = false;
                x = wifiGrid[k].coordinates.x;
                y = wifiGrid[k].coordinates.y;
#if 0  
                //if (fabs(pos.X - x) < mag_grid_size / 2 &&
                //fabs(pos.Y - y) < mag_grid_size / 2
                if (fabs(pos.X - x) < mag_grid_size   &&
                    fabs(pos.Y - y) < mag_grid_size 
                    ) 
#else
                distance = std::sqrt((pos.X - x)*(pos.X - x) + (pos.Y - y)*(pos.Y - y));
                if (distance < min_distance)
#endif
                {
                    //min_distance = distance;
                    //number = magneticGrid[k].magData.size();
                    //flag = true;
                    //break;
                    std::sort(wifiGrid[k].wifiData[0].scanWiFi.begin(), wifiGrid[k].wifiData[0].scanWiFi.end(), compare_RSSIMeasurement_by_mac);
                    int32_t  macaddressCounter = 0;
                    uint64_t prevMacAddress = 0;
                    for (auto scanWiFiIt = wifiGrid[k].wifiData[0].scanWiFi.begin(); scanWiFiIt != wifiGrid[k].wifiData[0].scanWiFi.end(); scanWiFiIt++)
                    {
                        uint64_t currentMacAddress = scanWiFiIt->mac;
                        //std::cout << macaddressCounter << "\t" << currentMacAddress << std::endl;

                        if (currentMacAddress != prevMacAddress)
                            macaddressCounter++;
                        prevMacAddress = currentMacAddress;
                    }

                    nearcell.number_of_measurements = wifiGrid[k].wifiData[0].scanWiFi.size() / macaddressCounter;
                    nearcell.X = x;
                    nearcell.Y = y;
                    nearcelllist.push_back(nearcell);
                }
            }
            //if (! flag) // no cells
            if (nearcelllist.empty()) // no cells
            {
                x = ((uint32_t)(pos.X)) + wifi_grid_size / 2;
                y = ((uint32_t)(pos.Y)) + wifi_grid_size / 2;
            }
            else // cell with maximum number
            {
                if (nearcelllist.size() > 1)
                {
                    std::sort(nearcelllist.begin(), nearcelllist.end(), compare_occupancy_of_positions_by_number_of_measurements);
                }
                nearcell = nearcelllist.back(); // the biggest element
                x = nearcell.X;
                y = nearcell.Y;
                number = nearcell.number_of_measurements;
            }
            it->number_of_measurements = number;
            //////////////////
        }
    }
}
