
#include <cmath>
#include <algorithm>
#include <string>

#include <stdint.h>

#include "CoordinateConverter.h"

#include "CalculateNumberOfMagMeasuments.hpp"
#include "OccupancyOfPosition.hpp"

#include "imm_mapobject.h"

namespace FPVerificator
{
    void calculate_number_of_mag_mesuments(
        // input
        Fpbl::MagneticGrid &magneticGrid, // grid with magnetic data
        double mag_grid_size,             // size of magnetic cell
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
            double min_distance = mag_grid_size * sqrt(2.0);
            double distance;

            nearcelllist.clear();

            for (uint32_t k = 0; k < magneticGrid.size(); k++)
            {
                flag = false;
                x = magneticGrid[k].coordinates.x;
                y = magneticGrid[k].coordinates.y;
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
                    nearcell.number_of_measurements = magneticGrid[k].magData.size();
                    nearcell.X = x;
                    nearcell.Y = y;
                    nearcelllist.push_back(nearcell);
                }
            }
            //if (! flag) // no cells
            if (nearcelllist.empty()) // no cells
            {
                x = ((uint32_t)(pos.X)) + mag_grid_size / 2;
                y = ((uint32_t)(pos.Y)) + mag_grid_size / 2;
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
