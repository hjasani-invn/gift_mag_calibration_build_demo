
// C includes
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

// Cpp includes
#include <iostream>
//#include <iomanip>
#include <fstream>
#include <sstream>

#include <string>
#include <stdint.h>

#include "Fpbl.hpp"

bool load_mag_grid_file( const std::string &fname, Fpbl::MagneticGrid     &magnetic_grid)
{
    std::ifstream infile(fname);
    std::string line;

    MagneticData  mag_data;
    Fpbl::MagneticCell  mag_cell;
    double  x_prev, y_prev;
    int16_t floor_prev;
    double  x, y; 
    int16_t floor;
    std::string  str;
    std::stringstream ss;
    long counter = 0;

    x_prev = -1.0;
    y_prev = -1.0;
    floor_prev = -1;

    while (std::getline(infile, line))
    {
        if (line == "")
            continue;

        counter++;

        ss << line;

        ss >> x;
        ss >> y;
        ss >> floor;

        if ((x != x_prev) || (y != y_prev) || (floor != floor_prev))
        {
            if (mag_cell.magData.size() > 0)
                magnetic_grid.push_back(mag_cell);

            std::cout << counter << "\t" << mag_cell.magData.size() << std::endl;
            std::cout << mag_cell.coordinates.x << "\t" << mag_cell.coordinates.y << "\t" << mag_cell.magData.size() << std::endl;

            mag_cell.coordinates.x = x;
            mag_cell.coordinates.y = y;
            mag_cell.coordinates.floor = floor;

            mag_cell.magData.clear();
        }

        mag_data.timestamp = 0;

        ss >> mag_data.mX;
        ss >> mag_data.mY;
        ss >> mag_data.mZ;
        ss >> mag_data.covarianceMatrix[0][0];
        ss >> mag_data.covarianceMatrix[0][1];
        ss >> mag_data.covarianceMatrix[0][2];
        ss >> mag_data.covarianceMatrix[1][0];
        ss >> mag_data.covarianceMatrix[1][1];
        ss >> mag_data.covarianceMatrix[1][2];
        ss >> mag_data.covarianceMatrix[2][0];
        ss >> mag_data.covarianceMatrix[2][1];
        ss >> mag_data.covarianceMatrix[2][2];

        ss.str("");
        ss.clear();

        mag_cell.magData.push_back(mag_data);

        x_prev = x;
        y_prev = y;
        floor_prev = floor;

    }
    if (mag_cell.magData.size() > 0)
        magnetic_grid.push_back(mag_cell);

 
    return true;
}

