
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
#include "OccupancyOfPosition.hpp"

using namespace FPVerificator;

bool save_position_occupancy( const std::string &fname, std::vector<OccupancyOfPosition> poslist)
{
        std::fstream f_occ;
        f_occ.open( fname.c_str(), std::ios::out );
        f_occ.precision( 6 );
        int width = 15;
        f_occ << std::right;

    for (auto it = poslist.begin(); it != poslist.end(); it++)
    {
        OccupancyOfPosition  pos = *it;
        f_occ << pos.X << "         " << pos.Y << "         " << pos.number_of_measurements << std::endl;
    }
    return true;
}

