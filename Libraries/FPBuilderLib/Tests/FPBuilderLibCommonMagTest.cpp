// FPBuilderLibCommonTest.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
//#include <windows.h>]

#include <string>
#include <fstream>
#include <iomanip>
#include <direct.h>

#include "GridBuilderPrivate.hpp"

                     // Magnetic
void magGeneration(Fpbl::GridBuilderPrivate *pStaticBuilder, std::fstream  &log)
{
    log << "Magnetic====================================";
    log << std::endl;
    log << std::right << std::endl;

    int width = 15;

    Fpbl::Venue venue = pStaticBuilder->getVenue();
    Fpbl::Grid grid = { Fpbl::CellType::CELL_SQUARE, 5.0 };


    Fpbl::MagneticVector mag;

    std::vector<uint64_t> magMeasurementTimestamps;
    std::vector<Fpbl::MagneticVector> magVectors;

    int N = 20 * 2;
    int M = 4;

    uint64_t timestamp = 1500000;
    mag.mX = 10;
    mag.mY = 11;
    mag.mZ = 12;

    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            log.width(width);
            log << timestamp;
            log.width(width);
            log << mag.mX;
            log.width(width);
            log << mag.mY;
            log.width(width);
            log << mag.mZ;
            log << std::endl;

            magMeasurementTimestamps.push_back(timestamp);
            magVectors.push_back(mag);

            pStaticBuilder->processMFPMeasurement(timestamp, mag);

            mag.mX += 1;       /**< MAC addres in decimal form */

            mag.mY += 1;
            mag.mZ += 1;

            timestamp += 2000000; // us
        }

    }


    Fpbl::MagneticGrid magGrid1;

    Fpbl::ReturnStatus status = pStaticBuilder->updateGridMFP(venue.id, grid, &magGrid1);

    int n = venue.sizeX / grid.size;
    int m = venue.sizeY / grid.size;

    //Assert::AreEqual((uint32_t)magGrid1.size(), (uint32_t)n * m);

    log.width(width);
    log << n;
    log.width(width);
    log << m;
    log.width(width);
    log << (uint16_t)magGrid1.size();
    log << std::endl;

    log << "magGrid====================================";
    log << std::endl;


    Fpbl::MagneticCell magCell;
    for (int i = 0; i < magGrid1.size(); i++)
    {
        magCell = magGrid1[i];

        log << "Position    ";
        log.width(width);
        log << magCell.coordinates.x;
        log.width(width);
        log << magCell.coordinates.y;
        log.width(width);
        log << magCell.coordinates.floor;
        log << std::endl;

        log << "magCell    "
            << magCell.magData.size();
        log << std::endl;

        for (auto scan_it = magCell.magData.begin(); scan_it != magCell.magData.end(); ++scan_it)
        {
            // for (auto meas_it = scan_it->scanBle.begin(); meas_it != scan_it->scanBle.end(); ++meas_it)
            {
                log.width(width);
                log << scan_it->mX;
                log.width(width);
                log << scan_it->mY;
                log.width(width);
                log << scan_it->mZ;
                log << std::endl;
            }
        }


        log << "*****************";
        log << std::endl;

    }

}

