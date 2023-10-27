// FPBuilderLibCommonTest.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
//#include <windows.h>]

#include <string>
#include <fstream>
#include <iomanip>
#include <direct.h>

#include "GridBuilderPrivate.hpp"


                 // Ble
void bleGeneration(Fpbl::GridBuilderPrivate *pStaticBuilder, std::fstream  &log)
{
    log << "Ble====================================";
    log << std::right << std::endl;

    int width = 15;

    Fpbl::Venue venue = pStaticBuilder->getVenue();
    Fpbl::Grid grid = { Fpbl::CellType::CELL_SQUARE, 5.0 };

    Fpbl::BleMeasurement ble;

    std::vector<uint64_t> bleMeasurementTimestamps;
    std::vector<Fpbl::BleMeasurement> bleMeasurements;

    int N = 20 * 2;
    int M = 4;

    uint64_t timestamp = 1500000;
    ble.timestamp = timestamp; /**< unix time [us] */
    ble.mac = 0;       /**< MAC addres in decimal form */
    ble.rssi = 0;        /**< RSSI value [dbm] */
    ble.frequency = 2500; /**< central channel frequency [MHz] */

    ble.major = 1;
    ble.minor = 1;
    ble.txPower = 20;
    ble.hasMAC;
    for (int i = 0; i < 16; i++)
        ble.uuid[i] = i;


    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            log.width(width);
            log << timestamp;
            log.width(width);
            log << ble.timestamp;
            log.width(width);
            log << ble.mac;
            log.width(width);
            log << (uint16_t)ble.rssi;
            log.width(width);
            log << ble.major;
            log.width(width);
            log << ble.minor;
            log.width(width);
            log << (uint16_t)ble.txPower;
            log << std::endl;

            bleMeasurementTimestamps.push_back(timestamp);
            bleMeasurements.push_back(ble);

            Fpbl::BleScanResult bleScan;
            bleScan.timestamp = timestamp;
            bleScan.scanBle.push_back(ble);


            pStaticBuilder->processBleMeasurement(timestamp, bleScan);

            ble.mac += 5;       /**< MAC addres in decimal form */

            ble.major += 1;
            ble.minor += 1;
            ble.txPower += 1;

            timestamp += 2000000; // us
            ble.timestamp = timestamp;

        }
        ble.rssi += 5;        /**< RSSI value [dbm] */


    }


    Fpbl::BleGrid bleGrid1;

    Fpbl::ReturnStatus status = pStaticBuilder->updateGridBLE(venue.id, grid, &bleGrid1);

    int n = venue.sizeX / grid.size;
    int m = venue.sizeY / grid.size;

    //Assert::AreEqual((uint32_t)bleGrid1.size(), (uint32_t)n * m);

    log.width(width);
    log << n;
    log.width(width);
    log << m;
    log.width(width);
    log << (uint16_t)bleGrid1.size();
    log << std::endl;

    log << "bleGrid====================================";
    log << std::endl;


    Fpbl::BleCell bleCell;
    for (int i = 0; i < bleGrid1.size(); i++)
    {
        bleCell = bleGrid1[i];

        log << "Position    ";
        log.width(width);
        log << bleCell.coordinates.x;
        log.width(width);
        log << bleCell.coordinates.y;
        log.width(width);
        log << bleCell.coordinates.floor;
        //log.width(width);
        //log << bleCell.bleData.size();
        log << std::endl;

        log << "bleCell    "
            << bleCell.bleData.size();
        log << std::endl;

        for (auto scan_it = bleCell.bleData.begin(); scan_it != bleCell.bleData.end(); ++scan_it)
        {
            for (auto meas_it = scan_it->scanBle.begin(); meas_it != scan_it->scanBle.end(); ++meas_it)
            {
                log.width(width);
                log << meas_it->timestamp;
                log.width(width);
                log << meas_it->mac;
                log.width(width);
                log << (int16_t)meas_it->rssi;
                log.width(width);
                log << meas_it->frequency;
                log.width(width);
                log << meas_it->major;
                log.width(width);
                log << meas_it->minor;
                log.width(width);
                log << (uint16_t)meas_it->txPower;
                log << std::endl;
            }
        }


        log << "*****************";
        log << std::endl;

    }

 }
 
