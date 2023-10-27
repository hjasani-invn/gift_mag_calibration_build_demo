// FPBuilderLibCommonTest.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
//#include <windows.h>]

#include <string>
#include <fstream>
#include <iomanip>
#include <direct.h>

#include "GridBuilderPrivate.hpp"


                // Wifi
void wifiGeneration(Fpbl::GridBuilderPrivate *pStaticBuilder, std::fstream  &log)
{
    log << "Wifi====================================";
    log << std::right << std::endl;

    int width = 15;

    Fpbl::Venue venue = pStaticBuilder->getVenue();
    Fpbl::Grid grid = { Fpbl::CellType::CELL_SQUARE, 5.0 };

    Fpbl::WiFiMeasurement wifi;

    std::vector<uint64_t> wiFiMeasurementTimestamps;
    std::vector<Fpbl::WiFiMeasurement> wiFiMeasurements;

    int N = 20 * 2;
    int M = 4;

    uint64_t timestamp = 1500000;
    wifi.timestamp = timestamp; /**< unix time [us] */
    wifi.mac = 0;       /**< MAC addres in decimal form */
    wifi.rssi = 0;        /**< RSSI value [dbm] */
    wifi.frequency = 2500; /**< central channel frequency [MHz] */


    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            log.width(width);
            log << timestamp;
            log.width(width);
            log << wifi.timestamp;
            log.width(width);
            log << wifi.mac;
            log.width(width);
            log << (uint16_t)wifi.rssi;
            log << std::endl;

            wiFiMeasurementTimestamps.push_back(timestamp);
            Fpbl::WiFiScanResult wifiScan;
            wifiScan.timestamp = timestamp;
            wifiScan.scanWiFi.push_back(wifi);

            pStaticBuilder->processWiFiMeasurement(timestamp, wifiScan);

            wifi.mac += 5;       /**< MAC addres in decimal form */

            timestamp += 2000000; // us
            wifi.timestamp = timestamp;

        }
        wifi.rssi += 5;        /**< RSSI value [dbm] */


    }


    Fpbl::WiFiGrid wifiGrid1;

    Fpbl::ReturnStatus status = pStaticBuilder->updateGridWiFi(venue.id, grid, &wifiGrid1);

    int n = venue.sizeX / grid.size;
    int m = venue.sizeY / grid.size;

    //Assert::AreEqual((uint32_t)wifiGrid1.size(), (uint32_t)n * m);

    log.width(width);
    log << n;
    log.width(width);
    log << m;
    log.width(width);
    log << (uint16_t)wifiGrid1.size();
    log << std::endl;

    log << "WifiGrid====================================";
    log << std::endl;


    Fpbl::WifiCell wifiCell;
    for (int i = 0; i < wifiGrid1.size(); i++)
    {
        wifiCell = wifiGrid1[i];

        log << "Position    ";
        log.width(width);
        log << wifiCell.coordinates.x;
        log.width(width);
        log << wifiCell.coordinates.y;
        log.width(width);
        log << wifiCell.coordinates.floor;
        //log.width(width);
        //log << wifiCell.wifiData.size();
        log << std::endl;

        log << "WifiCell    "
            << wifiCell.wifiData.size();
        log << std::endl;

        for (auto scan_it = wifiCell.wifiData.begin(); scan_it != wifiCell.wifiData.end(); ++scan_it)
        {
            for (auto meas_it = scan_it->scanWiFi.begin(); meas_it != scan_it->scanWiFi.end(); ++meas_it)
            {
                log.width(width);
                log << meas_it->timestamp;
                log.width(width);
                log << meas_it->mac;
                log.width(width);
                log << (int16_t)meas_it->rssi;
                log.width(width);
                log << meas_it->frequency;
                log << std::endl;
            }
        }
        log << "*****************";
        log << std::endl;

    }

}


