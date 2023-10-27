
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
#include "stringMac_to_intMac.h"

#if 0
bool load_wifi_grid_file( const std::string &fname, Fpbl::WiFiGrid     &wifi_grid)
{
    std::ifstream infile(fname);
    std::string line;

    WiFiScanResult  wifi_data;
    WiFiMeasurement scanWiFi;
    Fpbl::WifiCell  wifi_cell;
    double  x_prev, y_prev;
    int16_t floor_prev;
    double  x, y; 
    int16_t floor;
    std::string  str;
    std::stringstream ss;
    int16_t rssi;

    x_prev = -1.0;
    y_prev = -1.0;
    floor_prev = -1;
    long counter = 0;

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
            if (wifi_cell.wifiData.size() > 0)
                wifi_grid.push_back(wifi_cell);

            std::cout << x << "\t" << y << "\t" << wifi_cell.wifiData.size() << std::endl;

            wifi_cell.coordinates.x = x;
            wifi_cell.coordinates.y = y;
            wifi_cell.coordinates.floor = floor;

            wifi_cell.wifiData.clear();
        }

        wifi_data.timestamp = 0;
        scanWiFi.timestamp = 0;  
        
        ss >> str;
        scanWiFi.mac = stringMac_to_intMac(str);

        ss >> rssi;
        scanWiFi.rssi = (int8_t)rssi;
        ss >> scanWiFi.frequency;
        
        ss.str("");
        ss.clear();

        wifi_data.scanWiFi.push_back(scanWiFi);
        wifi_cell.wifiData.clear();
        wifi_cell.wifiData.push_back(wifi_data);

        //if (counter == 13038)
        //    counter++;
        //if (counter %  100000 == 0)
        //    std::cout << counter << std::endl;
        x_prev = x;
        y_prev = y;
        floor_prev = floor;

    }
    if (wifi_cell.wifiData.size() > 0)
        wifi_grid.push_back(wifi_cell);

 
    return true;
}
#else
bool load_wifi_grid_file(const std::string &fname, Fpbl::WiFiGrid     &wifi_grid)
{
    std::ifstream infile(fname);
    std::string line;

    WiFiScanResult  wifi_data;
    WiFiMeasurement wifiMeasurement;
    Fpbl::WifiCell  wifi_cell;
    double  x_prev, y_prev;
    int16_t floor_prev;
    double  x, y;
    int16_t floor;
    std::string  str;
    std::stringstream ss;
    int16_t rssi;

    x_prev = -1.0;
    y_prev = -1.0;
    floor_prev = -1;
    long counter = 0;

        while ( !infile.eof() )
        {
            counter++;

            infile >> x;
            infile >> y;
            infile >> floor;

            if ((x_prev != -1) && ((x != x_prev) || (y != y_prev) || (floor != floor_prev)))
            {
                wifi_cell.wifiData.push_back(wifi_data);
                if (wifi_cell.wifiData.size() > 0)
                     wifi_grid.push_back(wifi_cell);
                std::cout << counter << "\t" << wifi_cell.wifiData.size() << std::endl;
                std::cout << wifi_cell.coordinates.x << "\t" << wifi_cell.coordinates.y << "\t" << wifi_data.scanWiFi.size() << std::endl;

                wifi_data.scanWiFi.clear();
                wifi_cell.wifiData.clear();
            }

            wifiMeasurement.timestamp = 0;

            infile >> str;
            wifiMeasurement.mac = stringMac_to_intMac(str);

            infile >> rssi;
            wifiMeasurement.rssi = (int8_t)rssi;
            infile >> wifiMeasurement.frequency;

            wifi_data.timestamp = 0;
            wifi_data.scanWiFi.push_back(wifiMeasurement);
            //wifi_cell.wifiData.clear();
            //wifi_cell.wifiData.push_back(wifi_data);
            wifi_cell.coordinates.x = x;
            wifi_cell.coordinates.y = y;
            wifi_cell.coordinates.floor = floor;

            x_prev = x;
            y_prev = y;
            floor_prev = floor;

    }
        wifi_cell.wifiData.push_back(wifi_data);
        if (wifi_cell.wifiData.size() > 0)
            wifi_grid.push_back(wifi_cell);
        std::cout << counter << "\t" << wifi_cell.wifiData.size() << std::endl;
        std::cout << wifi_cell.coordinates.x << "\t" << wifi_cell.coordinates.y << "\t" << wifi_data.scanWiFi.size() << std::endl;

        wifi_data.scanWiFi.clear();
        wifi_cell.wifiData.clear();


    return true;
}
#endif
