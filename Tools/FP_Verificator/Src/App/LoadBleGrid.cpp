
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
bool load_ble_grid_file( const std::string &fname, Fpbl::BleGrid     &ble_grid)
{
    std::ifstream infile(fname);
    std::string line;

    BleScanResult  ble_data;
    Fpbl::BleCell  ble_cell;
    double  x_prev, y_prev;
    int16_t floor_prev;
    double  x, y; 
    int16_t floor;
    std::string  str;
    std::stringstream ss;


    x_prev = -1.0;
    y_prev = -1.0;
    floor_prev = -1;

    while (std::getline(infile, line))
    {
        if (line == "")
            continue;
        ss << line;

        ss >> x;
        ss >> y;
        ss >> floor;

        if ((x != x_prev) || (y != y_prev) || (floor != floor_prev))
        {
            if (ble_cell.bleData.size() > 0)
                ble_grid.push_back(ble_cell);

            ble_cell.coordinates.x = x;
            ble_cell.coordinates.y = y;
            ble_cell.coordinates.floor = floor;

            ble_cell.bleData.clear();
        }

        ble_data.timestamp = 0;

        ss >> ble_cell.coordinates.x;
        ss >> ble_cell.coordinates.y;
        ss >> ble_cell.coordinates.floor;

        ss.str("");
        ss.clear();

        ble_cell.bleData.push_back(ble_data);

        x_prev = x;
        y_prev = y;
        floor_prev = floor;

    }
    if (ble_cell.bleData.size() > 0)
        ble_grid.push_back(ble_cell);

 
    return true;
}
#else
bool load_ble_grid_file(const std::string &fname, Fpbl::BleGrid     &ble_grid)
{
    std::ifstream infile(fname);
    std::string line;

    BleScanResult  ble_data;
    BleMeasurement bleMeasurement;
    Fpbl::BleCell  ble_cell;
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

    while (!infile.eof())
    {
        counter++;

        infile >> x;
        infile >> y;
        infile >> floor;

        if ((x_prev != -1) && ((x != x_prev) || (y != y_prev) || (floor != floor_prev)))
        {
            ble_cell.bleData.push_back(ble_data);
            if (ble_cell.bleData.size() > 0)
                ble_grid.push_back(ble_cell);
            std::cout << counter << "\t" << ble_cell.bleData.size() << std::endl;
            std::cout << ble_cell.coordinates.x << "\t" << ble_cell.coordinates.y << "\t" << ble_data.scanBle.size() << std::endl;

            ble_data.scanBle.clear();
            ble_cell.bleData.clear();
        }

        bleMeasurement.timestamp = 0;

        infile >> str;
        bleMeasurement.mac = stringMac_to_intMac(str);

        infile >> rssi;
        bleMeasurement.rssi = (int8_t)rssi;
        infile >> bleMeasurement.frequency;

        ble_data.timestamp = 0;
        ble_data.scanBle.push_back(bleMeasurement);
        //wifi_cell.wifiData.clear();
        //wifi_cell.wifiData.push_back(wifi_data);
        ble_cell.coordinates.x = x;
        ble_cell.coordinates.y = y;
        ble_cell.coordinates.floor = floor;

        x_prev = x;
        y_prev = y;
        floor_prev = floor;

    }
    ble_cell.bleData.push_back(ble_data);
    if (ble_cell.bleData.size() > 0)
        ble_grid.push_back(ble_cell);
    std::cout << counter << "\t" << ble_cell.bleData.size() << std::endl;
    std::cout << ble_cell.coordinates.x << "\t" << ble_cell.coordinates.y << "\t" << ble_data.scanBle.size() << std::endl;

    ble_data.scanBle.clear();
    ble_cell.bleData.clear();


    return true;
}
#endif
