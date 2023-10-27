/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Saving/restoring BLA data
* \defgroup        ble_proximity
* \file            ble_proximity_helper.cpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date
*/

#include "ble_proximity_helper.hpp"
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>

#include "ble_proximity_helper.hpp"

#ifdef _MSC_VER
#   pragma warning( push )
#   pragma warning( pop )
#   pragma warning( disable : 4201 )
#   pragma warning( disable : 4505 )
#endif
#   include "dirent.h"

#define FIFO_PUSH_SIZE_MAX 20

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4706 )
#endif

#ifdef _MSC_VER
#pragma warning( pop )
#endif
void BLE_Proximity_Helper::pushMeas(int64_t timestamp, int n, const Fppe::BleScanResult &meas)
{
    fifo.push(meas);

    if (fifo.size() > FIFO_PUSH_SIZE_MAX)
    {
        fifo.pop();
    }
}

bool BLE_Proximity_Helper::readMeas(int64_t &timestamp, int &n, Fppe::BleScanResult &meas)
{
    bool ok = false;
    n = 0;
    if( fifo.size() > 0 )
    {
        meas = fifo.front();
        fifo.pop();
        timestamp = meas.timestamp;
        //leaveStrongBleMeasurement(meas);
        //n = meas.scanBle.size();
        n = fifo.size();
        ok = true;
    }
    return ok;
}

bool BLE_Proximity_Helper::readMeas(int64_t current_timestamp, int64_t &meas_timestamp, Fppe::BleScanResult &meas)
{
    bool ok = false;
    if (fifo.size() > 0)
    {
        meas = fifo.front();
        meas_timestamp = meas.timestamp;

        if (current_timestamp > meas_timestamp)
        {
            fifo.pop();
            ok = true;
        }
    }
    return ok;
}

void BLE_Proximity_Helper::pushBLEProxSolution(int64_t timestamp, const std::vector<WiFi_Location> &loc_list)
{
    ble_prox_pos_list_queue.push(std::make_pair(timestamp, loc_list));
    return (ble_prox_pos_list_queue.size() > FIFO_PUSH_SIZE_MAX) ? ble_prox_pos_list_queue.pop() : void(0);
}

bool BLE_Proximity_Helper::readBLEProxSolution(int64_t &timestamp, std::vector<WiFi_Location> &loc_list)
{
    bool ok = false;
    if (ble_prox_pos_list_queue.size() > 0)
    {
        std::pair <int64_t, const std::vector<WiFi_Location>> loclist_and_time= ble_prox_pos_list_queue.front();
        loc_list = loclist_and_time.second;
        timestamp = loclist_and_time.first;
        ble_prox_pos_list_queue.pop();
        ok = true;
    }
    return ok;
}

bool BLE_Proximity_Helper::max_min_time(int64_t &min_timestamp, int64_t &max_timestamp, Fppe::BleScanResult &meas)
{
    min_timestamp = 0;
    max_timestamp = 0;
    if (meas.scanBle.size() == 0)
        return false;
    min_timestamp = meas.scanBle.begin()->timestamp;
    max_timestamp = meas.scanBle.begin()->timestamp;
    for (auto it = meas.scanBle.begin(); it != meas.scanBle.end(); ++it)
    {
        if (min_timestamp > it->timestamp)
            min_timestamp = it->timestamp;
        if (max_timestamp < it->timestamp)
            max_timestamp = it->timestamp;
    }
    return true;
}

/**
* The method get beacon distance
* \return beacon distance
*/
double BLE_Proximity_Helper::getDistance(double measuredPower, double rssi)
{
    if (rssi == 0.0)
    {
        return -1.0;
    }
    double ratio = rssi * 1.0 / measuredPower;
    if (ratio < 1.0) 
    {
        return pow(ratio, 10.0);
    }
    double distance = 0.89976 * pow(ratio, 7.7095) + 0.111;
    return distance;
}

bool BLE_Proximity_Helper::getNextTimestamp( int64_t &timestamp )
{
    Fppe::BleScanResult last_result;
    bool ok = false;

    if( fifo.size() > 0 )
    {
        last_result = fifo.front();
        timestamp = last_result.timestamp;
        ok = true;
    }

    return ok;
}


std::vector<std::string> BLE_Proximity_Helper::parse( const std::string &s, const char &delim ) const
{
    std::stringstream buf( s );
    std::string token;
    std::vector<std::string> result;


    while( std::getline( buf, token, delim ) )
    {
        result.push_back( token );
    }

    return result;
}
