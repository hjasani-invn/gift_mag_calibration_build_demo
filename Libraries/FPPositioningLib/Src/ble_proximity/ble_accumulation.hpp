/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Accumulation of several in sequence following measurements
* \defgroup        ble_proximity        
* \file            ble_accumulation.hpp          
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018            
*/

#ifndef BLE_ACCUMULATIONT_H
#define BLE_ACCUMULATIONT_H

#include <string>
#include <map>
#include <list>
#include <iostream>
#include "Fppe.hpp"
#include "wifi_data.hpp"
#include "CoordinateConverter.h"
#include "ble_position.hpp"
#include "ble_mean.hpp"


class BLE_Accumulation
{
public:
    BLE_Accumulation(int N, int M )
    {
        set_BLE_Accumulation( N, M);
    }

    ~BLE_Accumulation()
    {
    }

    void set_BLE_Accumulation( int N, int M)
    {
        N_accumulation = N;
        M_accumulation = M;
    }

    bool full() const
    {
        return (data.size() == N_accumulation);
    }

    size_t size() const
    {
        return data.size();
    }

    void reset()
    {
        data.clear();
    }

    bool putCurrentBLEHash(BSSID  ble_hash, int64_t time_stamp)
    {
        data.push_back(BLE_mean(ble_hash, time_stamp));
 
        while (data.size() > N_accumulation)
            data.pop_front();

       // std::cout << "data.size() " << data.size() << std::endl;

        if (data.size() == N_accumulation)
            return true;
        else
            return false;
    }

    WiFi_Location GetLocation(std::map<BSSID, BLE_position> ble_position_map, uint64_t time_stamp)
    {
        //WiFi_Location location;
        BLE_position location;

        // make decision
        int max = 0;
        int number;
        //BSSID ble_hash;
        BLE_mean mean;

        uint64_t first_time_stamp = data.front().time_stamp;
        uint64_t last_time_stamp = data.back().time_stamp;
        
        if ((last_time_stamp - first_time_stamp) > time_stamp_interval)
        {
            data.pop_front();
            location.loc.valid = false;
            return location.loc;
        }
        if ((time_stamp - last_time_stamp) > time_stamp_interval)
        {
            data.clear();
            location.loc.valid = false;
            return location.loc;
        }

        for (auto it = data.cbegin(); it != data.cend(); ++it)
        {
            number = std::count(data.cbegin(), data.cend(), *it);
            if (number > max)
            {
                max = number;
                //ble_hash = *it;
                mean = *it;
            }
        }
        
        if (max >= M_accumulation)
        {
            location = ble_position_map[mean.bssid];
            location.loc.valid = true;
        }
        else
            location.loc.valid = false;

        return location.loc;
    }
private:
    uint16_t N_accumulation = 1;
    uint16_t M_accumulation = 1;
    std::list<BLE_mean> data;
    const uint64_t time_stamp_interval = 5000;

};

#endif // BLE_ACCUMULATIONT_H
