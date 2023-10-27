/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Data base for BLE proximity
* \defgroup        ble_proximity
* \file            ble_proximity_db.hpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/

#ifndef BLE_PROXIMITY_DB_H
#define BLE_PROXIMITY_DB_H

#include <string>
#include <map>
#include <list>
#include <iostream>
#include "Fppe.hpp"
#include "wifi_data.hpp"
#include "CoordinateConverter.h"
#include "ble_accumulation.hpp"


class BLE_Proximity_DB
{
    public:
        BLE_Proximity_DB(int N, int M, int rejThresh);
        virtual ~BLE_Proximity_DB();// {}

        void setBLEProximityLogic(int N, int M);

        bool readFormatBle5(const char* const pBleMap, const size_t bleFileSizeInBytes, const GeoLocConverter2D &converter);
        void setSingleCutoffThreshold(const int reject_threshold);
        void setMultipleCutoffThreshold(const int reject_threshold);
        //BLE_position GetLocation(const Fppe::BleScanResult  &measurement, uint64_t time_stamp) const;
        BLE_position GetLocation(const std::vector<Fppe::BleMeasCalibrated>  &measurement, uint64_t time_stamp) const;
        
        void GetLocation(const std::vector<Fppe::BleMeasCalibrated>  &measurement, uint64_t time_stamp, std::vector<WiFi_Location> &LocationList) const;
        //void GetLocation(const Fppe::BleScanResult  &measurement, uint64_t time_stamp, std::vector<WiFi_Location> &LocationList) const;
        std::map < BSSID, BLE_position> GetBLEProximityMap() const;
        size_t size() const;
        bool GetDbItem(BSSID  ble_hash, BLE_position &db_item) const;

    private:
        void string_to_ble(std::string const &s, uint8_t *ble);
        std::map < BSSID, BLE_position> ble_position_map;
        int SingleCutoffThreshold;
        int MultipleCutoffThreshold;
        BLE_Accumulation *ble_data;
};
#endif // BLE_PROXIMITY_DB_H
