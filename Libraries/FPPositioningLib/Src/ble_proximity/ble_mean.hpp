/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           BLE measurement struct
* \defgroup        ble_proximity
* \file            ble_mean.hpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/

#ifndef _BLE_MEAN_H
#define _BLE_MEAN_H

struct BLE_mean
{
    BLE_mean() :bssid(0), time_stamp(0) {};
    BLE_mean(BSSID  _ble_hash, int64_t _time_stamp) :bssid(_ble_hash), time_stamp(_time_stamp) {};
    BSSID bssid;
    int64_t time_stamp;

    bool operator==(const BLE_mean& mean) const
    { return  (this->bssid == mean.bssid); }
};
#endif // _BLE_MEAN_H
