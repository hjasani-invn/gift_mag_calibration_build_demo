/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           BLE location struct
* \defgroup        ble_proximity
* \file            ble_position.hpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/
#include "fpeDataTypes.h"

#ifndef _BLE_POSITION_H
#define _BLE_POSITION_H

struct  BLE_position
{
    WiFi_Location loc;
    //double power;
    double blp_height;
    int8_t txPower;
    int8_t rxPower;
	int8_t txPower_correction;
    double distance;
    BleBeaconType beaconType;
    uint16_t major;
    uint16_t minor;
};
#endif // _BLE_POSITION_H
