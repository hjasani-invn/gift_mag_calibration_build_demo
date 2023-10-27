/**
* \copyright       Copyright (C) TDK, LLC., 2013
* \project         IndoorSpirit
* \brief           Floor conversion class
* \file            FloorConverter.cpp
* \author          V. Pentyukhov
* \date            8.10.2018
*/

#include <math.h>
#include <iostream>
#include "eigen/Dense"
#include "FloorConverter.h"

//-----------------------------------------------------------------------------
// Class members
//-----------------------------------------------------------------------------

// Constructor
FloorConverter::FloorConverter()
{
    this->floors_count = 1;      /**< total floor number in venue */
    this->floor_height = 5;        /**< floor height [m] */
    this->floor_shift = 1;        /**< floor shift*/
    this->floor_zero_enable = false;
    return;
}

// Destructor
FloorConverter::~FloorConverter()
{
    return;
}

bool FloorConverter::SetFloorsParams(
    uint16_t floors_count,        /**< total floor number in venue */
    double   floor_height,        /**< floor height [m] */
    int16_t  floors_shift,        /**< floor shift*/
    bool     floor_zero_enable
    )
{
    this->floors_count = floors_count;
    this->floor_height = floor_height;
    this->floor_shift = floors_shift;
    this->floor_zero_enable = floor_zero_enable;

    return true;
}

bool FloorConverter::GetFloorsParams(
    uint16_t *floors_count,      /**< total floor number in venue */
    double   *floor_height,        /**< floor height [m] */
    int16_t  *floors_shift,        /**< floor shift*/
    bool     *floor_zero_enable
    ) const
{
    *floors_count = this->floors_count;
    *floor_height = this->floor_height;
    *floors_shift = this->floor_shift;
    *floor_zero_enable = this->floor_zero_enable;

    return true;
}

bool FloorConverter::SetCurrentRealFloor(int16_t real_floor)
{
    current_real_floor = real_floor;
    current_logical_floor = current_real_floor - floor_shift;
    if ((!floor_zero_enable) && (floor_shift < 0))
    {
        current_logical_floor -= (current_real_floor > 0) ? 1 : 0;
    }

    return true;
}

bool FloorConverter::SetCurrentLogicalFloor(int16_t logical_floor)
{
    current_logical_floor = logical_floor;
    current_real_floor = current_logical_floor + floor_shift;

    if ((!floor_zero_enable) && (floor_shift < 0))
    {
        current_real_floor += (current_real_floor >= 0) ? 1 : 0;
    }
    return true;
}

bool FloorConverter::GetCurrentRealFloor(int16_t *real_floor) const
{
    *real_floor = current_real_floor;
    return true;
}

int16_t FloorConverter::GetCurrentRealFloor(int16_t logical_floor) const
{
    int16_t real_floor = logical_floor + this->floor_shift;
    if ((!this->floor_zero_enable) && (this->floor_shift < 0))
    {
        real_floor += (real_floor >= 0) ? 1 : 0;
    }
    return real_floor;
}

bool FloorConverter::GetCurrentLogicalFloor(int16_t *logical_floor) const
{
    *logical_floor = current_logical_floor;
    return true;
}


