/**
* \copyright       Copyright (C) TDK, LLC., 2013
* \project         IndoorSpirit
* \brief           Floor conversion class
* \file            FloorConverter.h
* \author          V. Pentyukhov
* \date            18.08.2018
*/

/** \addtogroup COORDINATE_CONVERTER The coordinate converter class
*  Class GeoLocConverter2D 
*/

#ifndef _FLOOR_CONVERTER_H_
#define _FLOOR_CONVERTER_H_

// include standard integer types
#include <stdint.h>

// universal float types
#define float32_t   float
#define float64_t   double

/**
* FloorConverter class
* \ingroup FLOOR_CONVERTER
*/
class FloorConverter
{
    public:
        /**
        * FloorConverter constructor
        */
        FloorConverter();

        /**
        * FloorConverter destructor
        */
        ~FloorConverter();

        bool SetFloorsParams(
                            uint16_t floors_count,        /**< total floor number in venue */
                            double   floor_height,        /**< floor height [m] */
                            int16_t  floors_shift,        /**< floor shift*/
                            bool     floor_zero_enable
                           );

        bool GetFloorsParams( 
                            uint16_t *floors_count,      /**< total floor number in venue */
                            double   *floor_height,        /**< floor height [m] */
                            int16_t  *floors_shift,        /**< floor shift*/
                            bool     *zero_floor_enable
                           ) const;

        bool SetCurrentLogicalFloor(int16_t logical_floor);

        bool SetCurrentRealFloor(int16_t real_floor);

        bool GetCurrentRealFloor(int16_t *real_floor) const;
        int16_t GetCurrentRealFloor(int16_t logical_floor) const;

        bool GetCurrentLogicalFloor(int16_t *logical_floor) const;

    private:

    FloorConverter( const FloorConverter & ); //disable copy constructor
    const FloorConverter &operator=( const FloorConverter & ); //disable copy
    
    uint16_t floors_count;        /**< total floor number in venue */
    double   floor_height;        /**< floor height [m] */
    int16_t  floor_shift;        /**< floor shift*/
    bool     floor_zero_enable;
    int16_t  current_logical_floor;
    int16_t  current_real_floor;
};

#endif  // _FLOOR_CONVERTER_H_
