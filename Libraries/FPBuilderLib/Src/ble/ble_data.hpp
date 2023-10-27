/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Fingerprint base types and operations
* \file            wifi_data.hpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#ifndef BLE_DATA_H
#define BLE_DATA_H

#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <stdint.h>

#define RSSI_MAX    0  /**< rssi high limit */
#define RSSI_MIN -100  /**< rssi low limit */

#define IGNORE_FREQ_BAND_PREFIX 0 /** ignores frequency  band */



typedef uint64_t BSSID; /**< unique id */
typedef int      RSSI;  /**< signal strength */
typedef double   tProb; /**< probability */
typedef double   Coordinates; /**< coordinates type */


/** single AP data */
struct Ble_ApInfo
{
    BSSID    bssid;
    RSSI     rssi;

    //    static  BSSID string_to_mac2( const std::string &s );

    /**
    * parse bssid string and coverts it to internal type
    * \param[in] s bssid string in hex format, with ":" delimeter, supports 6 or 7 bytes format
    * \return BSSID uniqe id
    */
    static  BSSID string_to_mac( const std::string &s );
};

/** mesurement vector */
typedef std::vector< Ble_ApInfo>  Ble_Measurement;

/** location type 
* \ingroup         WiFi
*/
struct     Ble_Location
{
    /** default constructor */
     Ble_Location() : x( 0 ), y( 0 ), z( 0 ), valid( false ), p( 0 ) {};
    //WiFi_Location( const WiFi_Location &L )
    //{
    //    x = L.x;
    //    y = L.y;
    //    z = L.z;
    //    valid = L.valid;
    //    p = L.p;
    //}

    /**
    * initialize location
    * \param[in] _x,_y,_z Coordinates of the position in 3D space [m]
    * \param[in] _p Estimated probability for the solution
    * \param[in] _valid validity flag
    */
    void set( const Coordinates _x, const Coordinates _y, const Coordinates _z, const tProb _p, const bool _valid );

    /**
    * weighting operation
    * \return location weighted by multiplier
    */
    friend  Ble_Location operator* ( const  Ble_Location &L, const tProb &w );
    /**
    * weighting operation
    * \return location weighted by divider
    */
    friend  Ble_Location operator/( const  Ble_Location &L, const tProb &w );
    /**
    * weighting operation
    * \return location weighted by divider
    */
    friend  Ble_Location &operator/=(  Ble_Location &L, const tProb &w );
    /**
    * operator
    * \return sum of two location
    */
    friend  Ble_Location operator+( const  Ble_Location &L, const  Ble_Location &R );
    /**
    * operator
    * \return sum of two location
    */
    friend  Ble_Location &operator+=(  Ble_Location &L, const  Ble_Location &R );

    /**
    * The distance between the current location and transferred
    * \param[in] x0 location
    * \return Euclidian distance
    */
    Coordinates norm( const  Ble_Location &x0 ) const;

    /** Coordinate in 3D space [m] */
    ///@{
    Coordinates    x, y, z;
    ///@}
    bool     valid;         /**< validity flag */
    tProb    p;             /**< solution metric */

};

#endif
