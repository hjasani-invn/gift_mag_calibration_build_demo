/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Fingerprint base types and operations
* \file            wifi_data.cpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#define _CRT_SECURE_NO_WARNINGS
#include "wifi_data.hpp"
#include <cmath>
#include <algorithm>
#include <stdio.h> // ! gcc won't compile without it !

BSSID WiFi_ApInfo::string_to_mac( std::string const &s )
{
    unsigned int a[7];
    int last = -1;
    unsigned int band = 0;

    int cnt = std::count(s.begin(), s.end(), ':');

    if (cnt > 5)
    {
        int rc = sscanf( s.c_str(), "%x:%x:%x:%x:%x:%x:%x%n",
                     &band, a + 0, a + 1, a + 2, a + 3, a + 4, a + 5,
                     &last );
        if( rc != 7 || s.size() != (size_t)last )
        {
            //throw std::runtime_error( "invalid mac address format " + s );
            return 0;
        }
    }
    else
    {
        int rc = sscanf( s.c_str(), "%x:%x:%x:%x:%x:%x%n",
                     a + 0, a + 1, a + 2, a + 3, a + 4, a + 5,
                     &last );
        if( rc != 6 || s.size() != (size_t)last )
        {
            //throw std::runtime_error( "invalid mac address format " + s );
            return 0;
        }
    }

    return
        BSSID( band ) << 48 |
        BSSID( a[0] ) << 40 |
        BSSID( a[1] ) << 32 |
        BSSID( a[2] ) << 24 |
        BSSID( a[3] ) << 16 |
        BSSID( a[4] ) << 8 |
        BSSID( a[5] );
}

//BSSID WiFi_ApInfo::string_to_mac2( const std::string &s )
//{
//    BSSID mac = 0;
//    std::stringstream ss;
//    unsigned char delim;
//    unsigned short int val;
//
//    ss << s;
//    for( int i = 0; i < 6; ++i )
//    {
//        if( i != 5 )
//        {
//            ss >> std::hex >> val >> delim;
//        }
//        else
//        {
//            ss >> std::hex >> val;
//        }
//        if( ss.fail() == true || ss.bad() == true )
//        {
//            throw std::runtime_error( "invalid mac address format " + s );
//        }
//        mac |= ( static_cast<BSSID>( val ) << 8 * ( 5 - i ) );
//    }
//    return mac;
//}


WiFi_Location operator* ( const WiFi_Location &L, const tProb &w )
{
    WiFi_Location loc = L;
    loc.x *= w;
    loc.y *= w;
    loc.z *= w;
    loc.p *= w;
    return loc;
}

WiFi_Location operator/( const WiFi_Location &L, const tProb &w )
{
    WiFi_Location loc = L;
    loc.x /= w;
    loc.y /= w;
    loc.z /= w;
    loc.p /= w;
    return loc;
}

WiFi_Location &operator/=( WiFi_Location &L, const tProb &w )
{
    L.x /= w;
    L.y /= w;
    L.z /= w;
    L.p /= w;
    return L;
}

WiFi_Location operator+( const WiFi_Location &L, const WiFi_Location &R )
{
    WiFi_Location loc = L;
    loc.x += R.x;
    loc.y += R.y;
    loc.z += R.z;
    loc.p += R.p;
    return loc;
}

WiFi_Location &operator+=( WiFi_Location &L, const WiFi_Location &R )
{
    L.x += R.x;
    L.y += R.y;
    L.z += R.z;
    L.p += R.p;
    return L;
}

void WiFi_Location::set( const Coordinates _x, const Coordinates _y, const Coordinates _z,  const tProb _p , const bool _valid )
{
    x = _x;
    y = _y;
    z = _z;
    p = _p;
    valid = _valid;
}


Coordinates WiFi_Location::norm(const WiFi_Location &x0) const
{
    return sqrt( (x - x0.x)*(x - x0.x) + (y - x0.y)*(y - x0.y) + (z - x0.z)*(z - x0.z));
}
