/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2013
* \project         IndoorSpirit
* \brief           Coordinate conversion class
* \file            CoordinateConverter.cpp
* \author          D. Churikov
* \date            8.10.2013
*/

#include <math.h>
#include <iostream>
#include "eigen/Dense"
#include "CoordinateConverter.h"


#ifndef M_PI
#define M_PI (3.1415926535897932384626433832795)
#endif

#define CC_MIN_LAT0      (-M_PI/2.)
#define CC_MAX_LAT0      ( M_PI/2.)
#define CC_MIN_LON0      (-M_PI)
#define CC_MAX_LON0      ( M_PI)
#define CC_MAX_ALFA_LAT    1e10
#define CC_MAX_ALFA_LON    1e10
#define CC_MIN_HEADING   (-M_PI)
#define CC_MAX_HEADING   ( M_PI)
#define CC_MIN_LAT_DEGREES  (-89.9)
#define CC_MAX_LAT_DEGREES  ( 89.9)
#define CC_MIN_LON_DEGREES  (-180.)
#define CC_MAX_LON_DEGREES  ( 180.)
#define R_EARTH_A           ( 6378137)
#define R_EARTH_B           ( 6356752)

#ifndef _fsign
#define _fsign(a)  (((a)>=0) ? (1.) : (-1.))
#endif

//-----------------------------------------------------------------------------
// Support functions
//-----------------------------------------------------------------------------
static bool CalcDefScale_LLF2GEOrad( double lat0, double lon0, double &scale_lat, double &scale_lon )
{
    bool res = ( lat0 >= CC_MIN_LAT_DEGREES ) && ( lat0 <= CC_MAX_LAT_DEGREES ) &&
               ( lon0 >= CC_MIN_LON_DEGREES ) && ( lon0 <= CC_MAX_LON_DEGREES );

    if( res )
    {
        scale_lat = 2.*M_PI / ( 2.*M_PI * R_EARTH_B );
        scale_lon = 2.*M_PI / ( 2.*M_PI * R_EARTH_A * cos( lat0 ) );
    }
    else
    {
        scale_lat = 0;
        scale_lon = 0;
    }

    return res;
}

//-----------------------------------------------------------------------------
// Class members
//-----------------------------------------------------------------------------

// set frame interdependences parameters
bool GeoLocConverter2D::SetFrameParams( float64_t   new_lat0_deg,
                                        float64_t   new_lon0_deg,
                                        float64_t   new_alfa_lat,
                                        float64_t   new_alfa_lon,
                                        float64_t   new_heading )
{

    // conversion to degrees
    double new_lat0 = new_lat0_deg * M_PI / 180.;
    double new_lon0 = new_lon0_deg * M_PI / 180.;
    new_heading *= M_PI / 180.;

    // checking of input parameters
    if( new_lat0 < CC_MIN_LAT0 ) return 0;

    if( new_lat0 > CC_MAX_LAT0 ) return 0;

    if( new_lon0 < CC_MIN_LON0 ) return 0;

    if( new_lon0 > CC_MAX_LON0 ) return 0;

    if( new_alfa_lat < -CC_MAX_ALFA_LAT )    return 0;

    if( new_alfa_lat > CC_MAX_ALFA_LAT )     return 0;

    if( new_alfa_lon < -CC_MAX_ALFA_LON )    return 0;

    if( new_alfa_lon > CC_MAX_ALFA_LON )     return 0;

    //if( new_heading <= CC_MIN_HEADING )   return 0;

    //if( new_heading > CC_MAX_HEADING )    return 0;

    lat0 = new_lat0;
    lon0 = new_lon0;
    heading = new_heading;
    f_init = true;

    if(
        ( ( new_alfa_lat > -1e-15 ) && ( new_alfa_lat < 1e-15 ) ) ||
        ( ( new_alfa_lon > -1e-15 ) && ( new_alfa_lon < 1e-15 ) ) 
      )
    {
        CalcDefScale_LLF2GEOrad( new_lat0, new_lon0, alfa_lat, alfa_lon );
        alfa_lon = -alfa_lon; // to NWU local frame
    }
    else
    {
        alfa_lat = new_alfa_lat;
        alfa_lon = new_alfa_lon;
    }

    return f_init;
}

// get current frame transformation parameters
bool GeoLocConverter2D::GetFrameParams( float64_t   *cur_lat0,
                                        float64_t   *cur_lon0,
                                        float64_t   *cur_alfa_lat,
                                        float64_t   *cur_alfa_lon,
                                        float64_t   *cur_heading ) const
{
        *cur_lat0 = lat0 / M_PI * 180.;
        *cur_lon0 = lon0 / M_PI * 180.;
        *cur_alfa_lat = alfa_lat;
        *cur_alfa_lon = alfa_lon;
        *cur_heading = heading / M_PI * 180.;

    return f_init;
}

// get current azimut (local frame x-axis in GEO)
bool GeoLocConverter2D::GetFrameAzimut( float64_t &cur_heading) const
{
    cur_heading = heading / M_PI * 180.;
    return f_init;
}

// Convert Geo coordinates to local coordinates
bool GeoLocConverter2D:: Geo2Local( float64_t lat, float64_t lon, float64_t *x, float64_t *y ) const
{
    double sin_head = sin( heading );
    double cos_head = cos( heading );

    // conversion from degrees
    lat *= M_PI / 180;
    lon *= M_PI / 180;

    if( f_init )
    {
        *x = ( lat - lat0 ) * cos_head / alfa_lat + ( lon - lon0 ) * sin_head / alfa_lon;
        *y = -( lat - lat0 ) * sin_head / alfa_lat + ( lon - lon0 ) * cos_head / alfa_lon;

    }

    return f_init;
}

// Calculates transform matrix from Geo frame to Local frame
static void GetGeo2LocalMatrix2d(Eigen::Matrix2d &C, float64_t scale_lat, float64_t scale_lon, float64_t alfa)
{
	float64_t sin_alfa = sin(alfa);
	float64_t cos_alfa = cos(alfa);
	C << cos_alfa / scale_lat, sin_alfa / scale_lon,
		-sin_alfa / scale_lat, cos_alfa / scale_lon;
}

// Convert local coordinates to Geo coordinates
bool GeoLocConverter2D::Local2Geo( float64_t x, float64_t y, float64_t *lat, float64_t *lon ) const
{
    double sin_head = sin( heading );
    double cos_head = cos( heading );

    if( f_init )
    {
        *lat = lat0 + alfa_lat * ( x * cos_head - y * sin_head );
        *lon = lon0 + alfa_lon * ( x * sin_head + y * cos_head );
    }

    // conversion to degrees
    *lat *= 180 / M_PI;
    *lon *= 180 / M_PI;

    return f_init;
}


static void GetLocal2GeolMatrix2d(Eigen::Matrix2d &C, float64_t scale_lat, float64_t scale_lon, float64_t alfa)
{
	float64_t sin_alfa = sin(alfa);
	float64_t cos_alfa = cos(alfa);
	C << cos_alfa*scale_lat, -sin_alfa*scale_lat, 
		 sin_alfa*scale_lon,  cos_alfa*scale_lon;
}

// Convert Geo vector to local vector
bool GeoLocConverter2D::Geo2Local_vector(const float64_t v_geo[2], float64_t v_local[2]) const
{
    if (f_init)
    {
        Eigen::Matrix2d  C;
        GetGeo2LocalMatrix2d(C, alfa_lat, alfa_lon, heading);
        Eigen::Vector2d vg(v_geo[0], v_geo[1]);
        Eigen::Vector2d vl = C*vg;

        v_local[0] = vl(0);
        v_local[1] = vl(1);
    }

    return f_init;
}

// Convert vector in local coordinates to vector in Geo frame
bool GeoLocConverter2D::Local2Geo_vector(const float64_t v_local[2], float64_t v_geo[2]) const
{

    if (f_init)
    {
        Eigen::Matrix2d  C;
        GetLocal2GeolMatrix2d(C, alfa_lat, alfa_lon, heading);
        Eigen::Vector2d vl (v_local[0], v_local[1]);
        Eigen::Vector2d vg = C*vl;

        v_geo[0] = vg(0);
        v_geo[1] = vg(1);
    }

    return f_init;
}

// Convert covariance matrix in Geo coordinates to covariance matrix in local frame
bool GeoLocConverter2D::Geo2Local_cov_matrix(const float64_t C_geo[2][2], float64_t C_local[2][2]) const
{
    if (f_init)
    {
        // C_local = C*C_geo*C';
        Eigen::Matrix2d  C, C1, C2;
        C1 << C_geo[0][0], C_geo[0][1], C_geo[1][0], C_geo[1][1];
        GetGeo2LocalMatrix2d(C, 1, 1, heading);
        C2 = C*C1;
        C1 = C2*C.transpose();
        C_local[0][0] = C1(0, 0); C_local[0][1] = C1(0, 1);
        C_local[1][0] = C1(1, 0); C_local[1][1] = C1(1, 1);
    }

    return f_init;
}

// Convert covariance matrix in local coordinates to covariance matrix in Geo frame
bool GeoLocConverter2D::Local2Geo_cov_matrix(const float64_t C_local[2][2], float64_t C_geo[2][2]) const
{
    if (f_init)
    {
        // C_geo = C*C_local*C';
        Eigen::Matrix2d  C, C1, C2;
        C1 << C_local[0][0], C_local[0][1], C_local[1][0], C_local[1][1];
        GetLocal2GeolMatrix2d(C, 1, 1, heading);
        C2 = C*C1;
        C1 = C2*C.transpose();
        C_geo[0][0] = C1(0, 0); C_geo[0][1] = C1(0, 1);
        C_geo[1][0] = C1(1, 0); C_geo[1][1] = C1(1, 1);
    }

    return f_init;
}


// Convert Geo heading to local heading; degree version
bool GeoLocConverter2D:: Geo2Local_Heading( float64_t geo_heading, float64_t *local_heading ) const
{
    geo_heading *= M_PI / 180; // conversion from degrees
    Geo2Local_Heading_rad( geo_heading, local_heading );
    *local_heading *= 180. / M_PI;
    return f_init;
}

// Convert local heading to Geo heading; degree version
bool GeoLocConverter2D:: Local2Geo_Heading( float64_t local_heading, float64_t *geo_heading ) const
{
    local_heading *= M_PI / 180; // conversion from degrees
    Local2Geo_Heading_rad( local_heading, geo_heading );
    *geo_heading *= 180. / M_PI; // conversion to degrees
    return f_init;
}

// Convert Geo heading to local heading; radian version
bool GeoLocConverter2D:: Geo2Local_Heading_rad( float64_t geo_heading, float64_t *local_heading ) const
{
    if( f_init )
    {
        if( geo_heading > M_PI )
            geo_heading -= 2 * M_PI;

        if( geo_heading < -M_PI )
            geo_heading += 2 * M_PI;

        *local_heading = _fsign( alfa_lat * alfa_lon ) * geo_heading - heading;

        if( *local_heading > M_PI )
            *local_heading -= 2 * M_PI;

        if( *local_heading < -M_PI )
            *local_heading += 2 * M_PI;
    }

    return f_init;
}

// Convert local heading to Geo heading; radian version
bool GeoLocConverter2D:: Local2Geo_Heading_rad( float64_t local_heading, float64_t *geo_heading ) const
{
    if( f_init )
    {
        *geo_heading = -_fsign( alfa_lat * alfa_lon ) * ( local_heading + heading );

        if( *geo_heading > M_PI )
            *geo_heading -= 2 * M_PI;

        if( *geo_heading < -M_PI )
            *geo_heading += 2 * M_PI;
    }

    return f_init;
}
