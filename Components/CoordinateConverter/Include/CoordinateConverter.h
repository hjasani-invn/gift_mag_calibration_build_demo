/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2013
* \project         IndoorSpirit
* \brief           Coordinate conversion class
* \file            CoordinateConverter.h
* \author          D. Churikov
* \date            8.10.2013
*/

/** \addtogroup COORDINATE_CONVERTER The coordinate converter class
*  Class GeoLocConverter2D 
*/

#ifndef _COORDINATE_CONVERTER_H_
#define _COORDINATE_CONVERTER_H_

// include standard integer types
#include <stdint.h>


// !!! temporary define !!!
// universal float types
#define float32_t   float
#define float64_t   double
// !!! end of temporary define !!!

/**
* Coordinate converter class
* \ingroup COORDINATE_CONVERTER
*/
class GeoLocConverter2D
{
    public:
        /**
        * GeoLocConverter2D constructor
        */
        GeoLocConverter2D() : f_init(false), lat0(0), lon0(0), alfa_lat(1), alfa_lon(1), heading(0){};

        /**
        * GeoLocConverter2D destructor
        */
        ~GeoLocConverter2D() {};

        
        /**
        * Function returns initialysed status
        */
        bool IsInitialized() const { return f_init; }


        /**
        * Function sets frame conversion parameters
        * \param[in] lat0 - latitude of local frame origin point in earth frame, deg
        * \param[in] lon0 - longitude of local frame origin point in earth frame, deg
        * \param[in] alfa_lat - latitude linear transformation factor
        * \param[in] alfa_lon - longitude linear transformation factor
        * \param[in] heading - angle from geo North (x axis) to local frame x-axis, deg, counter clockwise
        * \returns "true" on success
        */
        bool SetFrameParams( float64_t   lat0,
                             float64_t   lon0,
                             float64_t   alfa_lat,
                             float64_t   alfa_lon,
                             float64_t   heading );

        /**
        * Function gets frame conversion parameters
        * \param[out] lat0 - latitude of local frame origin point in earth frame, deg
        * \param[out] lon0 - longitude of local frame origin point in earth frame, deg
        * \param[out] alfa_lat - latitude linear transformation factor
        * \param[out] alfa_lon - longitude linear transformation factor
        * \param[out] heading - angle from geo North (x axis) to local frame x-axis, deg, counter clockwise
        * \returns "true" if converter contains actual parameters (can be used in conversion functions)
        */
        bool GetFrameParams( float64_t   *lat0,
                             float64_t   *lon0,
                             float64_t   *alfa_lat,
                             float64_t   *alfa_lon,
                             float64_t   *heading ) const;

        /**
        * The function gets current azimut 
        * \param[out] heading - angle from geo North (x axis) to local frame x-axis, deg, counter clockwise
        */
        bool GetFrameAzimut(float64_t   &heading) const;

        /**
        * Function converts Geo coordinates to local coordinates.
        * As default is used the following notation for geo frame: Lattitude - geo_x; Longitude - geo_y.
        * Realy function transform coordinates from source Cartesiane frame to destination Cartesiane frame according parameters defined in SetFrameParams
        * \param[in] lat - latitude
        * \param[in] lon - longitude
        * \param[out] x - local X coordinate (from input lat-lon)
        * \param[out] y - local Y coordinate (from input lat-lon)
        * \returns "true" on success
        */
        bool Geo2Local( float64_t lat, float64_t lon, float64_t *x, float64_t *y ) const;

        /**
        * Function converts local coordinates to Geo coordinates
        * As default is used the following notation for geo frame: Lattitude - geo_x; Longitude - geo_y.
        * Realy function transform coordinates from source Cartesiane frame to destination Cartesiane frame according parameters defined in SetFrameParams
        * \param[in] x - local X coordinate
        * \param[in] y - local Y coordinate
        * \param[out] lat - latitude (from input X-Y)
        * \param[out] lon - longitude (from input X-Y)
        * \returns "true" on success
        */
        bool Local2Geo( float64_t x, float64_t y, float64_t *lat, float64_t *lon ) const;

        /**
        * Function converts vector in Geo frame to Local frame
        * As default is used the following notation for geo frame: Lattitude - geo_x; Longitude - geo_y.
        * Realy function transform vector from source Cartesiane frame to destination Cartesiane frame according parameters defined by SetFrameParams
        * \param[in] v_geo - 2D vector repesented in Geo frame
        * \param[out] v_local- 2D vector repesented in Local frame
        */
        bool Geo2Local_vector(const float64_t v_geo[2], float64_t v_local[2]) const;

        /**
        * Function converts vector in Local frame to Geo frame
        * As default is used the following notation for Geo frame: Lattitude - geo_x; Longitude - geo_y.
        * Realy function transform vector from source cartesiane frame to destination cartesiane frame according parameters defined by SetFrameParams
        * \param[in] v_local - 2D vector repesented in Localframe
        * \param[out] v_geo - 2D vector repesented in Geo frame
        */
        bool Local2Geo_vector(const float64_t v_local[2], float64_t v_geo[2]) const;

        /**
        * Function converts covariance matrix given in Geo frame to Local frame
        * As default is used the following notation for geo frame: Lattitude - geo_x; Longitude - geo_y.
        * \param[in] C_geo - 2D covariance matrix repesented in Geo frame
        * \param[out] C_local- 2D covariance matrix repesented in Local frame
        */
        bool Geo2Local_cov_matrix(const float64_t C_geo[2][2], float64_t C_local[2][2]) const;

        /**
        * Function converts covariance matrix given in Local frame to Geo frame
        * As default is used the following notation for geo frame: Lattitude - geo_x; Longitude - geo_y.
        * \param[in] C_local - 2D covariance matrix repesented in Geo frame
        * \param[out] C_geo- 2D covariance matrix repesented in Local frame
        */
        bool Local2Geo_cov_matrix(const float64_t C_local[2][2], float64_t C_geo[2][2]) const;

        /**
        * Function converts Geo heading to local heading, function uses degrees
        * \param[in] geo_heading - heading in Geo system, deg, [-360; 360]
        * \param[out] local_heading - heading in local system, deg
        * \returns "true" on success
        */
        bool Geo2Local_Heading( float64_t geo_heading, float64_t *local_heading ) const;

        /**
        * Function converts local heading to Geo heading, function uses degrees
        * \param[in] local_heading - heading in local system, deg, [-360; 360]
        * \param[out] geo_heading - heading in Geo system, deg
        * \returns "true" on success
        */
        bool Local2Geo_Heading( float64_t local_heading, float64_t *geo_heading )const;

        /**
        * Function converts Geo heading to local heading, function uses radians
        * \param[in] geo_heading - heading in Geo system, rad, [-2Pi;2Pi]
        * \param[out] local_heading - heading in local system, rad
        * \returns "true" on success
        */
        bool Geo2Local_Heading_rad( float64_t geo_heading, float64_t *local_heading ) const;

        /**
        * Function converts local heading to Geo heading, function uses radians
        * \param[in] local_heading - heading in local system, rad, [-2Pi;2Pi]
        * \param[out] geo_heading - heading in Geo system, rad
        * \returns "true" on success
        */
        bool Local2Geo_Heading_rad( float64_t local_heading, float64_t *geo_heading )const;


    private:
        bool        f_init;
        float64_t   lat0;
        float64_t   lon0;
        float64_t   alfa_lat;
        float64_t   alfa_lon;
        float64_t   heading;
};

#endif
