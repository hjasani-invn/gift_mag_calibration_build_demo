#ifndef VENUE_DATA_H
#define VENUE_DATA_H
#include <stdint.h>

typedef uint64_t venue_id; /**<  unique venue identifier type */

/** Venue type enumeration*/
typedef enum VenueTypeTag
{
    kDefaultVenue = 0, /**< default */
    kMallVenue = 1,    /**< mall/venue */
    kAisleVenue = 2,   /**< aisle/retail */
    kOfficeVenue = 3,   /**< office */
    kFactoryVenue = 4   /**< factory */
}VenueType;

/** defines cell configuration in fingerprint grid*/
typedef enum CellTypeTag
{
    CELL_SQUARE = 0,     /**< use square cells */
    CELL_HEXAGONAL = 1   /**< use hexagonal cells */
}CellType;

/** General venue information structure*/
typedef struct BaseVenueTypeTag
{
    venue_id id;                /**< venue identifier */
    double origin_lattitude;    /**< left corner lattitude [deg] [-90..+90] */
    double origin_longitude;    /**< bottom left corner longitude [deg] [-180..+180] */
    double origin_altitude;     /**< zero floor altitude [m] */
    double origin_azimuth;      /**< venue x-axix rotation to true north [deg] [-180..+180] */
    uint16_t floors_count;      /**< total floor number in venue */
    double floor_height;        /**< floor height [m] */
    double size_x;              /**< x axis venue size [m] */
    double size_y;              /**< y axis venue size [m] */

    double alfa;                /**< axis transforamtion param (scale factor) */
    double beta;                /**< axis transforamtion param (scale factor) */

    VenueType venue_type;       /**< venue type*/
    int16_t floor_shift;        /**< floor shift*/
    bool    floor_zero_enable;   /**< floor number zero is enabled*/

} BaseVenueType;


/** Old version of Venue information structure, defines transformation to local frame*/
#ifndef VENUE_DATA_HPP // to avoid conflict with old version of venue.hpp
#define VENUE_DATA_HPP

typedef struct VenueTag
{
    venue_id id;                /**< venue identifier */
    double origin_lattitude;    /**< left corner lattitude [deg] [-90..+90] */
    double origin_longitude;    /**< bottom left corner longitude [deg] [-180..+180] */
    double origin_altitude;     /**< zero floor altitude [m] */
    double origin_azimuth;      /**< venue x-axix rotation to true north [deg] [-180..+180] */
    uint16_t floors_count;      /**< total floor number in venue */
    double floor_height;        /**< floor height [m] */
    double size_x;              /**< x axis venue size [m] */
    double size_y;              /**< y axis venue size [m] */

    double alfa;                /**< axis transforamtion param (scale factor) */
    double beta;                /**< axis transforamtion param (scale factor) */

    double magX;                /**< average venue magnetic vector X [uT] */
    double magY;                /**< average venue magnetic vector Y [uT] */
    double magZ;                /**< average venue magnetic vector Z [uT] */

} Venue;
#endif //VENUE_DATA_HPP

#endif //VENUE_DATA_H
