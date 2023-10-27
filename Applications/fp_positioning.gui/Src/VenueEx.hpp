#ifndef VENUEEX_HPP
#define VENUEEX_HPP

#include <stdint.h>

#include <Venue.h>


/** Venue extation information*/

struct VenueEx : VenueTag
{
    double plan_size_x;              /**< x axis venue plan (picture) size [m] */
    double plan_size_y;              /**< y axis venue plan (picture) size [m] */
};

#endif //VENUEEX_HPP
