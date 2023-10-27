// FPBuilderLibCommonTest.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
//#include <windows.h>]

#include <string>
#include <fstream>
#include <iomanip>
#include <direct.h>

#include "GridBuilderPrivate.hpp"

void devicePositionGeneration(Fpbl::GridBuilderPrivate *pStaticBuilder, std::fstream  &log)
{
    log << std::right << std::endl;

    int width = 15;

    Fpbl::Venue venue;

    // 55.7352576, 37.6419266 Moscow Avrora Bisness Centre

    venue.id = 0;   /**< venue identifier */
    venue.originLattitude = 0;     /**< bottom left corner lattitude [rad] [-pi/2..pi/2] */
    venue.originLongitude = 0;     /**< bottom left corner longitude [rad] [-pi..pi] */
    venue.originAltitude = 0;      /**< zero floor altitude [m] */
    venue.originAzimuth = 0;       /**< venue y-axix rotation to true north [rad] [0..pi] */
    venue.floorsCount = 0;       /**< total floor number in venue */
    venue.sizeX = 100;               /**< x axis venue size [m] */
    venue.sizeY = 20;               /**< y axis venue size [m] */

    pStaticBuilder->setVenue(venue);

    Fpbl::Grid grid = { Fpbl::CellType::CELL_SQUARE, 5.0 };
    Fpbl::WiFiGrid wifiGrid;


    // Positions

    Fpbl::Position devicePosition;
    uint64_t timestamp;

    std::vector<uint64_t> posMeasurementTimestamps;
    std::vector<Fpbl::Position> posMeasurements;

    int N = 20 * 2;
    int M = 4 * 2;

    srand(1000);
    //srand(time(NULL));

    timestamp = 1000000; // us

    devicePosition.lattitude = 5.0 / 4;       /**< lattitude [rad] [-pi/2..pi/2]*/
    devicePosition.longitude = 5.0 / 4;       /**< longitude [rad] [-pi..pi] */
    devicePosition.altitude = 0;        /**< altitude (sea level?) [m] */

    devicePosition.floorNumber = 0;    /**< discrete floor number [0..32767], must be positive or zero */
    devicePosition.covarianceLatLon[0][0] = 0;
    devicePosition.covarianceLatLon[0][1] = 0;
    devicePosition.covarianceLatLon[1][0] = 0;
    devicePosition.covarianceLatLon[1][1] = 0;
    /** Lattitude/Longitude covariance matrix in column order [rad^2]\n
    *  cov(lat,lat), cov(lon,lat)\n
    *  cov(lat,lon), cov(lon,lon) */
    devicePosition.altitudeStd = 1;     /**< altitude standard deviation [m]     */
    devicePosition.floorStd = 1;        /**< altitude standard deviation [floor] */
    devicePosition.isValid = true;           /**< position validity flag */

    Fpbl::Attitude deviceAttitude;

    deviceAttitude.quaternion[0] = 1;
    deviceAttitude.quaternion[1] = 0;
    deviceAttitude.quaternion[2] = 0;
    deviceAttitude.quaternion[3] = 0;
    deviceAttitude.covarianceQuaternion[0][0] = 1;
    deviceAttitude.covarianceQuaternion[0][1] = 1;
    deviceAttitude.covarianceQuaternion[0][2] = 1;
    deviceAttitude.covarianceQuaternion[0][3] = 1;
    deviceAttitude.covarianceQuaternion[1][0] = 1;
    deviceAttitude.covarianceQuaternion[1][1] = 1;
    deviceAttitude.covarianceQuaternion[1][2] = 1;
    deviceAttitude.covarianceQuaternion[1][3] = 1;
    deviceAttitude.covarianceQuaternion[2][0] = 1;
    deviceAttitude.covarianceQuaternion[2][1] = 1;
    deviceAttitude.covarianceQuaternion[2][2] = 1;
    deviceAttitude.covarianceQuaternion[2][3] = 1;
    deviceAttitude.covarianceQuaternion[3][0] = 1;
    deviceAttitude.covarianceQuaternion[3][1] = 1;
    deviceAttitude.covarianceQuaternion[3][2] = 1;
    deviceAttitude.covarianceQuaternion[3][3] = 1;
    deviceAttitude.isValid = true;           /**< orientation validity flag */


    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            posMeasurementTimestamps.push_back(timestamp);
            posMeasurements.push_back(devicePosition);

            pStaticBuilder->processDevicePosition(timestamp, devicePosition);

            for (int k = 0; k < 20; k++)
                pStaticBuilder->processDeviceAttitude(timestamp + k * 50000, deviceAttitude);

            timestamp += 1000000; // us
            if (m % 2 == 0)
                devicePosition.lattitude += 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
            else
                devicePosition.lattitude -= 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/

        }

        if (m % 2 == 0)
            devicePosition.lattitude -= 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
        else
            devicePosition.lattitude += 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
        devicePosition.longitude += 5.0 / 2;       /**< longitude [rad] [-pi..pi] */

    }

    for (int i = 0; i < M*N; i++)
    {
        timestamp = pStaticBuilder->getDevicePositionTimestamp(i);
        devicePosition = pStaticBuilder->getDevicePosition(i);

        log.width(width);
        log << timestamp;
        log.width(width);
        log << devicePosition.lattitude;
        log.width(width);
        log << devicePosition.longitude;
        log << std::endl;
    }
}
