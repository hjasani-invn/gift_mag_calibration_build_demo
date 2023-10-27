/**
* \file CTpnData.h
* \brief C wrapper for definitions of TPN output interface
* \author Mostafa Elhoushi (melhoushi@invensense.com)
* \version 0.1
* \copyright InvenSense, all rights reserved
* \date Oct 27, 2016
*/

#ifndef C_TPN_DATA_H
#define C_TPN_DATA_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>


/** Structure of TPN position */
typedef struct CTpnPositionTag
{
    double lattitude;           ///< lattitude, [deg] [-90..90]
    double longitude;           ///< longitude, [deg] [-180..180]
    double user_heading;        ///< speed vector dicrection [deg] [-180..180] (user heading)

    double sigma_north;         ///< [m] position standard deviation in north direction
    double sigma_east;          ///< [m] position standard deviation in east direction
    double sigma_user_heading;  ///< [deg]

    double misalignment;        ///< user/device heading misalignment angle, [deg]
    double sigma_misalignment;  ///< user/device heading misalignment std, [deg]

    int16_t floor;              ///< floor number
    double altitude;            ///< altitude above sea level [m]
    double sigma_altitude;      ///< altitude standard deviation [m]

    int8_t navigation_phase;    ///< navigation phase flag; position is avaliable for navigation_phase > 0
    int8_t fidgeting_flag;      ///< fidjeting flag

    bool is_valid;              ///< position validity flag
} CTpnPosition;

/** Structure of TPN position */
typedef struct CTpnPositionExtraTag
{
    int16_t size;
    int8_t mode_of_transit;     ///< mode of transit, 1 - walking, 2 - elevator, 3 - stairs, 4 - escalator walking, 5 - escalator standing, 6 - fidgeting, 9 - running
} CTpnPositionExtra;

void CTpnPosition_print( CTpnPosition p, FILE * out );

/** Structure of device attitude of TPN */
typedef struct CTpnAttitudeTag
{
    int8_t orientation_id;      ///< orientation based on pitch [-1: vertical up; 0: horisontal; +1: vertical down]
    ///< axis definition is described in RTFPPL design document

    float roll;                 ///< device roll, [deg] [-180..180]
    float pitch;                ///< device pitch, [deg] [-90..90]
    float heading;              ///< device heading, [deg] [-180..180]

    float sigma_roll;           ///< device roll standard deviation
    float sigma_pitch;          ///< device pitch standard deviation
    float sigma_heading;        ///< device heading standard deviation

    bool is_valid;              ///< attitude validity flag
} CTpnAttitude;

void CTpnAttitude_print( CTpnAttitude a, FILE * out );

/** defines mag meassurements with uncertainties */
typedef struct CTpnMagneticMeasurementTag
{
    double mX;                  ///< magnetic field on x-axis [mG]
    double mY;                  ///< magnetic field on y-axis [mG]
    double mZ;                  ///< magnetic field on z-axis [mG]

    double sigma_mX;            ///<  x-axis magnetometer meassurement noise [mG]
    double sigma_mY;            ///<  y-axis magnetometer meassurement noise [mG]
    double sigma_mZ;            ///<  z-axis magnetometer meassurement noise [mG]

    double covarianceMatrix[3][3];      ///< mag bias error covariance matrix [mG^2], column order
    /* cov(mX, mX), cov(mY, mX), cov(mZ, mX)
    * cov(mX, mY), cov(mY, mY), cov(mZ, mY)
    * cov(mX, mZ), cov(mY, mZ), cov(mZ, mZ) */

    uint8_t level_of_calibration;    ///< this parameter describes a consistency class (level) of bias estimation and bias covariance matrix estimation
    ///< Class 0 - bias unreliable (even if the covariance seems small 1-2uT) maybe only few points were used. Note that the covariance stand for the cluster covariance.
    ///< Class 1- bias cannot be trusted
    ///< Class 2- bias maybe trusted
    ///< Class 3- bias can be trusted

    bool is_valid;              ///< attitude validity flag
} CTpnMagneticMeasurement;

void CTpnMagneticMeasurement_print( CTpnMagneticMeasurement m, FILE * out );

/** Structure of TPN PDR data */
typedef struct CTpnPdrTag
{
    double stride_length;       ///< stride length [m]
    bool is_valid;              ///< stride length validity flag

    double misalignment;        ///< user/device heading misalignment angle, [deg]
    double misalignment_p1;     ///< user/device heading misalignment angle from TPN phase 1, [deg]
    double misalignment_p2;     ///< user/device heading misalignment angle from TPN phase 2, [deg]

    uint8_t use_case;           ///< pdr use case [auto, general, pocket, etc.]
} CTpnPdr;

void CTpnPdr_print( CTpnPdr p, FILE * out );

/** TPN output structure*/
typedef struct CTpnOutputTag
{
    double timestamp;                       ///< timestamp [sec]
    CTpnPosition position;                  ///< TPN user position information
    CTpnAttitude attitude;                  ///< TPN device attitude information
    CTpnPdr      pdr;                       ///< Addidtional information from PDR

    CTpnMagneticMeasurement mag_meas;       ///< mag data synchronized with IRL output
} CTpnOutput;

void CTpnOutput_print( const CTpnOutput* o, FILE * out );

#endif //C_TPN_DATA_H
