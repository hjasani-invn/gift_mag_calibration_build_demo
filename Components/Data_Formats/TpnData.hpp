/**
* \file TpnData.hpp
* \brief Defines TPN output interface
* \author Mikhail Frolov (mfrolov@invensense.com)
* \version 0.1
* \copyright InvenSense, all rights reserved
* \date Jun 2, 2016
*/

#ifndef TPN_DATA_HPP
#define TPN_DATA_HPP

#include <stdint.h>
#include <ostream>
#include <iomanip>

/** Transit type enumeration*/
typedef enum ModeOfTransitTag    // 1 - walking, 2 - elevator, 3 - stairs, 4 - escalator walking, 5 - escalator standing,
                                 // 6 - fidgeting, 7 - conveyer walking, 7 - conveyer standing, 9 - running
{
    kWalking = 1, 
    kElevator = 2, 
    kStairs = 3, 
    kEscalatorWalking = 4, 
    kEscalatorStanding = 5, 
    kFidgeting = 6,
    kConveyorWalking = 7,
    kConveyorStanding = 8,
    kRunning = 9
} ModeOfTransit;

/** Structure of TPN position */
struct TpnPosition
{
    TpnPosition()
    {
        lattitude = 0.0;
        longitude = 0.0;
        user_heading = 0.0;
        sigma_north = 0.0;
        sigma_east = 0.0;
        sigma_user_heading = 0.0;
        misalignment = 0.0;
        sigma_misalignment = 0.0;
        floor = 0;
        altitude = 0.0;
        sigma_altitude = 0.0;
        navigation_phase = 0;
        fidgeting_flag = 0;
        mode_of_transit = 1;
        is_valid = false;
    }

    double lattitude;           ///< lattitude, [deg] [-90..90]
    double longitude;           ///< longitude, [deg] [-180..180]
    double user_heading;        ///< speed vector dicrection [deg] [-180..180] (user heading)

    double sigma_north;         ///< [m] position standard deviation in north direction
    double sigma_east;          ///< [m] position standard deviation in east direction
    double sigma_user_heading;  ///< [deg]

    double misalignment;        ///< user/device heading misalignment angle, [deg]
    double sigma_misalignment;  ///< user/device heading misalignment std, [deg]

    int16_t floor;              ///< tpn-floor number, starting from 1
    double altitude;            ///< altitude above sea level [m]
    double sigma_altitude;      ///< altitude standard deviation [m]

    int8_t navigation_phase;    ///< navigation phase flag; position is avaliable for navigation_phase > 0
    int8_t fidgeting_flag;      ///< fidgeting flag
    int8_t mode_of_transit;     ///< mode of transit, 1 - walking, 2 - elevator, 3 - stairs, 4 - escalator walking, 5 - escalator standing, 6 - fidgeting, 9 - running

    bool is_valid;              ///< position validity flag

    void print( std::basic_ostream<char> &out )
    {
		out << std::setprecision(12);
        out << ", " << lattitude;           ///< lattitude, [deg] [-90..90]
        out << ", " << longitude;           ///< longitude, [deg] [-180..180]
        out << ", " << user_heading;        ///< speed vector dicrection [deg] [-180..180] (user heading)

        out << ", " << sigma_north;         ///< [m] position standard deviation in north direction
        out << ", " << sigma_east;          ///< [m] position standard deviation in east direction
        out << ", " << sigma_user_heading;  ///< user heading uncertanties,[deg]; this field also indicates validity of itself, user_heading and misalignment parameters
        /// < these fields are invalid when sigma_user_heading <= 0

        out << ", " << misalignment;        ///< user/device heading misalignment angle [deg]
        out << ", " << sigma_misalignment;  ///< user/device heading misalignment std [deg]; reserved

        out << ", " << floor;               ///< floor number
        out << ", " << altitude;            ///< altitude above sea level [m]
        out << ", " << sigma_altitude;      ///< altitude standard deviation [m]

        out << ", " << static_cast<int> (navigation_phase);  ///< navigation phase flag; position is avaliable for navigation_phase > 0
        out << ", " << static_cast<int> (fidgeting_flag);    ///< fidgeting flag
        out << ", " << static_cast<int> (mode_of_transit);   ///< mode of transit

        out << ", " << static_cast<int> (is_valid);          ///< position validity flag
    }
};

/** Structure of device attitude of TPN */
struct TpnAttitude
{
    TpnAttitude()
    {
        orientation_id = 0;
        roll = 0.0;
        pitch = 0.0;
        heading = 0.0;
        sigma_roll = 0.0;
        sigma_pitch = 0.0;
        sigma_heading = 0.0;
        is_valid = false;
    }
    int8_t orientation_id;      ///< orientation based on pitch [-1: vertical up; 0: horisontal; +1: vertical down]
    ///< axis definition is described in RTFPPL design document

    float roll;                 ///< device roll, [deg] [-180..180]
    float pitch;                ///< device pitch, [deg] [-90..90]
    float heading;              ///< device heading, [deg] [-180..180]

    float sigma_roll;           ///< device roll standard deviation
    float sigma_pitch;          ///< device pitch standard deviation
    float sigma_heading;        ///< device heading standard deviation

    bool is_valid;              ///< attitude validity flag

    void print( std::basic_ostream<char> &out )
    {
        out << ", " << roll;                 ///< device roll, [deg] [-180..180]
        out << ", " << pitch;                ///< device pitch, [deg] [-90..90]
        out << ", " << heading;              ///< device heading, [deg] [-180..180]

        out << ", " << sigma_roll;           ///< device roll standard deviation
        out << ", " << sigma_pitch;          ///< device pitch standard deviation
        out << ", " << sigma_heading;        ///< device heading standard deviation

        out << ", " << static_cast<int> (is_valid);            ///< attitude validity flag
    }
};

/** defines mag meassurements with uncertainties */
struct TpnMagneticMeasurement
{
    TpnMagneticMeasurement()
    {
        mX = mY = mZ = 0.0;
        sigma_mX = sigma_mY = sigma_mZ = 0.0;
        covarianceMatrix[0][0] = covarianceMatrix[0][1] = covarianceMatrix[0][2] = 0.0;
        covarianceMatrix[1][0] = covarianceMatrix[1][1] = covarianceMatrix[1][2] = 0.0;
        covarianceMatrix[2][0] = covarianceMatrix[2][1] = covarianceMatrix[2][2] = 0.0;
        level_of_calibration = 0;
        is_valid = false;
    }

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

    void print( std::basic_ostream<char> &out )
    {
        out << ", " << mX;                  ///< magnetic field on x-axis [mG]
        out << ", " << mY;                  ///< magnetic field on y-axis [mG]
        out << ", " << mZ;                  ///< magnetic field on z-axis [mG]

        out << ",  " << sigma_mX;            ///<  x-axis magnetometer meassurement noise [mG]
        out << ", " << sigma_mY;            ///<  y-axis magnetometer meassurement noise [mG]
        out << ", " << sigma_mZ;            ///<  z-axis magnetometer meassurement noise [mG]

        for ( int i = 0; i < 3; i++ )
            for ( int j = 0; j < 3; j++ )
                out << ", " << covarianceMatrix[i][j];      ///< mag bias error covariance matrix [mG^2], column order

        out << ", " << static_cast<int> (level_of_calibration); ///< this parameter describes a consistency class of bias estimation and bias covariance matrix estimation
        out << ", " << static_cast<int> (is_valid);            ///< attitude validity flag
    }
};

/** Structure of TPN PDR data */
struct TpnPdr
{
    TpnPdr()
    {
        stride_length = 0.0;
        is_valid = false;
        misalignment = 0.0;
        misalignment_p1 = 0.0;
        misalignment_p2 = 0.0;
        use_case = 0;
    }

    double stride_length;       ///< stride length [m]
    bool is_valid;              ///< stride length validity flag

    double misalignment;        ///< user/device heading misalignment angle, [deg]
    double misalignment_p1;     ///< user/device heading misalignment angle from TPN phase 1, [deg]
    double misalignment_p2;     ///< user/device heading misalignment angle from TPN phase 2, [deg]

    uint8_t use_case;           ///< pdr use case [auto, general, pocket, etc.]

    void print( std::basic_ostream<char> &out )
    {
        out << ", " << stride_length;       ///< stride length [m]
        out << ", " << is_valid;              ///< stride length validity flag

    }
};

/** TPN output structure*/
struct TpnOutput
{
    TpnOutput()
    {
        timestamp = 0;
        position = TpnPosition();
        attitude = TpnAttitude();
        pdr = TpnPdr();
        mag_meas = TpnMagneticMeasurement();
    }

    double timestamp;           ///< timestamp [sec]
    TpnPosition position;       ///< TPN user position information
    TpnAttitude attitude;       ///< TPN device attitude information
    TpnPdr      pdr;            ///< Addidtional information from PDR

    TpnMagneticMeasurement mag_meas;    ///< mag data synchronized with IRL output

    void print( std::basic_ostream<char> &out )
    {
        out <<              static_cast<uint64_t>(timestamp * 1000 + 0.5); //timestamp in miliseconds;
        out << ", ";        position.print( out );
        out << ", ";        attitude.print( out );
        out << ", ";        pdr.print( out );
        out << ", ";        mag_meas.print( out );
        out << std::endl;
    }
};

#endif //TPN_DATA_HPP
