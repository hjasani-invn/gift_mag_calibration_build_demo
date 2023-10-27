/*****************************************************************************
*    Copyright (c) 2016 Invensense Inc
******************************************************************************/
/**
*   @project                MapperDataConverter
*   @brief                  Conversion of track data to TPN data
*   @author                 D. Churikov
*   @date                   14.12.2016
*/
/*****************************************************************************/
#include "TrackData2TPNData.h"
#include "tpn_quat_lib.hpp"

#include <iomanip>
#include <iostream>

const double ms_per_sec = 1000.;
const double rad_per_degree = 0.017453292519943;
const double mGauss_per_uTesla = 0.1;

//=======================================================================
// Quaretnion multiplication
static void quat_mlt(const FLOAT32 first[4], const FLOAT32 second[4], FLOAT32 result[4])
{
    double w1 = first[0];
    double x1 = first[1];
    double y1 = first[2];
    double z1 = first[3];

    double w2 = second[0];
    double x2 = second[1];
    double y2 = second[2];
    double z2 = second[3];

    result[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    result[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    result[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    result[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}


//=======================================================================
void SaveTpnData(std::ofstream &os, const TpnOutput &tpn_data)
{

    // time
    os << std::setprecision(0) << (int64_t)(tpn_data.timestamp * ms_per_sec);

    // position
    os << std::setprecision(10) << ", " << tpn_data.position.lattitude;
    os << std::setprecision(10) << ", " << tpn_data.position.longitude;

    os << std::setprecision(5) << ", " << tpn_data.position.sigma_north;
    os << std::setprecision(5) << ", " << tpn_data.position.sigma_east;

    os << std::setprecision(0) << ", " << tpn_data.position.floor;
    os << std::setprecision(5) << ", " << tpn_data.position.altitude;
    os << std::setprecision(5) << ", " << tpn_data.position.sigma_altitude;

    os << std::setprecision(5) << ", " << tpn_data.position.misalignment;
    os << std::setprecision(5) << ", " << tpn_data.position.user_heading;
    os << std::setprecision(5) << ", " << tpn_data.position.sigma_user_heading;

    // attitude
    os << std::setprecision(10) << ", " << tpn_data.attitude.roll;
    os << std::setprecision(10) << ", " << tpn_data.attitude.pitch;
    os << std::setprecision(10) << ", " << tpn_data.attitude.heading;

    os << std::setprecision(10) << ", " << tpn_data.attitude.sigma_roll;
    os << std::setprecision(10) << ", " << tpn_data.attitude.sigma_pitch;
    os << std::setprecision(10) << ", " << tpn_data.attitude.sigma_heading;

    os << std::setprecision(1) << ", " << (int)tpn_data.attitude.orientation_id;

    // magnetic data
    os << std::setprecision(5) << ", " << tpn_data.mag_meas.mX;
    os << std::setprecision(5) << ", " << tpn_data.mag_meas.mY;
    os << std::setprecision(5) << ", " << tpn_data.mag_meas.mZ;

    // flags
    os << std::setprecision(1) << ", " << (int)tpn_data.position.navigation_phase;
    os << std::setprecision(1) << ", " << (int)tpn_data.position.fidgeting_flag;

    // step length
    os << std::setprecision(5) << ", " << tpn_data.pdr.stride_length;

    // magnetic meas is_valid
    os << std::setprecision(5) << ", " << tpn_data.mag_meas.is_valid;

    os << std::endl;
}

//=======================================================================
void CalcTPNData(
    TpnOutput &tpn_data, 
    const GeoLocConverter2D &geo2local,
    int64_t sys_t,
    double x,
    double y,
    int16_t floor,
    const double q[4],
    double mag_vector[3],
    double p_event,
    double step_length,
    bool   mag_meas_is_valid
    )
{
    // time
    tpn_data.timestamp = (double)sys_t / ms_per_sec;

    // position
    double lat, lon;
    geo2local.Local2Geo(x, y, &lat, &lon);
    tpn_data.position.lattitude = lat;
    tpn_data.position.longitude = lon;

    tpn_data.position.sigma_north = 1.; // m, default value
    tpn_data.position.sigma_east = 1.; // m, default value

    tpn_data.position.floor = floor;
    tpn_data.position.altitude = 0; // m, default value
    tpn_data.position.sigma_altitude = 1.; // m, default value

    tpn_data.position.misalignment = 0; // m, default value
    
    tpn_data.position.sigma_user_heading = 2; // deg, default value

    tpn_data.position.is_valid = true;
    //tpn_data.position.is_valid = is_valid;

    // attitude conversion
    FLOAT32 ang[3], C[3][3], q_udf2ned[4], q_udf2mfp[4];
    const FLOAT32 sqrt_2_2 = sqrt(2) / 2.;
    const FLOAT32 q_mfp2ned[4] = { 0, -sqrt_2_2, -sqrt_2_2, 0 };  // MFP to NED frame conversion
    
    q_udf2mfp[0] = q[0];
    q_udf2mfp[1] = q[1];
    q_udf2mfp[2] = q[2];
    q_udf2mfp[3] = q[3];

    quat_mlt(q_mfp2ned, q_udf2mfp, q_udf2ned);

    {
        FLOAT32 q_udf2mfp1[4];
        const FLOAT32 q_ned2mfp[4] = { 0, sqrt_2_2, sqrt_2_2, 0 };  // MFP to NED frame conversion
        quat_mlt(q_ned2mfp, q_udf2ned, q_udf2mfp1);
        quaternion_to_dcm(q_udf2mfp, C);
        dcm_to_euler_angles(C, ang);
    }

    quaternion_to_dcm(q_udf2ned, C);
    dcm_to_euler_angles(C, ang);

    // attitude
    double local_heading;
    bool conversion_res = geo2local.GetFrameAzimut(local_heading);
    tpn_data.attitude.roll = ang[0] / rad_per_degree;
    tpn_data.attitude.pitch = ang[1] / rad_per_degree;
    tpn_data.attitude.heading = ang[2] / rad_per_degree - local_heading; // heading in NED
    tpn_data.position.user_heading = tpn_data.attitude.heading;

    tpn_data.attitude.sigma_roll = 0.5;
    tpn_data.attitude.sigma_pitch = 0.5;
    tpn_data.attitude.sigma_heading = 2;

    tpn_data.attitude.orientation_id = 0;
    tpn_data.attitude.is_valid = true;

    // magnetic data
    tpn_data.mag_meas.mX = mag_vector[0] / mGauss_per_uTesla;
    tpn_data.mag_meas.mY = mag_vector[1] / mGauss_per_uTesla;
    tpn_data.mag_meas.mZ = mag_vector[2] / mGauss_per_uTesla;

    tpn_data.mag_meas.sigma_mX = 1. / mGauss_per_uTesla; // mGauss, default value
    tpn_data.mag_meas.sigma_mY = 1. / mGauss_per_uTesla; // mGauss, default value
    tpn_data.mag_meas.sigma_mZ = 1. / mGauss_per_uTesla; // mGauss, default value

    tpn_data.mag_meas.sigma_mX = 0.5 / mGauss_per_uTesla; // mGauss, default value
    tpn_data.mag_meas.sigma_mY = 0.5 / mGauss_per_uTesla; // mGauss, default value
    tpn_data.mag_meas.sigma_mZ = 0.5 / mGauss_per_uTesla; // mGauss, default value

    memset(tpn_data.mag_meas.covarianceMatrix, 0, sizeof(tpn_data.mag_meas.covarianceMatrix));
    tpn_data.mag_meas.covarianceMatrix[0][0] = 1. / mGauss_per_uTesla; // mGauss, default value
    tpn_data.mag_meas.covarianceMatrix[1][1] = 1. / mGauss_per_uTesla; // mGauss, default value
    tpn_data.mag_meas.covarianceMatrix[2][2] = 1. / mGauss_per_uTesla; // mGauss, default value
        
    //tpn_data.mag_meas.is_valid = true;
    tpn_data.mag_meas.is_valid = mag_meas_is_valid;

    // flags
    tpn_data.position.navigation_phase = 1;
    tpn_data.position.fidgeting_flag = 0; // default value

    // steps
    tpn_data.pdr.stride_length = (p_event >=0.5) ? step_length : 0;
    tpn_data.pdr.is_valid = true;
}