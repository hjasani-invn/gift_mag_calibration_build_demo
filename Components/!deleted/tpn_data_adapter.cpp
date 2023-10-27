#define _USE_MATH_DEFINES
#include "tpn_data_adapter.hpp"
#include "tpn_quat_lib.hpp"
//#include <math.h>
#include <cmath>
#include <memory.h>
#include <fstream>
#include <iostream>

//#define DEBUG_CONVERTER_OUTPUT

const double max_pdr_data_gap = 0.5;    // sec
const double ms_per_sec = 1000;
const double rad_per_degree = 0.017453292519943;

//---------------------------------------------------------------------------
static void GEO2NED_local( double lat0, double h, double d_lat, double d_lon, double &d_n, double &d_e )
{
    static const double R_EARTH_A = 6378137; // Earth semi-major axis, m
    static const double R_EARTH_B = 6356752; // Earth semi-minor axis, m
    d_n = d_lat * ( R_EARTH_B + h );
    d_e = d_lon * ( R_EARTH_A + h ) * cos( lat0 );
}

static void quat_mlt( const FLOAT32 first[4], const FLOAT32 second[4], FLOAT32 result[4] )
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

//---------------------------------------------------------------------------
/*tpn_data_adapter_t::tpn_data_adapter_t()
{
    this->rt_fppe = 0;
    this->dbg_rtfppl_stream = 0;
}*/

#ifdef DEBUG_CONVERTER_OUTPUT
std::ofstream dbg_out;         ///< debug output stream
#endif
//---------------------------------------------------------------------------
TpnDataConverter::TpnDataConverter( Fppe::FPEngine *rt_fppe0 )
{
    this->rt_fppe = rt_fppe0;
    this->dbg_rtfppl_stream = 0;
    memset( &last_pdr_data, 0, sizeof( last_pdr_data ) );

#ifdef DEBUG_CONVERTER_OUTPUT
    dbg_out.open( "tpn_converter_out.txt", std::ios_base::out );
    set_tpn_converter_out_stream( dbg_out );
#endif
}

//---------------------------------------------------------------------------
TpnDataConverter::~TpnDataConverter()
{
#ifdef DEBUG_CONVERTER_OUTPUT
    dbg_out.close();
#endif
}

//---------------------------------------------------------------------------
void TpnDataConverter::set_pdr_data( const Fppe::TpnOutput &pdr_data )
{
    last_pdr_data = pdr_data;
}

//---------------------------------------------------------------------------
void TpnDataConverter::set_tpn_converter_out_stream( std::ostream &os )
{
    dbg_rtfppl_stream = &os;
}

//---------------------------------------------------------------------------
void TpnDataConverter::out_tpn_converter_data( Fppe::CoordinatesIncrement inc_data, Fppe::MagneticVector mag_vector )
{
    if ( ( this->dbg_rtfppl_stream ) && ( !this->dbg_rtfppl_stream->bad() ) )
    {
        *dbg_rtfppl_stream << inc_data.timestamp;
        //dbg_rtfppl_stream->precision(3);
        dbg_rtfppl_stream->precision( 18 );
        *dbg_rtfppl_stream << std::scientific;
        *dbg_rtfppl_stream << " , " << inc_data.d_x << " , " << inc_data.d_y;
        *dbg_rtfppl_stream << " ,  " << inc_data.d_floor;
        *dbg_rtfppl_stream << std::scientific;
        *dbg_rtfppl_stream << " , " << inc_data.covariance_yx[0][0];
        *dbg_rtfppl_stream << " , " << inc_data.covariance_yx[0][1];
        *dbg_rtfppl_stream << " , " << inc_data.covariance_yx[1][0];
        *dbg_rtfppl_stream << " , " << inc_data.covariance_yx[1][1];

        //*dbg_rtfppl_stream  << " ,  " << inc_data.d_floor_std;

        *dbg_rtfppl_stream << std::fixed;
        //dbg_rtfppl_stream->precision(12);
        *dbg_rtfppl_stream << "  ";

        for ( int i = 0; i < 4; i++ )  *dbg_rtfppl_stream << " , " << inc_data.attitude.quaternion[i]; // quaternion

        *dbg_rtfppl_stream << " ";


        *dbg_rtfppl_stream << std::scientific;
        //dbg_rtfppl_stream->precision(3);
#if (OUT_QUATERNION_DISPERSION_ONLY)

        for ( int i = 0; i < 4; i++ )  *dbg_rtfppl_stream << " , " << inc_data.attitude.covariance_quaternion[i]; // quaternion dispersion

#else

        for ( int i = 0; i < 4; i++ )
            for ( int j = 0; j < 4; j++ )
                *dbg_rtfppl_stream << " , " << inc_data.attitude.covariance_quaternion[i][j]; // quaternion covariance matrix

#endif

        *dbg_rtfppl_stream << std::fixed;
        *dbg_rtfppl_stream << " ,  " << inc_data.attitude.is_valid;

        *dbg_rtfppl_stream << "   , " << mag_vector.mX << " , " << mag_vector.mY << " , " << mag_vector.mZ;

        *dbg_rtfppl_stream << '\n';
    }
}

//---------------------------------------------------------------------------
bool TpnDataConverter::process_pdr_data( const Fppe::TpnOutput &pdr_data )
{
    Fppe::CoordinatesIncrement fppe_data;
    Fppe::MagneticVector fppe_mag_vector;

    bool is_valid = this->convert_pdr_data( pdr_data, &fppe_data, &fppe_mag_vector );

    //if ( is_valid && this->rt_fppe)
    {
        //this->rt_fppe->processIncrements(fppe_data);
        //this->rt_fppe->processMFP(mag_vector);
    }

    if ( is_valid )
        this->out_tpn_converter_data( fppe_data, fppe_mag_vector );

    this->set_pdr_data( pdr_data );

    return is_valid;
}

//---------------------------------------------------------------------------
bool TpnDataConverter::convert_pdr_data( const Fppe::TpnOutput &pdr_data, Fppe::CoordinatesIncrement *fppe_data, Fppe::MagneticVector *fppe_mag_vector )
{
    //Fppe::CoordinatesIncrement _fppe_data;

    double dtau = this->last_pdr_data.timestamp - pdr_data.timestamp; // data gap time

    if ( std::abs( dtau ) < max_pdr_data_gap ) // abs macros to support forward/backward operations
    {
        if ( this->last_pdr_data.position.is_valid && pdr_data.position.is_valid )
        {
            //calc_pos_increment;
            fppe_data->is_step = ( pdr_data.pdr.stride_data_validity && pdr_data.pdr.stride_length > 0 );
            fppe_data->timestamp = floor( pdr_data.timestamp * ms_per_sec + 0.5 );

            this->convert_position_data( this->last_pdr_data.position, pdr_data.position, fppe_data );

            if ( pdr_data.attitude.is_valid )
                this->convert_attitude_data( pdr_data.attitude, &fppe_data->attitude ); //convert attitude;

            fppe_data->attitude.is_valid = pdr_data.attitude.is_valid;
        }
    }

    this->convert_magnetic_data( pdr_data.timestamp, pdr_data.mag_data, fppe_mag_vector );

    return this->last_pdr_data.position.is_valid && pdr_data.position.is_valid;
}

void TpnDataConverter::convert_magnetic_data( const double &time_tag, const Fppe::MagneticVector  &pdr_mag_vector, Fppe::MagneticVector *fppe_mag_vector )
{
    static double mGauss_per_uTesla = 0.1;
    fppe_mag_vector->timestamp = ( int64_t )( time_tag * ms_per_sec );
    fppe_mag_vector->mX = pdr_mag_vector.mX * mGauss_per_uTesla;
    fppe_mag_vector->mY = pdr_mag_vector.mY * mGauss_per_uTesla;
    fppe_mag_vector->mZ = pdr_mag_vector.mZ * mGauss_per_uTesla;

    // DEBUG LOGIC: conversion: UDF to Phone Meas Coord Sysytem  (PMSC)
    // xu -> yp, yu -> xp, zu -> -zp
    /*float tm_flt = fppe_mag_vector.mY;
    fppe_mag_vector.mY = fppe_mag_vector.mX;
    fppe_mag_vector.mX = tm_flt;
    fppe_mag_vector.mZ = -fppe_mag_vector.mZ;
    */
}

const FLOAT32 sqrt_2_2 = sqrt( 2 ) / 2.;
const FLOAT32 q1[4] = { sqrt_2_2, 0, sqrt_2_2, 0 };     // UDF to Internal Up conversion
const FLOAT32 q0[4] = { 1., 0, 0., 0 };                 // UDF to Internal Horizontal conversion
const FLOAT32 q_1[4] = { sqrt_2_2, 0, -sqrt_2_2, 0 };   // UDF to Internal Down conversion
const FLOAT32 q_dev2udf[4] = { 0, sqrt_2_2, sqrt_2_2, 0 };  // Device Coordinat System (Android) to UDF conversion
const FLOAT32 q_ned2mfp[4] = { 0, sqrt_2_2, sqrt_2_2, 0 };  // NED to MFP frame conversion
//---------------------------------------------------------------------------
void TpnDataConverter::convert_attitude_data( const Fppe::TpnAttitude &tpn_att, Fppe::Attitude *att )
{
    FLOAT32 ang[3];
    FLOAT32 Cbn[3][3];
    ang[0] = tpn_att.roll    * rad_per_degree;
    ang[1] = tpn_att.pitch   * rad_per_degree;
    ang[2] = tpn_att.heading * rad_per_degree;
    euler_angles_to_dcm( ang, Cbn ); // calc dcm

    // calc quaternion from internal TPN frame to qNED
    FLOAT32 q[4];
    dcm_to_quaternion( Cbn, q );

    // calc quaternion from internal UDF frame to qNED: q_udf2qned = q_int2ned*q_udf2int
    if ( tpn_att.orientation_id == -1 )  quat_mlt( q, q_1, q );

    if ( tpn_att.orientation_id == 0 )   quat_mlt( q, q0, q );

    if ( tpn_att.orientation_id == 1 )   quat_mlt( q, q1, q );

    // DEBUG LOGIC: device Meas Coord Sysyt (Android) to qNED: q_dev2qned = q_udf2qned*q_dev2udf
    //quat_mlt(q, q_dev2udf, q);

    // calc quaternion to quazy FP frame - it is FP frame ritated on h angel
    quat_mlt( q_ned2mfp, q, q );

    // save calculated quaternion
    att->quaternion[0] = q[0];
    att->quaternion[1] = q[1];
    att->quaternion[2] = q[2];
    att->quaternion[3] = q[3];

    // calc quaternion covariance matrix
    // TODO: conversion of Euler angle dispersion to quaternion covariance matrix
    //memset(att.covariance_quaternion, 0, sizeof(Fppe::Attitude::covariance_quaternion)); // ! check this memset
    memset( att->covariance_quaternion, 0, sizeof( att->covariance_quaternion ) ); // ! check this memset
    att->covariance_quaternion[0][0] = 1e-3; // ! debug value
    att->covariance_quaternion[1][1] = 1e-3; // ! debug value
    att->covariance_quaternion[2][2] = 1e-3; // ! debug value
    att->covariance_quaternion[3][3] = 1e-3; // ! debug value

    att->is_valid = tpn_att.is_valid;
}


void TpnDataConverter::convert_position_data( const Fppe::TpnPosition &tpn_pos0, const Fppe::TpnPosition &tpn_pos1, Fppe::CoordinatesIncrement *pos_inc )
{
    double d_north, d_east, d_down, d_up;
    double d_lat = ( tpn_pos1.lattitude - tpn_pos0.lattitude ) * rad_per_degree;
    double d_lon = ( tpn_pos1.longitude - tpn_pos0.longitude ) * rad_per_degree;
    double alpha = 1;
    //double pos_sigma = (tpn_pos1.sigma_longitude - tpn_pos0.sigma_longitude) * alpha;
    double pos_sigma = 0.1; // 0.07 // position sigma for 10 Hz input

    GEO2NED_local( tpn_pos0.lattitude * rad_per_degree, tpn_pos0.height, d_lat, d_lon, d_north, d_east );
    d_down = -( tpn_pos1.height - tpn_pos0.height ); // sign '-' becouse it is down direction


    // NED to NWU conversion
    double pdr_scale = 1;// 1/1.2;
    pos_inc->d_x = d_north * pdr_scale;
    pos_inc->d_y = -d_east * pdr_scale;
    d_up = -d_down;

    // delta floor calculation
    //pos_inc.d_floor = dh / 5; //! 5 m is floor height - debug value
    pos_inc->d_floor = 0;       //! debug value
    pos_inc->d_floor_std = 0.0; //! debug value


    pos_inc->covariance_yx[0][0] = pos_sigma * pos_sigma;   pos_inc->covariance_yx[0][1] = 0.0;
    pos_inc->covariance_yx[1][0] = 0.0;                     pos_inc->covariance_yx[1][1] = pos_sigma * pos_sigma;

}
