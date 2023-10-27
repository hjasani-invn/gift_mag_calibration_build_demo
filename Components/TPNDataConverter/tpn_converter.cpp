#define _USE_MATH_DEFINES
#include "tpn_converter.hpp"
#include "tpn_quat_lib.hpp"
#include <math.h>
#include <iostream>  // for std::cout and std::cerr

static const double ms_per_sec = 1000.;
static const double rad_per_degree = 0.017453292519943;
static const double mGauss_per_uTesla = 0.1;


#ifndef _fsign
#define _fsign(a)  (((a)>=0) ? (1.) : (-1.))
#endif


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


TpnConverter::TpnConverter()
{
    dbg_rtfppl_stream = &stub_dbg_rtfppl_stream;
}

TpnConverter::TpnConverter(const BaseVenueType &venue)
{
    dbg_rtfppl_stream = &stub_dbg_rtfppl_stream;
    SetVenueFrameParams( venue );

    floor_converter.SetFloorsParams(
        venue.floors_count,        /**< total floor number in venue */
        venue.floor_height,        /**< floor height [m] */
        venue.floor_shift,        /**< floor shift*/
        venue.floor_zero_enable
        );
}


// set venue frame parameters
bool TpnConverter::SetVenueFrameParams(const BaseVenueType &venue)
{
    floor_converter.SetFloorsParams(
        venue.floors_count,        /**< total floor number in venue */
        venue.floor_height,        /**< floor height [m] */
        venue.floor_shift,        /**< floor shift*/
        venue.floor_zero_enable
        );
    return SetVenueFrameParams(venue.origin_lattitude, venue.origin_longitude, venue.alfa, venue.beta, venue.origin_azimuth);
}

bool TpnConverter::SetVenueFrameParams( float64_t lat0, float64_t lon0, float64_t lat_scale, float64_t lon_scale, float64_t azimut )
{
    bool res = geo2local.SetFrameParams( lat0, lon0, lat_scale, lon_scale, azimut );
    geo2local.GetFrameParams( &lat0, &lon0, &lat_scale, &lon_scale, &azimut ); // copy data from geo2local
    return  res &&
            ned2local.SetFrameParams( 0, 0, _fsign( lat_scale ), _fsign( lon_scale ), azimut );
}


MffAttitude TpnConverter::ConvertAttitudeData( double timestamp, const TpnAttitude &tpn_att )
{
    const FLOAT32 sqrt_2_2 = sqrt( 2 ) / 2.;
    const FLOAT32 q1[4] = { sqrt_2_2, 0, sqrt_2_2, 0 };     // UDF to Internal Up conversion
    const FLOAT32 q0[4] = { 1., 0, 0., 0 };                 // UDF to Internal Horizontal conversion
    const FLOAT32 q_1[4] = { sqrt_2_2, 0, -sqrt_2_2, 0 };   // UDF to Internal Down conversion
    const FLOAT32 q_ned2mfp[4] = { 0, sqrt_2_2, sqrt_2_2, 0 };  // NED to MFP frame conversion
    //    const FLOAT32 q_dev2udf[4] = { 0, sqrt_2_2, sqrt_2_2, 0 };  // Device Coordinat System (Android) to UDF conversion

    FLOAT32 ang[3] = {0, 0, 0};
    FLOAT32 Cbn[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
    ang[0] = tpn_att.roll    * rad_per_degree;
    ang[1] = tpn_att.pitch   * rad_per_degree;
    double local_heading;
    bool conversion_res = ned2local.GetFrameAzimut( local_heading );
    ang[2] = ( local_heading + tpn_att.heading ) * rad_per_degree;

    euler_angles_to_dcm( ang, Cbn ); // calc dcm

    // calc quaternion from internal TPN frame to qNED
    FLOAT32 q[4];
    dcm_to_quaternion( Cbn, q );

    // calc quaternion from internal UDF frame to qFF (qNED): q_udf2qned = q_int2ned*q_udf2int
    if ( tpn_att.orientation_id == -1 )  quat_mlt( q, q_1, q );
    else if ( tpn_att.orientation_id == 0 )   quat_mlt( q, q0, q );
    else if ( tpn_att.orientation_id == 1 )   quat_mlt( q, q1, q );
    else conversion_res = false; // uncknown frame

    // DEBUG LOGIC: device Meas Coord Sysyt (Android) to qFF (qNED): q_dev2qned = q_udf2qned*q_dev2udf
    //quat_mlt(q, q_dev2udf, q);

    // calc quaternion to quazy FP frame - it is FP frame ritated on gamma angel
    quat_mlt( q_ned2mfp, q, q );

    // save calculated quaternion
    MffAttitude att;
    att.timestamp = ( int64_t )( timestamp * ms_per_sec + 0.5 );
    att.quaternion[0] = q[0];
    att.quaternion[1] = q[1];
    att.quaternion[2] = q[2];
    att.quaternion[3] = q[3];

    // Euler angle standard deviation
    if ( tpn_att.orientation_id == 0 )
    {
        att.heading_std = tpn_att.sigma_heading * rad_per_degree;
        att.pitch_std = tpn_att.sigma_pitch * rad_per_degree;
        att.roll_std = tpn_att.sigma_roll * rad_per_degree;
    }
    else
    {
        att.heading_std = tpn_att.sigma_roll * rad_per_degree;
        att.pitch_std = tpn_att.sigma_pitch * rad_per_degree;
        att.roll_std = tpn_att.sigma_heading * rad_per_degree;
    }

    att.is_valid = tpn_att.is_valid && conversion_res;

    if ( ( this->dbg_rtfppl_stream ) && ( !this->dbg_rtfppl_stream->bad() ) )
        OutAttitudeData( att );


    return att;
}

MagneticMeasurement TpnConverter::ConvertMagneticData( double timestamp, const TpnMagneticMeasurement &tpn_mag_meas )
{
    const double uTesla_per_mGauss = 0.1;
    MagneticMeasurement mag_data;
    mag_data.timestamp = ( int64_t )( timestamp * ms_per_sec + 0.5 );

    mag_data.mX = tpn_mag_meas.mX * uTesla_per_mGauss;
    mag_data.mY = tpn_mag_meas.mY * uTesla_per_mGauss;
    mag_data.mZ = tpn_mag_meas.mZ * uTesla_per_mGauss;

    mag_data.sigma_mX = tpn_mag_meas.sigma_mX * uTesla_per_mGauss;
    mag_data.sigma_mY = tpn_mag_meas.sigma_mY * uTesla_per_mGauss;
    mag_data.sigma_mZ = tpn_mag_meas.sigma_mZ * uTesla_per_mGauss;

    // warning: validity of mag data is not translated now

    //const double mag_sensor_noise = 1.;
    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 3; j++ )
        {
            mag_data.covarianceMatrix[i][j] = tpn_mag_meas.covarianceMatrix[i][j] * uTesla_per_mGauss * uTesla_per_mGauss;
        }

    if ( ( this->dbg_rtfppl_stream ) && ( !this->dbg_rtfppl_stream->bad() ) )
        OutMagneticData( mag_data );

    return mag_data;
}

FfPosition TpnConverter::ConvertPositionData( double timestamp, const TpnPosition &tpn_pos )
{
    FfPosition pos;
    double x, y;
    //std::cout << "floor   " << tpn_pos.floor << std::endl;
    pos.timestamp = ( int64_t )( timestamp * ms_per_sec + 0.5 );

    // NWD conversion - this convertion is provided by ÑoordinateÑonverter
    geo2local.Geo2Local( tpn_pos.lattitude, tpn_pos.longitude, &x, &y );
    pos.x = x;
    pos.y = y;

    // convert real floor number into locical floor number
    floor_converter.SetCurrentRealFloor(tpn_pos.floor);

    floor_converter.GetCurrentLogicalFloor(&pos.floor_number);
    //std::cerr << tpn_pos.floor << "    " << pos.floor_number << std::endl;

    pos.floor_std = 0.00; // this data is not available in TPN/IRL data
    pos.altitude = tpn_pos.altitude;
    pos.altitude_std = tpn_pos.sigma_altitude;

    // unscertainty calculation
    double cov_xy_ned[2][2];
    cov_xy_ned[0][0] = tpn_pos.sigma_north * tpn_pos.sigma_north;
    cov_xy_ned[0][1] = cov_xy_ned[1][0] = 0.0;
    cov_xy_ned[1][1] = tpn_pos.sigma_east * tpn_pos.sigma_east;
    ned2local.Geo2Local_cov_matrix( cov_xy_ned, pos.covariance_xy );

    // flags
    pos.fidgeting_flag = tpn_pos.fidgeting_flag;
    pos.navigation_phase = tpn_pos.navigation_phase;
	pos.mode_of_transit = tpn_pos.mode_of_transit;

    // position validity
    pos.is_valid = tpn_pos.is_valid;

    if ( ( this->dbg_rtfppl_stream ) && ( !this->dbg_rtfppl_stream->bad() ) )
        OutPositionData( pos );

    return pos;
}

bool TpnConverter::SetFloorsParams(
    uint16_t floors_count,        /**< total floor number in venue */
    double   floor_height,        /**< floor height [m] */
    int16_t  floors_shift,        /**< floor shift*/
    bool     floor_zero_enable
    )
{
    return floor_converter.SetFloorsParams(
        floors_count,        /**< total floor number in venue */
        floor_height,        /**< floor height [m] */
        floors_shift,        /**< floor shift*/
        floor_zero_enable
        );
}

bool TpnConverter::GetFloorsParams(
    uint16_t *floors_count,      /**< total floor number in venue */
    double   *floor_height,        /**< floor height [m] */
    int16_t  *floors_shift,        /**< floor shift*/
    bool     *zero_floor_enable
    ) const
{
    return floor_converter.GetFloorsParams(
        floors_count,      /**< total floor number in venue */
        floor_height,        /**< floor height [m] */
        floors_shift,        /**< floor shift*/
        zero_floor_enable
        );
}

bool TpnConverter::SetCurrentLogicalFloor(int16_t logical_floor)
{
    return floor_converter.SetCurrentLogicalFloor(logical_floor);
}

bool TpnConverter::SetCurrentRealFloor(int16_t real_floor)
{
    return floor_converter.SetCurrentRealFloor(real_floor);
}

bool TpnConverter::GetCurrentLogicalFloor(int16_t *logical_floor) const
{
    return floor_converter.GetCurrentLogicalFloor(logical_floor);
}

bool TpnConverter::GetCurrentRealFloor(int16_t *real_floor) const
{
    return floor_converter.GetCurrentRealFloor(real_floor);
}


void TpnConverter::SetTpnConverterOutStream( std::ostream &os )
{
    dbg_rtfppl_stream = &os;
}

void TpnConverter::OutTpnConverterData( FfPosition &pos, MffAttitude &att, MagneticMeasurement &mag_meas )
{
    if ( ( this->dbg_rtfppl_stream ) && ( !this->dbg_rtfppl_stream->bad() ) )
    {
        OutPositionData( pos );
        OutAttitudeData( att );
        OutMagneticData( mag_meas );
    }
}

std::ostream ** TpnConverter::getLogDescriptor()
{
    return &dbg_rtfppl_stream;
}


void TpnConverter::OutPositionData( const FfPosition &pos )
{
    *dbg_rtfppl_stream << '\n';

    // position
    dbg_rtfppl_stream->precision( 0 );
    *dbg_rtfppl_stream << std::fixed << pos.timestamp;

    dbg_rtfppl_stream->precision( 3 );
    *dbg_rtfppl_stream << std::fixed;
    *dbg_rtfppl_stream << "  , " << pos.x << " , " << pos.y;
    *dbg_rtfppl_stream << std::scientific;
    *dbg_rtfppl_stream << "  , " << pos.covariance_xy[0][0];
    *dbg_rtfppl_stream << " , " << pos.covariance_xy[0][1];
    *dbg_rtfppl_stream << " , " << pos.covariance_xy[1][0];
    *dbg_rtfppl_stream << " , " << pos.covariance_xy[1][1];

    dbg_rtfppl_stream->precision( 3 );
    *dbg_rtfppl_stream << std::fixed;
    *dbg_rtfppl_stream << "  , " << pos.floor_number << " , " << pos.floor_std;
    *dbg_rtfppl_stream << "  , " << pos.altitude << " , " << pos.altitude_std;
    *dbg_rtfppl_stream << "  , " << ( int )pos.navigation_phase;
    *dbg_rtfppl_stream << "  , " << ( int )pos.fidgeting_flag;
	*dbg_rtfppl_stream << "  , " << (int)pos.mode_of_transit;
    *dbg_rtfppl_stream << "  , " << ( int )pos.is_valid;
}

void TpnConverter::OutAttitudeData( const MffAttitude &att )
{
    dbg_rtfppl_stream->precision( 0 );
    *dbg_rtfppl_stream << "    , ";
    *dbg_rtfppl_stream << std::fixed << att.timestamp;

    *dbg_rtfppl_stream << std::fixed;
    dbg_rtfppl_stream->precision( 12 );
    *dbg_rtfppl_stream << "  ";

    for ( int i = 0; i < 4; i++ )  *dbg_rtfppl_stream << " , " << att.quaternion[i]; // quaternion

    dbg_rtfppl_stream->precision( 3 );
    *dbg_rtfppl_stream << std::fixed;
    *dbg_rtfppl_stream << "  , " << att.roll_std;
    *dbg_rtfppl_stream << "  , " << att.pitch_std;
    *dbg_rtfppl_stream << "  , " << att.heading_std;
    *dbg_rtfppl_stream << "  , " << att.is_valid;
}

void TpnConverter::OutMagneticData( const MagneticMeasurement &mag_meas )
{
    // magnetic
    dbg_rtfppl_stream->precision( 0 );
    *dbg_rtfppl_stream << "    , ";
    *dbg_rtfppl_stream << std::fixed << mag_meas.timestamp;
    dbg_rtfppl_stream->precision( 3 );
    *dbg_rtfppl_stream << "   , " << mag_meas.mX << " , " << mag_meas.mY << " , " << mag_meas.mZ;
    *dbg_rtfppl_stream << "   , " << mag_meas.sigma_mX << " , " << mag_meas.sigma_mY << " , " << mag_meas.sigma_mZ;
    *dbg_rtfppl_stream << "  ";

    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 3; j++ )
            *dbg_rtfppl_stream << " , " <<  mag_meas.covarianceMatrix[i][j];
}