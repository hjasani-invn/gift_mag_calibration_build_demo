#include "TpnData.hpp"
#include "CoordinateConverter.h"

#include <fstream>
#include <stdint.h>

//=======================================================================
/// The function outs TPN data to specified stream
///
/// @param[in]  os - stream to output data
/// @param[in]  tpn_data - tpn data structure
//=======================================================================
void SaveTpnData(std::ofstream &os, const TpnOutput &tpn_data);

//=======================================================================
/// The function calculates posiiton, orientation quaternion and magnetic data into TPN format
///
/// @param[out]  tpn_data - tpn data structure
/// @param[out]  geo2local - coordinate tonverter object, geo<->FF conversion
/// @param[in]  sys_t - Android system time, ms
/// @param[in]  x - x-coordinate of position in filter frame, m
/// @param[in]  y - y-coordinate of position in filter frame, m
/// @param[in]  floor - current floor number
/// @param[in]  q - orientation quaternion, UDF to MFP frame
/// @param[in]  mag_vector - magnetic vector in ???(UDF), m
/// @param[in]  p_event - probability metric of step event
/// @param[in]  step_length - step length, m
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
    bool   is_valid
    );


