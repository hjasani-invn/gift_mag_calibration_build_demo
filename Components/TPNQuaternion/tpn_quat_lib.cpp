/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @project     RTFPPL
* @file        tpn_quat_lib.cpp
* @author      D Churikov
* @date        8 Apr 2016
* @brief       Quaternion to Eyler angles transformation functions
*              Source code provided by ICA
*/
/*****************************************************************************/

#include <math.h>
#include "tpn_quat_lib.hpp"

/**
* @brief           Function to calculate direction cosines matrix from attitude angle
* @details
* @param[in] att: attitude vector of type float including roll, pitch and heading in radians
* @param[out]      Cbn: A 3x3 direction cosines matrix of type float calculated by using the attitude vector
* @return          NONE
*/
void euler_angles_to_dcm( FLOAT32 att[3], FLOAT32 Cbn[3][3] )
{
    FLOAT32 cr, cp, ch, sr, sp, sh;

    cr = cos( att[0] );   cp = cos( att[1] );   ch = cos( att[2] );
    sr = sin( att[0] );   sp = sin( att[1] );   sh = sin( att[2] );

    Cbn[0][0] = cp * ch;
    Cbn[0][1] = -cr * sh + sr * sp * ch;
    Cbn[0][2] = sr * sh + cr * sp * ch;

    Cbn[1][0] = cp * sh;
    Cbn[1][1] = cr * ch + sr * sp * sh;
    Cbn[1][2] = -sr * ch + cr * sp * sh;

    Cbn[2][0] = -sp;
    Cbn[2][1] = sr * cp;
    Cbn[2][2] = cr * cp;

}

/**
* @brief           Function to calculate quaternions from direction cosines matrix
* @details
* @param[in] C: A 3x3 direction cosines matrix of type float containing attitude information
* @param[out]      q: quaternion vector related to C in float
* @return          NONE
*/
void dcm_to_quaternion( FLOAT32 C[3][3], FLOAT32 q[4] )
{
    FLOAT32 Tr = 0, Pq[4], a;
    UINT8 i, max_id;

    Tr = C[0][0] + C[1][1] + C[2][2];

    Pq[0] = 1 + Tr;
    Pq[1] = 1 + 2.0f * C[0][0] - Tr;
    Pq[2] = 1 + 2.0f * C[1][1] - Tr;
    Pq[3] = 1 + 2.0f * C[2][2] - Tr;

    max_id = 0;

    for ( i = 0; i < 4; i++ )
    {
        if ( Pq[i] > Pq[max_id] )    max_id = i;
    }

    switch ( max_id )
    {
        case 0:
            q[0] = 0.5f * sqrt( Pq[0] );
            a = 0.25f / q[0];
            q[1] = ( C[2][1] - C[1][2] ) * a;
            q[2] = ( C[0][2] - C[2][0] ) * a;
            q[3] = ( C[1][0] - C[0][1] ) * a;
            break;

        case 1:
            q[1] = 0.5f * sqrt( Pq[1] );
            a = 0.25f / q[1];
            q[0] = ( C[2][1] - C[1][2] ) * a;
            q[2] = ( C[1][0] + C[0][1] ) * a;
            q[3] = ( C[0][2] + C[2][0] ) * a;
            break;

        case 2:
            q[2] = 0.5f * sqrt( Pq[2] );
            a = 0.25f / q[2];
            q[0] = ( C[0][2] - C[2][0] ) * a;
            q[1] = ( C[1][0] + C[0][1] ) * a;
            q[3] = ( C[2][1] + C[1][2] ) * a;
            break;

        case 3:
            q[3] = 0.5f * sqrt( Pq[3] );
            a = 0.25f / q[3];
            q[0] = ( C[1][0] - C[0][1] ) * a;
            q[1] = ( C[0][2] + C[2][0] ) * a;
            q[2] = ( C[2][1] + C[1][2] ) * a;
            break;
    }

    if ( q[0] < 0 )
    {
        for ( i = 0; i < 4; i++ ) q[i] = -q[i];
    }
}

/**
* @brief           Function to calculate direction cosines matrix from quaternions
* @details
* @param[in] quar: quaternion vector in float
* @param[out]      C: A 3x3 direction cosines matrix of type float
* @return          NONE
*/
void quaternion_to_dcm( FLOAT32 quar[4], FLOAT32 C[3][3] )
{
    C[0][0] = quar[0] * quar[0] + quar[1] * quar[1] - quar[2] * quar[2] - quar[3] * quar[3];
    C[0][1] = 2.0f * ( quar[1] * quar[2] - quar[0] * quar[3] );
    C[0][2] = 2.0f * ( quar[1] * quar[3] + quar[0] * quar[2] );

    C[1][0] = 2.0f * ( quar[1] * quar[2] + quar[0] * quar[3] );
    C[1][1] = quar[0] * quar[0] - quar[1] * quar[1] + quar[2] * quar[2] - quar[3] * quar[3];
    C[1][2] = 2.0f * ( quar[2] * quar[3] - quar[0] * quar[1] );

    C[2][0] = 2.0f * ( quar[1] * quar[3] - quar[0] * quar[2] );
    C[2][1] = 2.0f * ( quar[2] * quar[3] + quar[0] * quar[1] );
    C[2][2] = quar[0] * quar[0] - quar[1] * quar[1] - quar[2] * quar[2] + quar[3] * quar[3];
}

/**
* @brief           Function to calculate attitude angles from direction cosines matrix
* @details
* @param[in] Cbn: A 3x3 direction cosines matrix of type float
* @param[out]      attitude: attitude vector of type float including roll, pitch and heading in radians calculated from the DCM
* @return          NONE
*/
void dcm_to_euler_angles( FLOAT32 Cbn[3][3], FLOAT32 attitude[3] )
{
    // between +/-pi
    attitude[0] = atan2( Cbn[2][1], Cbn[2][2] );
    // between +/-pi/2
    attitude[1] = atan( -Cbn[2][0] / sqrt( SQR( Cbn[2][1] ) + SQR( Cbn[2][2] ) ) );
    // between +/-pi
    attitude[2] = atan2( Cbn[1][0], Cbn[0][0] );
}
