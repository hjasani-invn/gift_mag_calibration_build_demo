/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @project     RTFPPL
* @file        tpn_quat_lib.hpp
* @author      D Churikov
* @date        8 Apr 2016
* @brief       Interface of quaternion to Eyler angles transformation functions
*
*/
/*****************************************************************************/
#ifndef TPN_QUAT_LIB_HPP
#define TPN_QUAT_LIB_HPP
#include <stdint.h>

// TPN types support
typedef uint8_t UINT8;  ///< 8-byte unsigned int type
typedef double FLOAT32;  ///< 32-byte float type

#define SQR(a)  ((a)*(a))   ///< Square macro

/**
* @brief           Function to calculate direction cosines matrix from attitude angle
* @details
* @param[in] att: attitude vector of type float including roll, pitch and heading in radians
* @param[out]      Cbn: A 3x3 direction cosines matrix of type float calculated by using the attitude vector
* @return          NONE
*/
void euler_angles_to_dcm( FLOAT32 att[3], FLOAT32 Cbn[3][3] );


/**
* @brief           Function to calculate quaternions from direction cosines matrix
* @details
* @param[in] C: A 3x3 direction cosines matrix of type float containing attitude information
* @param[out]      q: quaternion vector related to C in float
* @return          NONE
*/
void dcm_to_quaternion( FLOAT32 C[3][3], FLOAT32 q[4] );


/**
* @brief           Function to calculate direction cosines matrix from quaternions
* @details
* @param[in] quar: quaternion vector in float
* @param[out]      C: A 3x3 direction cosines matrix of type float
* @return          NONE
*/
void quaternion_to_dcm( FLOAT32 quar[4], FLOAT32 C[3][3] );

/**
* @brief           Function to calculate attitude angles from direction cosines matrix
* @details
* @param[in] Cbn: A 3x3 direction cosines matrix of type float
* @param[out]      attitude: attitude vector of type float including roll, pitch and heading in radians calculated from the DCM
* @return          NONE
*/
void dcm_to_euler_angles( FLOAT32 Cbn[3][3], FLOAT32 attitude[3] );
#endif