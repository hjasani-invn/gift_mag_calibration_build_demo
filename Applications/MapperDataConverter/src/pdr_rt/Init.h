/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  data type for User parameters
 *   @file                   Init.h
 *   @author                 M. Zhokhova
 *   @date                   09.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#ifndef _INIT_H_
#define _INIT_H_

#include "config.h"
#include "matrix.h"

//==========================================================
/// Enumeration type of attitude state
//==========================================================
typedef enum
{
  eAS_Uncknown = 0,
  eAS_Rough = 1,
  eAS_Fine = 2
} eAttitudeState;

//==========================================================
/// Enumeration type of phone2body matrix state
//==========================================================
typedef enum
{
  I2P_Unknown = 0,
  I2P_yplus = 1,
  I2P_xplus = 2,
	I2P_yminus = 3,
	I2P_xminus = 4
} eI2PState;

//==========================================================
/// Attitude parameters
//==========================================================
typedef struct tStepAttitude
{
  Matrix Cp2b; // Phone frame to User body frame transition matrix
  //Matrix Cp2b0; // Phone frame to User body frame transition matrix
  Matrix Cp2b1; // Phone frame to User body frame transition matrix
  Matrix Cb2n; // User body to NED frame transition matrix
  double fi; // roll, rad
  double teta; // pitch, rad
  double psi; // heading, rad
  int16_t index; // sample index of current attitude data associating in sensor measuring arrays 
  double time; // time of current attitude data associating in sensor measuring arrays 
  double headingInc; // heading increment
  double acc_scale_factor;
  double acc_mean_g;

  eAttitudeState fAttitudeState;
}
tStepAttitude;

//==========================================================
/// Sensor parameters
//==========================================================
typedef struct tSensorParams
{
  int8_t IsUpdate;
  double acc_scale_factor;
  double acc_mean_g;
}
tSensorParams;

//==========================================================
/// Enumeration type for user male
//==========================================================
typedef enum
{
  es_Female = 0,    ///< Female
  es_Male = 1       ///< Male
//es_Child = 2
} eSex;


typedef struct tUserParams
{
  int16_t height; // growth of user, cm
  eSex sex;   // sex of user
  double K;   // coef.
  int8_t AttUpdateFlag; // enable/disable transition matrix update during step
  tStepAttitude Att;  // attitude of user
  //eAttitudeState IsAttitudeInit; // flag of initial attitude of user validity
  tSensorParams SensorParams; // sensor parameters
	Matrix Ci2p; // Instrumental frame to Phone frame transition matrix
	eI2PState fI2PState; // state of I2P orientation
	double HeadingCompensation;
}
tUserParams;

typedef struct tRawData
{
  int64_t st[RAW_FLT_DATA];
  double t[RAW_FLT_DATA];
  double data[3][RAW_FLT_DATA];
  int16_t cntData;
}
tRawData;

// filtered data
typedef struct tFltData
{
  double t[RAW_FLT_DATA];
  double data[3][RAW_FLT_DATA]; // data filtering
  int16_t cntData;
}
tFltData;

// gyro vectors type
typedef struct tVectGyr
{
	double data[3];
}
tVectGyr;

//=================================================================================
/// Initialization User parameters
///
/// @param[in]  userHeight - growth of user [cm]
/// @param[in]  userSex    - sex of user 
/// @param[out] UserParams - pointer to user parameters structure
//=================================================================================
extern void InitUserParams(int16_t userHeight, eSex userSex, tUserParams* UserParams);

#endif // _INIT_H_