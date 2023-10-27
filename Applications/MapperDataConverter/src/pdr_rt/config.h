/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  project configuration
 *   @file                   config.h
 *   @author                 M. Zhokhova
 *   @date                   09.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define PDR_DEBUG  //define for extended debug data from PDR
#define PDR_ACCERT_DEBUG   1

#if 1
// TODO remove this block!
// config.h must contain only necessary defines!

#ifdef _MSC_VER
 #if  _MSC_VER < 1600
  #include "stdint.h"
 #else 
  #include <stdint.h>
 #endif
#else
 #include <stdint.h>
#endif

#ifndef Max
 #define Max(x,y) (((x)>=(y))?(x):(y))
#endif

#ifndef Min
 #define Min(x,y) (((x)<=(y))?(x):(y))
#endif

#endif

#define MAX_SAMPLE_RATE (200) // Hz, maximum sample rate
#define SYNC_SMP_RATE   (50)  // Hz, desired sampling rate of all data after synchronization
#define MAX_STEPS       (10)  // maximum steps/stands
#define MAX_EXTREM      (120) // the maximum number of extremes
#define MAX_TIME_SAMPLE_RATE (3.0) // maximum amount of time to determine the sample rate of data from sensors
#define MAX_DATA_GAP_DURATION (0.5) // sec,  maximum data gap duration
#define RAW_FLT_DATA         ((int16_t)(MAX_SAMPLE_RATE*MAX_TIME_SAMPLE_RATE) + 40) // buffer for 1 sec 200Hz
#define MIN_TIME_DATA        (0.1) // [ms] 0.2 = 0.18s min step duration + 1/50Hz
#define MAX_LENGTH_DATA      ((int16_t)(MAX_SAMPLE_RATE*MIN_TIME_DATA) + 10) // MIN_TIME_DATA for 200Hz + delta

//===================================================================================
// Enumeration type oà sensor type
enum eSensorType
{
  est_Accelerometer,
  est_Gyro,
  est_Magnetometer
};


#endif // _CONFIG_H_

