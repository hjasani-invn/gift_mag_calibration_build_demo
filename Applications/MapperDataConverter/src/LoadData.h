/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  Load sensor data, definition names of data files
 *   @file                   LoadData.h
 *   @author                 M. Zhokhova, D.Churikov
 *   @date                   22.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#ifndef _LOAD_DATA_H_
#define _LOAD_DATA_H_

#include "pdr_rt/Init.h"
#include "Model_Main2.h"
#include <stdio.h>
#include <string>

//==============================================================================
// Load Data configuration
//==============================================================================

#define NEW_DATA_FORMAT   1

#if INPUT_DATA_TYPE == 1
    #define REAL_TIME_EMULATION   1
#else
    #define REAL_TIME_EMULATION   0
#endif
/*
#if NEW_DATA_FORMAT
  #define REAL_TIME_EMULATION   1  
#else
  #define RAW_DATA_SUPPORT      0
  #define REAL_TIME_EMULATION   1
#endif
*/
//==============================================================================
// Specific constants
//==============================================================================
#define TIME_SCALE (1.e3)
#define MAX_LENGTH_STR 1000
#define MIN_STEP_DATA (int64_t)(MIN_TIME_DATA*TIME_SCALE)
#define IDX_ACC       (0)
#define IDX_GYRO      (1)
#define IDX_MAGN      (2)
#define IDX_MAG_BIAS  (3)

typedef struct tRawDataFile
{
  double data[3][RAW_FLT_DATA];
  double data_bias[3][RAW_FLT_DATA];
  int64_t ts[RAW_FLT_DATA];
  int16_t cntLine;
}
tRawDataFile;

//==============================================================================
// Specific types definition
//==============================================================================
typedef struct tLoadData
{
  int64_t timeEnd;
  FILE* fSens[4];
  char buff[4][MAX_LENGTH_STR];
  int32_t LastBlockID[4];
  tRawDataFile accData;  // acceleration from the accelerometers
  tRawDataFile gyrData;  // angular velocity (w) measured by the gyroscopes
  tRawDataFile magnData; // magnetic field strength from magnetic compass
  //tRawDataFile magnBias; // magnetic sensor bias
}
tLoadData;

//=========================================================================
/// Definition names of data files
///
/// @param[in]  path      - pointer to a file folder
/// @param[out] loadData  - to data structure
/// @return Zero if successful
//=========================================================================
extern int8_t GetDataFileNames(const char* in_path, tLoadData* loadData);

extern bool FindFile(std::string dir, std::string mask, std::string sensor_name, std::string &file_name);

//=========================================================================
/// Load data from files
///
/// @param[in/out] LoadData - poiner to data for loading data from file
/// @return Zero if successful
//=========================================================================
extern int8_t LoadDataProcess(tLoadData* LoadData);

#endif // _LOAD_DATA_H_