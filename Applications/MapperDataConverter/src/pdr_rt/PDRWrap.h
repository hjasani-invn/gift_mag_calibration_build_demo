/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  Wraping module for android
 *   @file                   PDRWrap.h
 *   @author                 M. Zhokhova
 *   @date                   26.08.2013
 *   @version                1.0
 */
/*****************************************************************************/

#ifndef _PDR_WRAP_H_
#define _PDR_WRAP_H_

#include "config.h"
#include "PDR_data.h"
/* Guard C code in headers, while including them from C++ */
#ifdef  __cplusplus
#define FPPE_BEGIN_DECLS extern "C" {
#define FPPE_END_DECLS    }
#else
#define FPPE_BEGIN_DECLS
#define FPPE_END_DECLS
#endif


FPPE_BEGIN_DECLS

//===================================================================================
/// Initialization PDR module
//===================================================================================
void aw_InitPDR( const char *file_name_marker);

//===================================================================================
/// Set new parameters of user
///
/// @param[in]  height  - user height
/// @param[in]  userSex - user sex
/// @param[out] errFlag - 0 if user parameters are correct; user parameters applied
///                      -1 if user sex is incorrect; user parameters not applied
///                      -2 if user height is incorrect; user parameters not applied
//===================================================================================
void aw_InitUserParams(int16_t height, int8_t sex, int8_t* errFlag);

//===================================================================================
/// Get step length parameter of user
//===================================================================================
double getUserParamK();

//===================================================================================
/// Set step length parameter of user
//===================================================================================
int setUserParamK(double k);

//===================================================================================
/// Set new data from the sensors and process them: determinates user steps 
/// and theirs parameters. Current sensor data are resetted after processing.
///
/// @param[in]  aAcc          - accelerometer data array
/// @param[in]  tsAcc         - system time stamps of accelerometer data
/// @param[in]  AccLineCount  - number of data lines in accelerometer data array
/// @param[in]  aGyro         - gyro data array
/// @param[in]  tsGyro        - system time stamps of gyro data
/// @param[in]  GyroLineCount - number of data lines in gyro data array
/// @param[in]  aMag          - magnetic compas data array
/// @param[in]  tsMag         - system time stamps of magnetic data
/// @param[in]  MagLineCount  - number of data lines in magnetic compas data array
/// @param[out] cntSteps      - pointer to counter of detected steps
/// @param[out] errFlag       - 0 new sensor data were set
///                            -1 if accelerometer data are incorrect; 
///                            -2 if gyro data are incorrect; 
///                            -3 if magnetic data are incorrect; 
//===================================================================================
void aw_ProcessPDR(double *aAcc[3], int64_t *tsAcc, int16_t AccLineCount,
                   double *aGyro[3], int64_t *tsGyro, int16_t GyroLineCount,
                   double *aMag[3], int64_t *tsMag, int16_t MagLineCount, 
                   double aMagBias[3],
                   int8_t* cntSteps, int8_t* errFlag);

//===================================================================================
/// Get step count of current PDR results
///
/// @param[out] StepCount - pointer to buffer for requested value of step count
//===================================================================================
void aw_GetStepCount(int8_t *StepCount);

//===================================================================================
/// The function scroll Steps array on specified step count (delete steps from arry)
///
/// @param[out] StepCount -  step count
//===================================================================================
void aw_ScrollSteps(int8_t StepCount);

//===================================================================================
/// Get array of step lengths from current PDR results
///
/// @param[out] StepLengths - pointer to buffer for requested array
/// @param[in]  N           - maximal number of elements in array
/// @param[out] StepCount   - pointer to buffer for current step count value
//===================================================================================
void aw_GetStepLengths(double *StepLengths, int8_t N, int8_t *StepCount);

//====================================================================================
/// Get array of step direction increments from current PDR results
///
/// @param[out] StepHeadingIncrements - pointer to buffer for requested array
/// @param[in]  N                     - maximal number of elements in array
/// @param[out] StepCount             - pointer to buffer for current step count value
//====================================================================================
void aw_GetStepHeadingIncrements(double *StepHeadingIncrements, 
                                 int8_t N, int8_t *StepCount);
//=========================================================================
/// Get array of step start timestamps from current PDR results
///
/// @param[out] StepStartTimeStamps - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//=========================================================================
void aw_GetStepStartTimeStamps(int64_t *StepStartTimeStamps, int8_t N, int8_t *StepCount);


//===================================================================================
/// Get array of step end timestamps from current PDR results
///
/// @param[out] StepEndTimeStamps - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetStepEndTimeStamps(int64_t *StepEndTimeStamps, int8_t N, int8_t *StepCount);

//===================================================================================
/// Get array of average magnetic magnitude on stap from current PDR results
///
/// @param[out] AverageMagneticAtStep - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetAverageMagneticOnStep(double *AverageMagneticAtStep, int8_t N, int8_t *StepCount);

//===================================================================================
/// Get array of  magnetic magnitude at step start from current PDR results
///
/// @param[out] MagneticAtStepStart - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetMagneticAtStepStart(double *MagneticAtStepStart, int8_t N, int8_t *StepCount);

//===================================================================================
/// Get array of  magnetic magnitude at step end from current PDR results
///
/// @param[out] MagneticAtStepEnd - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetMagneticAtStepEnd(double *MagneticAtStepEnd, int8_t N, int8_t *StepCount);


//====================================================================================
/// Get array of step direction increments from current PDR results
///
/// @param[out] StepMagneticHeading   - pointer to buffer for requested array
/// @param[in]  N                     - maximal number of elements in array
/// @param[out] StepCount             - pointer to buffer for current step count value
//====================================================================================
void aw_GetStepMagneticHeading(double *StepMagneticHeading, 
                                 int8_t N, int8_t *StepCount);

//===================================================================================
/// Get array of (a_max-a_min) at step from current PDR results
///
/// @param[out] step_DeltaAcc - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
void aw_GetAccelerationDeltas(double *step_DeltaAcc, int8_t N, int8_t *StepCount);

//===================================================================================
/// Get array of pEvent at step from current PDR results
///
/// @param[out] step_PEvent - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetPEvents( float *step_PEvent, int8_t N, int8_t *StepCount);

//===================================================================================
/// Get array of tPFData structures
///
/// @param[out] step_PEvent - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetPFData( tPFData *pPFData, int8_t N, int8_t *StepCount);
//===================================================================================
/// Get array of tPFData structures for virtual steps
///
/// @param[out] step_PEvent - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetVirtStepData( tPFData *pPFData, int8_t N, int8_t *StepCount );
//===================================================================================
/// The function closes all reports
///
void aw_CloseReports();

FPPE_END_DECLS
#endif // _PDR_WRAP_H_
