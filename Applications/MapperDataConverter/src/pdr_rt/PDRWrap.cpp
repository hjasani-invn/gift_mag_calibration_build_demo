/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  Wraping module for android
 *   @file                   PDRWrap.cpp
 *   @author                 M. Zhokhova
 *   @date                   26.08.2013
 *   @version                1.0
 */
/*****************************************************************************/

#include <stdio.h>
#include <memory.h>
#include <string.h>
#include "PDRWrap.h"
#include "PDR.h"
#include "DR.h"
#include "Report.h"
#include <math.h>
#include <assert.h>
//#include "model_config.h"

#ifdef __ANDROID__
#   include <android/log.h>
#	ifdef LOGI
#		undef LOGI
#	endif
#   define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "FusionFilter", __VA_ARGS__))
#else
#   define LOGI(...)
#endif

#define USER_PARAMS_K_MIN   (0.2)
#define USER_PARAMS_K_MAX   (1.0)

//=========================================================================
/// Constants
//=========================================================================
const double sys_tyme_scale = 1.e3; /// scale of system tyme stamp

//=========================================================================
/// Static data
//=========================================================================
static tData Data;                          // sensor data structure
static tStepsInfo Steps;                    // StepInfo structure - contains detected steps
static tStepsInfo VirtualSteps;				// StepInfo structure - contains unconfirmed steps for extrapolation
static tPDR PDR;                            // PDR structure
static tUserParams UserParams;              // Structure of User Parameters
static int8_t fIsUserParamsInit = 0;        // initialization flag of User Parameters Structure
static int16_t default_height = 175;        // default user height
static eSex default_userSex = es_Male;      // default user sex
static int32_t CallCounter = 0;
static tDREKF DREKF(SYNC_SMP_RATE);
//static int callCounter = 0;

//===================================================================================
/// The function closes all reports
///
void aw_CloseReports()
{
  ReportPDR::PDRReport(&PDR, &Data, &UserParams, &Steps, 1);
  ReportPDR::DestroyReports();
  CallCounter = 0;
}

//===================================================================================
/// Set new parameters of user
///
/// @param[in]  height  - user height
/// @param[in]  userSex - user sex
/// @param[out] errFlag - 0 if user parameters are correct; user parameters applied
///                      -1 if user sex is incorrect; user parameters not applied
///                      -2 if user height is incorrect; user parameters not applied
//===================================================================================
void aw_InitUserParams(int16_t height, int8_t sex, int8_t* errFlag)
{
  eSex eUserSex;

  if (sex == 0)
    eUserSex = es_Female;
  else if (sex == 1)
    eUserSex = es_Male;
  else
  {
    *errFlag = -2;
    return;
  }
  if ((height < 100) || (height > 260))
  {
    *errFlag = -1;
    return;
  }
  InitUserParams(height, eUserSex, &UserParams);
  fIsUserParamsInit = 1;
  *errFlag = 0;

  //callCounter = 0;
  return;
}

//===================================================================================
/// Get step length parameter of user
//===================================================================================
double getUserParamK()
{
  return UserParams.K;
}

//===================================================================================
/// Set step length parameter of user
//===================================================================================
int setUserParamK(double k)
{
  if ((k >= USER_PARAMS_K_MIN) && (k <= USER_PARAMS_K_MAX))
  {
    UserParams.K = k;
    return 1;
  }
  else
    return 0;
}

//===================================================================================
/// Initialization PDR module
//===================================================================================
void aw_InitPDR( const char *file_name_marker)
{
//  !TODO
//  aw_CloseReports(); // exclude for pc-model //Possibly bug here
  ReportPDR::InitReportNames( file_name_marker);
  
  // check User parameters initialization
  if( fIsUserParamsInit == 0 )
  { // set defaul parameters
    memset(&UserParams, 0, sizeof(tUserParams));
    InitUserParams(default_height, default_userSex, &UserParams);
  }
  // initialization of data structure
  memset(&Data, 0, sizeof(tData));
  // initialization of steps info structure
  memset(&Steps, 0, sizeof(tStepsInfo)); 
  memset(&PDR, 0, sizeof(tPDR));
  PDR.pDREKF = &DREKF; // obtain DREKF instance
}

//===================================================================================
int8_t CheckInputData(double *aAcc[3], int64_t *tsAcc, int16_t AccLineCount,
                      double *aGyro[3], int64_t *tsGyro, int16_t GyroLineCount,
                      double *aMag[3], int64_t *tsMag, int16_t MagLineCount,
                      double aMagBias[3])
{
  int8_t errFlag = 0;

  // check data size
  if ((AccLineCount <= 0))
    ReportPDR::LogReport( "Warning: no data from acc at %d call\n", CallCounter);
  if ((GyroLineCount <= 0))
    ReportPDR::LogReport( "Warning: no data from gyro at %d call\n", CallCounter);
  if ((MagLineCount <= 0))
    ReportPDR::LogReport( "Warning: no data from magnetic at %d call\n", CallCounter);

  // check data size
  if (AccLineCount > MAX_LENGTH_DATA)
  {
    errFlag = -1;
    ReportPDR::LogReport( "ERROR: incorrect input data size (%d) from acc at %d call\n", AccLineCount, CallCounter);
    //assert(0);
  }
  else if ((GyroLineCount > MAX_LENGTH_DATA))
  {
    errFlag = -1;
    ReportPDR::LogReport( "ERROR: incorrect input data size (%d) from gyro at %d call\n", GyroLineCount, CallCounter);
    //assert(0);
  }
  else if ((MagLineCount > MAX_LENGTH_DATA))
  {
    errFlag = -1;
    ReportPDR::LogReport( "ERROR: incorrect input data size (%d) from magnetic at %d call\n", MagLineCount, CallCounter);
    //assert(0);
  }
  // check data pointers
  else if ((aAcc == 0)  || (aAcc[0] == 0)  || (aAcc[1] == 0)  || (aAcc[2] == 0)  ||
           (aGyro == 0) || (aGyro[0] == 0) || (aGyro[1] == 0) || (aGyro[2] == 0) ||
           (aMag== 0)   || (aMag[0] == 0)  || (aMag[1] == 0)  || (aMag[2] == 0)  ||
           (tsAcc== 0)  || (tsGyro == 0)   || (tsMag == 0)    || (aMagBias == 0))
    {
    errFlag = -2;
    ReportPDR::LogReport( "ERROR: incorrect input data pointer  at %d call\n", CallCounter);
    assert(0);
  }
  return errFlag;
}

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
///                            -4 incorrect sample rate for one second
//===================================================================================
void aw_ProcessPDR(double *aAcc[3], int64_t *tsAcc, int16_t AccLineCount,
                   double *aGyro[3], int64_t *tsGyro, int16_t GyroLineCount,
                   double *aMag[3], int64_t *tsMag, int16_t MagLineCount, 
                   double aMagBias[3],
                   int8_t* cntSteps, int8_t* errFlag)
{
  int16_t i, j;
  double *pAcc[3];
  double *pGyro[3];
  double *pMag[3];

  // general operations
  Steps.cntPfSteps = 0;
  *cntSteps = 0;
  *errFlag = 0;
  CallCounter++;

  if (CallCounter == 844)
    CallCounter = CallCounter+0;
    

  //LOGI("PDR_process");

  for( i = 0; i < 3; ++i )
  {
    pAcc[i] = aAcc[i];
    pGyro[i] = aGyro[i];
    pMag[i] = aMag[i];
  }
  
  if (*errFlag = CheckInputData(aAcc, tsAcc, AccLineCount,
                                aGyro, tsGyro, GyroLineCount,
                                aMag, tsMag, MagLineCount,
                                aMagBias))
  {// input data error
    return;
  }

  j = Data.accData.rawData.cntData;
 // place input sensor data into tData structure
 for( i = 0; i < AccLineCount; i++ )
 {
    Data.accData.rawData.st[j] = tsAcc[i];
    Data.accData.rawData.t[j] = ((double)tsAcc[i])/sys_tyme_scale;
    Data.accData.rawData.data[0][j] = pAcc[0][i];
    Data.accData.rawData.data[1][j] = pAcc[1][i];
    Data.accData.rawData.data[2][j] = pAcc[2][i];
    ReportPDR::SensorLog(est_Accelerometer, CallCounter, Data.accData.rawData.t[j],
                                              Data.accData.rawData.data[0][j],
                                              Data.accData.rawData.data[1][j],
                                              Data.accData.rawData.data[2][j]);
    j++;
  }
  Data.accData.rawData.cntData += AccLineCount;
  Data.accData.bias[0] = 0;
  Data.accData.bias[1] = 0;
  Data.accData.bias[2] = 0;

  j = Data.gyrData.rawData.cntData;
  
  Data.gyrData.bias[0] = 0;
  Data.gyrData.bias[1] = 0;
  Data.gyrData.bias[2] = 0;
  
  for( i = 0; i < GyroLineCount; i++ )
  {
    Data.gyrData.rawData.st[j] = tsGyro[i];
    Data.gyrData.rawData.t[j] = ((double)tsGyro[i])/sys_tyme_scale;
    Data.gyrData.rawData.data[0][j] = pGyro[0][i];
    Data.gyrData.rawData.data[1][j] = pGyro[1][i];
    Data.gyrData.rawData.data[2][j] = pGyro[2][i];
    //ReportPDR::SensorLog(est_Gyro, CallCounter, Data.gyrData.rawData.t[j],
    //                                 Data.gyrData.rawData.data[0][j],
    //                                 Data.gyrData.rawData.data[1][j],
    //                                 Data.gyrData.rawData.data[2][j]);
    ReportPDR::SensorLogEx(est_Gyro, CallCounter, Data.gyrData.rawData.t[j],
                                     Data.gyrData.rawData.data[0][j],
                                     Data.gyrData.rawData.data[1][j],
                                     Data.gyrData.rawData.data[2][j],
                                     Data.gyrData.bias[0],
                                     Data.gyrData.bias[1],
                                     Data.gyrData.bias[2]);
    j++;
  }
  Data.gyrData.rawData.cntData += GyroLineCount;
  
  Data.magnData.bias[0] = aMagBias[0];
  Data.magnData.bias[1] = aMagBias[1];
  Data.magnData.bias[2] = aMagBias[2];

  j = Data.magnData.rawData.cntData;
  for( i = 0; i < MagLineCount; i++ )
  {
    Data.magnData.rawData.st[j] = tsMag[i];
    Data.magnData.rawData.t[j] = ((double)tsMag[i])/sys_tyme_scale;
    Data.magnData.rawData.data[0][j] = pMag[0][i];
    Data.magnData.rawData.data[1][j] = pMag[1][i];
    Data.magnData.rawData.data[2][j] = pMag[2][i];
    //ReportPDR::SensorLog(est_Magnetometer, CallCounter, Data.magnData.rawData.t[j],
    //                                         Data.magnData.rawData.data[0][j],
    //                                         Data.magnData.rawData.data[1][j],
    //                                         Data.magnData.rawData.data[2][j]);
    ReportPDR::SensorLogEx(est_Magnetometer, CallCounter, Data.magnData.rawData.t[j],
                                             Data.magnData.rawData.data[0][j],
                                             Data.magnData.rawData.data[1][j],
                                             Data.magnData.rawData.data[2][j],
                                             Data.magnData.bias[0],
                                             Data.magnData.bias[1],
                                             Data.magnData.bias[2]);
    j++;
  }
  Data.magnData.rawData.cntData += MagLineCount;
  
  //*errFlag = 0;

  // PDR processing
  if( ProcessPDR(&PDR, &Data, &UserParams, &Steps, &VirtualSteps) )
  {
    *errFlag = -4; // incorect sample rate
    ReportPDR::LogReport( "Warning: PDR error at %d call\n", CallCounter);
    return;
  }

   // report
   //reportPrintf(Steps);
  ReportPDR::StepReport(CallCounter, &Steps);
  ReportPDR::ScreenStepReport(&Steps);
  ReportPDR::PDRReport(&PDR, &Data, &UserParams, &Steps, 0);
  
  /*
  for (i=0; i < Steps.cntPfSteps; i++)
  {
    if (fabs(Steps.pfData[i].magnAvgBF[0]) < 1)
      i = i+1;
    ReportPDR::FusionFilterLog("PDRwrap %.3f %f %f %f", Steps.pfData[i].timeStart,
                                                       Steps.pfData[i].magnAvgBF[0],
                                                       Steps.pfData[i].magnAvgBF[1],
                                                       Steps.pfData[i].magnAvgBF[2]);
  }*/

  *cntSteps = Steps.cntPfSteps;
  return;
}

//===================================================================================
/// Get step count of current PDR results
///
/// @param[out] StepCount - pointer to buffer for requested value of step count
//===================================================================================
void aw_GetStepCount(int8_t *StepCount)
{
  *StepCount = Steps.cntPfSteps;
  return;
}

//===================================================================================
/// The function scroll Steps array on specified step count (delete steps from arry)
///
/// @param[out] StepCount -  step count
//===================================================================================
void aw_ScrollSteps(int8_t StepCount)
{
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable: 6385)
#endif
  int8_t i, scroll_cnt;
  scroll_cnt = (StepCount < Steps.cntPfSteps) ? StepCount : Steps.cntPfSteps;
  for (i = scroll_cnt; i < Steps.cntPfSteps; i++)
    Steps.pfData[i-scroll_cnt] = Steps.pfData[scroll_cnt];
  Steps.cntPfSteps -= scroll_cnt;
  return;
#ifdef _MSC_VER
#pragma warning (pop)
#endif
}

//===================================================================================
/// Get array of step lengths from current PDR results
///
/// @param[out] StepLengths - pointer to buffer for requested array
/// @param[in]  N           - maximal number of elements in array
/// @param[out] StepCount   - pointer to buffer for current step count value
//===================================================================================
void aw_GetStepLengths(double *StepLengths, int8_t N, int8_t *StepCount)
{
  int i;

  if( Steps.cntPfSteps == 0 )
  {
    *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
    StepLengths[i] = Steps.pfData[i].length;
  }
  return;
}

//====================================================================================
/// Get array of step direction increments from current PDR results
///
/// @param[out] StepHeadingIncrements - pointer to buffer for requested array
/// @param[in]  N                     - maximal number of elements in array
/// @param[out] StepCount             - pointer to buffer for current step count value
//====================================================================================
void aw_GetStepHeadingIncrements(double *StepHeadingIncrements, 
                                 int8_t N, int8_t *StepCount)
{
  int i;

  if( Steps.cntPfSteps == 0 )
  {
    *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
      StepHeadingIncrements[i] = Steps.pfData[i].headingInc;
  }
  return;
}

//=========================================================================
/// Get array of step start timestamps from current PDR results
///
/// @param[out] StepStartTimeStamps - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//=========================================================================
void aw_GetStepStartTimeStamps(int64_t *StepStartTimeStamps, int8_t N, int8_t *StepCount)
{
  int i;

  if (Steps.cntPfSteps == 0)
  {
    *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for (i = 0; i < N; i++)
      StepStartTimeStamps[i] = (int64_t)(Steps.pfData[i].timeStart*sys_tyme_scale);
  }
  return;
}

//===================================================================================
/// Get array of step end timestamps from current PDR results
///
/// @param[out] StepEndTimeStamps - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetStepEndTimeStamps(int64_t *StepEndTimeStamps, int8_t N, int8_t *StepCount)
{
  int i;

  if( Steps.cntPfSteps == 0 )
  {
  *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
    StepEndTimeStamps[i] = (int64_t)(Steps.pfData[i].timeEnd*sys_tyme_scale);
  }
  return;
}

//===================================================================================
/// Get array of average magnetic magnitude on stap from current PDR results
///
/// @param[out] AverageMagneticAtStep - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetAverageMagneticOnStep(double *AverageMagneticAtStep, int8_t N, int8_t *StepCount)
{
  int i;

  if( Steps.cntPfSteps == 0 )
  {
  *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
      AverageMagneticAtStep[i] = (Steps.pfData[i].magnAverage);
  }
  return;
}

//===================================================================================
/// Get array of  magnetic magnitude at step start from current PDR results
///
/// @param[out] MagneticAtStepStart - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetMagneticAtStepStart(double *MagneticAtStepStart, int8_t N, int8_t *StepCount)
{
  int i;

  if( Steps.cntPfSteps == 0 )
  {
  *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
      MagneticAtStepStart[i] = (Steps.pfData[i].magnStart);
  }
  return;
}


//===================================================================================
/// Get array of  magnetic magnitude at step end from current PDR results
///
/// @param[out] MagneticAtStepEnd - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetMagneticAtStepEnd(double *MagneticAtStepEnd, int8_t N, int8_t *StepCount)
{
  int i;

  if( Steps.cntPfSteps == 0 )
  {
  *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
      MagneticAtStepEnd[i] = (Steps.pfData[i].magnEnd);
  }
  return;
}

//====================================================================================
/// Get array of step direction increments from current PDR results
///
/// @param[out] StepMagneticHeading   - pointer to buffer for requested array
/// @param[in]  N                     - maximal number of elements in array
/// @param[out] StepCount             - pointer to buffer for current step count value
//====================================================================================
void aw_GetStepMagneticHeading(double *StepMagneticHeading, 
                                 int8_t N, int8_t *StepCount)
{
  int i;

  if( Steps.cntPfSteps == 0 )
  {
    *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
        StepMagneticHeading[i] = Steps.pfData[i].heading;
  }
  return;
}
//===================================================================================
/// Get array of (a_max-a_min) at step from current PDR results
///
/// @param[out] step_DeltaAcc - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetAccelerationDeltas(double *step_DeltaAcc, int8_t N, int8_t *StepCount)
{
	int i;

  if( Steps.cntPfSteps == 0 )
  {
  *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
      step_DeltaAcc[i] = Steps.pfData[i].acc_span;
  }
  return;
}
//===================================================================================
/// Get array of pEvent at step from current PDR results
///
/// @param[out] step_PEvent - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetPEvents( float *step_PEvent, int8_t N, int8_t *StepCount)
{
	int i;

  if( Steps.cntPfSteps == 0 )
  {
  *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
      step_PEvent[i] = Steps.pfData[i].pEvent;
  }
  return;
}

//===================================================================================
/// Get array of tPFData structures
///
/// @param[out] step_PEvent - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetPFData( tPFData *pPFData, int8_t N, int8_t *StepCount)
{
  int i;

  if( Steps.cntPfSteps == 0 )
  {
    *StepCount = 0;
  }
  else
  {
    *StepCount = Steps.cntPfSteps;
    N = (N <= Steps.cntPfSteps) ? N : Steps.cntPfSteps;
    for( i = 0; i < N; i++ )
      memcpy(&pPFData[i], &Steps.pfData[i], sizeof(tPFData));
  }
  return;
}

//===================================================================================
/// Get array of tPFData structures for virtual steps
///
/// @param[out] step_PEvent - pointer to buffer for requested array
/// @param[in] N - maximal number of elements in array
/// @param[out] StepCount - pointer to buffer for current step count value
///
//===================================================================================
void aw_GetVirtStepData( tPFData *pPFData, int8_t N, int8_t *StepCount)
{
  int i;

  if( VirtualSteps.cntPfSteps == 0 )
  {
    *StepCount = 0;
  }
  else
  {
    *StepCount = VirtualSteps.cntPfSteps;
    N = (N <= VirtualSteps.cntPfSteps) ? N : VirtualSteps.cntPfSteps;
    for( i = 0; i < N; i++ )
      memcpy(&pPFData[i], &VirtualSteps.pfData[i], sizeof(tPFData));
  }

  return;
}
