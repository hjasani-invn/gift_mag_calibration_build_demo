/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  PDR module
 *   @file                   PDR.cpp
 *   @author                 M. Zhokhova
 *   @date                   10.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#include <math.h>
#include <memory.h>
#include <string.h>
#include <assert.h>
//#include <stdio.h>
#include "PDR.h"
#include "DR.h"


//#ifdef PDR_DEBUG
#include "Report.h"
//#endif

static void getStepLength(double K, tStepsInfo* Steps);
static void shiftFilteredData(tPDR* PDR, tData* Data);
static int getModeAndSteps(tPDR* PDR, tStepsInfo* Steps, int16_t cntTime, int8_t* idxStart, int8_t* cntSteps);
static void prepareDataForPF(tData* Data, tStepsInfo* Steps, int8_t idxStart, int8_t cntSteps);
static void shiftSynchronizedData(tPDR* PDR, tData* Data, tStepsInfo* Steps);

//=========================================================================
static void ResetPDR(tPDR* PDR, tData* Data, tStepsInfo* Steps)
{
  ReportPDR::LogReport("\nWARNING: RESET OF PDR!");
  memset(Data, 0, sizeof(tData));
  memset(Steps, 0, sizeof(tStepsInfo)); 
  
  tDREKF* pDREKF = PDR->pDREKF; // obtain DREKF instance
  memset(PDR, 0, sizeof(tPDR));
  PDR->pDREKF = pDREKF; // obtain DREKF instance
  pDREKF->ResetEKF();
  
  PDR->isStart = 0;
}

//=========================================================================
static bool CheckInternalData(tData* Data, tStepsInfo* Steps)
{
  bool is_valid = true, flag;
  // filter data checking
  flag = (Data->accData.fltData.cntData >= 0) && (Data->accData.fltData.cntData <= RAW_FLT_DATA );
  is_valid = is_valid && flag;
#if PDR_ACCERT_DEBUG
  if (!flag)    ReportPDR::LogReport("\nPDR Error: internal data corrupted (acc,%d)", Data->accData.fltData.cntData);
#endif
  flag = (Data->gyrData.fltData.cntData >= 0) && (Data->gyrData.fltData.cntData <= RAW_FLT_DATA );
  is_valid = is_valid && flag;
#if PDR_ACCERT_DEBUG
  if (!flag)    ReportPDR::LogReport("\nPDR Error: internal data corrupted (gyr,%d)", Data->gyrData.fltData.cntData);
#endif
  flag = (Data->magnData.fltData.cntData >= 0) && (Data->magnData.fltData.cntData <= RAW_FLT_DATA );
  is_valid = is_valid && flag;
#if PDR_ACCERT_DEBUG
  if (!flag)    ReportPDR::LogReport("\nPDR Error: internal data corrupted (magn,%d)", Data->magnData.fltData.cntData);
#endif
  // synchro data checking
  flag = (Data->cntTime >= 0) && (Data->cntTime <= SYNC_DATA);
  is_valid = is_valid && flag;
#if PDR_ACCERT_DEBUG
  if (!flag)    ReportPDR::LogReport("\nPDR Error: internal data corrupted (cntTime,%d)", Data->cntTime);
#endif
  // steps data checking
  flag = (Steps->cntPfSteps >= 0) && (Steps->cntPfSteps <= MAX_STEPS);
  is_valid = is_valid && flag;
#if PDR_ACCERT_DEBUG
  if (!flag)    ReportPDR::LogReport("\nPDR Error: internal data corrupted (cntPfSteps,%d)", Steps->cntPfSteps);
#endif
  flag = (Steps->cntSteps >= 0) && (Steps->cntSteps <= MAX_STEPS);
  is_valid = is_valid && flag;
#if PDR_ACCERT_DEBUG
  if (!flag)    ReportPDR::LogReport("\nPDR Error: internal data corrupted (cntSteps,%d)", Steps->cntSteps);
#endif
  // to do: add PDR structure arrays checking
  return is_valid;
}

//=========================================================================
static bool IsGapInSensorData(tSensData * pSensData)
{
  int i;
  double dt;
  bool is_data_gap = false;

  // check first sample
  if ((pSensData->rawData.cntData > 0) && (pSensData->fltData.cntData > 0))
  {
    dt = pSensData->rawData.t[0] - pSensData->fltData.t[pSensData->fltData.cntData-1];
    is_data_gap = is_data_gap || (dt > MAX_DATA_GAP_DURATION);
  }
  for (i = 1; i < pSensData->rawData.cntData; i++)
  {
    dt = pSensData->rawData.t[i] - pSensData->rawData.t[i-1];
    is_data_gap = is_data_gap || (dt > MAX_DATA_GAP_DURATION);
  }
  return is_data_gap;
}

//=========================================================================
static bool IsDataGap(tData* Data, tPDR *pPDR)
{
  bool is_data_gap = false;

  //if( pPDR->isStart == 1 )
  {
    is_data_gap = is_data_gap || IsGapInSensorData(&Data->accData);
    is_data_gap = is_data_gap || IsGapInSensorData(&Data->gyrData);
    is_data_gap = is_data_gap || IsGapInSensorData(&Data->magnData);
  }
  return is_data_gap;
}

//=========================================================================
/// PDR process
///
/// @param[in]  PDR        - pointer to instance
/// @param[in]  Data       - pointer to data
/// @param[in]  UserParams - pointer to user parameters structure
/// @param[out] Steps      - pointer to steps inforamtion
/// @return Zero if successful, 1 - incorrect sample rate for one second
//=========================================================================
int8_t ProcessPDR(tPDR* PDR,
                  tData* Data,
                  tUserParams* UserParams, 
                  tStepsInfo* Steps,
				  tStepsInfo* VirtualSteps)
{
  int8_t idxStart, cntSteps;
#ifdef PDR_DEBUG
  int8_t i;
#endif

  // check internalm data state
  if (!CheckInternalData(Data, Steps))
  {
    ReportPDR::LogReport("\nERROR: PDR DATA CORRUPTED AT PDR START (%lf)",(double)Data->accData.rawData.st[Data->accData.rawData.cntData]);
    assert(0);
    ResetPDR(PDR, Data, Steps);
  }

  // check data gap
  if (IsDataGap(Data, PDR))
  {
    ReportPDR::LogReport("\nWarning: Input data gap. PDR skiped portion of data (%lf)",(double)Data->accData.rawData.st[Data->accData.rawData.cntData]);
    ResetPDR(PDR, Data, Steps);
    return 0;
  }

  // filtering sensor data: accelerometers, gyroscopes and magnetic compass
  if( PDR->isStart == 0 )
  {
    if( InitFilteringSensorData(PDR, Data) ) // determine sample rate and initialization filters
    {
      if( (Data->accData.rawData.t[Data->accData.rawData.cntData-1] - Data->accData.rawData.t[0]) > MAX_TIME_SAMPLE_RATE )
      {
        // Shift data to one second - incorrect sample rate.
        ReportPDR::LogReport("\nWarning: Incorrect input data sample rate. Can't start PDR");
        Data->accData.rawData.cntData = 0;
        Data->gyrData.rawData.cntData = 0;
        Data->magnData.rawData.cntData = 0;
        return 1;
      }
      return 0;
    }
    Data->sampleRate = SYNC_SMP_RATE; // desired sampling rate of all data after synchronization
    PDR->isStart = 1;
    PDR->pDREKF->InitEKF(Data->sampleRate);
  }
  else
    FilteringSensorData(PDR, Data);

  // synchronization & clipping sensor data 
  if(( Data->accData.fltData.cntData > 0 ) &&
     ( Data->gyrData.fltData.cntData > 0 ) &&
     ( Data->magnData.fltData.cntData > 0 ))
    SynchronizSensorData(Data);

  shiftFilteredData(PDR, Data);

  if( Data->cntTime == 0 ) // buffer with the synchronized data is empty
    return 0;

  // step detection
  StepDetector(&PDR->isWalking,
                Data->magnitudeData.magnitudeSync,
                PDR->idxStart,
                Data->timeSync,
                Data->cntTime,
                /*Data->sampleRate,*/
                &UserParams->SensorParams,
                Steps->spData,
				VirtualSteps->spData,
                &Steps->cntSteps,
				&VirtualSteps->cntSteps);

#ifdef PDR_DEBUG
  for (i = 0; i < Steps->cntSteps; i++)
    Steps->spData[i].acc_mean_g = UserParams->SensorParams.acc_mean_g;
#endif

  // get initial heading
  if( (PDR->InitHeading == 0) && (Data->cntTime > INIT_HEADING_TIME) )
  {
    GetHeadingAndMatrixes(Data, Steps, 0, 0, &PDR->prevAtt, INIT_HEADING, UserParams, PDR->pDREKF);
    PDR->InitHeading = 1;
  }
  
  // heading estimation for virtual steps
  tDREKF virtDREKF(SYNC_SMP_RATE);
  virtDREKF = *(PDR->pDREKF);
  VirtStepsDeltaHeadings(Data, VirtualSteps, VirtualSteps->cntSteps, &virtDREKF);

  // prepare virtual steps for PF
  VirtualSteps->cntPfSteps = 0;
 // prepareDataForPF(Data, VirtualSteps, 0, VirtualSteps->cntSteps);
  
  // determination of the mode and the number of steps/stands for issuing
  if( getModeAndSteps(PDR, Steps, Data->cntTime, &idxStart, &cntSteps) )
  {
    Steps->cntPfSteps = 0;
	return 0; // the results are not ready
  }

  // heading estimation
  GetHeadingAndMatrixes(Data, Steps, idxStart, cntSteps, &PDR->prevAtt, IN_STEP_HEADING, UserParams, PDR->pDREKF);
  // step length estimation
  getStepLength(UserParams->K, Steps);

  // prepare data for PF
  Steps->cntPfSteps = 0;
  prepareDataForPF(Data, Steps, idxStart, cntSteps);

  // shift synchronized data
  shiftSynchronizedData(PDR, Data, Steps);

  // check internal data state
  if (!CheckInternalData(Data, Steps))
  {
    ReportPDR::LogReport("\nERROR: PDR DATA CORRUPTED AT PDR END (%lf)",(double)Data->accData.rawData.st[Data->accData.rawData.cntData]);
    assert(0);
    ResetPDR(PDR,Data, Steps);
  }
  
  return 0;
}

//=========================================================================
/// calculates legth of steps for step series
///
/// @param[in]     K     - coefficient from user parameters
/// @param[in/out] Steps - pointer to steps inforamtion
//=========================================================================
static void getStepLength(double K, tStepsInfo* Steps)
{
  int8_t i;
  double accAmplitude;

  for( i = 0; i < Steps->cntSteps; i++ )
  {
    accAmplitude = Steps->spData[i].accMax - Steps->spData[i].accMin;
    if(accAmplitude < 0.)
    {
      ReportPDR::LogReport("\nError: (getStepLength) negative acc amplitude of step %f ignored. t-step = %.3f",
                              accAmplitude, Steps->spData[i].timeStart);
      accAmplitude = 0;
      assert(0);
    }
      Steps->spData[i].length = (float)(K*sqrt(sqrt(accAmplitude)));
  }
  for( i = 0; i < Steps->cntSteps; i++ )
  {
    if( (Steps->spData[i].pEvent != 0.) && (Steps->spData[i].length == 0.) )
    {
      if( Steps->cntSteps > MIN_STEPS )
        Steps->spData[i].length = (float)((Steps->spData[i-1].length + Steps->spData[i+1].length)/2.); // space between two steps
      else
        Steps->spData[i].length = Steps->spData[i-1].length; // two steps and space
    }
  }
}

//=========================================================================
/// shift the filtered data
///
/// @param[in]     PDR   - pointer to instance
/// @param[in/out] Data - pointer to data
//=========================================================================
static void shiftFilteredData(tPDR* PDR, tData* Data)
{
  int16_t i, cntStart;
  int8_t j;

  // shift accelerometer data
  if( Data->accData.fltData.cntData > PDR->filterAcc.cntMinStep )
  {
#ifdef PDR_DEBUG
    // fir data loging
    for( i = 0; i < (Data->accData.fltData.cntData - PDR->filterAcc.cntMinStep); i++ )
    {
      ReportPDR::FIRDataLog(est_Accelerometer,
                            Data->accData.fltData.t[i],
                            Data->accData.fltData.data[0][i],
                            Data->accData.fltData.data[1][i],
                            Data->accData.fltData.data[2][i]);
    }
#endif
    // move accelerometer data
    cntStart = 0;
    assert((Data->accData.fltData.cntData - PDR->filterAcc.cntMinStep) <= RAW_FLT_DATA);
    for( i = Data->accData.fltData.cntData - PDR->filterAcc.cntMinStep; i >= 0; i-- )
    {
      if( Data->accData.fltData.t[i] < Data->timeSync[Data->cntTime-1] )
      {
        cntStart = i;
        assert((Data->accData.fltData.cntData - cntStart) <= RAW_FLT_DATA);
        assert((Data->accData.fltData.cntData - cntStart) >= 0);
        break;
      }
    }

    assert((cntStart) <= RAW_FLT_DATA);
    assert((cntStart) >= 0);
    assert((Data->accData.fltData.cntData - cntStart) <= RAW_FLT_DATA);
    assert((Data->accData.fltData.cntData - cntStart) >= 0);
    for( i = 0; i < (Data->accData.fltData.cntData - cntStart); i++ )
    {
      Data->accData.fltData.t[i] = Data->accData.fltData.t[cntStart + i];
      Data->magnitudeData.magnitudeFlt[i] = Data->magnitudeData.magnitudeFlt[cntStart + i];
      for( j = 0; j < 3; j++ )
      {
        assert((cntStart + i) <= RAW_FLT_DATA);
        Data->accData.fltData.data[j][i] = Data->accData.fltData.data[j][cntStart + i];
      }
    }
    Data->accData.fltData.cntData = Data->accData.fltData.cntData - cntStart;
    assert((Data->accData.fltData.cntData) <= RAW_FLT_DATA);
    assert((Data->accData.fltData.cntData) >= 0);
  }

  // shift gyro data
  if( Data->gyrData.fltData.cntData > PDR->filterGyr.cntMinStep )
  {
#ifdef PDR_DEBUG
    // fir data loging
    for( i = 0; i < (Data->gyrData.fltData.cntData - PDR->filterGyr.cntMinStep); i++ )
    {
      ReportPDR::FIRDataLog(est_Gyro,
                            Data->gyrData.fltData.t[i],
                            Data->gyrData.fltData.data[0][i],
                            Data->gyrData.fltData.data[1][i],
                            Data->gyrData.fltData.data[2][i]);
    }
#endif
	cntStart = 0;
	for( i = Data->gyrData.fltData.cntData - PDR->filterGyr.cntMinStep; i >= 0; i-- )
	{
		if( Data->gyrData.fltData.t[i] < Data->timeSync[Data->cntTime-1] )
		{
		      cntStart = i;
                      assert((Data->gyrData.fltData.cntData - cntStart) <= RAW_FLT_DATA);
                      assert((Data->gyrData.fltData.cntData - cntStart) >= 0);
			break;
		}
	}
    for( i = 0; i < (Data->gyrData.fltData.cntData - cntStart); i++ )
    {
      Data->gyrData.fltData.t[i] = Data->gyrData.fltData.t[cntStart + i];
      for( j = 0; j < 3; j++ )
        Data->gyrData.fltData.data[j][i] = Data->gyrData.fltData.data[j][cntStart + i];
    }
    Data->gyrData.fltData.cntData = Data->gyrData.fltData.cntData - cntStart;
  }

  // shift magnetic data
  if( Data->magnData.fltData.cntData > PDR->filterMagn.cntMinStep )
  {
#ifdef PDR_DEBUG
    // fir data loging
    for( i = 0; i < (Data->magnData.fltData.cntData - PDR->filterMagn.cntMinStep); i++ )
    {
      ReportPDR::FIRDataLog(est_Magnetometer,
                            Data->magnData.fltData.t[i],
                            Data->magnData.fltData.data[0][i],
                            Data->magnData.fltData.data[1][i],
                            Data->magnData.fltData.data[2][i]);
    }
#endif
	cntStart = 0;
	for( i = Data->magnData.fltData.cntData - PDR->filterMagn.cntMinStep; i >= 0; i-- )
	{
		if( Data->magnData.fltData.t[i] < Data->timeSync[Data->cntTime-1] )
		{
                   assert((Data->magnData.fltData.cntData - cntStart) <= RAW_FLT_DATA);
                   assert((Data->magnData.fltData.cntData - cntStart) >= 0);
			cntStart = i;
			break;
		}
	}
    for( i = 0; i < (Data->magnData.fltData.cntData - cntStart); i++ )
    {
      Data->magnData.fltData.t[i] = Data->magnData.fltData.t[cntStart + i];
      for( j = 0; j < 3; j++ )
        Data->magnData.fltData.data[j][i] = Data->magnData.fltData.data[j][cntStart + i];
    }
    Data->magnData.fltData.cntData = Data->magnData.fltData.cntData - cntStart;
  }
}

//============================================================================
/// determination of the mode and the number of steps/stands for issuing
///
/// @param[in]  PDR      - pointer to instance
/// @param[in]  Steps    - pointer to steps inforamtion
/// @param[in]  cntTime  - counter of time
/// @param[out] idxStart - pointer to index of the first step/stand
/// @param[out] cntSteps - pointer to counter of steps/stands
/// @return 1 if the results are not ready
//============================================================================
static int getModeAndSteps(tPDR* PDR, 
                           tStepsInfo* Steps, 
                           int16_t cntTime,
                           int8_t* idxStart, 
                           int8_t* cntSteps)
{
	switch( PDR->isWalking ) 
	{
	case STANDS: // defined intervals stop
		{
			if( Steps->cntSteps < MIN_STANDS ) // give out previous step when the current step is completed
			{
				*idxStart = 0;
				*cntSteps = 0;
				Steps->cntSteps = 0;
				return 1;
			}
			*idxStart = 0;
			*cntSteps = 1;
			break;
		}
	case SPACE_START_STEPS: // defined interval stop before the first step in a series of steps
		{
			*idxStart = 0;
			*cntSteps = Steps->cntSteps;
			break;
		}
	case START_STEPS: // expect definition of a series of steps
		{
			*idxStart = 0;
			*cntSteps = 0;
			if( cntTime < THESH_BUFF ) // if the definition of a series of steps take more than 11 seconds, then the shift input
			{
				Steps->cntSteps = 0;
				return 1;
			}
			break;
		}
	case SKIP_START_STEPS: // defined false step, then defined interval stop
		{
			*idxStart = 0;
			*cntSteps = 0;
			break;
		}
	case STEPS_FIRST: // defined first steps in a series of steps beginning
		{
			*idxStart = 0;
			*cntSteps = Steps->cntSteps;
			break;
		}
	case STEPS:
		{
			if( Steps->cntSteps < MIN_STEPS ) // the number steps required to obtain the result
			{
				*idxStart = 0;
				*cntSteps = 0;
				return 1;
			}
			*idxStart = REMAIN_STEPS; 
			*cntSteps = Steps->cntSteps - REMAIN_STEPS; // the first two steps old, defined one new step or two new steps
			break;
		}
	case STEPS_END: // the end of a series of steps, the last steps defined: no new steps, one new step, two new step
		{
			*idxStart = REMAIN_STEPS; 
			*cntSteps = Steps->cntSteps - REMAIN_STEPS;
			break;
		}
	case STEPS_SEPARATE: // separate steps and intervals stop before them
		{
			*idxStart = 0;
			*cntSteps = Steps->cntSteps;
			break;
		}
	default:
		{
			*idxStart = 0; 
			*cntSteps = 0;
			Steps->cntSteps = 0;
			return 1;
		}
	}
	return 0;
}

//============================================================================
/// prepare data for PF
///
/// @param[in]     Data     - pointer to data
/// @param[in/out] Steps    - pointer to steps inforamtion
/// @param[in]     idxStart - pointer to index of the first step/stand
/// @param[in]     cntSteps - pointer to counter of steps/stands
//============================================================================
static void prepareDataForPF(tData* Data,
                             tStepsInfo* Steps, 
                             int8_t idxStart, 
                             int8_t cntSteps)
{
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable:6385)
#endif
  int8_t i, k;
  int16_t j;
  double magnData[3];

  for( i = 0; i < cntSteps; i++ )
  {
    tPFData &pfData = Steps->pfData[Steps->cntPfSteps];
    tStepParams &stepData = Steps->spData[i+idxStart];

#ifdef PDR_DEBUG
	pfData.DTWcombined = stepData.DTWcombined;
	pfData.acc_mean_g = stepData.acc_mean_g;
#endif
    pfData.timeStart = stepData.timeStart;
    pfData.timeEnd = stepData.timeEnd;
    pfData.pEvent = stepData.pEvent;
    pfData.length = stepData.length;
    pfData.acc_span = stepData.accMax - stepData.accMin;
    pfData.speed = (float)(pfData.length/(pfData.timeEnd - pfData.timeStart));
    pfData.heading = -Steps->attData[i+idxStart].psi;
    pfData.headingInc = -Steps->attData[i+idxStart].headingInc;
    pfData.magnStart = (float)sqrt(Data->magnData.syncData[0][stepData.idxStart]*Data->magnData.syncData[0][stepData.idxStart] +
      Data->magnData.syncData[1][stepData.idxStart]*Data->magnData.syncData[1][stepData.idxStart] +
      Data->magnData.syncData[2][stepData.idxStart]*Data->magnData.syncData[2][stepData.idxStart]);
    pfData.magnEnd = (float)sqrt(Data->magnData.syncData[0][stepData.idxEnd]*Data->magnData.syncData[0][stepData.idxEnd] +
      Data->magnData.syncData[1][stepData.idxEnd]*Data->magnData.syncData[1][stepData.idxEnd] +
      Data->magnData.syncData[2][stepData.idxEnd]*Data->magnData.syncData[2][stepData.idxEnd]);
    magnData[0] = magnData[1] = magnData[2] = 0.;
    for( j = stepData.idxStart; j <= stepData.idxEnd; j++ )
      for( k = 0; k < 3; k++ )
        magnData[k] += Data->magnData.syncData[k][j];
    for( k = 0; k < 3; k++ )
      magnData[k] /= (j - stepData.idxStart);
    pfData.magnAverage = (float)sqrt(magnData[0]*magnData[0] + magnData[1]*magnData[1] + magnData[2]*magnData[2]);

    // magnetic data accuracy factor - obtained from magnetic sensor meassurements
    pfData.magnWorstAccuracy = 0; // now it not supported
    pfData.magnMeanAccuracy = 0; //  now it not supported

    magnData[0] = (float)-1.;
    for( j = stepData.idxStart; j <= stepData.idxEnd; j++ )
    {
      magnData[1] = (float)sqrt(Data->magnData.syncData[0][j]*Data->magnData.syncData[0][j] +
        Data->magnData.syncData[1][j]*Data->magnData.syncData[1][j] + 
        Data->magnData.syncData[2][j]*Data->magnData.syncData[2][j]);
      if( magnData[0] == -1. )
        magnData[0] = magnData[1];
      else if( magnData[1] < magnData[0] )
      magnData[0] = magnData[1];
    }
    /*pfData.magnWorstAccuracy = magnData[0];
    if( pfData.magnWorstAccuracy > 0. )
      pfData.magnMeanAccuracy = Steps->pfData[i].magnAverage;
    else
      pfData.magnMeanAccuracy = 0.;
      */
    // Additional MFP
    pfData.fAttitudeState = Steps->attData[i+idxStart].fAttitudeState;
    CopyMatrix( &pfData.Cp2b, &Steps->attData[i+idxStart].Cp2b);
    CopyMatrix( &pfData.Cp2b1, &Steps->attData[i+idxStart].Cp2b1);
    pfData.pitch = Steps->attData[i+idxStart].teta;
    pfData.roll = Steps->attData[i+idxStart].fi;
    pfData.t_att = Steps->attData[i+idxStart].time;
    pfData.magnAvgPF[0] = stepData.meanMagnetic[0];
    pfData.magnAvgPF[1] = stepData.meanMagnetic[1];
    pfData.magnAvgPF[2] = stepData.meanMagnetic[2];
     
    Matrix meanMagnMrx, MB;
    CreateMatrix(&meanMagnMrx, 3, 1);
    meanMagnMrx.Matr[0][0] = pfData.magnAvgPF[0];
    meanMagnMrx.Matr[1][0] = pfData.magnAvgPF[1];
    meanMagnMrx.Matr[2][0] = pfData.magnAvgPF[2];
    MultMatrix(&MB, &pfData.Cp2b, &meanMagnMrx);
    pfData.magnAvgBF[0] = MB.Matr[0][0];
    pfData.magnAvgBF[1] = MB.Matr[1][0];
    pfData.magnAvgBF[2] = MB.Matr[2][0];
    
    for( j = stepData.idxStart; j <= stepData.idxEnd; j++ )
      for( k = 0; k < 3; k++ )
        pfData.SyncMagnData[k][j-stepData.idxStart] = Data->magnData.syncData[k][j];
    pfData.magnSampleCount = j - stepData.idxStart;
    
    pfData.SampleCount = stepData.cntMagData;

    for (j = 0; j < stepData.cntMagData; j++)
    {
        pfData.sample_time[j] = Data->timeSync[j + stepData.idxStart];
        //pfData.SyncMagnData[0][j] = Data->magnData.syncData[0][j + stepData.idxStart];
        //pfData.SyncMagnData[1][j] = Data->magnData.syncData[1][j + stepData.idxStart];
        //pfData.SyncMagnData[2][j] = Data->magnData.syncData[2][j + stepData.idxStart];

        CopyMatrix( &pfData.Cp2ne_set[j], &stepData.Cp2b[j]);

#if 0 //  debug out
        { 
            static FILE * fStepsOut = 0;
            if (fStepsOut == 0)
            {
                fStepsOut = fopen("data_dbg.txt", "wt");
            }
            if (fStepsOut != 0)
                fprintf(fStepsOut, "%.0lf  %8.6lf %8.6lf %8.6lf  %8.6lf %8.6lf %8.6lf  %8.6lf %8.6lf %8.6lf  %8.6lf %8.6lf \n",
                Data->timeSync[stepData.idxStart+j]*1e3,
                stepData.Cp2b[j].Matr[0][0],
                stepData.Cp2b[j].Matr[0][1],
                stepData.Cp2b[j].Matr[0][2],
                stepData.Cp2b[j].Matr[1][0],
                stepData.Cp2b[j].Matr[1][1],
                stepData.Cp2b[j].Matr[1][2],
                stepData.Cp2b[j].Matr[2][0],
                stepData.Cp2b[j].Matr[2][1],
                stepData.Cp2b[j].Matr[2][2],
                pfData.pitch,
                pfData.roll);
        }
#endif
    }

#if 0
    { //  debug out
      static FILE * fStepsOut = 0;
      if (fStepsOut == 0)
      {
        fStepsOut = fopen("data_dbg1.txt", "wt");
      }
      if (fStepsOut != 0)
        for (j = 0; j < pfData.magnSampleCount; j++)
          fprintf(fStepsOut, "%.3lf   %7.3lf %7.3lf %7.3lf   %7.3lf %7.3lf\n",
                            Data->timeSync[stepData.idxStart+j],
                            pfData.SyncMagnData[0][j],
                            pfData.SyncMagnData[1][j],
                            pfData.SyncMagnData[2][j],
                            pfData.pitch,
                            pfData.roll);
    }
#endif

    // enuMagnData
    //if (!stepData.ubfMagnSamples.empty())
    if ((stepData.cntMagData > 0) && (stepData.cntMagData < SIZE_OF_ONE_STEP))
    {
      //for (std::list<vector_3d>::iterator magn_sample = stepData.ubfMagnSamples.begin(); magn_sample != stepData.ubfMagnSamples.end(); magn_sample++)
      for (j = 0; j < stepData.cntMagData; j++)
      {// UBF = ENU wthiou heading rotation
        pfData.enuMagnData[0][j] =  stepData.ubfMagData[0][j];  //magn_sample->x;
        pfData.enuMagnData[1][j] =  stepData.ubfMagData[1][j];  //magn_sample->y;
        pfData.enuMagnData[2][j] =  stepData.ubfMagData[2][j];  //magn_sample->z;
      }
      pfData.magnSampleCount = stepData.cntMagData;
    }
    else 
      pfData.magnSampleCount = 0;

#if 0
    { //  debug out
      static FILE * fStepsOut = 0;
      if (fStepsOut == 0)
      {
        fStepsOut = fopen("data_dbg.txt", "wt");
      }
      if (fStepsOut != 0)
        for (j = 0; j < stepData.cntMagData; j++)
          fprintf(fStepsOut, "%.0lf  %8.6lf %8.6lf %8.6lf  %8.6lf %8.6lf %8.6lf  %8.6lf %8.6lf %8.6lf  %8.6lf %8.6lf \n",
                            Data->timeSync[stepData.idxStart+j]*1e3,
                            stepData.Cp2b[j].Matr[0][0],
                            stepData.Cp2b[j].Matr[0][1],
                            stepData.Cp2b[j].Matr[0][2],
                            stepData.Cp2b[j].Matr[1][0],
                            stepData.Cp2b[j].Matr[1][1],
                            stepData.Cp2b[j].Matr[1][2],
                            stepData.Cp2b[j].Matr[2][0],
                            stepData.Cp2b[j].Matr[2][1],
                            stepData.Cp2b[j].Matr[2][2],
                            pfData.pitch,
                            pfData.roll);
    }
#endif

    Steps->cntPfSteps++;
  }
#ifdef _MSC_VER
#pragma warning (pop)
#endif
}

//==========================================================================
/// shift synchronized data
///
/// @param[in] PDR      - pointer to instance
/// @param[in] Data     - pointer to data
/// @param[in] Steps    - pointer to steps inforamtion
//==========================================================================
static void shiftSynchronizedData(tPDR* PDR, 
                                  tData* Data, 
                                  tStepsInfo* Steps)
{
	int16_t i, shift;
	int8_t j;

	switch( PDR->isWalking )
	{
	case STANDS:
		{
			shift = Steps->spData[1].idxStart; // shift the first stop interval
			PDR->idxStart = 0;
			Steps->cntSteps = 0;
			break;
		}
	case SPACE_START_STEPS:
		{
			shift = Steps->spData[Steps->cntSteps - 1].idxEnd+1; // shift of all stop intervals 
			PDR->idxStart = 0;
			Steps->cntSteps = 0;
			break;
		}
	case START_STEPS: // if the definition of a series of steps take more than 11 seconds, then the shift input
		{
			shift = MIN_DATA_SYNC;
			PDR->idxStart = 0;
			Steps->cntSteps = 0;
			break;
		}
	case SKIP_START_STEPS:
		{
			shift = 0; // shift of all stop intervals 
			PDR->idxStart = Steps->spData[Steps->cntSteps - 1].idxEnd+1;
			Steps->cntSteps = 0;
			break;
		}
	case STEPS_FIRST:
	case STEPS:
		{
			shift = Steps->spData[1].idxStart; // shift the one step
			PDR->idxStart = Steps->spData[Steps->cntSteps - 1].idxEnd+1 - shift; // new analysis begins with the end of the last step
			CopyStepToStep(&Steps->spData[0], &Steps->spData[1]);
			if( Steps->cntSteps > MIN_STEPS ) // in a series of steps can be determined by four steps: one of the steps is the space between the steps
				CopyStepToStep(&Steps->spData[1], &Steps->spData[3]); // skip the steps is the space between the steps			
			else
				CopyStepToStep(&Steps->spData[1], &Steps->spData[2]);
			Steps->cntSteps = REMAIN_STEPS;
			for( j = 0; j < Steps->cntSteps; j++ )
			{
				Steps->spData[j].idxEnd -= shift;
				Steps->spData[j].idxMax -= shift;
				Steps->spData[j].idxMin -= shift;
				Steps->spData[j].idxStart -= shift;
			}
			break;
		}
	case STEPS_END:
	case STEPS_SEPARATE:
		{
			shift = Steps->spData[Steps->cntSteps - 1].idxEnd + 1; // shift all the steps
			PDR->idxStart = 0;
			Steps->cntSteps = 0;
			break;
		}
	default:
		{
			shift = 0;
			PDR->idxStart = 0;
			Steps->cntSteps = 0;
		}
	}
#if SYNC_DATA_LOGS_ON
  {// output sync gyro data
    static FILE * fOutGyr = 0;
    if (fOutGyr == 0)
      fOutGyr = fopen("c-rt_sync_gyr.txt","wt");
    if (fOutGyr != 0)
      for( i = 0; i < shift; i++ )
        fprintf(fOutGyr, "%.6f,%20.15f,%20.15f,%20.15f\n",
                Data->timeSync[i], Data->gyrData.syncData[0][i],
                Data->gyrData.syncData[1][i], Data->gyrData.syncData[2][i]);
  }
  {// output sync acc data
    static FILE * fOutAcc = 0;
    if (fOutAcc == 0)
      fOutAcc = fopen("c-rt_sync_acc.txt","wt");
    if (fOutAcc != 0)
      for( i = 0; i < shift; i++ )
        fprintf(fOutAcc, "%.6f,%20.15f,%20.15f,%20.15f\n",
                Data->timeSync[i], Data->accData.syncData[0][i],
                Data->accData.syncData[1][i], Data->accData.syncData[2][i]);
  }
  {// output sync magnetic data
    static FILE * fOutMagn = 0;
    if (fOutMagn == 0)
      fOutMagn = fopen("c-rt_sync_mg.txt","wt");
    if (fOutMagn != 0)
      for( i = 0; i < shift; i++ )
        //fprintf(fOutMagn, "%.6f,%20.15f,%20.15f,%20.15f\n",
        fprintf(fOutMagn, "%.0f,%20.15f,%20.15f,%20.15f\n",
                Data->timeSync[i]*1000., Data->magnData.syncData[0][i],
                Data->magnData.syncData[1][i], Data->magnData.syncData[2][i]);
  }
#endif

  for( i = 0; i < Data->cntTime - shift; i++ )
	{
		Data->timeSync[i] = Data->timeSync[shift + i];
		Data->magnitudeData.magnitudeSync[i] = Data->magnitudeData.magnitudeSync[shift + i];
		for( j = 0; j < 3; j++ )
		{
			Data->accData.syncData[j][i] = Data->accData.syncData[j][shift + i];
			Data->gyrData.syncData[j][i] = Data->gyrData.syncData[j][shift + i];
			Data->magnData.syncData[j][i] = Data->magnData.syncData[j][shift + i];
		}
	}
	//printf("sift=%d cnt_time%d \n",shift, Data->cntTime);
	Data->cntTime -= shift;
	//if(Data->cntTime < 0)
	//  Data->cntTime = Data->cntTime;
}