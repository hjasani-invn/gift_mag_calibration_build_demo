/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  data types for PDR module
 *   @file                   PDR.h
 *   @author                 M. Zhokhova
 *   @date                   10.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#ifndef _PDR_H_
#define _PDR_H_

#include "Init.h"
#include "FIRfilter.h"
#include "DR.h"
#include "PDR_data.h"


// step detector defines
// #define MIN_05_STEP_DURATION      (0.09)
// #define MIN_STEP_DURATION         (2.*MIN_05_STEP_DURATION)
// #define MAX_05_STEP_DURATION      (0.6)
// #define MAX_STEP_DURATION         (2.*MAX_05_STEP_DURATION)

// #define SIZE_OF_ONE_STEP  ((int16_t)(MAX_STEP_DURATION*SYNC_SMP_RATE) + 10) // 1.2s for 50 Hz + delta
#define SYNC_DATA         (int16_t)(SYNC_SMP_RATE*MAX_STEPS*MAX_STEP_DURATION) // buffer for max steps = MAX_STEPS*MAX_STEP_DURATION = 12s for 50Hz 
#define THESH_BUFF        (int16_t)(SYNC_DATA - SYNC_SMP_RATE*MAX_STEP_DURATION) // SYNC_DATA - 1 steps for 50 Hz
#define INIT_HEADING_TIME (int16_t)(SYNC_SMP_RATE*0.5) // 0.5s for 50Hz
#define MIN_DATA_SYNC     (int16_t)(MIN_TIME_DATA*SYNC_SMP_RATE)

#define EST_INTERVAL    (0.5) // estimation interval duration [s]
#define SIZE_ROUGH_INTERVAL (int16_t)(EST_INTERVAL*SYNC_SMP_RATE)

#define TIME_OUT_STANDS (0.5) // 1.

#define STANDS            (0)
#define SPACE_START_STEPS (1) // skip false step at the beginning of a series of steps
#define START_STEPS       (2)
#define SKIP_START_STEPS  (3) // stop interval before the first step series of steps 
#define STEPS_FIRST       (4) // first steps in a series of steps beginning
#define STEPS             (5)
#define STEPS_END         (6)
#define STEPS_SEPARATE    (7) // separate steps: 2, 3

#define DTW_NO_STEPS  (0)
#define DTW_STEPS     (1)
#define DTW_SKIP      (2) // the space between the first and second steps or skip false steps
#define DTW_SPACE     (3) // the space between the second and the third step in a series of steps 
                           // or in a series of steps beginning

#define MAX_SMP        (2) // the maximum distance between two adjacent steps

#define MIN_STANDS   (4) // the number stands required to obtain the result
#define MIN_STEPS    (3) // the number steps required to obtain the result
#define REMAIN_STEPS (2) // 

#define FILTERING_INIT    (0)
#define FILTERING_PROCESS (1)

#define INIT_HEADING    (0)   // initialization heading and matrixes
#define IN_STEP_HEADING (1)   // calculation heading and matrixes "in step"

#define MAX_SIGMA_ACC (0.07) // maximal allowed acceleration magnitude dispersion
#define MAX_SIGMA_GYR (0.05) // maximal allowed data from gyroscopes magnitude dispersion

#define MAX_SIGMA_ACC_ROUGH (0.5) // maximal allowed acceleration magnitude dispersion
#define MAX_SIGMA_GYR_ROUGH (0.25) // maximal allowed data from gyroscopes magnitude dispersion

#define MIN_STEP_AMPLITUDE_FACTOR (0.06) // min step amplitude factor
#define MAX_STEP_AMPLITUDE_FACTOR (2.) // max step amplitude factor

//#define START_WALK_THRESH_DTW         (0.000153420312182887) // start of walk threshold DTW (optimal by Kotik)
#define START_WALK_THRESH_DTW         (50*0.000153420312182887) // start of walk threshold DTW (optimal by Kotik)
#define END_WALK_THRESH_DTW           (0.906286021829204) // end of walk threshold DTW (optimal by Kotik)

#ifndef PDR_DEBUG
#define MEAN_G            (double)(9.81)
#else
#define MEAN_G            (double)(9.81)
//#define MEAN_G             (double)(9.870190038) //(9.774169322) //(11.008307688) //(9.900830396) //(9.868431929)//(9.8100000)// (10.838173798) //  (9.925271325)
#define FIX_MEAN_G
#endif

#define REJECT_THRESH             (0.5)
#define DTW_COEFF    (float)(0.24)

#define STEP_CUTOFF_PROBABILITY   (0.5)
#define QUARTER_P_EVENT           (0.25)
#define HALF_P_EVENT              (0.5)
#define SKIP_P_EVENT              (0.)
#define SPACE_P_EVENT             (0.5)

// tDREKF class predefinition
class tDREKF;

typedef struct tMagnitudeAcc
{
  double magnitudeRaw[RAW_FLT_DATA]; // magnitude
  double magnitudeFlt[RAW_FLT_DATA]; // magnitude filtering
  double magnitudeSync[SYNC_DATA]; // synchronized magnitude
}
tMagnitudeAcc;

// data from sensors
typedef struct tSensData
{
  tRawData rawData;
  tFltData fltData;
  double syncData[3][SYNC_DATA]; // synchronized data
  int16_t index_delay_flt;
  float smpRate; // round sample rate of raw data
  float smpRateFIR; // sample rate of applied FIR // debug field
  double bias[3];
}
tSensData;

typedef struct tData
{
  // acceleration from the accelerometers
  tSensData accData;
  tMagnitudeAcc magnitudeData;
  // angular velocity (w) measured by the gyroscopes
  tSensData gyrData;
  // magnetic field strength from magnetic compass
  tSensData magnData;
  // common data
  float sampleRate;
  double timeSync[SYNC_DATA]; // generated uniform time scale 
  int16_t cntTime;
}
tData;


// // data for particle filter
// typedef struct tPFData
// {
//   // base data for particle filter
//   double DTWcombined; // combined DTW metric
//   double timeStart; // time of step/stand start, sec or msec, (double/uint48)
//   double timeEnd; // time of step/stand end, sec or msec, (double/uint48)
//   float pEvent; // probability of correct of event detection
//   float length; // time of step length, m
//   float speed; // average speed during step, m/sec
//   double heading; // step/stang heading, rad
//   double headingInc; // step/stang heading increment, rad
//   float magnStart; // magnetic vector magnitude at start point of step/stand
//   float magnEnd; // magnetic vector magnitude at end point of step/stand
//   float magnAverage; // average magnetic vector magnitude in step/stand, mTesla
//   double magnWorstAccuracy; // worst accuracy of magnetic measurement during step/stand, mTesla
//   float magnMeanAccuracy;  // mean accuracy of magnetic measurement during step/stand, mTesla
//   double acc_span;          // maximal acceleration deviation during step/ m/s/s

//   // Additional data for MFP
//   eAttitudeState fAttitudeState; // flag of initial attitude of user validity
//   double t_att; // time of current attitude data associating
//   Matrix Cp2b; // Phone frame to User body frame transition matrix
//   Matrix Cp2b1; // Phone frame to User body frame transition matrix
//   double magnAvgPF[3]; // average magnetic vector magnitude in step/stand, mTesla
//   double magnAvgBF[3]; // average magnetic vector magnitude in step/stand, mTesla
//   double SyncMagnData[3][SIZE_OF_ONE_STEP]; // magnetic samples in phone frame (instrumental)
//   double enuMagnData[3][SIZE_OF_ONE_STEP]; // magnetic samples in ENU frame without heading rotation
//   double sample_time[SIZE_OF_ONE_STEP];
//   long magnSampleCount;
//   long SampleCount;

//   Matrix Cp2ne_set[SIZE_OF_ONE_STEP]; // set of matrixes Phone->NE 

//   double pitch;          // rad, phone pitch angle
//   double roll;           // rad, phone roll angle

//   //double SyncDataDelay; // 
//   //double RawMagnData[3][SIZE_OF_ONE_STEP];
//   //long cntRawMagnData;

//   // debug data
// #ifdef PDR_DEBUG
//   double acc_mean_g;
// #endif
// }
// tPFData;


// data of Step Detector and Step Length Estimator
typedef struct tStepParams
{
  int16_t idxMax; // index of maximum acceleration in data sensor arrays 
  int16_t idxMin; // index of minimal acceleration in data sensor arrays
  int16_t idxStart; // index of start step/stand point in data sensor arrays
  int16_t idxEnd; // index of end step/stand point in data sensor arrays
  double timeStart; // time of step/stand start
  double timeMax; // time of maximum acceleration point
  double timeMin; // time of minimal acceleration point
  double timeEnd; // time of step/stand end
  double accMax; // maximum acceleration during step
  double accMin; // minimal acceleration during step
  float pEvent; // probability of correct of event detection
  int8_t isWalking; // indicator of step series 
	double DTWideal; // DTW metric of step vs ideal step
  double DTWnext; // DTW metric of step vs next step
	double DTWprev; // DTW metric of step vs previous step
  double DTWskip; // DTW metric of step vs next+1 step (or prev+1 step)
  double DTWcombined; // combined DTW metric, equals to dDTWideal*dDTWskip*DTWnext
  float length;           

  double meanMagnetic[3];

  double ubfMagData[3][SIZE_OF_ONE_STEP]; // magnetic samples in ENU frame without heading rotation
  Matrix Cp2b[SIZE_OF_ONE_STEP];
  int16_t cntMagData;

#ifdef PDR_DEBUG
  double acc_mean_g;
#endif
}
tStepParams;

typedef struct tStepsInfo
{
	tStepParams spData[MAX_STEPS];
	tStepAttitude attData[MAX_STEPS];
	tPFData pfData[MAX_STEPS];
	int8_t cntSteps;
	int8_t cntPfSteps;
}
tStepsInfo;

// extremum data
typedef struct tExtrem
{
	int16_t idx;
	float sign; 
	double magnitude;
	double time;
}
tExtrem;

#define FIR100_DL (101) // response FIR filter 200Hz
#define FIR50_DL  (51)  // response FIR filter 100Hz
#define FIR24_DL  (25)  // response FIR filter 50Hz  (48Hz)
#define FIR12_DL  (13)  // response FIR filter 25Hz  (24Hz)
#define FIR8_DL   (9)   // response FIR filter 16Hz
#define FIR4_DL   (5)   // response FIR filter 10Hz

typedef struct tFilter
{
	tFIRfilter FIR[4];
	double buffDelayLine[4][FIR100_DL];
	int16_t delay;
	double delay_t;
	int16_t cntMinStep;
}
tFilter;

typedef struct tPDR
{
	int8_t isStart;
	int8_t isWalking; // indicator of step series 
	int16_t idxStart;
	int8_t InitHeading;
	tFilter filterAcc;
	tFilter filterGyr;
	tFilter filterMagn;
	tStepAttitude prevAtt;
  tDREKF *pDREKF; // pointer to DREKF instance
}
tPDR;

//=========================================================================
/// PDR process
///
/// @param[in]  PDR        - pointer to instance
/// @param[in]  Data       - pointer to data
/// @param[in]  UserParams - pointer to user parameters structure
/// @param[out] Steps      - pointer to steps inforamtion
/// @return Zero if successful, 1 - incorrect sample rate for one second
//=========================================================================
extern int8_t ProcessPDR(tPDR* PDR,
						 tData* Data,
						 tUserParams* UserParams, 
						 tStepsInfo* Steps,
						 tStepsInfo* VirtualSteps);

//==============================================================
/// Initialize filtering sensor data: accelerometers, gyroscopes 
/// and magnetic compass
///
/// @param[in]     PDR  - pointer to instance
/// @param[in/out] Data - pointer to data
/// @return Zero if successful
//==============================================================
extern int8_t InitFilteringSensorData(tPDR* PDR, tData* Data);

//==============================================================
/// Filtering sensor data: accelerometers, gyroscopes and
/// magnetic compass
///
/// @param[in]     PDR  - pointer to instance
/// @param[in/out] Data - pointer to data
//==============================================================
extern void FilteringSensorData(tPDR* PDR, tData* Data);

//=========================================================================
/// Synchronizes filtered data from all sensors
///
/// @param[in/out]  Data - pointer to filtered sensor data
//=========================================================================
extern void SynchronizSensorData(tData* Data);

//==============================================================================
/// Calculation heading and orientation matrices for steps and for static
/// intervals
///
/// @param[in]     Data       - pointer to data
/// @param[in]     Steps      - pointer to steps inforamtion
/// @param[in]     idxStart   - index of start step
/// @param[in]     cntSteps   - counter of step
/// @param[in/out] prevAtt    - pointer to previous attitude information
/// @param[in]     isInitStep - flag: initializatin (0) or calculate heading 
///                 and orientation matrices in step and in stand intervals (1)
/// @param[out]    UserParams - pointer to user parameters structure with 
///                initial orientation; it is calculated for flag=0 only
/// @param[in/out]    pDREKF  - DREKF instance
//==============================================================================
extern void GetHeadingAndMatrixes(tData* Data,
                                  tStepsInfo* Steps, 
                                  int8_t idxStart, 
                                  int8_t cntSteps, 
                                  tStepAttitude* prevAtt,
                                  int8_t isInitStep,
                                  tUserParams* UserParams,
                                  tDREKF* pDREKF);
//==============================================================================
/// Calculation heading for virtual steps and static intervals
/// @param[in]     Data       - pointer to data
/// @param[in]     VirtSteps      - pointer to steps inforamtion
/// @param[in]     cntVirtSteps   - counter of step
/// @param[in/out] prevAtt    - pointer to previous attitude information
/// @param[in/out]    pDREKF  - DREKF instance
//==============================================================================
extern void VirtStepsDeltaHeadings(tData* Data, 
									  tStepsInfo* VirtSteps, 
									  int8_t cntVirtSteps, 
									  tDREKF* pDREKF);
//=========================================================================
/// Step detection
///
/// @param[in]  isWalking    - pointer to flag 
/// @param[in]  magnitude    - pointer to the magnitude accelerations 
/// @param[in]  idxStart     - start index
/// @param[in]  timeSync     - pointer to uniform time scale
/// @param[in]  cntTime      - counter of time scale
/// @param[in]  SensorParams - pointer to sensor parameters structure
/// @param[out] Step         - pointer to steps inforamtion
/// @param[out] cntSteps     - counter of steps
//=========================================================================
extern void StepDetector(int8_t* isWalking,
                         double* magnitude, 
                         int16_t idxStart,
                         double* timeSync, 
                         int16_t cntTime, 
                         tSensorParams* SensorParams,
                         tStepParams* Step,
						 tStepParams* VirtStep,
                         int8_t* cntSteps,
						 int8_t* cntVirtSteps);

//=========================================================================
/// Obtaining steps of extremums
///
/// @param[in]  timeSync     - pointer to uniform time scale
/// @param[in]  cntTime      - counter of time scale
/// @param[in]  magnitude    - pointer to the magnitude accelerations 
/// @param[in]  idxStart     - start index
/// @param[in]  SensorParams - pointer to sensor parameters structure
/// @param[out] Step         - pointer to steps inforamtion
/// @param[out] cntSteps     - counter of steps
//=========================================================================
extern void ExtremToSteps(double* timeSync, 
					      int16_t cntTime,
					      double* magnitude, 
						  int16_t idxStart,
					      tSensorParams* SensorParams,
					      tStepParams* Step, 
					      int8_t* cntSteps);

//=========================================================================
/// copy step to step
///
/// @param[in]  stepDst - pointer to steps inforamtion
/// @param[out] stepSrc - pointer to steps inforamtion
//=========================================================================
extern void CopyStepToStep(tStepParams* stepDst, tStepParams* stepSrc);

#endif // _PDR_H_