#ifndef _PDR_DATA_H_
#define _PDR_DATA_H_
#include "matrix.h"
#include "Init.h"




#define MIN_05_STEP_DURATION      (0.09)
#define MIN_STEP_DURATION         (2.*MIN_05_STEP_DURATION)
#define MAX_05_STEP_DURATION      (0.6)
#define MAX_STEP_DURATION         (2.*MAX_05_STEP_DURATION)

#define SIZE_OF_ONE_STEP  ((int16_t)(MAX_STEP_DURATION*SYNC_SMP_RATE) + 10) // 1.2s for 50 Hz + delta



// data for particle filter
typedef struct tPFData
{
  // base data for particle filter
  double DTWcombined; // combined DTW metric
  double timeStart; // time of step/stand start, sec or msec, (double/uint48)
  double timeEnd; // time of step/stand end, sec or msec, (double/uint48)
  float pEvent; // probability of correct of event detection
  float length; // time of step length, m
  float speed; // average speed during step, m/sec
  double heading; // step/stang heading, rad
  double headingInc; // step/stang heading increment, rad
  float magnStart; // magnetic vector magnitude at start point of step/stand
  float magnEnd; // magnetic vector magnitude at end point of step/stand
  float magnAverage; // average magnetic vector magnitude in step/stand, mTesla
  double magnWorstAccuracy; // worst accuracy of magnetic measurement during step/stand, mTesla
  float magnMeanAccuracy;  // mean accuracy of magnetic measurement during step/stand, mTesla
  double acc_span;          // maximal acceleration deviation during step/ m/s/s

  // Additional data for MFP
  eAttitudeState fAttitudeState; // flag of initial attitude of user validity
  double t_att; // time of current attitude data associating
  Matrix Cp2b; // Phone frame to User body frame transition matrix
  Matrix Cp2b1; // Phone frame to User body frame transition matrix
  double magnAvgPF[3]; // average magnetic vector magnitude in step/stand, mTesla
  double magnAvgBF[3]; // average magnetic vector magnitude in step/stand, mTesla
  double SyncMagnData[3][SIZE_OF_ONE_STEP]; // magnetic samples in phone frame (instrumental)
  double enuMagnData[3][SIZE_OF_ONE_STEP]; // magnetic samples in ENU frame without heading rotation
  double sample_time[SIZE_OF_ONE_STEP];
  long magnSampleCount;
  long SampleCount;

  Matrix Cp2ne_set[SIZE_OF_ONE_STEP]; // set of matrixes Phone->NE 

  double pitch;          // rad, phone pitch angle
  double roll;           // rad, phone roll angle

  //double SyncDataDelay; // 
  //double RawMagnData[3][SIZE_OF_ONE_STEP];
  //long cntRawMagnData;

  // debug data
#ifdef PDR_DEBUG
  double acc_mean_g;
#endif
}
tPFData;


#endif