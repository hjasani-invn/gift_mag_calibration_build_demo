/*****************************************************************************
 *    Copyright (c) 2013-2014 Spirit Corp.
******************************************************************************/
/**
 *   @project                HIOL
 *   @brief                  Dead Reconing realization for pedestian motion
 *   @file                   DR.h
 *   @author                 B.Oblakov, D.Churikov
 *   @date                   9.06.2013
 *   @version                1.0
 */
/*****************************************************************************/
#ifndef _DR_H_
#define _DR_H_

#include "matrix.h"
#include "Init.h"

#define DR_EST_DEBUG_OUT

#define N_ST                8
#define N_MEAS              3
#define N_MEAS_ANG          5

// data for extended Kalman Filter 
typedef struct
{
  double     tau;          // time between two consecutive IMU data [sec]
  Matrix     Xest;         // EKF: EKF state vector
  Matrix     Papost;       // EKF: covariance matrix
  Matrix     MQ;           // EKF: State noise matrix
} tEKF;

// DR - EKF class

class tDREKF
{
public:
  tDREKF(double sampleRate); //InitDR();
  ~tDREKF() {};
  void InitEKF(double SmpRate);
  void ResetEKF();
  void ResetEKF(double *accMean);
  double CalculateHeadingInc( double    *pAcclEq, // DREstimation
                              int32_t    nEq,
                              tVectGyr  *pVectGyr,
                              int16_t    cntGyr,
															bool CalledFromStep);
  double getMeanFi()    {return mean_fi;}
  double getMeanTeta()  {return (mean_teta - GL_Compensation);}
private:
  void IterationEKF (Matrix *MR, Matrix *ZMeas, int8_t  NumMeas);
  void RotateBF();
  double *ConvertVectorToBF (double *inVec, double *bfVec);
  void setMeasCov(bool FineMeasFlag);

private:
  tEKF       EKF;
  double Heading;
  bool fInitEKF;
  bool fInitStateVector;
  double GL_Compensation;
  double mean_fi, mean_teta; // pitch and roll in equiposition heading

   double r_fi, r_teta; // current fi and teta meas covariance
};


#endif // _DR_H_
