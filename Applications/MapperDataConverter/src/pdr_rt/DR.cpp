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

#include <math.h>
#include <cmath>
#include <cassert>

#include "DR.h"
#include "matrix.h"

#include <stdio.h>
#ifdef DR_EST_DEBUG_OUT
#include <stdio.h>
#include <fstream>
#endif

// mathematical defines
#ifndef M_PI
#define M_PI  (3.14159265358979323846)
#endif

#ifndef M_2PI
#define M_2PI (6.28318530717958647693)
#endif

#ifndef M_PI2
#define M_PI2 (1.57079632679489662)
#endif

// Gimbal lock compensation version

//#define GL_default
#define GL_new

//=========================================================================
// DR EKE parameters
#if 0
// source configuration (VDR based)
#define COV_YAW_DOT         (2.5e-1)
#define COV_PITCH           (2.5e-1)
#define COV_ROLL            (2.5e-1)
#define COV_PITCH_DOT       (2.5e-1)
#define COV_ROLL_DOT        (2.5e-1)
#define COV_BIAS            (1.2e-3)

#define Q_YAW_DOT           (5.0e-2)
#define Q_PITCH             (1.0e-2)
#define Q_ROLL              (1.0e-2)
#define Q_PITCH_DOT         (5.0e-2)
#define Q_ROLL_DOT          (5.0e-2)
#define Q_BIAS              (1.0e-9)

#define MR_GYRO             (1.0e-3)   // [rad/sec]
#define MR_TT               (2.e-1)//(1.e-1)   // [rad] // teta (pitch) meassurement covariance
#define MR_FI               (2.e-1)//(1.e-1)   // [rad] // fi (roll) meassurement covariance
#define MR_TT_ROUGH         MR_TT
#define MR_FI_ROUGH         MR_FI
#define MR_TT_FINE          MR_TT
#define MR_FI_FINE          MR_FI
#else
// PDR frendly configuration
#define COV_YAW_DOT         (2.5e-1)
#define COV_PITCH           (2.5e-1)
#define COV_ROLL            (2.5e-1)
#define COV_PITCH_DOT       (2.5e-2)
#define COV_ROLL_DOT        (2.5e-2)
#define COV_BIAS            (1.2e-3)

#define Q_YAW_DOT           (10.0e-1) 
#define Q_PITCH             (1.0e-2)
#define Q_ROLL              (1.0e-2)
#define Q_PITCH_DOT         (10.0e-1)
#define Q_ROLL_DOT          (10.0e-1)
#define Q_BIAS              (1.0e-9)

#define MR_GYRO             (1.0e-3)   // [rad/sec]
#define MR_TT_ROUGH         (20.e-1)//(1.e-1)   // [rad] // Can be increased up to 100
#define MR_FI_ROUGH         (20.e-1)//(1.e-1)   // [rad] // Can be increased up to 100
#define MR_TT_FINE         (1.e-1)   // [rad] // Please do not increase this value.
#define MR_FI_FINE         (1.e-1)   // [rad] // Please do not increase this value.
#endif

//=========================================================================
/// Attitude estimation for two angles - pitch and roll
///
/// @param[in]  accMean - pointer to values of accelerations
/// @param[out] *pTeta  - pointer to pitch angle
/// @param[out] *pFi    - pointer to roll angle
//=========================================================================
static void AttitudeEst(double *accMean, double *pTeta, double *pFi)
{
	double gm;
	double tetaMeas, fiMeas;

	gm = sqrt(accMean[0]*accMean[0] + accMean[1]*accMean[1] + accMean[2]*accMean[2]);

	tetaMeas = asin(accMean[0]/gm);
	fiMeas   = -atan2(accMean[1], -accMean[2]);

	*pTeta   = tetaMeas;
	*pFi     = fiMeas;
}

//=========================================================================
/// Constructor - Initialization DR module
tDREKF::tDREKF(double SmpRate)
{
	InitEKF(SmpRate);
}

//=========================================================================
/// Initialization DR module
///
/// @param[in] SmpRate     - data sample rate
//=========================================================================
void tDREKF::InitEKF(double SmpRate)
{
	double QYawDot,
		QPitch,
		QRoll,
		QPitchDot,
		QRollDot,
		QBias;

	EKF.tau = 1.0/SmpRate;

	QYawDot   = Q_YAW_DOT   * 200.0 * EKF.tau;
	QPitch    = Q_PITCH     * 200.0 * EKF.tau;
	QRoll     = Q_ROLL      * 200.0 * EKF.tau;
	QPitchDot = Q_PITCH_DOT * 200.0 * EKF.tau;
	QRollDot  = Q_ROLL_DOT  * 200.0 * EKF.tau;
	QBias     = Q_BIAS      * 200.0 * EKF.tau;

	// State noise matrix Q initialization
	CreateMatrix(&EKF.MQ, N_ST, N_ST);
	EKF.MQ.Matr[0][0]     = QYawDot   * QYawDot;    // Yaw dot for L-frame --> P-frame
	EKF.MQ.Matr[1][1]     = QPitch    * QPitch;     // Pitch for L-frame --> B-frame
	EKF.MQ.Matr[2][2]     = QRoll     * QRoll;      // Roll  for L-frame --> B-frame
	EKF.MQ.Matr[3][3]     = QPitchDot * QPitchDot;  // Pitch dot
	EKF.MQ.Matr[4][4]     = QRollDot  * QRollDot;   // Roll  dot
	EKF.MQ.Matr[5][5]     = QBias     * QBias;      // Gyro X bias
	EKF.MQ.Matr[6][6]     = QBias     * QBias;      // Gyro Y bias
	EKF.MQ.Matr[7][7]     = QBias     * QBias;      // Gyro Y bias

	ResetEKF();

	this->fInitEKF = true;
	this->Heading = 0;
	this->GL_Compensation = 0;
}

//=========================================================================
/// Reset State vector and covariance matrix
///
//=========================================================================
void tDREKF::ResetEKF()
{
	// XestSupp (Supplementary EKF state vector) initialization
	CreateMatrix(&EKF.Xest, N_ST, 1);
	EKF.Xest.Matr[0][0]   = 0.;                   // Yaw dot for L-frame --> P-frame
	EKF.Xest.Matr[1][0]   = 0.;                   // Pitch for L-frame --> B-frame
	EKF.Xest.Matr[2][0]   = -180.*M_PI/180.;      // Roll  for L-frame --> B-frame
	EKF.Xest.Matr[3][0]   = 0.;                   // Pitch dot
	EKF.Xest.Matr[4][0]   = 0.;                   // Roll  dot
	EKF.Xest.Matr[5][0]   = 0.;                   // Gyro X bias
	EKF.Xest.Matr[6][0]   = 0.;                   // Gyro Y bias
	EKF.Xest.Matr[7][0]   = 0.;                   // Gyro Z bias

	// Base EKF a posteriori covariance matrix initialization
	CreateMatrix(&EKF.Papost, N_ST, N_ST);
	EKF.Papost.Matr[0][0] = COV_YAW_DOT   * COV_YAW_DOT;    // Yaw dot for L-frame --> P-frame
	EKF.Papost.Matr[1][1] = COV_PITCH     * COV_PITCH;      // Pitch for L-frame --> B-frame
	EKF.Papost.Matr[2][2] = COV_ROLL      * COV_ROLL;       // Roll  for L-frame --> B-frame
	EKF.Papost.Matr[3][3] = COV_PITCH_DOT * COV_PITCH_DOT;  // Pitch dot
	EKF.Papost.Matr[4][4] = COV_ROLL_DOT  * COV_ROLL_DOT;   // Roll  dot
	EKF.Papost.Matr[5][5] = COV_BIAS      * COV_BIAS;       // Gyro X bias
	EKF.Papost.Matr[6][6] = COV_BIAS      * COV_BIAS;       // Gyro Y bias
	EKF.Papost.Matr[7][7] = COV_BIAS      * COV_BIAS;       // Gyro Z bias

	this->fInitStateVector = false;
	this->GL_Compensation = 0;

  setMeasCov(false);
}



//=========================================================================
/// Iteration of extended Kalman Filter
///
/// @param[in]     MR      - pointer to measurement noise matrix
/// @param[in]     ZMeas   - pointer to measurement vector
/// @param[in]     NumMeas - dimension of the measurement vector
/// @param[in/out] pEKF    - pointer to data for extended Kalman Filter 
//=========================================================================
void tDREKF::IterationEKF(Matrix *MR,Matrix *ZMeas, int8_t  NumMeas)
{
	Matrix Xpred, Fder, Zpred, Hder, MI, Papri, K; 
	Matrix K0, Kinv;
	int8_t i;
	double dZ;

	///< INITIALIZATION OF VECTORS AND MATRICES FOR EKF
	CreateMatrix(&Xpred, N_ST, 1);

	CreateMatrix(&Fder, N_ST, N_ST);
	for(i=0; i<N_ST; i++)
		Fder.Matr[i][i] = 1.0;

	CreateMatrix(&Zpred, NumMeas, 1);
	CreateMatrix(&Hder, NumMeas, N_ST);

	CreateMatrix(&MI, N_ST, N_ST);
	for (i=0; i<N_ST; i++)
	{
		MI.Matr[i][i] = 1.;
	}


	///< STATE VECTOR PREDICTION
	CopyMatrix( &Xpred ,&EKF.Xest);
	Xpred.Matr[1][0] = EKF.Xest.Matr[1][0] + EKF.Xest.Matr[3][0]*EKF.tau;
	Xpred.Matr[2][0] = EKF.Xest.Matr[2][0] + EKF.Xest.Matr[4][0]*EKF.tau;

	///< STATE VECTOR PARTIAL DERIVATIVES
	Fder.Matr[1][3] =  EKF.tau;
	Fder.Matr[2][4] =  EKF.tau;

	///< MEASUREMENT VECTOR PREDICTION
	Zpred.Matr[0][0] =  Xpred.Matr[4][0]                       - Xpred.Matr[0][0]*sin(Xpred.Matr[1][0])                       + Xpred.Matr[5][0];
	Zpred.Matr[1][0] =  Xpred.Matr[3][0]*cos(Xpred.Matr[2][0]) + Xpred.Matr[0][0]*sin(Xpred.Matr[2][0])*cos(Xpred.Matr[1][0]) + Xpred.Matr[6][0];
	Zpred.Matr[2][0] = -Xpred.Matr[3][0]*sin(Xpred.Matr[2][0]) + Xpred.Matr[0][0]*cos(Xpred.Matr[2][0])*cos(Xpred.Matr[1][0]) + Xpred.Matr[7][0];

	if (NumMeas == 5)
	{
		// --------------------------------
		Zpred.Matr[3][0] =  Xpred.Matr[1][0];

		dZ = Zpred.Matr[3][0] - ZMeas->Matr[3][0];
		if (dZ >= M_PI2)
		{
			Xpred.Matr[1][0] -= M_PI;
			Zpred.Matr[3][0]  = Xpred.Matr[1][0];
		}
		if (dZ < -M_PI2)
		{
			Xpred.Matr[1][0] += M_PI;
			Zpred.Matr[3][0]  = Xpred.Matr[1][0];
		}

		// --------------------------------
		Zpred.Matr[4][0] =  Xpred.Matr[2][0];

		dZ = Zpred.Matr[4][0] - ZMeas->Matr[4][0];
		if (dZ >= M_PI)
		{
			Xpred.Matr[2][0] -= M_2PI;
			Zpred.Matr[4][0]  = Xpred.Matr[2][0];
		}
		if (dZ < -M_PI)
		{
			Xpred.Matr[2][0] += M_2PI;
			Zpred.Matr[4][0]  = Xpred.Matr[2][0];
		}
	}

	///< MEASUREMENT VECTOR PARTIAL DERIVATIVES
	Hder.Matr[0][0] = -sin(Xpred.Matr[1][0]);
	Hder.Matr[0][1] = -Xpred.Matr[0][0]*cos(Xpred.Matr[1][0]);
	Hder.Matr[0][4] = 1.;
	Hder.Matr[0][5] = 1.;

	Hder.Matr[1][0] =  sin(Xpred.Matr[2][0])*cos(Xpred.Matr[1][0]);
	Hder.Matr[1][1] = -Xpred.Matr[0][0]*sin(Xpred.Matr[2][0])*sin(Xpred.Matr[1][0]);
	Hder.Matr[1][2] = -Xpred.Matr[3][0]*sin(Xpred.Matr[2][0]) + Xpred.Matr[0][0]*cos(Xpred.Matr[2][0])*cos(Xpred.Matr[1][0]);
	Hder.Matr[1][3] =  cos(Xpred.Matr[2][0]);
	Hder.Matr[1][6] = 1.;

	Hder.Matr[2][0] =  cos(Xpred.Matr[2][0])*cos(Xpred.Matr[1][0]);
	Hder.Matr[2][1] = -Xpred.Matr[0][0]*cos(Xpred.Matr[2][0])*sin(Xpred.Matr[1][0]);
	Hder.Matr[2][2] = -Xpred.Matr[3][0]*cos(Xpred.Matr[2][0]) - Xpred.Matr[0][0]*sin(Xpred.Matr[2][0])*cos(Xpred.Matr[1][0]);
	Hder.Matr[2][3] = -sin(Xpred.Matr[2][0]);
	Hder.Matr[2][7] = 1.;

	if (NumMeas == 5)
	{
		Hder.Matr[3][1] = 1.;
		Hder.Matr[4][2] = 1.;
	}

	///< EKF UPDATE
	// Papri = Fder*Papost*Fder' + MQ;
	MultMatrix(&Papri, &Fder, &EKF.Papost);
	MultMatrix2T_(&Papri, &Papri, &Fder);
	SumMatrix(&Papri, &Papri, &EKF.MQ);

	// K = Papri*Hder'/(Hder*Papri*Hder' + MR);
	MultMatrix2T_(&K, &Papri, &Hder);
	MultMatrix(&K0, &Hder, &Papri);
	MultMatrix2T_(&K0, &K0, &Hder);
	SumMatrix(&K0, &K0, MR);
	InverseMatrix(&Kinv, &K0);
	MultMatrix(&K, &K, &Kinv);

	// Xest = Xpred + K*(Zmeas - Zpred);
	DiffMatrix(&K0, ZMeas, &Zpred);
	MultMatrix(&K0, &K, &K0);
	SumMatrix(&EKF.Xest, &Xpred, &K0);

	// Papost = (MI - K*Hder)*Papri;
	MultMatrix(&K0, &K, &Hder);
	DiffMatrix(&MI, &MI, &K0);
	MultMatrix(&EKF.Papost, &MI, &Papri);

	return;
}

//=========================================================================
// The function estimates angular and orientation parameters of user motion
// param [in] pEKF         - DREKF data instance
// param [in] pAcclEq      - pointer to accelaration data in equiopoise point
// param [in] nEq          - 
// param [in] pVectGyr     - pointer to gyro data
// param [in] cntGyr       - count of gyro data
// param [in] RoughAccMeas  - rough accelerometer data flag; 1 - rough acc data, 0 - fine acc data
// return headingInc  - heading increment [rad]
//=========================================================================
double tDREKF::CalculateHeadingInc(double    *pAcclEq,
	                                 int32_t    nEq,
	                                 tVectGyr  *pVectGyr,
	                                 int16_t    cntGyr,
	                                 bool RoughAccMeas)
{
	int16_t i, n, n_meas;
	double Teta = 0, Fi = 0, HeadingPrev;
	Matrix MR, Zmeas;
	double vectAcc[3], vectGyr[3];

#ifdef DR_EST_DEBUG_OUT
	static int jj = 0, cnt = 0;
#endif

	HeadingPrev = this->Heading;

	if (this->fInitEKF == false)
	{
		assert(0);
		return 0;
	}

	if (this->fInitStateVector == false)
	{
		ResetEKF();
		if ((nEq >= 0) && (nEq < cntGyr))
		{// set start values
			ConvertVectorToBF(pAcclEq, vectAcc);
			AttitudeEst(vectAcc, &Teta, &Fi);
			EKF.Xest.Matr[1][0] = Teta;    // Pitch for L-frame --> B-frame
			EKF.Xest.Matr[2][0] = Fi;      // Roll  for L-frame --> B-frame
			this->fInitStateVector = true;
		}
	}

  // set accelerometer meassurement covariance
  setMeasCov(!RoughAccMeas);

	///< ATTITUDE ESTIMATION AND EKF
	for (n=0; n < cntGyr; n++)
	{
#ifdef DR_EST_DEBUG_OUT
		if (++cnt <= 1)  continue;

    if (n == nEq)
		{// prepare matrixes and obtain acc meassurements
			ConvertVectorToBF(pAcclEq, vectAcc);
			AttitudeEst(vectAcc, &Teta, &Fi);      // Attitude estimation
		}
#endif

		if (n == nEq)
		{// prepare matrixes and obtain acc meassurements
			n_meas = N_MEAS_ANG;
			CreateMatrix(&MR, n_meas, n_meas);  // Measurement noise matrix R initialization
			CreateMatrix(&Zmeas, n_meas, 1);        // Measurement vector initialization

			MR.Matr[3][3] = r_teta;             // Pitch [rad]^2
			MR.Matr[4][4] = r_fi;             // Roll [rad]^2

			ConvertVectorToBF(pAcclEq, vectAcc);
			AttitudeEst(vectAcc, &Teta, &Fi);      // Attitude estimation

			Zmeas.Matr[3][0]  = Teta;
			Zmeas.Matr[4][0]  = Fi;
		}
		else
		{// prepare matrixes for gyro only meassuremnts
			n_meas = N_MEAS;
			CreateMatrix(&MR, n_meas, n_meas);  // Measurement noise matrix R initialization
			CreateMatrix(&Zmeas, n_meas, 1);        // Measurement vector initialization
		}

		// gyro measurement noise matrix R
		MR.Matr[0][0] = MR_GYRO * MR_GYRO;         // gyroscopes omega X [rad/s]^2
		MR.Matr[1][1] = MR_GYRO * MR_GYRO;         // gyroscopes omega Y [rad/s]^2
		MR.Matr[2][2] = MR_GYRO * MR_GYRO;         // gyroscopes omega Z [rad/s]^2

		// gyro measurement vector
		ConvertVectorToBF(pVectGyr[n].data, vectGyr);
		for (i=0; i<N_MEAS; i++)
		{
			//Zmeas.Matr[i][0] = pVectGyr[n].data[i];
			Zmeas.Matr[i][0] = vectGyr[i];
		}

		IterationEKF(&MR, &Zmeas, n_meas);

		// ******** Gimbal lock compensation
		double test = fmod(this->EKF.Xest.Matr[1][0], M_PI);
		
		if ( test >= 0 )
		{
			if ( std::abs(test - M_PI/2) < 0.5 )
			{
				this->EKF.Xest.Matr[1][0] -= M_PI/2;
				this->GL_Compensation -= M_PI/2;
			}
		}

		else // angle is < 0
		{
			if ( std::abs(test + M_PI/2 ) < 0.5 )
			{
				this->EKF.Xest.Matr[1][0] += M_PI/2;
				this->GL_Compensation += M_PI/2;
			}
		}
		// ********

		if (n == nEq)
		{
			mean_fi = this->EKF.Xest.Matr[2][0];   //roll    // save from mean point // this data actual for steps only
			mean_teta = this->EKF.Xest.Matr[1][0]; // pitch  // save from mean point // this data actual for steps only
		}
		this->Heading = this->Heading + this->EKF.Xest.Matr[0][0] * this->EKF.tau;

#ifdef DR_EST_DEBUG_OUT
#if 0
		{ //  debug out
			static FILE * fStepsOut = 0;
			if (fStepsOut == 0)
			{
				fStepsOut = fopen("data_dbg_dr_hi.txt", "wt");
			}

			if (fStepsOut != 0)
				fprintf(fStepsOut, "%10d  %10.5f  %10.5f %10.5f  %10.5f %10.5f\n",
				jj,
				this->Heading,
				this->EKF.Xest.Matr[1][0],
				Teta,
				this->EKF.Xest.Matr[2][0],
				Fi);
		}
#endif
#endif

	}
	
	return (this->Heading - HeadingPrev);
}

double* tDREKF::ConvertVectorToBF (double *inVec, double *bfVec)
{
	// for my updated version of compensating GL
	if ( ( GL_Compensation < 0.1 ) && ( GL_Compensation > -0.1 ) )
	{
		bfVec[0] =  inVec[0];
		bfVec[1] =  inVec[1];
		bfVec[2] =  inVec[2];
	}

	if ( GL_Compensation >= 0.1 )
	{
		bfVec[0] =  inVec[2];
		bfVec[1] =  inVec[1];
		bfVec[2] = -inVec[0];

	}

	if ( GL_Compensation <= -0.1 )
	{
		bfVec[0] = -inVec[2];
		bfVec[1] =  inVec[1];
		bfVec[2] =  inVec[0];

	}
	return bfVec;
}

void tDREKF::setMeasCov(bool flag)
{
  if (flag)
  {
    r_fi = MR_FI_FINE;
    r_teta = MR_TT_FINE;
  }
  else
  {
    r_fi = MR_FI_ROUGH;
    r_teta = MR_TT_ROUGH;
  }
}