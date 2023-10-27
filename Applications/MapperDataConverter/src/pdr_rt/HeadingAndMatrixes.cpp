/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  calculation heading and matrixes
 *   @file                   HeadingAndMatrixes.cpp
 *   @author                 M. Zhokhova
 *   @date                   18.07.2013
 *   @version                1.0
 */
/*****************************************************************************/
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "PDR.h"
#include "Report.h"
//#include "DREstimation.h"

static tVectGyr vectGyr[SYNC_DATA];

static void getInitialHeading(tData* Data, tUserParams* UserParams);
static void getInitialData(tData* Data, int16_t idxStart, int16_t idxEnd, double estInterval,
                           double* timeInit, int16_t* idxInit, double* meanAcc, double* meanMagn);
static void getInitialDataRough(tData* Data, double estInterval, double* timeInit, int16_t* idxInit,
                                double* meanAcc, double* meanMagn);
static void Instrumental2PhoneMatrix(double* meanAcc, tUserParams* UserParams);
static void phone2NEDMatrix_v3(double* meanAcc, double* meanMagn, tStepAttitude* Att, tUserParams* UserParams,
                               bool CalledFromStand);
static void Phone2ENMatrix(double* meanAcc, Matrix &Cp2b);
static void getHeading(Matrix Cp2b, double* meanMagn, Matrix* Cb2n, double* psi);
static void getHeadingOfStep(tData* Data, tStepsInfo* Steps, int8_t idxStart, int8_t cntSteps,
                             tStepAttitude* prevAtt, tUserParams* UserParams, tDREKF *pDREKF);
static int8_t onStepWithAttitudeUpdate(tData* Data, tSensorParams* SensorParams, tStepParams* step,
                                       tStepAttitude* attData, tUserParams* UserParamst, tDREKF *pDREKF);
static void onStepWithoutAttitudeUpdate(tData* Data, tSensorParams *SensorParams, tStepParams* step,
                                        tStepAttitude* prevAtt, tStepAttitude* AttUserParams,
                                        tStepAttitude* attData, tDREKF *pDREKF);
static int8_t onStandWithAttitudeUpdate(tData* Data, tStepParams* step, tStepAttitude* attData,
                                        tUserParams* UserParams, tDREKF *pDREKF);
static void onStandWithoutAttitudeUpdate(tData* Data, tStepParams* step, tStepAttitude* prevAtt,
                                         tStepAttitude* AttUserParams, tStepAttitude* attData, tDREKF *pDREKF);
static int8_t getInitialHeading_Fine(tData* Data, tStepParams* step, tStepAttitude* attData, tUserParams* UserParams, double meanAcc[3]);
static void rotateCb2n(Matrix* Cb2n, double headingInc);
static void getStepData_v2(tData* Data, tSensorParams* SensorParams, tStepParams* step,
                           tVectGyr* GyrData, int16_t* cntGyrData, double* meanAcc,
                           double* meanMagn, double* time, int16_t* idx);
static void getDeltaHeading_v2(Matrix* Cp2b,	tData* Data, tStepParams* step, int16_t idxC, double* headingInc);
static void ConvertMagneticData(Matrix *Cp2b, tData* Data, tStepParams* Step, int16_t idxC);
static void copyAttToAtt(tStepAttitude* attDst, tStepAttitude* attSrc);

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
void GetHeadingAndMatrixes(tData* Data,
                           tStepsInfo* Steps,
                           int8_t idxStart,
                           int8_t cntSteps,
                           tStepAttitude* prevAtt,
                           int8_t isInitStep,
                           tUserParams* UserParams,
                           tDREKF* pDREKF)
{
	if( isInitStep == INIT_HEADING )
		getInitialHeading(Data, UserParams);
	else
		getHeadingOfStep(Data, Steps, idxStart, cntSteps, prevAtt, UserParams, pDREKF);
	return;
}
//==============================================================================
/// Calculation heading for virtual steps and static intervals
/// @param[in]     Data       - pointer to data
/// @param[in]     VirtSteps      - pointer to steps inforamtion
/// @param[in]     cntVirtSteps   - counter of step
/// @param[in/out] prevAtt    - pointer to previous attitude information
/// @param[in/out]    pDREKF  - DREKF instance
//==============================================================================
void VirtStepsDeltaHeadings(tData* Data,
								tStepsInfo* VirtSteps,
								int8_t cntVirtSteps,
								tDREKF* pDREKF)
{
	int16_t cntGyr;
	tStepParams* step;

	for ( int i = 0; i < cntVirtSteps; i++ )
	{
		step = &(VirtSteps->spData[i]);
		cntGyr = step->idxEnd - step->idxStart + 1;
		for ( int j = step->idxStart; j <= step->idxEnd; j++ )
		{
			for ( int k = 0; k < 3; k++ )
			{
				vectGyr[j-step->idxStart].data[k] = Data->gyrData.syncData[k][j];
			}
		}

		VirtSteps->attData[i].headingInc = pDREKF->CalculateHeadingInc( 0, -1, vectGyr, cntGyr, true );
	}
}
//=========================================================================================
/// Initialization heading and orientation matrices: determinates motionless interval,
/// calculates heading and orientation matrices for this interval
///
/// @param[in]  Data       - pointer to data
/// @param[out] UserParams - pointer to user parameters structure with initial orientation
//=========================================================================================
static void getInitialHeading(tData* Data, tUserParams* UserParams)
{
	double estInterval = EST_INTERVAL; // estimation interval duration [s]
	double timeInit;
	double meanAcc[3], meanMagn[3];
	int16_t idxInit;

	// initial time determination
	getInitialDataRough(Data, estInterval, &timeInit, &idxInit, meanAcc, meanMagn);
	if( timeInit == 0. )
	{
		memset(&UserParams->Att, 0, sizeof(tStepAttitude));
    UserParams->Att.fAttitudeState = eAS_Uncknown;
	}
	else
	{
		phone2NEDMatrix_v3(meanAcc, meanMagn, &UserParams->Att, UserParams, 1);
    Phone2ENMatrix(meanAcc, UserParams->Att.Cp2b1);
    UserParams->Att.index = idxInit;
		UserParams->Att.time = timeInit;
		UserParams->Att.headingInc = 0.;
		UserParams->Att.fAttitudeState = eAS_Rough; // rough initialization flag
	}

	return;
}

//===========================================================================
/// Determines time interval suitable for initialization orientation and
/// calculates average vectors of sensors for this interval
///
/// @param[in]  Data        - pointer to data
/// @param[in]  idxStart    - start index for data
/// @param[in]  idxEnd      - end index for data
/// @param[in]  estInterval - estimation interval duration
/// @param[out] timeInit    - time in measurement arrays for which calculate
///                           average of vectors
/// @param[out] idxInit     - index of measuring sample in measurement arrays
///                           for which calculate average of vectors
/// @param[out] meanAcc     - pointer to means of acceleration vectors
/// @param[out] meanMagn    - pointer to means of magnetic vectors
/// @return Zero if successful
//===========================================================================
static void getInitialData(tData* Data,
						   int16_t idxStart,
						   int16_t idxEnd,
						   double estInterval,
						   double* timeInit,
						   int16_t* idxInit,
						   double* meanAcc,
						   double* meanMagn)
{
#pragma warning(push)
#pragma warning (disable: 6385) // to avoid warning
	double tau;
	double meanMagnitudeAcc, meanMagnitudeGyr; // mean of acceleration/data from gyroscopes magnitude
	double sigmaMagnitudeAcc, sigmaMagnitudeGyr; // standard deviation of acceleration/data from gyroscopes magnitude
	int16_t idxTau, i, j;
	int8_t k;
	double tmpMeanAcc[3], tmpMeanMagn[3];
	double magnitudeSyncGyr[SIZE_OF_ONE_STEP]; // gyro vector

	sigmaMagnitudeAcc = 0.0;
	sigmaMagnitudeGyr = 0.0;

	for( i = 0; i < 3; i++ )
		tmpMeanAcc[i] = tmpMeanMagn[i] = 0.;

	tau = estInterval/2.;
	idxTau = (int16_t)(estInterval*Data->sampleRate);

	if( idxTau < 10 ) // Too small sample rate or estimation interval. No heading and attitude initialization.
	{
	    *idxInit = 0;
	    *timeInit = 0.;
		return;
	}

	for( i = idxStart; i <= idxEnd; i++ )
	{
		magnitudeSyncGyr[i-idxStart] = sqrt(Data->gyrData.syncData[0][i]*Data->gyrData.syncData[0][i] +
			Data->gyrData.syncData[1][i]*Data->gyrData.syncData[1][i] +
			Data->gyrData.syncData[2][i]*Data->gyrData.syncData[2][i]);
	}

	for( i = idxStart; i <= (idxEnd - idxTau); i++ )
	{
		meanMagnitudeAcc = 0.; meanMagnitudeGyr = 0.;
		for( j = i; j <= (i+idxTau); j++ )
		{
			meanMagnitudeAcc += Data->magnitudeData.magnitudeSync[j];
			meanMagnitudeGyr += magnitudeSyncGyr[j-idxStart];
		}
		meanMagnitudeAcc /= (j-i); // == idxTau + 1
		meanMagnitudeGyr /= (j-i);
		sigmaMagnitudeAcc = 0.; sigmaMagnitudeGyr = 0.;
		for( j = i; j <= (i+idxTau); j++ )
		{
			sigmaMagnitudeAcc += (Data->magnitudeData.magnitudeSync[j] - meanMagnitudeAcc)*
				(Data->magnitudeData.magnitudeSync[j] - meanMagnitudeAcc);
			sigmaMagnitudeGyr += (magnitudeSyncGyr[j-idxStart] - meanMagnitudeGyr)*
				(magnitudeSyncGyr[j-idxStart] - meanMagnitudeGyr);
		}
		sigmaMagnitudeAcc /= (j-i-1); // == idxTau
		sigmaMagnitudeGyr /= (j-i-1);
		sigmaMagnitudeAcc = sqrt(sigmaMagnitudeAcc);
		sigmaMagnitudeGyr = sqrt(sigmaMagnitudeGyr);

		if( (sigmaMagnitudeAcc < MAX_SIGMA_ACC) && (sigmaMagnitudeGyr < MAX_SIGMA_GYR) )
		{
			*idxInit = i + (int16_t)floor(idxTau/2.);
			*timeInit = Data->timeSync[*idxInit];
			for( j = i; j <= (i+idxTau); j++ )
			{
				for( k = 0; k < 3; k++ )
				{
					tmpMeanAcc[k] += Data->accData.syncData[k][j];
					tmpMeanMagn[k] += Data->magnData.syncData[k][j];
				}
			}
			for( k = 0; k < 3; k++ )
			{
				meanAcc[k] = tmpMeanAcc[k]/(j-i);
				meanMagn[k] = tmpMeanMagn[k]/(j-i);
			}
			return;
		}
	}

	// Can not init heading
	*idxInit = 0;
	*timeInit = 0.;
	return;
#pragma warning(pop)
}

//=============================================================================
/// Determines time interval suitable for initialization rough orientation and
/// calculates average vectors of sensors for this interval
///
/// @param[in]  Data        - pointer to data
/// @param[in]  estInterval - estimation interval duration
/// @param[out] timeInit    - time in measurement arrays for which calculate
///                           average of vectors
/// @param[out] idxInit     - index of measuring sample in measurement arrays
///                           for which calculate average of vectors
/// @param[out] meanAcc     - pointer to means of acceleration vectors
/// @param[out] meanMagn    - pointer to means of magnetic vectors
/// @return Zero if successful
//=============================================================================
static void getInitialDataRough(tData* Data,
								double estInterval,
							    double* timeInit,
								int16_t* idxInit,
								double* meanAcc,
								double* meanMagn)
{
	double meanMagnitudeAcc, meanMagnitudeGyr; // mean of acceleration/data from gyroscopes magnitude
	double sigmaMagnitudeAcc, sigmaMagnitudeGyr; // standard deviation of acceleration/data from gyroscopes magnitude
	int16_t idxTau, i;
	int8_t k;
	double tmpMeanAcc[3], tmpMeanMagn[3];
	double magnitudeSyncGyr[SIZE_ROUGH_INTERVAL+1]; // gyro vector

	for( i = 0; i < 3; i++ )
		tmpMeanAcc[i] = tmpMeanMagn[i] = 0.;

	idxTau = (int16_t)(estInterval*Data->sampleRate);

	if( idxTau < 10 ) // Too small sample rate or estimation interval. No heading and attitude initialization.
	{
		*idxInit = 0;
		*timeInit = 0.;
		return;
	}

	for( i = 0; i <= idxTau; i++ )
	{
		magnitudeSyncGyr[i] = sqrt(Data->gyrData.syncData[0][i]*Data->gyrData.syncData[0][i] +
			Data->gyrData.syncData[1][i]*Data->gyrData.syncData[1][i] +
			Data->gyrData.syncData[2][i]*Data->gyrData.syncData[2][i]);
	}
	meanMagnitudeAcc = 0.; meanMagnitudeGyr = 0.;
	for( i = 0; i <= idxTau; i++ )
	{
		meanMagnitudeAcc += Data->magnitudeData.magnitudeSync[i];
		meanMagnitudeGyr += magnitudeSyncGyr[i];
	}
	meanMagnitudeAcc /= i;
	meanMagnitudeGyr /= i;
	sigmaMagnitudeAcc = 0.; sigmaMagnitudeGyr = 0.;
	for( i = 0; i <= idxTau; i++ )
	{
		sigmaMagnitudeAcc += (Data->magnitudeData.magnitudeSync[i] - meanMagnitudeAcc)*
			(Data->magnitudeData.magnitudeSync[i] - meanMagnitudeAcc);
		sigmaMagnitudeGyr += (magnitudeSyncGyr[i] - meanMagnitudeGyr)*
			(magnitudeSyncGyr[i] - meanMagnitudeGyr);
	}
	sigmaMagnitudeAcc /= (i-1);
	sigmaMagnitudeGyr /= (i-1);
	sigmaMagnitudeAcc = sqrt(sigmaMagnitudeAcc);
	sigmaMagnitudeGyr = sqrt(sigmaMagnitudeGyr);
	if( (sigmaMagnitudeAcc < MAX_SIGMA_ACC_ROUGH) && (sigmaMagnitudeGyr < MAX_SIGMA_GYR_ROUGH) )
	{
		*idxInit = (int16_t)floor(idxTau/2.);
		*timeInit = Data->timeSync[*idxInit];
		for( i = 0; i <= idxTau; i++ )
		{
			for( k = 0; k < 3; k++ )
			{
				tmpMeanAcc[k] += Data->accData.syncData[k][i];
				tmpMeanMagn[k] += Data->magnData.syncData[k][i];
			}
		}
		for( k = 0; k < 3; k++ )
		{
			meanAcc[k] = tmpMeanAcc[k]/i;
			meanMagn[k] = tmpMeanMagn[k]/i;
		}
		return;
	}
	// Can not init heading and orientation.
	*idxInit = 0;
	*timeInit = 0.;
}
//===========================================================================
/// Calculates instrumental to phone coordinate system conversion matrix
///
/// @param[in] meanAcc  - pointer to means of acceleration vectors
/// @param[out] UserParams - pointer to user parameters structure
//===========================================================================
static void Instrumental2PhoneMatrix(double* meanAcc, tUserParams* UserParams)
{
	double incl = M_PI/4;

	double y_plus[2], y_minus[2], x_plus[2], x_minus[2];
	float weight_y_plus = 1.0f;
	float weight_y_minus = 0.5f;
	float weight_x_plus = 0.7f;
	float weight_x_minus = 0.7f;
	float y_minus_threshold = 10.0f;
	float x_minus_threshold = 5.0f;
	float x_plus_threshold = 5.0f;
	double y1, y2, x1, x2;
	eI2PState prev_I2Pstate = UserParams->fI2PState;
	double heading_compensation = 0;

	y_plus[0] = 0;
	y_plus[1] = MEAN_G * sin(incl);
	y_minus[0] = 0;
	y_minus[1] = -MEAN_G * sin(incl);
	x_plus[0] = MEAN_G * sin(incl);
	x_plus[1] = 0;
	x_minus[0] = -MEAN_G * sin(incl);
	x_minus[1] = 0;

	y1 = weight_y_plus * y_plus[1] * meanAcc[1]; // + 0 * y_plus[0] (a scalar product result)
	y2 = weight_y_minus * y_minus[1] * meanAcc[1];
	x1 = weight_x_plus * x_plus[0] * meanAcc[0];
	x2 = weight_x_minus * x_minus[0] * meanAcc[0];

	if ( ( y1 >= y2 ) && ( y1 >= x1 ) && ( y1 >= x2 ) )
	{
		UserParams->fI2PState = I2P_yplus;
	}
	else if ( (x1 >= y2 ) && (x1 >= x2) )
	{
		if ( x1 < x_plus_threshold )
		{
			UserParams->fI2PState = I2P_yplus;
		}
		else
		{
			UserParams->fI2PState = I2P_xplus;
		}
	}
	else if ( x2 >= y2 )
	{
		if ( x2 < x_minus_threshold )
		{
			UserParams->fI2PState = I2P_yplus;
		}
		else
		{
			UserParams->fI2PState = I2P_xminus;
		}
	}
	else
	{
		if ( y2 < y_minus_threshold )
		{
			UserParams->fI2PState = I2P_yplus;
		}
		else
		{
			UserParams->fI2PState = I2P_yminus;
		}
	}

	if ( prev_I2Pstate != I2P_Unknown )
	heading_compensation = (UserParams->fI2PState - prev_I2Pstate) * M_PI/2;

	// conversion to traditional system
	// Ci2p - transition matrix: instrumental (measurement) android frame to traditional body frame
	if  ( UserParams->fI2PState == I2P_yplus )
	{
		UserParams->Ci2p.Matr[0][0] = 0.; UserParams->Ci2p.Matr[0][1] = 1.;
		UserParams->Ci2p.Matr[1][0] = 1.; UserParams->Ci2p.Matr[1][1] = 0.;
	}
	else if ( UserParams->fI2PState == I2P_xplus )
	{
		UserParams->Ci2p.Matr[0][0] = 1.; UserParams->Ci2p.Matr[0][1] = 0.;
		UserParams->Ci2p.Matr[1][0] = 0.; UserParams->Ci2p.Matr[1][1] = -1.;
	}
	else if ( UserParams->fI2PState == I2P_xminus )
	{
		UserParams->Ci2p.Matr[0][0] = -1.; UserParams->Ci2p.Matr[0][1] = 0.;
		UserParams->Ci2p.Matr[1][0] = 0.; UserParams->Ci2p.Matr[1][1] = 1.;
	}
	else if ( UserParams->fI2PState == I2P_yminus )
	{
		UserParams->Ci2p.Matr[0][0] = 0.; UserParams->Ci2p.Matr[0][1] = -1.;
		UserParams->Ci2p.Matr[1][0] = -1.; UserParams->Ci2p.Matr[1][1] = 0.;
	}

	UserParams->Ci2p.Matr[0][2] = 0.;
	UserParams->Ci2p.Matr[1][2] = 0.;
	UserParams->Ci2p.Matr[2][0] = 0.;
	UserParams->Ci2p.Matr[2][1] = 0.;
	UserParams->Ci2p.Matr[2][2] = -1.;

	if ( heading_compensation != 0 )
	{
		UserParams->HeadingCompensation = heading_compensation;
	}
}

//===========================================================================
/// Calculates orientation matrix and Euler angles by specified acceleration
/// (current g-vector estimation) and magnetic vector
///
/// @param[in] meanAcc  - pointer to means of acceleration vectors
/// @param[in] meanMagn - pointer to means of magnetic vectors
/// @param[out] Att     - pointer to attitude of user
/// @param[out] UserParams - pointer to user parameters structure
/// @param[in] CalledFromStand - flag, shows if the function was called from stand of step
//===========================================================================
static void phone2NEDMatrix_v3(double* meanAcc,
							   double* meanMagn,
							   tStepAttitude* Att,
								 tUserParams* UserParams,
								 bool CalledFromStand)
{
	Matrix meanAccMtx, GP, C1, C2, C3;
	double sinFi, cosFi, sinTeta, cosTeta;

	CreateMatrix(&meanAccMtx, 3, 1);

#if(1) // on/off instrumental frame autorotation
	if ( CalledFromStand == 1 )  // updating Ci2p matrix
	{
		Instrumental2PhoneMatrix(meanAcc, UserParams);
	}
#endif

	meanAccMtx.Matr[0][0] = -meanAcc[0];
	meanAccMtx.Matr[1][0] = -meanAcc[1];
	meanAccMtx.Matr[2][0] = -meanAcc[2];

	MultMatrix(&GP, &UserParams->Ci2p, &meanAccMtx);

	sinFi = GP.Matr[1][0]/sqrt(GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);
	cosFi = GP.Matr[2][0]/sqrt(GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);
	Att->fi = atan2(GP.Matr[1][0], GP.Matr[2][0]);

	CreateMatrix(&C1, 3, 3);

	C1.Matr[0][0] = 1.; C1.Matr[0][1] = 0.; C1.Matr[0][2] = 0.;
	C1.Matr[1][0] = 0.; C1.Matr[1][1] = cosFi; C1.Matr[1][2] = -sinFi;
	C1.Matr[2][0] = 0.; C1.Matr[2][1] = sinFi; C1.Matr[2][2] = cosFi;

	sinTeta = GP.Matr[0][0]/sqrt(GP.Matr[0][0]*GP.Matr[0][0] + GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);
	cosTeta = sqrt(GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0])/
		sqrt(GP.Matr[0][0]*GP.Matr[0][0] + GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);
	Att->teta = atan2(GP.Matr[0][0], sqrt(GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]));
        //asin(sinTeta);
	// !!! different from PDR demo ( Att->teta = asin(sinTeta); there)

	CreateMatrix(&C2, 3, 3);

	C2.Matr[0][0] = cosTeta; C2.Matr[0][1] = 0.; C2.Matr[0][2] = -sinTeta;
	C2.Matr[1][0] = 0.; C2.Matr[1][1] = 1.; C2.Matr[1][2] = 0.;
	C2.Matr[2][0] = sinTeta; C2.Matr[2][1] = 0.; C2.Matr[2][2] = cosTeta;

	MultMatrix(&C3, &C2, &C1);
	MultMatrix(&Att->Cp2b, &C3, &UserParams->Ci2p);

	getHeading(Att->Cp2b, meanMagn, &Att->Cb2n, &Att->psi);
}

//===========================================================================
/// Calculates orientation matrix from phone(instrumental) frame to current
//  Earth level frame (EN = ENU without heading rotation)
///
/// @param[in] meanAcc  - pointer to means of acceleration vectors
/// @param[in] Cp2b - calculated matrix
//===========================================================================
static void Phone2ENMatrix(double* meanAcc, Matrix &Cp2b)
{
	Matrix GP, Rx, Ry, Rz3, GP1;
	double sinFi, cosFi, sinTeta, cosTeta;

	// !!! TODO gimbal lock compensation !!!

	CreateMatrix(&GP, 3,1);
	GP.Matr[0][0] = meanAcc[0];
	GP.Matr[1][0] = meanAcc[1];
	GP.Matr[2][0] = meanAcc[2];

	double gm = sqrt(GP.Matr[0][0]*GP.Matr[0][0] + GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);

	double fiMeas   = -atan2(GP.Matr[0][0], GP.Matr[2][0]);
	double tetaMeas = asin(GP.Matr[1][0]/gm);

	sinTeta = sin(tetaMeas);//GP.Matr[1][0]/sqrt(GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);
	cosTeta = cos(tetaMeas);//GP.Matr[2][0]/sqrt(GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);

	CreateMatrix(&Rx, 3, 3);

	Rx.Matr[0][0] = 1.;     Rx.Matr[0][1] = 0.;         Rx.Matr[0][2] = 0.;
	Rx.Matr[1][0] = 0.;     Rx.Matr[1][1] = cosTeta;    Rx.Matr[1][2] = -sinTeta;
	Rx.Matr[2][0] = 0.;     Rx.Matr[2][1] = sinTeta;    Rx.Matr[2][2] = cosTeta;

	sinFi = sin(fiMeas);
	cosFi = cos(fiMeas);

	//sinTeta = GP.Matr[0][0]/sqrt(GP.Matr[0][0]*GP.Matr[0][0] + GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);
	//cosTeta = sqrt(GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0])/
	//	sqrt(GP.Matr[0][0]*GP.Matr[0][0] + GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]);
	//Att->teta = atan2(GP.Matr[0][0], sqrt(GP.Matr[1][0]*GP.Matr[1][0] + GP.Matr[2][0]*GP.Matr[2][0]));

	CreateMatrix(&Ry, 3, 3);

	Ry.Matr[0][0] = cosFi;    Ry.Matr[0][1] = 0.;     Ry.Matr[0][2] = sinFi;
	Ry.Matr[1][0] = 0.;       Ry.Matr[1][1] = 1.;     Ry.Matr[1][2] = 0.;
	Ry.Matr[2][0] = -sinFi;   Ry.Matr[2][1] = 0.;     Ry.Matr[2][2] = cosFi;

	MultMatrix(&Cp2b, &Rx, &Ry);
	MultMatrix(&GP1, &Cp2b, &GP);
}

//================================================================================
/// Calculates heading and user body to NED transition matrix by specified
/// phone frame to user body frame transition matrix and magnetic vector.
/// Magnetic declination and inclination is not accounted in current version
///
/// @param[in]  Cp2b     - phone frame to user body frame transition matrix
/// @param[in]  meanMagn - pointer to means of magnetic vectors
/// @param[out] Cb2n     - pointer to user body frame to NED transition matrix
/// @param[out] psi      - pointer to current heading
//================================================================================
static void getHeading(Matrix Cp2b, double* meanMagn, Matrix* Cb2n, double* psi)
{
	Matrix meanMagnMrx, MB;
	double sinPsi, cosPsi;

	CreateMatrix(&meanMagnMrx, 3, 1);

	meanMagnMrx.Matr[0][0] = meanMagn[0];
	meanMagnMrx.Matr[1][0] = meanMagn[1];
	meanMagnMrx.Matr[2][0] = meanMagn[2];

	MultMatrix(&MB, &Cp2b, &meanMagnMrx);

	*psi = atan2(MB.Matr[1][0], MB.Matr[0][0]);
	sinPsi = MB.Matr[1][0]/sqrt(MB.Matr[0][0]*MB.Matr[0][0] + MB.Matr[1][0]*MB.Matr[1][0]);
	cosPsi = MB.Matr[0][0]/sqrt(MB.Matr[0][0]*MB.Matr[0][0] + MB.Matr[1][0]*MB.Matr[1][0]);

	CreateMatrix(Cb2n, 3, 3);

	Cb2n->Matr[0][0] = cosPsi;  Cb2n->Matr[0][1] = sinPsi; Cb2n->Matr[0][2] = 0.;
	Cb2n->Matr[1][0] = -sinPsi; Cb2n->Matr[1][1] = cosPsi; Cb2n->Matr[1][2] = 0.;
	Cb2n->Matr[2][0] = 0.;      Cb2n->Matr[2][1] = 0.;     Cb2n->Matr[2][2] = 1.;

	return;
}

//==============================================================================
/// Calculation heading and orientation matrices for steps and for static
/// intervals
///
/// @param[in]     Data       - pointer to data
/// @param[in]     Steps      - pointer to steps inforamtion
/// @param[in]     idxStart   - index of start step
/// @param[in]     cntSteps   - counter of step
/// @param[in/out] prevAtt    - pointer to previous attitude information
/// @param[in]     UserParams - pointer to user parameters structure with
///                 initial orientation
//==============================================================================
static void getHeadingOfStep(tData* Data,
							 tStepsInfo* Steps,
							 int8_t idxStart,
							 int8_t cntSteps,
						     tStepAttitude* prevAtt,
							 tUserParams* UserParams,
               tDREKF* pDREKF)
{
	int16_t i;

	for( i = 0; i < cntSteps; i++ )
	{
		if( Steps->spData[i+idxStart].pEvent != 0 )
		{
			// attitude update is required
			if( (UserParams->Att.fAttitudeState == eAS_Uncknown) || UserParams->AttUpdateFlag )
			{
				if( onStepWithAttitudeUpdate(Data, &UserParams->SensorParams, &Steps->spData[i+idxStart], &Steps->attData[i+idxStart], UserParams, pDREKF) )
				{ // attitude update is possible
					copyAttToAtt(&UserParams->Att, &Steps->attData[i+idxStart]);
					UserParams->Att.fAttitudeState = eAS_Fine;
				}
				else // attitude update is not possible
					onStepWithoutAttitudeUpdate(Data, &UserParams->SensorParams, &Steps->spData[i+idxStart], prevAtt, &UserParams->Att, &Steps->attData[i+idxStart], pDREKF);
			}
			else // attitude update is NOT required
				onStepWithoutAttitudeUpdate(Data, &UserParams->SensorParams, &Steps->spData[i+idxStart], prevAtt, &UserParams->Att, &Steps->attData[i+idxStart], pDREKF);
		}
		else // it is a stand
		{
			if( UserParams->AttUpdateFlag || ( UserParams->Att.fAttitudeState != eAS_Fine ) ) // attitude update is required
			{
				if( onStandWithAttitudeUpdate(Data, &Steps->spData[i+idxStart], &Steps->attData[i+idxStart], UserParams, pDREKF) ) // attitude update is possible
				{
					copyAttToAtt(&UserParams->Att, &Steps->attData[i+idxStart]);
					UserParams->Att.fAttitudeState = eAS_Fine;
					if (!UserParams->SensorParams.IsUpdate)
					{
						UserParams->SensorParams.acc_mean_g = Steps->attData[i+idxStart].acc_mean_g;
						UserParams->SensorParams.acc_scale_factor = Steps->attData[i+idxStart].acc_scale_factor;
						UserParams->SensorParams.IsUpdate = 1;
					}
				}
				else // attitude update is not possible
				{
					onStandWithoutAttitudeUpdate(Data, &Steps->spData[i+idxStart], prevAtt, &UserParams->Att, &Steps->attData[i+idxStart], pDREKF);
				}
			}
			else // attitude update is NOT required
			{
				onStandWithoutAttitudeUpdate(Data, &Steps->spData[i+idxStart], prevAtt, &UserParams->Att, &Steps->attData[i+idxStart], pDREKF);
			}
		}
		copyAttToAtt(prevAtt, &Steps->attData[i+idxStart]);
	}
}

//===============================================================================
/// Calculation heading and orientation matrices for steps with attitude update
///
/// @param[in]  Data    - pointer to structure with sensor meassurements
/// @param[in]  SensorParams  - pointer to sensor params data
/// @param[in]  step    - pointer to steps inforamtion
/// @param[out] attData - pointer to attitude data
/// @return one if successful
//===============================================================================
static int8_t onStepWithAttitudeUpdate(tData* Data,
                                       tSensorParams* SensorParams,
                                       tStepParams* step,
                                       tStepAttitude* attData,
                                       tUserParams* UserParams,
                                       tDREKF *pDREKF)
{
  int16_t cntGyr, idx;
  double meanAcc[3]/*, meanMagn[3]*/; // mean of acceleration/data from magnetic compass magnitude
  double time;
  int8_t ret_param;

  int16_t i;

  getStepData_v2(Data, SensorParams, step, vectGyr, &cntGyr, meanAcc, step->meanMagnetic, &time, &idx);
  ReportPDR::AttDebugReport(step, time, meanAcc, step->meanMagnetic);
  if( time != 0 )
  {
    attData->index = idx;
    attData->time = time;
    attData->fAttitudeState = eAS_Fine;

		for( i = step->idxStart; i < step->idxEnd; ++i )
		{
			if( Data->timeSync[i] >= attData->time )
				break;
		}
		    
    attData->headingInc = pDREKF->CalculateHeadingInc( meanAcc, i - (step->idxStart), vectGyr, cntGyr, true);

#if 1 // use gyrovertical for attitude calculation
    double gyro_g0[3], tmp_g;
    gyro_g0[0] = sin(pDREKF->getMeanTeta());
    tmp_g = sqrt(1-gyro_g0[0]*gyro_g0[0]);
    gyro_g0[1] = -tmp_g*sin(pDREKF->getMeanFi());
    gyro_g0[2] = -tmp_g*cos(pDREKF->getMeanFi());
 
    phone2NEDMatrix_v3(gyro_g0, step->meanMagnetic, attData, UserParams, 0);
    Phone2ENMatrix(gyro_g0, attData->Cp2b1);
#else
    phone2NEDMatrix_v3(meanAcc, step->meanMagnetic, attData, UserParams, 0);
    Phone2ENMatrix(meanAcc, attData->Cp2b1);
#endif

    ConvertMagneticData(&attData->Cp2b1, Data, step, i);

    ret_param = 1;
  }
  else
  {
    memset( attData, 0, sizeof(tStepAttitude));
    //attData->headingInc = 0;
    ret_param = 0; // StepWitoutAttitude update will be used for delta heading calculation
  }

  return ret_param;
}

//=================================================================================
/// Calculation heading and orientation matrices for steps without attitude update
///
/// @param[in]  Data          - pointer to structure with sensor meassurements
/// @param[in]  SensorParams  - pointer to sensor params data
/// @param[in]  step          - pointer to step inforamtion
/// @param[in]  prevAtt       - pointer to previuos attitude inforamtion
/// @param[out] AttUserParams - pointer to attitude structure of user parameters
/// @param[out] attData       - pointer to attitude data
//=================================================================================
static void onStepWithoutAttitudeUpdate(tData* Data,
                                        tSensorParams *SensorParams,
                                        tStepParams* step,
                                        tStepAttitude* prevAtt,
                                        tStepAttitude* AttUserParams,
                                        tStepAttitude* attData,
                                        tDREKF *pDREKF)
{
#pragma warning(push)
#pragma warning(disable: 6385)
  int16_t cntGyr, idx, j, k;
  double meanAcc[3]/*, meanMagn[3]*/; // mean of acceleration/data from magnetic compass magnitude
  double time;

  if( prevAtt->fAttitudeState != eAS_Uncknown )
  {
    copyAttToAtt(attData, prevAtt);
    attData->fAttitudeState = eAS_Rough;
  }
  else if( AttUserParams->fAttitudeState != eAS_Uncknown)
  {
    copyAttToAtt(attData, AttUserParams);
    attData->fAttitudeState = eAS_Rough;
  }
  else
    memset(attData, 0, sizeof(tStepAttitude));
  if( attData->fAttitudeState != eAS_Uncknown )
  {// update heading & delta heading if attitude is known
    getStepData_v2(Data, SensorParams, step, vectGyr, &cntGyr, meanAcc, step->meanMagnetic, &time, &idx);
    if (time != 0.)
    {// update heading from g & magnetic
      getHeading(attData->Cp2b, step->meanMagnetic, &attData->Cb2n, &attData->psi);
    }
    else
    {// update heading by gyro delta heading from last step
      //attData->psi += attData->headingInc;
      attData->psi -= attData->headingInc;
      rotateCb2n(&attData->Cb2n, attData->headingInc);
    }
    //getDeltaHeading_v2(&attData->Cp2b1, Data, step, step->idxStart, &attData->headingInc);
    ConvertMagneticData(&attData->Cp2b1, Data, step, step->idxStart);
    //cntGyr = step->idxEnd - step->idxStart+1;
    //DREstimation( 0, -1, vectGyr, cntGyr, Data->sampleRate, &attData->headingInc);
  }
  else // unknown attitude
  {
    memset( attData, 0, sizeof(tStepAttitude));
  }

  cntGyr = step->idxEnd - step->idxStart + 1;
  for (j = step->idxStart; j <= step->idxEnd; j++)
    for( k = 0; k < 3; k++ )
      vectGyr[j-step->idxStart].data[k]= Data->gyrData.syncData[k][j];
  
  attData->headingInc = pDREKF->CalculateHeadingInc( 0, -1, vectGyr, cntGyr, true);
#pragma warning(pop)
}

//===============================================================================
/// Calculation heading and orientation matrices for static interval with
/// attitude update
///
/// @param[in]  Data    - pointer to structure with sensor meassurements
/// @param[in]  step    - pointer to steps inforamtion
/// @param[out] attData - pointer to attitude data
/// @return one if successful
//===============================================================================
static int8_t onStandWithAttitudeUpdate(tData* Data,
                                        tStepParams* step,
                                        tStepAttitude* attData,
                                        tUserParams* UserParams,
                                        tDREKF *pDREKF)
{
  int16_t cntGyr, i, j;
  int8_t k;
  int8_t ret_param;
  double meanAcc[3];

  if( getInitialHeading_Fine(Data, step, attData, UserParams, meanAcc) )
  {
    for( i = step->idxStart; i < step->idxEnd; i++ )
      if( Data->timeSync[i] >= attData->time )
        break;

    int16_t idx = i-step->idxStart;

    //getDeltaHeading_v2(&attData->Cp2b1, Data, step, i, &attData->headingInc);
    ConvertMagneticData(&attData->Cp2b1, Data, step, i);

    cntGyr = step->idxEnd - step->idxStart + 1;
    for( i = step->idxStart; i <= step->idxEnd; i++ )
      for( j = 0; j < 3; j++ )
        vectGyr[i - step->idxStart].data[j] = Data->gyrData.syncData[j][i];

    attData->headingInc = pDREKF->CalculateHeadingInc( meanAcc, idx, vectGyr, cntGyr, false);
    attData->headingInc = attData->headingInc + UserParams->HeadingCompensation;
    UserParams->HeadingCompensation = 0;

    ret_param = 1;
  }
  else
  {
    //mean magnetic calculation
    i = (step->idxStart + step->idxEnd)/2;
    for (k = 0; k < 3; k++)
    {
      step->meanMagnetic[k] = Data->magnData.syncData[k][i];
    }
    attData->headingInc = 0;
    ret_param = 0; // StepWitoutAttitude update will be used for delta heading calculation
  }

  return ret_param;
}

//=================================================================================
/// Calculation heading and orientation matrices for for static interval
/// without attitude update
///
/// @param[in]  Data          - pointer to structure with sensor meassurements
/// @param[in]  step          - pointer to step inforamtion
/// @param[in]  prevAtt       - pointer to previuos attitude inforamtion
/// @param[out] AttUserParams - pointer to attitude structure of user parameters
/// @param[out] attData       - pointer to attitude data
//=================================================================================
static void onStandWithoutAttitudeUpdate(tData* Data,
                                         tStepParams* step,
                                         tStepAttitude* prevAtt,
                                         tStepAttitude* AttUserParams,
                                         tStepAttitude* attData,
                                         tDREKF *pDREKF)
{
#pragma warning (push)
#pragma warning (disable: 6385)
  int16_t cntGyr, i, j;
  int8_t k;

  if( prevAtt->fAttitudeState != eAS_Uncknown )
  {
    copyAttToAtt(attData, prevAtt);
    attData->fAttitudeState = eAS_Rough;
  }
  else if( AttUserParams->fAttitudeState != eAS_Uncknown)
  {
    copyAttToAtt(attData, AttUserParams);
    attData->fAttitudeState = eAS_Rough;
  }
  else
    memset(attData, 0, sizeof(tStepAttitude));
  if( attData->fAttitudeState != eAS_Uncknown )
  {
    //attData->psi += attData->headingInc; // heading correction (rough) on start of current step
    attData->psi -= attData->headingInc; // heading correction (rough) on start of current step
    rotateCb2n(&attData->Cb2n, attData->headingInc);
    //cntGyr = 0;

    //getDeltaHeading_v2(&attData->Cp2b1, Data, step, step->idxStart, &attData->headingInc);
    ConvertMagneticData(&attData->Cp2b1, Data, step, step->idxStart);

    //mean magnetic calculation
    i = (step->idxStart + step->idxEnd)/2;
    for (k = 0; k < 3; k++)
    {
      step->meanMagnetic[k] = Data->magnData.syncData[k][i];
    }
  }

  cntGyr = step->idxEnd - step->idxStart + 1;
  for (j = step->idxStart; j <= step->idxEnd; j++)
    for( k = 0; k < 3; k++ )
      vectGyr[j-step->idxStart].data[k]= Data->gyrData.syncData[k][j];
    		
  attData->headingInc = pDREKF->CalculateHeadingInc( 0, -1, vectGyr, cntGyr, true);
#pragma warning (pop)
}

//===============================================================================
/// Calculates fine heading and fine orientation matrices from
/// data of stand interval
///
/// @param[in]  Data    - pointer to structure with sensor meassurements
/// @param[in]  step    - pointer to steps inforamtion
/// @param[out] attData - pointer to attitude data
/// @return one if successful
//===============================================================================
static int8_t getInitialHeading_Fine(tData* Data,
                                     tStepParams* step,
                                     tStepAttitude* attData,
                                     tUserParams* UserParams,
                                     double *meanAcc)
{
  double estInterval = EST_INTERVAL; // estimation interval duration [s]
  double timeInit;
  //double meanAcc[3]/*, meanMagn[3]*/;
  int16_t idxInit;
  double meanAccMagnitude;

  // initial data determination
  getInitialData(Data, step->idxStart, step->idxEnd, estInterval, &timeInit, &idxInit, meanAcc, step->meanMagnetic);
  if( timeInit != 0 )
  {
    phone2NEDMatrix_v3(meanAcc, step->meanMagnetic, attData, UserParams, 1);
    Phone2ENMatrix(meanAcc, attData->Cp2b1);
    attData->fAttitudeState = eAS_Fine;
    attData->index = step->idxStart + idxInit - 1;
    attData->time = timeInit;
    attData->headingInc = 0.;
    // Current accelerometer scale factor
    meanAccMagnitude = sqrt(meanAcc[0]*meanAcc[0] + meanAcc[1]*meanAcc[1] + meanAcc[2]*meanAcc[2]);
#ifndef PDR_DEBUG
    attData->acc_scale_factor = MEAN_G/meanAccMagnitude;
    attData->acc_mean_g = meanAccMagnitude;
#else
    #ifndef FIX_MEAN_G
    attData->acc_scale_factor = MEAN_G/meanAccMagnitude;
    attData->acc_mean_g = meanAccMagnitude;
    #else
    attData->acc_scale_factor = 1;
    attData->acc_mean_g = MEAN_G;
    #endif
#endif
    return 1;
	}
  else return 0;
}

//=================================================================================
/// The function rotates Cb2n0 matrix on angle dpsi
///
/// @param[in/out]  Cb2n       - pointer to matrix
/// @param[in]      headingInc -  heading increment
//=================================================================================
static void rotateCb2n(Matrix* Cb2n, double headingInc)
{
	double sinPsi, cosPsi;
	Matrix C;

	sinPsi = sin(headingInc);
	cosPsi = cos(headingInc);

	CreateMatrix(&C, 3, 3);

	C.Matr[0][0] = cosPsi;  C.Matr[0][1] = sinPsi; C.Matr[0][2] = 0.;
	C.Matr[1][0] = -sinPsi; C.Matr[1][1] = cosPsi; C.Matr[1][2] = 0;
	C.Matr[2][0] = 0;       C.Matr[2][1] = 0.;     C.Matr[2][2] = 1.;

	MultMatrix(Cb2n, Cb2n, &C);
}

//===============================================================================
/// The function  acceleration and magnetic vector for equilibrium point of step
/// (where a = g)
///
/// @param[in]  Data         - pointer to structure with sensor meassurements
/// @param[in]  SensorParams - pointer to sensor params data
/// @param[in]  step         - pointer to step inforamtion
/// @param[out] vectGyr      - pointer to gyro vector in equilibrium point
/// @param[out] cntGyr       - pointer to counter of gyro vector
/// @param[out] meanAcc      - pointer to acceleration vector in equilibrium point
/// @param[out] meanMagn     - pointer to magnetic vector in equilibrium point
/// @param[out] time         - time of calculated vectors
/// @param[out] idx          - index of measuring sample in measurement arrays
///                            corresponding to time
//===============================================================================
static void getStepData_v2(tData* Data,
                           tSensorParams* SensorParams,
                           tStepParams* step,
                           tVectGyr* GyrData,
                           int16_t* cntGyrData,
                           double* meanAcc,
                           double* meanMagn,
                           double* time,
                           int16_t* idx)
{
#pragma warning (push)
#pragma warning (disable: 6385)
	int16_t i = 0, j;
	int8_t k;
	double deltaAcc, flag;
	double acc_magnitude_i = 0, acc_magnitude_i1 = 0;

	for( j = step->idxStart; j <= step->idxEnd; j++ )
		for( k = 0; k < 3; k++ )
			GyrData[j-step->idxStart].data[k]= Data->gyrData.syncData[k][j];
	*cntGyrData = (j-step->idxStart);

	flag = 1;
	if( step->idxMax < step->idxMin )
	{
		for( i = step->idxMax; i < step->idxMin-1; i++ )
		{
		  acc_magnitude_i = Data->magnitudeData.magnitudeSync[i] - SensorParams->acc_mean_g;
		  acc_magnitude_i1 = Data->magnitudeData.magnitudeSync[i+1] - SensorParams->acc_mean_g;
		  flag = acc_magnitude_i*acc_magnitude_i1;
		  if( flag < 0. )
			break;
		}
	}

	if (flag < 0.)
	{
		deltaAcc = -acc_magnitude_i/(acc_magnitude_i1 - acc_magnitude_i);

		*idx = (int16_t)(((1. - deltaAcc)*i) + (deltaAcc*(i+1)));
		*time = (1. - deltaAcc)*Data->timeSync[i] + deltaAcc*Data->timeSync[i+1];
		for( k = 0; k < 3; k++ )
		{
			meanAcc[k] = (1. - deltaAcc)*Data->accData.syncData[k][i] + deltaAcc*Data->accData.syncData[k][i+1];
			meanMagn[k] = (1. - deltaAcc)*Data->magnData.syncData[k][i] + deltaAcc*Data->magnData.syncData[k][i+1];
		}
	}
	else
	{
		*idx = 0;
		*time = 0.;
		for( k = 0; k < 3; k++ )
		{
			meanAcc[k] = 0.;
			meanMagn[k] = 0.;
		}
	}
#pragma warning (pop)
}

//================================================================================
/// calculates heading increment during one step or one stand interval.
/// This is first adjustment and it is working for phone in hand orientation only.
///
/// @param[in]  Cp2b      - pointer to phone frame to user body frame transition matrix
/// @param[in]  Data      - pointer to data
/// @param[in]  idxStart	- starting index of current step
/// @param[in]  idxEnd		- ending index of current step
/// @param[in]  idxC			- index in step at which Cp2b was calculated
/// @param[out] headingInc - pointer to magnetic vector in equilibrium point
//================================================================================
static void getDeltaHeading_v2(Matrix* Cp2b,
	                             tData* Data,
                               tStepParams* Step,
	                             int16_t idxC,
	                             double* headingInc)
{
#pragma warning( push )
#pragma warning( disable: 6385)
	Matrix SkewSymmW;
	CreateMatrix(&SkewSymmW, 3, 3);
	Matrix ExpM;
	CreateMatrix(&ExpM, 3, 3);
	Matrix Cp2b_copy;
	CreateMatrix(&Cp2b_copy, 3, 3);
	CopyMatrix(&Cp2b_copy, Cp2b);
  Matrix MG;
	CreateMatrix(&MG, 3, 1);

	double dt = 1/Data->sampleRate;

	Matrix X_p_0;		CreateMatrix(&X_p_0, 3, 1);
	Matrix X_p_1;		CreateMatrix(&X_p_1, 3, 1);

	X_p_0.Matr[0][0] = X_p_1.Matr[0][0] = 1;
	X_p_0.Matr[1][0] = X_p_1.Matr[1][0] = 0;
	X_p_0.Matr[2][0] = X_p_1.Matr[2][0] = 0;

  //std::list <vector_3d> ubfMagnSamples;

	for( int i = idxC; i > Step->idxStart; --i )
	{
		Skew_Symmetric(&SkewSymmW, Data->gyrData.syncData[0][i-1] * (-dt), Data->gyrData.syncData[1][i-1] * (-dt), Data->gyrData.syncData[2][i-1] * (-dt));
		RoughExpM(&ExpM, &SkewSymmW);
		MultMatrix(Cp2b, Cp2b, &ExpM);
    // save magnetic vector in User Body Frame (NED without heading rotation)
    MG.Matr[0][0] = Data->magnData.syncData[0][i-1];
    MG.Matr[1][0] = Data->magnData.syncData[1][i-1];
    MG.Matr[2][0] = Data->magnData.syncData[2][i-1];
    MultMatrix(&MG,Cp2b,&MG);
    Step->ubfMagData[0][i-Step->idxStart-1] = MG.Matr[0][0];
    Step->ubfMagData[1][i-Step->idxStart-1] = MG.Matr[1][0];
    Step->ubfMagData[2][i-Step->idxStart-1] = MG.Matr[2][0];
	}

	MultMatrix(&X_p_0, Cp2b, &X_p_0);

	for( int i = idxC; i < (Step->idxEnd + 1); ++i )
	{
		Skew_Symmetric(&SkewSymmW, Data->gyrData.syncData[0][i] * (dt), Data->gyrData.syncData[1][i] * (dt), Data->gyrData.syncData[2][i] * (dt));
		RoughExpM(&ExpM, &SkewSymmW);
		MultMatrix(&Cp2b_copy, &Cp2b_copy, &ExpM);
   // save magnetic vector in User Body Frame (NED without heading rotation)
    MG.Matr[0][0] = Data->magnData.syncData[0][i];
    MG.Matr[1][0] = Data->magnData.syncData[1][i];
    MG.Matr[2][0] = Data->magnData.syncData[2][i];
    MultMatrix(&MG,&Cp2b_copy,&MG);
    Step->ubfMagData[0][i-Step->idxStart] = MG.Matr[0][0];
    Step->ubfMagData[1][i-Step->idxStart] = MG.Matr[1][0];
    Step->ubfMagData[2][i-Step->idxStart] = MG.Matr[2][0];
	}

  //Step->ubfMagnSamples = ubfMagnSamples;
  Step->cntMagData = Step->idxEnd - Step->idxStart + 1;

	MultMatrix(&X_p_1, &Cp2b_copy, &X_p_1);

	double abs_X_p_0 = sqrt(X_p_0.Matr[0][0] * X_p_0.Matr[0][0] + X_p_0.Matr[1][0] * X_p_0.Matr[1][0]);
	double abs_X_p_1 = sqrt(X_p_1.Matr[0][0] * X_p_1.Matr[0][0] + X_p_1.Matr[1][0] * X_p_1.Matr[1][0]);
	// scalar product ( X_p_0|xy ) * ( X_p_1|xy )
	double delta_h = (X_p_0.Matr[0][0] * X_p_1.Matr[0][0] + X_p_0.Matr[1][0] * X_p_1.Matr[1][0]);
	delta_h = delta_h / (abs_X_p_0 * abs_X_p_1);
	delta_h = acos(delta_h);
	// vector product ( X_p_0|xy ) x ( X_p_1|xy ) - provides the sign of rotation angle
	if ( (X_p_0.Matr[0][0]*X_p_1.Matr[1][0] - X_p_0.Matr[1][0]*X_p_1.Matr[0][0]) < 0 )
		{ delta_h = -delta_h; }
	CopyMatrix(Cp2b, &Cp2b_copy);

	*headingInc = delta_h;

#pragma warning (pop)
}

//================================================================================
/// calculates heading increment during one step or one stand interval.
/// This is first adjustment and it is working for phone in hand orientation only.
///
/// @param[in]  Cp2b      - pointer to phone frame to user body frame transition matrix
/// @param[in]  Data      - pointer to data
/// @param[in]  idxStart	- starting index of current step
/// @param[in]  idxEnd		- ending index of current step
/// @param[in]  idxC			- index in step at which Cp2b was calculated
/// @param[out] headingInc - pointer to magnetic vector in equilibrium point
//================================================================================
static void ConvertMagneticData(Matrix *Cp2b,
	                             tData* Data,
                               tStepParams* Step,
	                             int16_t idxC)
{
#pragma warning( push )
#pragma warning( disable: 6385)
	Matrix SkewSymmW;
	CreateMatrix(&SkewSymmW, 3, 3);
	Matrix ExpM;
	CreateMatrix(&ExpM, 3, 3);
	Matrix Cp2b_copy;
	CreateMatrix(&Cp2b_copy, 3, 3);
	CopyMatrix(&Cp2b_copy, Cp2b);
  Matrix MG;
	CreateMatrix(&MG, 3, 1);

	double dt = 1/Data->sampleRate;

	for( int i = idxC; i > Step->idxStart; --i )
	{
		Skew_Symmetric(&SkewSymmW, Data->gyrData.syncData[0][i-1] * (-dt), Data->gyrData.syncData[1][i-1] * (-dt), Data->gyrData.syncData[2][i-1] * (-dt));
		RoughExpM(&ExpM, &SkewSymmW);
		MultMatrix(Cp2b, Cp2b, &ExpM);
    // save magnetic vector in User Body Frame (NED without heading rotation)
    MG.Matr[0][0] = Data->magnData.syncData[0][i-1];
    MG.Matr[1][0] = Data->magnData.syncData[1][i-1];
    MG.Matr[2][0] = Data->magnData.syncData[2][i-1];
    MultMatrix(&MG,Cp2b,&MG);
    Step->ubfMagData[0][i-Step->idxStart-1] = MG.Matr[0][0];
    Step->ubfMagData[1][i-Step->idxStart-1] = MG.Matr[1][0];
    Step->ubfMagData[2][i-Step->idxStart-1] = MG.Matr[2][0];
    CopyMatrix(&Step->Cp2b[i - Step->idxStart - 1], Cp2b);
	}

	for( int i = idxC; i < (Step->idxEnd + 1); ++i )
	{
		Skew_Symmetric(&SkewSymmW, Data->gyrData.syncData[0][i] * (dt), Data->gyrData.syncData[1][i] * (dt), Data->gyrData.syncData[2][i] * (dt));
		RoughExpM(&ExpM, &SkewSymmW);
		MultMatrix(&Cp2b_copy, &Cp2b_copy, &ExpM);
   // save magnetic vector in User Body Frame (NED without heading rotation)
    MG.Matr[0][0] = Data->magnData.syncData[0][i];
    MG.Matr[1][0] = Data->magnData.syncData[1][i];
    MG.Matr[2][0] = Data->magnData.syncData[2][i];
    MultMatrix(&MG,&Cp2b_copy,&MG);
    Step->ubfMagData[0][i-Step->idxStart] = MG.Matr[0][0];
    Step->ubfMagData[1][i-Step->idxStart] = MG.Matr[1][0];
    Step->ubfMagData[2][i-Step->idxStart] = MG.Matr[2][0];
    CopyMatrix(&Step->Cp2b[i - Step->idxStart], &Cp2b_copy);
	}

  Step->cntMagData = Step->idxEnd - Step->idxStart + 1;
	CopyMatrix(Cp2b, &Cp2b_copy);

#pragma warning (pop)
}


//================================================================================
/// copy attitude parameters
///
/// @param[in]  attSrc - pointer to source strucuture
/// @param[out] attDst - pointer to output structure
//================================================================================
static void copyAttToAtt(tStepAttitude* attDst, tStepAttitude* attSrc)
{
	CopyMatrix(&attDst->Cb2n, &attSrc->Cb2n);
	CopyMatrix(&attDst->Cp2b, &attSrc->Cp2b);
  CopyMatrix(&attDst->Cp2b1, &attSrc->Cp2b1);

	attDst->fi = attSrc->fi;
	attDst->teta = attSrc->teta;
	attDst->psi = attSrc->psi;
	attDst->time = attSrc->time;
	attDst->index = attSrc->index;
	attDst->headingInc = attSrc->headingInc;
  attDst->fAttitudeState = attSrc->fAttitudeState;
}