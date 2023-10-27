/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  synchronization module
 *   @file                   SynchronizSensorData.cpp
 *   @author                 M. Zhokhova
 *   @date                   17.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#include <math.h>
#include <memory.h>
#include "PDR.h"
#include <assert.h>

static void createTimeScale(double* timeAcc, int16_t cntAcc, double* timeGyr, int16_t cntGyr, double* timeMagn, 
							int16_t cntMagn, double sampleRate, double* timeSync, int16_t* cntTime);
static void excludeEqualValues(tFltData* fltData);
static void interpolateData(double* timeSync, int16_t cntTimeStart, int16_t cntTimeEnd, 
														double* fltData, double* timeFlt, int16_t cntFltData,
                            double* syncData, double bias);

//=========================================================================
/// Synchronizes filtered data from all sensors
///
/// @param[in/out]  Data - pointer to filtered sensor data
//=========================================================================
void SynchronizSensorData(tData* Data)
{
  int16_t cntTimeStart, j;

  cntTimeStart = Data->cntTime;
  // decrease accelerometer data for needed sample rate
  createTimeScale(Data->accData.fltData.t, Data->accData.fltData.cntData,
                  Data->gyrData.fltData.t, Data->gyrData.fltData.cntData, Data->magnData.fltData.t, 
                  Data->magnData.fltData.cntData, Data->sampleRate, Data->timeSync, &Data->cntTime);
  //excludeEqualValues(&Data->accData.fltData);

  // interpolate data on scale time by cubic polynoms
  for( j = 0; j < 3; j++ )
    interpolateData(Data->timeSync, cntTimeStart, Data->cntTime,
                    Data->accData.fltData.data[j], Data->accData.fltData.t, Data->accData.fltData.cntData,
                    Data->accData.syncData[j], Data->accData.bias[j]);
  interpolateData(Data->timeSync, cntTimeStart, Data->cntTime,
                  Data->magnitudeData.magnitudeFlt, Data->accData.fltData.t, Data->accData.fltData.cntData,
                  Data->magnitudeData.magnitudeSync, 0);

  //excludeEqualValues(&Data->gyrData.fltData);
  // interpolate data on scale time by cubic polynoms
  for( j = 0; j < 3; j++ )
    interpolateData(Data->timeSync, cntTimeStart, Data->cntTime,
                    Data->gyrData.fltData.data[j], Data->gyrData.fltData.t, Data->gyrData.fltData.cntData,
                    Data->gyrData.syncData[j], Data->gyrData.bias[j]);

  //excludeEqualValues(&Data->magnData.fltData);
  // interpolate data on scale time by cubic polynoms
  for( j = 0; j < 3; j++ )
    interpolateData(Data->timeSync, cntTimeStart, Data->cntTime,
                    Data->magnData.fltData.data[j],Data->magnData.fltData.t, Data->magnData.fltData.cntData,
                    Data->magnData.syncData[j], Data->magnData.bias[j]);
}

//=========================================================================
/// Generate uniform time scale with specific sample rate,
/// to improve the interpolation take the previous time point 
///
/// @param[in]  time       - pointer to filtered sensor data
/// @param[in]  cntData    - counter of filtered data
/// @param[in]  sampleRate - sampling rate
/// @param[out] timeSync   - uniform time scale
/// @param[out] cntTime    - counter of time scale
//=========================================================================
static void createTimeScale(double* timeAcc, 
							int16_t cntAcc,
							double* timeGyr, 
							int16_t cntGyr,
							double* timeMagn, 
							int16_t cntMagn,
							double sampleRate, 
							double* timeSync,
							int16_t* cntTime)
{
	double timeEnd, period;
	int16_t i;

	period = 1./sampleRate;
	if( *cntTime == 0 )
	{
		timeSync[0] = timeAcc[0];
    if( timeGyr[0] > timeSync[0])
      timeSync[0] = timeGyr[0];
    if( timeMagn[0] > timeSync[0])
      timeSync[0] = timeMagn[0];
		i = 0;
	}
	else
		i = *cntTime-1;

	timeEnd = timeAcc[cntAcc-1];
	if( timeGyr[cntGyr-1] < timeEnd )
		timeEnd = timeGyr[cntGyr-1];
	if( timeMagn[cntMagn-1] < timeEnd )
		timeEnd = timeMagn[cntMagn-1];

	while( timeSync[i] < (timeEnd-period) ) 
	{
		i++;
		timeSync[i] = timeSync[i-1] + period;
	}
	*cntTime = i;
}

//=========================================================================
/// Exclude equal values
///
/// @param[in/out]  fltData - pointer to filtered sensor data
//=========================================================================
static void excludeEqualValues(tFltData* fltData)
{
	int16_t i;

	for( i = 1; i < (fltData->cntData-2); i++ )
	{
		if( (fltData->t[i] - fltData->t[i-1]) <= 0 )
			fltData->t[i] += Min(((fltData->t[i+1] - fltData->t[i-1])/2.), 0.001);
	}
	i;
	if( (fltData->t[i] - fltData->t[i-1]) <= 0 )
		fltData->t[i] += Min(((fltData->t[i-1] - fltData->t[i-2])/2.), 0.001);
}

//=========================================================================
/// calculating the first derivative of the function
///
/// @param[in]  fltData    - pointer to filtered sensor data
/// @param[in]  cntFltData - counter of filtered data
/// @param[in]  time       - time scale for filtered data
/// @param[in]  idxData    - index for derivate
/// @param[out] deriv      - pointer to derivate
//=========================================================================
static void calcFirstDeriv(double* fltData, 
						   int16_t cntFltData, 
						   double* time,
						   int16_t idxData, 
						   double* deriv)
{
	double delta[2], d;
	double df;
	double deltaDf[2];

	if( idxData == 0 )
	{
		// calculation of at a boundary point
		delta[0] = (fltData[1] - fltData[0])/(time[1] - time[0]); 
		delta[1] = (fltData[2] - fltData[1])/(time[2] - time[1]);
		d = (((time[2] + time[1] - 2.*time[0])*delta[0]) - ((time[1] - time[0])*delta[1]))/(time[2]-time[0]);
		if( ((d >= 0.) && (delta[0] < 0.)) || ((d < 0.) && (delta[0] >= 0.)) )
			*deriv = 0.;
		else
		{
			// calculation the first derivative for next index 
			deltaDf[0] = (fltData[1] - fltData[0])/(time[1] - time[0]); 
			deltaDf[1] = (fltData[2] - fltData[1])/(time[2] - time[1]);
			if( ((deltaDf[0] < 0.) && (deltaDf[1] > 0.)) || ((deltaDf[0] > 0.) && (deltaDf[1] < 0.)) 
				|| (deltaDf[0] == 0.) || (deltaDf[1] == 0.) ) // signs of the numbers are the same or equal zero
			{
				df = 0.;
			}
			else // signs of the numbers are different
			{ // x(k+1) - x(k) == x(k) - x(k-1)
				if( (time[2] - time[1]) == (time[1] - time[0]) )
					df = 2./((1./deltaDf[0]) + (1./deltaDf[1]));
				else // not equal
				{
					df = 3.*(time[2] - time[0])/
						(((2.*time[2]-time[1]-time[0])/deltaDf[0]) + ((time[2]+time[1]-2.*time[0])/deltaDf[1]));
				}
			}
			if( (((d >= 0.) && (df < 0)) || ((d < 0.) && (df >= 0.))) && (fabs(d) > 3*delta[0]) )
				*deriv = 3.*delta[0];
			else
				*deriv = d;
		}
	}
	else if( idxData == (cntFltData-1) )
	{
    assert((time[cntFltData-1] - time[cntFltData-2]) != 0.);
    assert((time[cntFltData-2] - time[cntFltData-3]) != 0.);
    assert((time[cntFltData-3]-time[cntFltData-1]) != 0.);
		delta[0] = (fltData[cntFltData-1] - fltData[cntFltData-2])/(time[cntFltData-1] - time[cntFltData-2]); 
		delta[1] = (fltData[cntFltData-2] - fltData[cntFltData-3])/(time[cntFltData-2] - time[cntFltData-3]);
		d = (((time[cntFltData-3] + time[cntFltData-2] - 2.*time[cntFltData-1])*delta[0]) - ((time[cntFltData-2] - time[cntFltData-1])*delta[1]))/
			(time[cntFltData-3]-time[cntFltData-1]);
		if( ((d >= 0.) && (delta[0] < 0.)) || ((d < 0.) && (delta[0] >= 0.)) )
			*deriv = 0.;
		else 
		{
			// calculation the first derivative for previous index 
			delta[0] = (fltData[cntFltData-2] - fltData[cntFltData-3])/(time[cntFltData-2] - time[cntFltData-3]); 
			delta[1] = (fltData[cntFltData-1] - fltData[cntFltData-2])/(time[cntFltData-1] - time[cntFltData-2]);
			if( ((delta[0] < 0.) && (delta[1] > 0.)) || ((delta[0] > 0.) && (delta[1] < 0.)) 
				|| (delta[0] == 0.) || (delta[1] == 0.) ) // signs of the numbers are the same or equal zero
			{
				df = 0.;
			}
			else // signs of the numbers are different
			{ // x(k+1) - x(k) == x(k) - x(k-1)
				if( (time[cntFltData-1] - time[cntFltData-2]) == (time[cntFltData-2] - time[cntFltData-3]) )
					df = 2./((1./delta[0]) + (1./delta[1]));
				else // not equal
				{
					df = 3.*(time[cntFltData-1] - time[cntFltData-3])/
						(((2.*time[cntFltData-1]-time[cntFltData-2]-time[cntFltData-3])/delta[0]) + ((time[cntFltData-1]+time[cntFltData-2]-2.*time[cntFltData-3])/delta[1]));
				}
			}
			if( (((d >= 0.) && (df < 0)) || ((d < 0.) && (df >= 0.))) && (fabs(d) > 3*delta[0]) )
				*deriv = 3.*delta[0];
			else
				*deriv = d;
		}
	}
	else
	{
		// delta(k) = (y(k+1)- y(k))/(x(k+1) - x(k))
		delta[0] = (fltData[idxData] - fltData[idxData-1])/(time[idxData] - time[idxData-1]); 
		delta[1] = (fltData[idxData+1] - fltData[idxData])/(time[idxData+1] - time[idxData]);
		if( ((delta[0] < 0.) && (delta[1] > 0.)) || ((delta[0] > 0.) && (delta[1] < 0.)) 
			|| (delta[0] == 0.) || (delta[1] == 0.) ) // signs of the numbers are the same or equal zero
		{
			*deriv = 0.;
		}
		else // signs of the numbers are different
		{ // x(k+1) - x(k) == x(k) - x(k-1)
			if( (time[idxData+1] - time[idxData]) == (time[idxData] - time[idxData-1]) )
				*deriv = (2./((1./delta[0]) + (1./delta[1])));
			else // not equal
			{
				*deriv = (3.*(time[idxData+1] - time[idxData-1])/
					(((2.*time[idxData+1]-time[idxData]-time[idxData-1])/delta[0]) + ((time[idxData+1]+time[idxData]-2.*time[idxData-1])/delta[1])));
			}
		}
	}
}

//=========================================================================
/// Exclude equal values
///
/// @param[in]  timeSync     - uniform time scale
/// @param[in]  cntTimeStart - counter of time scale
/// @param[in]  cntTimeEnd   - counter of time scale
/// @param[in]  fltData      - pointer to filtered sensor data
/// @param[in]  timeFlt      - pointer to time scale for filtered data
/// @param[in]  cntFltData   - counter of filtered data
/// @param[out] syncData     - pointer to synchronized data
//=========================================================================
static void interpolateData(double* timeSync, 
							int16_t cntTimeStart,
							int16_t cntTimeEnd,
							double* fltData,
							double* timeFlt,
							int16_t cntFltData,
							double* syncData,
              double bias)
{
	double delta[2];
	int16_t i, k;
	double df[2];

	// interpolate
	if( cntTimeStart == 0 )
		k = 0;
	else
		k = cntTimeStart-1;

	for( ; k < cntTimeEnd; k++ )
	{
		for(i = 0 ; i < cntFltData; i++ )
		{
			if( timeSync[k] <= timeFlt[i] )
				break;
		}
    if( timeSync[k] == timeFlt[i] )
		{
			syncData[k] = fltData[i];
			continue;
		}
		if( i == 0 )
			i = 1;
		else if( i == cntFltData )
			i = cntFltData-1;
		delta[0] = timeSync[k] - timeFlt[i-1];
		delta[1] = (fltData[i] - fltData[i-1])/(timeFlt[i] - timeFlt[i-1]);
		calcFirstDeriv(fltData, cntFltData, timeFlt, i-1, &df[0]); // i-1
		calcFirstDeriv(fltData, cntFltData, timeFlt, i, &df[1]); // i
		syncData[k] = (fltData[i-1] + df[0]*delta[0] + 
			((3*delta[1] - df[1] - 2.*df[0])/(timeFlt[i] - timeFlt[i-1]))*delta[0]*delta[0] + 
			((df[0] + df[1] - 2.*delta[1])/((timeFlt[i] - timeFlt[i-1])*(timeFlt[i] - timeFlt[i-1])))*delta[0]*delta[0]*delta[0]);
    syncData[k] -= bias;
	}
}
