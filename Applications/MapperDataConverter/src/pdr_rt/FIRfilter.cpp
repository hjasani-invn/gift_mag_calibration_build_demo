/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  floor FIR filter implementation
 *   @file                   FIRfilter.cpp
 *   @author                 D. Vishin, M. Zhokhova
 *   @date                   14.12.2011, 15.07.2013
 *   @version                3.0
 */
/*****************************************************************************/

#include "FIRfilter.h"

//====================================================================
/// FIR filter initialization
///
/// @param[in]  FIRfilter        - pointer to instance
/// @param[in]  ptrPulseResponse - pointer to pulse response
/// @param[in]  length           - length of FIR filter
/// @param[in]  ptrBuff          - pointer to buffer for delay line
/// @param[in]  evenOddFIR       - even/odd pulse response FIR filter
//====================================================================
void InitFIRfilter(tFIRfilter* FIRfilter,
				           const double* ptrPulseResponse,
				           uint16_t length,
				           void* ptrBuff,
									 int8_t evenOddFIR)
{
	FIRfilter->ptrPulseResponse = ptrPulseResponse;
	FIRfilter->length = length;
	FIRfilter->evenOddFIR = evenOddFIR;

	InitDelayLineDouble(&FIRfilter->stateDelayLine, ptrBuff, --length);
	ClrBuffDouble(&FIRfilter->stateDelayLine);
}

//===============================================================
/// Put to FIR filter
///
/// @param[in]  FIRfilter - pointer to instance
/// @param[in]  sample    - current sample
/// @return filtered value
//===============================================================
double PutFIRfilter(tFIRfilter* FIRfilter, double sample)
{
	double sum = 0;
	uint16_t c1, c2;
	const double* ptrPulseResp = FIRfilter->ptrPulseResponse;

	c1 = FIRfilter->length;

	if( FIRfilter->evenOddFIR ) // even
	{
		c1 >>= 1;
	}
	else // odd
	{
		c1--;
		c1 >>= 1;
	}
	c2 = c1;

	do 
	{
		sample = PutToDelayLineDouble(&FIRfilter->stateDelayLine, sample);
		sum += (sample *(*ptrPulseResp++));
	} while( --c1 ) ;
	if( !FIRfilter->evenOddFIR ) // odd
	{
		sample = PutToDelayLineDouble(&FIRfilter->stateDelayLine, sample);
		sum += (sample *(*ptrPulseResp));
	}
	ptrPulseResp--;
	do 
	{
		sample = PutToDelayLineDouble(&FIRfilter->stateDelayLine, sample);
		sum += (sample *(*ptrPulseResp--));
	} while( --c2 ) ;

	PutToDelayLineDouble(&FIRfilter->stateDelayLine, sample);
	
	return sum;
}