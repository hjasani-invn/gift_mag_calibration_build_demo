/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  floor FIR filter interface
 *   @file                   FIRfilter.h
 *   @author                 D. Vishin, M. Zhokhova
 *   @date                   14.12.2011, 12.07.2013
 *   @version                3.0
 */
/*****************************************************************************/
 
#ifndef _FIRFILTER_H_
#define _FIRFILTER_H_

#include "DelayLine.h"

typedef struct tFIRfilter 
{
	tDelayLineDouble stateDelayLine;

	const double* ptrPulseResponse; // pointer to pulse response FIR filter
	uint16_t length;
	int8_t evenOddFIR; // even(1)/odd(0) pulse response FIR filter
}
tFIRfilter;


//====================================================================
/// FIR filter initialization
///
/// @param[in]  FIRfilter        - pointer to instance
/// @param[in]  ptrPulseResponse - pointer to pulse response
/// @param[in]  length           - length of FIR filter
/// @param[in]  ptrBuff          - pointer to buffer for delay line
/// @param[in]  evenOddFIR       - even/odd pulse response FIR filter
//====================================================================
extern void InitFIRfilter(tFIRfilter* FIRfilter,
					                const double* ptrPulseResponse,
					                uint16_t length,
					                void* ptrBuff,
													int8_t evenOddFIR);

//===============================================================
/// Put to FIR filter
///
/// @param[in]  this   - pointer to instance
/// @param[in]  sample - current sample
/// @return filtered value
//===============================================================
extern double PutFIRfilter(tFIRfilter* FIRfilter, double sample);


#endif //_FIRFILTER_H_

