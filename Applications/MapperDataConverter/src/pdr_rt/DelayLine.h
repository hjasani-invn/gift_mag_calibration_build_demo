/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  Delay line interface
 *   @file                   DelayLine.h
 *   @author                 D. Vishin, M. Zhokhova
 *   @date                   14.12.2011, 12.07.2013
 *   @version                3.0
 */
/*****************************************************************************/

#ifdef _MSC_VER
 #if  _MSC_VER < 1600
  #include "stdint.h"
 #else 
  #include <stdint.h>
 #endif
#else
 #include <stdint.h>
#endif

// century
#ifndef _DELAY_LINE_H_
#define _DELAY_LINE_H_

typedef struct tDelayLine 
{
	void* ptrBuff;
	uint16_t fieldSize;
	uint16_t length;
	uint16_t index;
} 
tDelayLine;

typedef struct tDelayLineDouble
{
	tDelayLine dl;
} 
tDelayLineDouble;

//=================================================================================================
/// Initialization of delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @param[in]  ptrBuff   - pointer to buffer
/// @param[in]  length    - length of delay line
//=================================================================================================
extern void InitDelayLineDouble(tDelayLineDouble* DelayLine, void *ptrBuff, uint16_t length);

//=======================================================================================
/// Clearing buffer of delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @return error
//=======================================================================================
extern int8_t ClrBuffDouble(tDelayLineDouble* DelayLine);

//=======================================================================================
/// Put the value in the delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @param[in]  Obj       - value
/// return value of the delay line
//=======================================================================================
extern double PutToDelayLineDouble(tDelayLineDouble* DelayLine, double Obj);

//=======================================================================================
/// Get value from the delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @param[in]  delay     - delay
/// @return value of the delay line
//=======================================================================================
extern double GetDelayLineDouble(tDelayLineDouble* DelayLine, uint16_t delay);

//===========================================================================================
/// Set the value of the delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @param[in]  Obj       - value
/// @param[in]  delay     - delay
/// @return error
//===========================================================================================
extern int8_t SetDelayLineDouble(tDelayLineDouble* DelayLine, double Obj, uint16_t delay);

#endif //_DELAY_LINE_H_
