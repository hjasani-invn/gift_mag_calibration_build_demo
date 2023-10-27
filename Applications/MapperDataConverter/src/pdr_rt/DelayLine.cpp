/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  Delay line implementation
 *   @file                   DelayLine.h
 *   @author                 D. Vishin, M. Zhokhova
 *   @date                   14.12.2011
 *   @version                2.0
 */
/*****************************************************************************/

#include "DelayLine.h"

//=======================================================================
/// Shift in the buffer for delay
///
/// @param[in]  delayLine - pointer to instance
/// @param[in]  delay     - shift in the buffer
/// @return pointer to buffer
//=======================================================================
static void* getPtrDelayLine(tDelayLine* delayLine, uint16_t delay)
{
	int16_t index = delayLine->index - (int16_t)delay;
	if( index < 0 ) 
	{
		index += delayLine->length;
	}

	return  (void*)(((char*) delayLine->ptrBuff) + index*delayLine->fieldSize);
}

//================================================================
/// Increment the buffer
///
/// @param[in]  delayLine - pointer to instance
/// @return pointer to buffer
//================================================================
static void* newDelayLine(tDelayLine* delayLine)
{
	uint16_t index = delayLine->index;
	if( ++index == delayLine->length ) 
	{
		index = 0; // wrap index
	}
	delayLine->index = index;

	return (void *) (((char*) delayLine->ptrBuff) + index*delayLine->fieldSize);
}

//=======================================================================================
/// Initialization of delay line
///
/// @param[in]  delayLine - pointer to instance
/// @param[in]  ptrBuff   - pointer to buffer
/// @param[in]  fieldSize - size of field
/// @param[in]  length    - length of delay line
//=======================================================================================
static void initDelayLine(tDelayLine* delayLine, 
						              void* ptrBuff, 
						              uint16_t fieldSize, 
						              uint16_t length)
{
	delayLine->fieldSize = fieldSize;
	delayLine->length = length;
	delayLine->ptrBuff = ptrBuff;
	delayLine->index = 0;
}

// base universal delay line functions
//=======================================================================================
/// Put the value in the delay line
///
/// @param[in]  delayLine - pointer to instance
/// @param[in]  ptrObj    - pointer to value
//=======================================================================================
/*static void putDelayLine(tDelayLine* delayLine, void* ptrObj)
{
	uint16_t c1 = delayLine->fieldSize;
	char* ptrBuffByte = (char*)newDelayLine(delayLine);
	char* ptrObjByte = (char*)ptrObj;
	do 
	{
		*ptrBuffByte++ = *ptrObjByte++;
	} while( --c1 );

	return;
}*/

//=======================================================================================
/// Get value from the delay line
///
/// @param[in]  delayLine - pointer to instance
/// @param[in]  ptrObj    - pointer to value
/// @param[in]  delay     - delay
//=======================================================================================
/*static void getDelayLine(tDelayLine* delayLine, void* ptrObj, uint16_t delay)
{
	uint16_t	c1 = delayLine->fieldSize;
	char* ptrObjByte = (char*)ptrObj;
	char* ptrBuffByte = (char*)getPtrDelayLine(delayLine, delay);

	do
	{
		*ptrObjByte++ = *ptrBuffByte++;
	} while( --c1 );

	return;
}*/

//=======================================================================================
/// Set the value of the delay line
///
/// @param[in]  delayLine - pointer to instance
/// @param[in]  ptrObj    - pointer to value
/// @param[in]  delay     - delay
//=======================================================================================
/*static void setDelayLine(tDelayLine* delayLine, void* ptrObj, uint16_t delay)
{
	uint16_t c1 = delayLine->fieldSize;
	char* ptrObjByte = (char*)ptrObj;
	char* ptrBuffByte = (char*)getPtrDelayLine(delayLine, delay);

	do 
	{
		*ptrBuffByte++ = *ptrObjByte++;
	} while( --c1 );

	return;
}*/

/*
Double delayLine functions
*/

//=========================================================================================
/// Initialization of delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @param[in]  ptrBuff   - pointer to buffer
/// @param[in]  length    - length of delay line
//=========================================================================================
void InitDelayLineDouble(tDelayLineDouble* DelayLine, void *ptrBuff, uint16_t length)
{
	initDelayLine((tDelayLine*)DelayLine, ptrBuff, sizeof(double), length);
}

//=======================================================================================
/// Clearing buffer of delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @return error
//=======================================================================================
int8_t ClrBuffDouble(tDelayLineDouble* DelayLine)
{
	uint16_t c1 = DelayLine->dl.length;
	double* ptr = (double*)DelayLine->dl.ptrBuff;

	if( !c1 ) 
	{
		return 0;
	}
	do 
	{
		*ptr++ = 0.0;
	} while( --c1 );

	return 1;
}

//=======================================================================================
/// Put the value in the delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @param[in]  Obj       - value
/// return value of the delay line
//=======================================================================================
double PutToDelayLineDouble(tDelayLineDouble* DelayLine, double Obj)
{
	double outObj;
	double* ptr;

	if( !DelayLine->dl.length ) 
	{
		return Obj;
	}
	ptr = (double*)newDelayLine((tDelayLine*)DelayLine);
	outObj = *ptr; *ptr = Obj;

	return outObj;
}

//=======================================================================================
/// Get value from the delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @param[in]  delay     - delay
/// @return value of the delay line
//=======================================================================================
double GetDelayLineDouble(tDelayLineDouble* DelayLine, uint16_t delay)
{
	double* ptr;

	if( !DelayLine->dl.length ) 
	{
		return 0;
	}
	ptr = (double*)getPtrDelayLine((tDelayLine*)DelayLine, delay);

	return *ptr;
}

//=======================================================================================
/// Set the value of the delay line of double type
///
/// @param[in]  DelayLine - pointer to instance
/// @param[in]  Obj       - value
/// @param[in]  delay     - delay
/// @return error
//=======================================================================================
int8_t SetDelayLineDouble(tDelayLineDouble* DelayLine, double Obj, uint16_t delay)
{
	double* ptr;

	if( !DelayLine->dl.length ) 
	{
		return 0;
	}
	ptr = (double*)getPtrDelayLine((tDelayLine*)DelayLine, delay);
	*ptr = Obj;

	return 1;
}