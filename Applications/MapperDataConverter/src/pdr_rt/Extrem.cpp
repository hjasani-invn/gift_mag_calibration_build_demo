/*****************************************************************************
*    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
*   @project                PDR project
*   @brief                  extremum detection module
*   @file                   Extrem.cpp
*   @author                 M. Zhokhova
*   @date                   26.11.2013
*   @version                1.0
*/
/*****************************************************************************/

#include <math.h>
#include <memory.h>
#include <string.h>
#include <assert.h>
#include "PDR.h"
#include "Report.h"

static tExtrem extrem[MAX_EXTREM];

static void rejectFirstMinimum( tExtrem* extrem, int16_t* cntExtrem );
static void rejectLastMaximum( tExtrem* extrem, int16_t* cntExtrem );
static void baseStepConstraints( tExtrem* extrem, int16_t cntExtrem, double min_step_amplitude_threshold );
static void rejectDoublePeaks( tExtrem* extrem, int16_t* cntExtrem );
static void clearRejected( tExtrem* extrem, int16_t* cntExtrem, float rejectThresh );
static void rejectSingleExtremums( tExtrem* extrem, int16_t* cntExtrem );
static void rejectHugeSteps( tExtrem* extrem, int16_t cntExtrem, double max_step_amplitude_threshold );
static void restoreBoundaryExtremums( tExtrem* extrem, int16_t cntExtrem );
static void getSteps( tExtrem* extrem, int16_t cntExtrem, double* timeSync, int16_t cntTime, double* magnitude,
                     tStepParams* Step, int8_t* cntSteps );

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
void ExtremToSteps( double* timeSync,
                   int16_t cntTime,
                   double* magnitude,
                   int16_t idxStart,
                   tSensorParams* SensorParams,
                   tStepParams* Step,
                   int8_t* cntSteps )
{
  int16_t i, cntExtrem;
  double daMinus, daPlus;

  cntExtrem = 0;

  // extremum detection
  for( i = idxStart + 1; i < cntTime - 1; i++ )
  {
    daMinus = magnitude[i] - magnitude[i - 1];
    daPlus = magnitude[i] - magnitude[i + 1];

    if( ( daMinus * daPlus ) >= 0. ) //  extremum is detected
    {
      extrem[cntExtrem].idx = i;

      if( daPlus > 0. )
        extrem[cntExtrem].sign = 1.;
      else if( daPlus < 0. )
        extrem[cntExtrem].sign = -1.;
      else
        extrem[cntExtrem].sign = 0.;

      extrem[cntExtrem].magnitude = magnitude[i];
      extrem[cntExtrem].time = timeSync[i];
      cntExtrem++;
    }
  }

  // false step rejection: heuristic constraints
  rejectFirstMinimum( extrem, &cntExtrem );
  rejectLastMaximum( extrem, &cntExtrem );

  // base constraints
  baseStepConstraints( extrem, cntExtrem, SensorParams->acc_mean_g * MIN_STEP_AMPLITUDE_FACTOR );

  // reject double peaks
  rejectDoublePeaks( extrem, &cntExtrem );
  clearRejected( extrem, &cntExtrem, REJECT_THRESH );

  // reject single extremums
  rejectSingleExtremums( extrem, &cntExtrem );
  clearRejected( extrem, &cntExtrem, REJECT_THRESH );

  // reject huge steps
  rejectHugeSteps( extrem, cntExtrem, SensorParams->acc_mean_g * MAX_STEP_AMPLITUDE_FACTOR );
  clearRejected( extrem, &cntExtrem, REJECT_THRESH );

  // restore boundary steps
  restoreBoundaryExtremums( extrem, cntExtrem );
  clearRejected( extrem, &cntExtrem, REJECT_THRESH );
  rejectFirstMinimum( extrem, &cntExtrem );
  rejectLastMaximum( extrem, &cntExtrem );

  // base constraints
  baseStepConstraints( extrem, cntExtrem, SensorParams->acc_mean_g * MIN_STEP_AMPLITUDE_FACTOR );
  clearRejected( extrem, &cntExtrem, REJECT_THRESH );
  rejectSingleExtremums( extrem, &cntExtrem );
  clearRejected( extrem, &cntExtrem, REJECT_THRESH );

  // get steps
  // AHTUNG! huge step re-inspection!
  // reject huge steps
  rejectHugeSteps( extrem, cntExtrem, SensorParams->acc_mean_g * MAX_STEP_AMPLITUDE_FACTOR );
  clearRejected( extrem, &cntExtrem, REJECT_THRESH );

  getSteps( extrem, cntExtrem, timeSync, cntTime, magnitude, Step, cntSteps );
}

//=========================================================================
/// reject first minimum
///
/// @param[in] extrem    - pointer to extremum array
/// @param[in] cntExtrem - pointer to number of extremum
//=========================================================================
static void rejectFirstMinimum( tExtrem* extrem, int16_t* cntExtrem )
{
  int16_t i, j;

  // find first maximum
  for( i = 0; i < *cntExtrem; i++ )
  {
    if( extrem[i].sign > 0 )
      break;
  }

  for( j = i; j < *cntExtrem; j++ )
  {
    extrem[j - i].idx = extrem[j].idx;
    extrem[j - i].sign = extrem[j].sign;
    extrem[j - i].magnitude = extrem[j].magnitude;
    extrem[j - i].time = extrem[j].time;
  }

  *cntExtrem -= i;
}

//=========================================================================
/// reject last maximum
///
/// @param[in] extrem    - pointer to extremum array
/// @param[in] cntExtrem - pointer to number of extremum
//=========================================================================
static void rejectLastMaximum( tExtrem* extrem, int16_t* cntExtrem )
{
  int16_t i;

  // find last minimum
  for( i = ( *cntExtrem - 1 ); i >= 0; i-- )
  {
    if( extrem[i].sign < 0. )
      break;
  }

  *cntExtrem = ( i + 1 );
}

//=========================================================================
/// base constraints
///
/// @param[in] extrem    - pointer to extremum array
/// @param[in] cntExtrem - number of extremum
//=========================================================================
static void baseStepConstraints( tExtrem* extrem, int16_t cntExtrem, double min_step_amplitude_threshold )
{
  int16_t i;
  double deltaMagnitude, deltaTime;

  for( i = 1; i < cntExtrem; i++ )
  {
    if( extrem[i].sign == 0. ) continue;

    deltaMagnitude = extrem[i - 1].magnitude - extrem[i].magnitude;
    deltaTime = extrem[i].time - extrem[i - 1].time;

    if( deltaTime < MAX_05_STEP_DURATION )
    {
      if( ( fabs( deltaMagnitude ) < min_step_amplitude_threshold ) ||
        ( deltaTime < MIN_05_STEP_DURATION ) )
      {
        extrem[i - 1].sign *= 0.5;
        extrem[i].sign *= 0.5;
      }
    }
  }
}

//=========================================================================
/// reject double peaks
///
/// @param[in] extrem    - pointer to extremum array
/// @param[in] cntExtrem - pointer to number of extremum
//=========================================================================
static void  rejectDoublePeaks( tExtrem* extrem, int16_t* cntExtrem )
{
  int16_t i;

  rejectFirstMinimum( extrem, cntExtrem );
  rejectLastMaximum( extrem, cntExtrem );

  for( i = 0; i < ( *cntExtrem - 2 ); i++ )
  {
    // compare two max or two min
    if( ( fabs( extrem[i].sign ) == 0.5 ) && ( extrem[i].sign == extrem[i + 2].sign ) )
    {
      if( ( extrem[i + 2].time - extrem[i].time ) < ( 0.5 * MAX_05_STEP_DURATION ) )
      {
        if( extrem[i].sign == -0.5 )
        {
          if( fabs( extrem[i].magnitude ) < fabs( extrem[i + 2].magnitude ) )
            extrem[i + 2].sign *= 0.5;
          else
            extrem[i].sign *= 0.5;
        }
        else
        {
          if( fabs( extrem[i].magnitude ) > fabs( extrem[i + 2].magnitude ) )
            extrem[i + 2].sign *= 0.5;
          else
            extrem[i].sign *= 0.5;
        }

        extrem[i + 1].sign *= 0.5;
      }
    }
  }
}

//=============================================================================
/// clear rejected
///
/// @param[in] extrem       - pointer to extremum array
/// @param[in] cntExtrem    - pointer to number of extremum
/// @param[in] rejectThresh - threshold for clear rejected
//=============================================================================
static void  clearRejected( tExtrem* extrem, int16_t* cntExtrem, float rejectThresh )
{
  int16_t i, idx;

  idx = 0;

  for( i = 0; i < *cntExtrem; i++ )
  {
    if( fabs( extrem[i].sign ) >= rejectThresh )
    {
      extrem[idx].idx = extrem[i].idx;
      extrem[idx].sign = extrem[i].sign;
      extrem[idx].magnitude = extrem[i].magnitude;
      extrem[idx].time = extrem[i].time;
      idx++;
    }
  }

  *cntExtrem = idx;
}

//=========================================================================
/// reject single extremums
///
/// @param[in] extrem    - pointer to extremum array
/// @param[in] cntExtrem - pointer to number of extremum
//=========================================================================
static void rejectSingleExtremums( tExtrem* extrem, int16_t* cntExtrem )
{
  int16_t i;

  rejectFirstMinimum( extrem, cntExtrem );

  for( i = 0; i < ( *cntExtrem - 1 ); i++ )
  {
    if( ( extrem[i].sign * extrem[i + 1].sign ) > 0. ) // min,min || max,max
    {
      if( extrem[i].sign > 0. )
        extrem[i].sign = 0.; // reject first maximum
      else
        extrem[i + 1].sign = 0.; // reject last minimum
    }
  }

  rejectLastMaximum( extrem, cntExtrem );
}

//=========================================================================
/// reject huge steps
///
/// @param[in] extrem    - pointer to extremum array
/// @param[in] cntExtrem - number of extremum
//=========================================================================
static void rejectHugeSteps( tExtrem* extrem, int16_t cntExtrem, double max_step_amplitude_threshold )
{
  int16_t i;
  double deltaMagnitude, deltaTime;

  for( i = 0; i < ( cntExtrem - 1 ); i++ )
  {
    if( extrem[i].sign > 0. )
    {
      deltaTime = extrem[i + 1].time - extrem[i].time;
      deltaMagnitude = extrem[i].magnitude - extrem[i + 1].magnitude;

      if( ( deltaTime > MAX_05_STEP_DURATION ) || ( fabs( deltaMagnitude ) > max_step_amplitude_threshold ) )
      {
        extrem[i].sign = 0.;
        extrem[i + 1].sign = 0.;
      }
    }
  }
}

//=========================================================================
/// restore boundary extremums
///
/// @param[in] extrem    - pointer to extremum array
/// @param[in] cntExtrem - number of extremum
//=========================================================================
static void restoreBoundaryExtremums( tExtrem* extrem, int16_t cntExtrem )
{
  int16_t i;
  double deltaTime;

  // maximums
  for( i = 0; i < ( cntExtrem - 1 ); i++ )
  {
    if( extrem[i].sign == 0.5 ) // max
    {
      if( i > 0 )
        deltaTime = extrem[i].time - extrem[i - 1].time;
      else
        deltaTime = ( 2.*MAX_05_STEP_DURATION );

      if( ( extrem[i + 1].sign == -1. ) && ( deltaTime > MAX_05_STEP_DURATION ) )
        extrem[i].sign = 1.; // restore maximum
    }
  }

  // minimums
  for( i = 1; i < cntExtrem; i++ )
  {
    if( extrem[i].sign == -0.5 ) // minimum
    {
      if( i < ( cntExtrem - 1 ) )
        deltaTime = extrem[i + 1].time - extrem[i].time;
      else
        deltaTime = ( 2.*MAX_05_STEP_DURATION );

      if( ( extrem[i - 1].sign == 1. ) && ( deltaTime > MAX_05_STEP_DURATION ) )
        extrem[i].sign = -1.; // restore minimum
    }
  }
}

//=========================================================================
/// Obtain steps from extremum information
///
/// @param[in]  extrem    - pointer to extremum array
/// @param[in]  cntExtrem - number of extremum
/// @param[in]  timeSync  - pointer to uniform time scale
/// @param[in]  cntTime   - counter of time scale
/// @param[in]  magnitude - pointer to the magnitude accelerations
/// @param[out] Step      - pointer to steps inforamtion
/// @param[out] cntSteps  - counter of steps
//=========================================================================
static void getSteps( tExtrem* extrem,
                      int16_t cntExtrem,
                      double* timeSync,
                      int16_t cntTime,
                      double* magnitude,
                      tStepParams* Step,
                      int8_t* cntSteps )
{
  int16_t i;
  int8_t idx;

  idx = *cntSteps;

  for( i = 0; i < cntExtrem; i = i + 2 )
  {
    memset( &Step[idx], 0, sizeof( tStepParams ) );
    Step[idx].idxMax = extrem[i].idx;
    Step[idx].idxMin = extrem[i + 1].idx;
    Step[idx].pEvent = fabs( extrem[i].sign * extrem[i + 1].sign );
    Step[idx].DTWcombined = 100.;
    Step[idx].isWalking = 0;

    // bounds detection
    if (idx <= 0 )
    { // there are no available stap before
      Step[idx].idxStart = extrem[i].idx - ( int16_t )( ceil( ( extrem[i + 1].idx - extrem[i].idx ) / 2. ) );
    }
    else if( ( extrem[i].time - Step[idx - 1].timeMin ) > MAX_05_STEP_DURATION )
    { // curent and previouse steps have no bound tugether
      Step[idx].idxStart = extrem[i].idx - ( int16_t )( ceil( ( extrem[i + 1].idx - extrem[i].idx ) / 2. ) );
    }
    else
    { // bound tugether
      Step[idx].idxStart = Step[idx-1].idxEnd + 1;
    }

    if( i >= ( cntExtrem - 2 ) )
    { // last pair of extremums
      Step[idx].idxEnd = extrem[i + 1].idx + ( int16_t )( ceil( ( extrem[i + 1].idx - extrem[i].idx ) / 2. ) );
    }
    else if( ( extrem[i + 2].time - extrem[i + 1].time ) > MAX_05_STEP_DURATION )
    { // curent and next steps have no bound tugether
      Step[idx].idxEnd = extrem[i + 1].idx + ( int16_t )( ceil( ( extrem[i + 1].idx - extrem[i].idx ) / 2. ) );
    }
    else
    { // bound tugether
      Step[idx].idxEnd = ( int16_t )( floor( ( extrem[i + 1].idx + extrem[i + 2].idx ) / 2. ) );
    }

    //if( Step[idx].idxEnd <= cntTime-1 )
    {
      Step[idx].idxStart = Max( Step[idx].idxStart, 0 );
      Step[idx].idxEnd = Min( Step[idx].idxEnd, cntTime - 1 );
      Step[idx].timeStart = timeSync[Step[idx].idxStart];
      Step[idx].timeEnd = timeSync[Step[idx].idxEnd];
      Step[idx].timeMax = timeSync[Step[idx].idxMax];
      Step[idx].timeMin = timeSync[Step[idx].idxMin];
      Step[idx].accMax = magnitude[Step[idx].idxMax];
      Step[idx].accMin = magnitude[Step[idx].idxMin];

      // check step overlaping
      if( ( idx > 0 ) && ( ( Step[idx].idxStart - Step[idx - 1].idxEnd ) < 1 ))
      {
        ReportPDR::LogReport("\nError: (GetSteps) overlaping steps at %f", timeSync[Step[idx].idxStart]);
        assert(0);
      }
      else if ( ( idx > 0 ) && ( ( Step[idx].idxStart - Step[idx - 1].idxEnd ) > 1 ) &&
                ( ( Step[idx].idxStart - Step[idx - 1].idxEnd ) < 4 ))
      {
        ReportPDR::LogReport("\nWarning: (GetSteps) small gap between steps at %f", timeSync[Step[idx].idxStart]);
        //assert(0);
      }

      // check spep validity
      if (Step[idx].accMax <= Step[idx].accMin)
      {
        ReportPDR::LogReport("\nWarning: (GetSteps) invalid step skipped %f", timeSync[Step[idx].idxStart]);
      }
      else
        idx++; // save current new step
    }
  }

  *cntSteps = idx;
}