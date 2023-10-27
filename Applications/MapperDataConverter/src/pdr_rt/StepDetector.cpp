/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  step detection module
 *   @file                   StepDetector.cpp
 *   @author                 M. Zhokhova, Y Kotik, D Churikov
 *   @date                   18.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <memory.h>
#include <string.h>
#include "PDR.h"

//=========================================================================
// specific types
enum eDTWMode // DTW mode enumeration
{
  edm_ideal,
  edm_next,
  edm_prev,
  edm_skip_backward,
  edm_skip_forward
};


//=========================================================================
// static functions
static double DTW( double* t, int8_t tsize, double* r, int8_t rsize );
static void DTWCalcOneStepOfThreeSteps(tStepParams* Step, int8_t idx0, int8_t idx1,  int8_t idx2, double* magnitude, double mean_g);
static void stepDTW(tStepParams* Step, int16_t stepIndex, double* magnitude, double mean_g, eDTWMode mode);
static int8_t rejectByDTW(tStepParams* Step, int8_t cntSteps, double* magnitude, double mean_g);
static void clearRejectedSteps(tStepParams* Step, int8_t* cntSteps, tStepParams* VirtStep, int8_t* cntVirtSteps);
static void placeStepsInStepsArray(tStepParams* Step, int8_t* cntSteps, double* timeSync);
static void savingIdxStartFirstStep(tStepParams* Step, int8_t cntSteps, int16_t* idxStartFirstStep);
static void placeStandsInStepsArray(tStepParams* Step, int8_t* cntSteps, double* timeSync, int16_t cntTime, int16_t idxStartFirstStep, int16_t idxStart, bool CalledForVirt);
static void divideStandsInStepsArray(tStepParams* Step, int8_t* cntSteps, double* timeSync, int8_t mode);
static void placeStandsInVirtualSteps(tStepParams* Step, int8_t* cntSteps, double* timeSync, tStepParams* VirtStep, int8_t* cntVirtSteps, int16_t idxStart, int16_t cntTime, int8_t mode);

//=========================================================================
/// Step detection
///
/// @param[in]  isWalking    - pointer to flag 
/// @param[in]  magnitude    - pointer to the magnitude accelerations 
/// @param[in]  idxStart     - start index
/// @param[in]  timeSync     - pointer to uniform time scale
/// @param[in]  cntTime      - counter of time scale
/// @param[in]  SensorParams - pointer to sensor parameters structure
/// @param[out] Step         - pointer to steps inforamtion
/// @param[out] cntSteps     - counter of steps
//=========================================================================
void StepDetector(int8_t* isWalking,
					double* magnitude, 
					int16_t idxStart,
					double* timeSync, 
					int16_t cntTime, 
					tSensorParams* SensorParams,
					tStepParams* Step,
					tStepParams* VirtStep,
					int8_t* cntSteps,
					int8_t* cntVirtSteps)
{
	int8_t i, resDTW = DTW_NO_STEPS;
	int16_t idxStartFirstStep = 0;
	*cntVirtSteps = 0;

	ExtremToSteps(timeSync, cntTime, magnitude, idxStart, SensorParams, Step, cntSteps);

	if( (*isWalking != STANDS) && (*cntSteps == 0) ) // no steps
		*isWalking = STANDS;
	if( *isWalking == SPACE_START_STEPS ) // stop interval before first step
		*isWalking = START_STEPS;
	if( *isWalking == STEPS_FIRST ) // given several first steps
		*isWalking = STEPS;
	// finished a series of steps or had several separate steps or defined false step
	if( (*isWalking == STEPS_END) || (*isWalking == STEPS_SEPARATE) || (*isWalking == SKIP_START_STEPS) ) 
		*isWalking = STANDS;

	// if there are steps you perform calculation of DTW
	if( *cntSteps != 0 )
	{
		if( *isWalking == STANDS )
			*isWalking = SPACE_START_STEPS;
		// separate steps: 3 or the end of a series of steps
		if( ((*cntSteps == MIN_STEPS) || (*cntSteps == REMAIN_STEPS)) && ((timeSync[cntTime-1] - Step[*cntSteps-1].timeEnd) > MAX_STEP_DURATION) )
		{
			if( *cntSteps == MIN_STEPS )
				resDTW = rejectByDTW(Step, *cntSteps, magnitude, SensorParams->acc_mean_g);
			if( (*cntSteps == REMAIN_STEPS) && (*isWalking != STEPS) )
			{
				for( i = 0; i < *cntSteps; i++ )
					Step[i].pEvent *= DTW_COEFF;
			}
			if( *isWalking == STEPS ) // the end of a series of steps
				*isWalking = STEPS_END; 
			else // separate steps (*isWalking = START_STEPS) or (*isWalking = SPACE_START_STEPS)
			{
				if( resDTW == DTW_STEPS )
					*isWalking = STEPS_SEPARATE;
				else
					*isWalking = STANDS;
			}
		}
		else if( *cntSteps > MIN_STEPS ) // sequence of steps, four steps 
		{
			resDTW = rejectByDTW(Step, *cntSteps, magnitude, SensorParams->acc_mean_g);
			if( (*isWalking == STEPS) && (resDTW == DTW_NO_STEPS) )
				*isWalking = STEPS_END; // the end of a series of steps
			if( *isWalking != STEPS ) // beginning of a series of steps
			{
				if( resDTW == DTW_SKIP ) // the space between the first and second steps or skip false steps
					*isWalking = SKIP_START_STEPS;
				if( resDTW == DTW_SPACE ) // the space between the second and the third step in a series of steps beginning
					*isWalking = STEPS_SEPARATE;
				if( resDTW == DTW_STEPS ) // the beginning of a series of steps
					*isWalking = STEPS_FIRST;
			}
		}
		else // one step or series of steps in the process of determining the next step
		{
			if( (*cntSteps == 1) && ((timeSync[cntTime-1] - Step[*cntSteps-1].timeEnd) > MAX_STEP_DURATION) ) // one step
			{
				Step[*cntSteps-1].pEvent *= DTW_COEFF;
				*isWalking = STANDS;
			}
			else // series of steps in the process of determining the next step
			{
				for( i = 0; i < *cntSteps; i++ )
					if( Step[i].isWalking != 1. )
						Step[i].pEvent *= DTW_COEFF;
			}
		}

		if( resDTW == DTW_SPACE ) // the space between the second and the third step in a series of steps or in a series of steps beginning
			placeStepsInStepsArray(Step, cntSteps, timeSync);

		// saving index start of first step
		if( (*isWalking == SPACE_START_STEPS) || (*isWalking == SKIP_START_STEPS) )
		{
			savingIdxStartFirstStep(Step, *cntSteps, &idxStartFirstStep);
			if( idxStartFirstStep == 0 )
				*isWalking = START_STEPS;
		}

		clearRejectedSteps(Step, cntSteps, VirtStep, cntVirtSteps);
	}

	// forming intervals stop
	if( (*isWalking != START_STEPS) && (*isWalking != STEPS) )
	{
		// before real step array is filled with stands, fill virtual array with stand(s) here to send to FF
		placeStandsInVirtualSteps(Step, cntSteps, timeSync, VirtStep, cntVirtSteps, idxStart, cntTime, *isWalking);

		// no step candidates are in buffer
		// 1 stand interval
		placeStandsInStepsArray(Step, cntSteps, timeSync, cntTime, idxStartFirstStep, 0, 0);
		// cntSteps always == 1 here
		
		// Then stand interval is divided into 0.5 second intervals in this function

    divideStandsInStepsArray(Step, cntSteps, timeSync, *isWalking);
	}

}

//=========================================================================
/// DTW calculation for two vectors
static double DTW( double* t, int tsize, double* r, int rsize )
{
  int k = 0;
  int k1 = 0;

  while (k < tsize)
  {
    t[k1] = t[k];
    k1++;
    k = k + 2;
  }
  tsize = k1;

  k = 0; k1 = 0;

  while (k < rsize)
  {
    r[k1] = r[k];
    k1++;
    k = k + 2;
  }
  rsize = k1;

  double D[35][35];
  
  for (int i = 0; i < tsize; i++)
  {
    for (int j = 0; j < rsize; j++)
    {
      D[i][j] = (t[i]-r[j]) * (t[i]-r[j]);
    }
  }

  double d[35][35];
  
  d[0][0] = D[0][0];
  for (int i = 1; i < tsize; i++)
  {
    d[i][0] = D[i][0] + d[i-1][0];
  }
  for (int j = 1; j < rsize; j++)
  {
    d[0][j] = D[0][j] + d[0][j-1];
  }

  double min = 0;

  for (int i = 1; i < tsize; i++)
  {
    for (int j = 1; j < rsize; j++)
    {
      if ((d[i-1][j] <= d[i-1][j-1]) && (d[i-1][j] <= d[i][j-1]))
      {
        min = d[i-1][j];
      }
      else
      {
        if ((d[i-1][j-1] <= d[i-1][j]) && (d[i-1][j-1] <= d[i][j-1]))
        {
          min = d[i-1][j-1];
        }
        else
        {
          min = d[i][j-1];
        }
      }
      d[i][j] = D[i][j] + min;
    }
  }

  return d[tsize - 1][rsize - 1];
}

//=========================================================================
/// DTW analysis
///
/// @param[in/out] Step    - pointer to step inforamtion
/// @param[in]  cntSteps   - counter of steps
/// @param[in]  magnitude  - pointer to the magnitude accelerations 
/// @param[in]  sampleRate - sample rate of data
/// @return 1 if the DTW analysis is made
//=========================================================================
static int8_t rejectByDTW(tStepParams* Step, 
                      int8_t cntSteps,
                      double* magnitude, 
                      double mean_g)
{
  int8_t i, dtw0, dtw1, dtw2, res = DTW_NO_STEPS;
  
  // walking detection
  if( Step[1].isWalking )
  {
    DTWCalcOneStepOfThreeSteps(Step, 2, 1, 0, magnitude, mean_g);
    if( Step[2].DTWcombined < END_WALK_THRESH_DTW )
    {
      Step[2].isWalking = 1;
        if( (Step[2].idxStart - Step[1].idxEnd) > MAX_SMP )
      {
        res = DTW_SPACE;
//        if( cntSteps == MIN_STEPS )
//          res = res;
      }
      else
        res = DTW_STEPS;
    }
    else
      Step[2].isWalking = 0;
  }
  else // new series detection
  {
    if( (Step[1].idxStart - Step[0].idxEnd) > MAX_SMP )
    {
      Step[0].pEvent = SKIP_P_EVENT;
      for( i = 1; i < cntSteps; i++ )
        Step[i].pEvent *= DTW_COEFF;
      res = DTW_SKIP;
      return res;
    }
    else if( (Step[2].idxStart - Step[1].idxEnd) > MAX_SMP )
    {
      Step[0].pEvent = SKIP_P_EVENT;
      Step[1].pEvent = SKIP_P_EVENT;
      res = DTW_SKIP;
      for( i = 2; i < cntSteps; i++ )
        Step[i].pEvent *= DTW_COEFF;
      return res;
    }
    else
    {
      DTWCalcOneStepOfThreeSteps(Step, 0, 1, 2, magnitude, mean_g);
      DTWCalcOneStepOfThreeSteps(Step, 1, 0, 2, magnitude, mean_g);
      DTWCalcOneStepOfThreeSteps(Step, 2, 1, 0, magnitude, mean_g);
      dtw0 = (Step[0].DTWcombined < START_WALK_THRESH_DTW);
      dtw1 = (Step[1].DTWcombined < START_WALK_THRESH_DTW);
      dtw2 = (Step[2].DTWcombined < START_WALK_THRESH_DTW);
      if( dtw0 && dtw1 && dtw2  ) // these are three steps
      {
        Step[0].isWalking = 1;
        Step[1].isWalking = 1;
        Step[2].isWalking = 1;
        res = DTW_STEPS;
      }
      else
      {
        Step[0].pEvent = SKIP_P_EVENT;
        for( i = 1; i < cntSteps; i++ )
          Step[i].pEvent *= DTW_COEFF;
        res = DTW_SKIP;
        return res;
      }
    }
  }

  // step probability correction
  for( i = 0; i < cntSteps; i++ )
  {
    if( Step[i].isWalking == 0 )
    {
      Step[i].pEvent *= QUARTER_P_EVENT;
      if( Step[i].DTWcombined > START_WALK_THRESH_DTW )
        Step[i].pEvent *= HALF_P_EVENT;
      if( Step[i].DTWcombined > END_WALK_THRESH_DTW )
        Step[i].pEvent *= HALF_P_EVENT;
    }
    else
    {
      if( Step[i].DTWcombined <= START_WALK_THRESH_DTW )
        Step[i].pEvent = (float)Min((Step[i].pEvent*2.), 1.);
      if( (cntSteps != MIN_STEPS) && (Step[i].pEvent < HALF_P_EVENT) ) // in series steps
        Step[i].pEvent = (float)Max(Step[i].pEvent, HALF_P_EVENT);
    }
  }
  return res;
}

//=========================================================================
/// DTW analyzes for step in step array: 
/// 0,1,2 or 1,0,2 or 2,1,0
///
/// @param[in/out] Step       - pointer to step inforamtion
/// @param[in]     idx0       - index of verifiable steps
/// @param[in]     idx1       - index of first step
/// @param[in]     idx2       - index of second step
/// @param[in]     magnitude  - pointer to the magnitude accelerations 
/// @param[in]     mean_g  - current mean g value
//=========================================================================
static void DTWCalcOneStepOfThreeSteps(tStepParams* Step, 
                                       int8_t idx0,
                                       int8_t idx1,
                                       int8_t idx2,
                                       double* magnitude, 
                                       double mean_g)
{
	stepDTW(Step, idx0, magnitude, mean_g, edm_ideal);

	if ( Step[idx0].idxStart < Step[idx1].idxStart )   // 0 < 1   
	{     // 012 , 201, 021
		stepDTW(Step, idx0, magnitude, mean_g, edm_next);
		if  ( Step[idx0].idxStart > Step[idx2].idxStart )
		{
			// 201
			stepDTW(Step, idx0, magnitude, mean_g, edm_prev);
			Step[idx0].DTWcombined = Step[idx0].DTWnext * Step[idx0].DTWprev * Step[idx0].DTWideal;
		}
		else
		{   // 012 , 021
			stepDTW(Step, idx0, magnitude, mean_g, edm_skip_forward);
			Step[idx0].DTWcombined = Step[idx0].DTWnext * Step[idx0].DTWskip * Step[idx0].DTWideal;
		}
	}
	else  // 0 > 1
	{     // 102, 120, 210
		stepDTW(Step, idx0, magnitude, mean_g, edm_prev);
		if ( Step[idx0].idxStart < Step[idx2].idxStart ) // 0 < 2 
		{   // 102
			stepDTW(Step, idx0, magnitude, mean_g, edm_next);
			Step[idx0].DTWcombined = Step[idx0].DTWnext * Step[idx0].DTWprev * Step[idx0].DTWideal;
		}
		else // 0 > 2 
		{   // 120, 210
			stepDTW(Step, idx0, magnitude, mean_g, edm_skip_backward);
			Step[idx0].DTWcombined = Step[idx0].DTWprev * Step[idx0].DTWskip * Step[idx0].DTWideal;
		}
	}
}

//=========================================================================

static void stepDTW( tStepParams* Step, int16_t stepIndex, double* magnitude, double mean_g, eDTWMode mode)
{
  double magnitudeStep0[SIZE_OF_ONE_STEP], magnitudeStep1[SIZE_OF_ONE_STEP];
  double magnitudeStepIdeal[SIZE_OF_ONE_STEP];
  double norm = 0;
  int Step0Size, Step1Size, j;
  double x;

  norm = 0;
  j = 0;
  for (int i = Step[stepIndex].idxStart; i <= Step[stepIndex].idxEnd; i++)
  {
    magnitudeStep0[j] = magnitude[i] - mean_g;
    norm += magnitudeStep0[j] * magnitudeStep0[j];
    j++;
  }

  Step0Size = j;

  norm = sqrt(norm);
  for (int i = 0; i < Step0Size; i++)
  {
    magnitudeStep0[i] = magnitudeStep0[i] / norm;
  } // we have normalized our target "step"
    
  if (mode == edm_ideal)
  {
    norm = 0;
    for (int i = 0; i < 29; i++)
    {
      x = i * 2 * M_PI / 29;
      magnitudeStepIdeal[i] = sin(x);
      norm += magnitudeStepIdeal[i] * magnitudeStepIdeal[i];
    }

    norm = sqrt(norm);
    for (int i = 0; i < 29; i++)
    {
      magnitudeStepIdeal[i] = magnitudeStepIdeal[i] / norm;
    } // we have normalized "ideal step"

    Step[stepIndex].DTWideal = DTW(magnitudeStepIdeal, 30, magnitudeStep0, Step0Size);
  }

  if (mode == edm_next)
  {
    norm = 0;
    j = 0;
    for (int i = Step[stepIndex+1].idxStart; i <= Step[stepIndex+1].idxEnd; i++)
    {
      magnitudeStep1[j] = magnitude[i] - mean_g;
      norm += magnitudeStep1[j] * magnitudeStep1[j];
      j++;
    }
    Step1Size = j;

    norm = sqrt(norm);
    for (int i = 0; i < Step1Size; i++)
    {
      magnitudeStep1[i] = magnitudeStep1[i] / norm;
    } // we have normalized next "step"

    Step[stepIndex].DTWnext = DTW(magnitudeStep1, Step1Size, magnitudeStep0, Step0Size);
  }

  if (mode == edm_prev)
  {
    norm = 0;
    j = 0;
    for (int i = Step[stepIndex-1].idxStart; i <= Step[stepIndex-1].idxEnd; i++)
    {
      magnitudeStep1[j] = magnitude[i] - mean_g;
      norm += magnitudeStep1[j] * magnitudeStep1[j];
      j++;
    }
    Step1Size = j;

    norm = sqrt(norm);
    for (int i = 0; i < Step1Size; i++)
    {
      magnitudeStep1[i] = magnitudeStep1[i] / norm;
    } // we have normalized previous "step"

    Step[stepIndex].DTWprev = DTW(magnitudeStep1, Step1Size, magnitudeStep0, Step0Size);
  }

  if (mode == edm_skip_backward)
  {
    norm = 0;
    j = 0;
    for (int i = Step[stepIndex-2].idxStart; i <= Step[stepIndex-2].idxEnd; i++)
    {
      magnitudeStep1[j] = magnitude[i] - mean_g;
      norm += magnitudeStep1[j] * magnitudeStep1[j];
      j++;
    }
    Step1Size = j;

    norm = sqrt(norm);
    for (int i = 0; i < Step1Size; i++)
    {
      magnitudeStep1[i] = magnitudeStep1[i] / norm;
    } // we have normalized previous-1 "step"

    Step[stepIndex].DTWskip = DTW(magnitudeStep1, Step1Size, magnitudeStep0, Step0Size);
  }

  if (mode == edm_skip_forward)
  {
    norm = 0;
    j = 0;
    for (int i = Step[stepIndex+2].idxStart; i <= Step[stepIndex+2].idxEnd; i++)
    {
      magnitudeStep1[j] = magnitude[i] - mean_g;
      norm += magnitudeStep1[j] * magnitudeStep1[j];
      j++;
    }
    Step1Size = j;

    norm = sqrt(norm);
    for (int i = 0; i < Step1Size; i++)
    {
      magnitudeStep1[i] = magnitudeStep1[i] / norm;
    } // we have normalized next+1 "step"

    Step[stepIndex].DTWskip = DTW(magnitudeStep1, Step1Size, magnitudeStep0, Step0Size);
  }
}

//=========================================================================
/// clear rejected steps
///
/// @param[in/out] Steps    - pointer to steps inforamtion
/// @param[in/out] cntSteps - pointer to counter of steps
//=========================================================================
static void clearRejectedSteps(tStepParams* Step, int8_t* cntSteps, tStepParams* VirtStep, int8_t* cntVirtSteps)
{
	// function takes first n "good" steps out of N total steps
	// result : total steps array contains first n steps and cntSteps becomes equal to n
	// example: 3 steps total (1,2,3),  1st is bad, 2nd and 3rd steps are good
	// the result cntSteps == 2, step array is (2,3)
	// ! now some additional virtual step logic is included
	int8_t i, idx;
	int8_t virt_idx;

	if( *cntSteps == 0 )
		return;
	

	idx = 0;
	virt_idx = 0;
	for( i = 0; i < *cntSteps; i++ )
	{
		if(Step[i].pEvent >= STEP_CUTOFF_PROBABILITY ) // clear current step
		{
			if( idx != i )
				CopyStepToStep(&Step[idx], &Step[i]); 
			idx++;
		}
		else
		{
			// converting rejected steps to virtual steps for extrapolation
			CopyStepToStep(&VirtStep[virt_idx], &Step[i]);
			VirtStep[virt_idx].pEvent = 1;
			virt_idx++;
		}
	}
	*cntSteps = idx;
	*cntVirtSteps = virt_idx;
}

//=========================================================================
/// place steps in steps array
///
/// @param[in/out]  Steps    - pointer to steps inforamtion
/// @param[in/out]  cntSteps - pointer to counter of steps
/// @param[in]      timeSync - pointer to uniform time scale
//=========================================================================
static void placeStepsInStepsArray(tStepParams* Step, 
                   int8_t* cntSteps, 
                   double* timeSync)
{
  int8_t i, cnt;
  tStepParams tmpStep[MAX_STEPS];

  cnt = *cntSteps;
  for( i = 2; i < cnt; i++ )
    CopyStepToStep(&tmpStep[i-2], &Step[i]);
  i = 2;
  Step[i].idxStart = Step[i-1].idxEnd + 1; 
  Step[i].idxEnd = tmpStep[0].idxStart - 1; 
  Step[i].idxMax = (int16_t)floor((Step[i].idxStart+Step[i].idxEnd)/2.);
  Step[i].timeStart = timeSync[Step[i].idxStart];
  Step[i].timeEnd = timeSync[Step[i].idxEnd];
  Step[i].timeMax = timeSync[Step[i].idxMax];
  Step[i].pEvent = SPACE_P_EVENT;
  Step[i].accMax = 0.;
  Step[i].accMin = 0.;
  Step[i].DTWideal = 0.;
  Step[i].DTWnext = 0.;
  Step[i].DTWprev = 0.;
  Step[i].DTWskip = 0.;
  Step[i].DTWcombined = 100.;
  Step[i].idxMin = 0;
  Step[i].isWalking = 0;
  Step[i].length = 0.;
  Step[i].timeMin = 0.;
  cnt++;
  for( i = 3; i < cnt; i++ )
    CopyStepToStep(&Step[i], &tmpStep[i-3]);
  *cntSteps = cnt;
  return;
}

//=========================================================================
/// saving index start of first step
///
/// @param[in]  Steps             - pointer to steps inforamtion
/// @param[in]  cntSteps          - pointer to counter of steps
/// @param[out] idxStartFirstStep - pointer to index
//=========================================================================
static void savingIdxStartFirstStep(tStepParams* Step, 
                  int8_t cntSteps, 
                  int16_t* idxStartFirstStep)
{
  int8_t idx;

  for( idx = 0; idx < cntSteps; idx++ )
  {
    if( Step[idx].pEvent != SKIP_P_EVENT )
    {
      *idxStartFirstStep = Step[idx].idxStart;
      return;
    }
  }
}

//=========================================================================
/// place stands in steps array
///
/// @param[in/out]  Steps             - pointer to steps inforamtion
/// @param[in/out]  cntSteps          - pointer to counter of steps
/// @param[in]      timeSync          - pointer to uniform time scale
/// @param[in]      cntTime           - counter of time scale
/// @param[in]      idxStartFirstStep - index start of first step
//=========================================================================
static void placeStandsInStepsArray(tStepParams* Step, 
	int8_t* cntSteps,
	double* timeSync, 
	int16_t cntTime,
	int16_t idxStartFirstStep,
	int16_t idxStart,
	bool CalledForVirt)
{
	int8_t i, idx;
	tStepParams tmpStep[MAX_STEPS];

	idx = 0;
	// first stand before all steps
	// adds one long stand before 1st step, or if 0 steps - until (cntTime-1)
	// here cntSteps = num of actual steps (not stands)
	if( *cntSteps == 0 )
	{	
		Step[idx].idxStart = idxStart; // 0 when called for real steps
		if( idxStartFirstStep == 0 )
			Step[idx].idxEnd = cntTime-1;
		else
			Step[idx].idxEnd = idxStartFirstStep - 1;
		Step[idx].timeStart = timeSync[Step[idx].idxStart];
		Step[idx].timeEnd = timeSync[Step[idx].idxEnd];
		Step[idx].idxMax = (int16_t)floor((Step[idx].idxStart+Step[idx].idxEnd)/2.);
		Step[idx].timeMax = timeSync[Step[idx].idxMax];
		Step[idx].timeMin = 0.;
		Step[idx].pEvent = 0;
		Step[idx].accMax = 0.;
		Step[idx].accMin = 0.;
		Step[idx].DTWideal = 0.;
		Step[idx].DTWnext = 0.;
		Step[idx].DTWprev = 0.;
		Step[idx].DTWskip = 0.;
		Step[idx].DTWcombined = 100.;
		Step[idx].idxMin = 0;
		Step[idx].isWalking = 0;
		Step[idx].length = 0.;
		idx++;	// idx == 1
		*cntSteps = idx; // *cntSteps == 1
		return;
	}
	else if( Step[0].idxStart > 0 )
	{
		// if there are steps, we place one stand before the 1st step, and the idx == 1
		tmpStep[idx].idxStart = idxStart; // 0 when called for real steps
		tmpStep[idx].idxEnd = Step[0].idxStart - 1;
		tmpStep[idx].timeStart = timeSync[tmpStep[idx].idxStart];
		tmpStep[idx].timeEnd = timeSync[tmpStep[idx].idxEnd];
		tmpStep[idx].idxMax = (int16_t)floor((tmpStep[idx].idxStart+tmpStep[idx].idxEnd)/2.);
		tmpStep[idx].timeMax = timeSync[tmpStep[idx].idxMax];
		tmpStep[idx].pEvent = 0;
		tmpStep[idx].accMax = 0.;
		tmpStep[idx].accMin = 0.;
		tmpStep[idx].DTWideal = 0.;
		tmpStep[idx].DTWnext = 0.;
		tmpStep[idx].DTWprev = 0.;
		tmpStep[idx].DTWskip = 0.;
		tmpStep[idx].DTWcombined = 100.;
		tmpStep[idx].idxMin = 0;
		tmpStep[idx].isWalking = 0;
		tmpStep[idx].length = 0.;
		tmpStep[idx].timeMin = 0.;
		idx++; // idx == 1
	}

	for( i = 0; i < *cntSteps; i++ )
	{
		CopyStepToStep(&tmpStep[idx], &Step[i]);
		idx++;
		// if *cntSteps is ever > 1 here, then ( Step[0].idxStart == 0 ) 
		// so,  idx == 0 (initially)
		// in this case nothing changes, steps are copied to tmpStep[], and then back again
		
		// if *cntSteps == 1 and ( Step[0].idxStart > 0 ), then idx == 1 (initially)
		// so we copy Step[0] to tmpStep[1] and idx == 2
		// the next 3 lines of code make Step[0] == stand, Step[1] == Step[0]
	}

	for( i = 0; i < idx; i++ )
		CopyStepToStep(&Step[i], &tmpStep[i]);
	*cntSteps = idx;
}

//=========================================================================
/// divide stands in steps array
///
/// @param[in/out]  Steps    - pointer to steps inforamtion
/// @param[in/out]  cntSteps - pointer to counter of steps
/// @param[in]      timeSync - pointer to uniform time scale
/// @param[in]      mode     - mode of steps
//=========================================================================
static void divideStandsInStepsArray(tStepParams* Step, 
	int8_t* cntSteps,
	double* timeSync, 
	int8_t mode)
{
	int8_t i, idx;
	int16_t idxStart, idxEnd;
	double deltaTime;
	tStepParams tmpStep[MAX_STEPS];
	double timeThr;

	if ( mode == STANDS )
		timeThr = MAX_STEP_DURATION + TIME_OUT_STANDS; // (1.2 + 0.5) seconds
	else
		timeThr = TIME_OUT_STANDS; // 0.5 seconds

	idx = 0;
	for ( i = 0; i < *cntSteps; i++ )
	{
		deltaTime = Step[i].timeEnd - Step[i].timeStart;
		if ( (Step[i].pEvent == 0) && (deltaTime > timeThr) ) // a too long stand
		{
			idxEnd = idxStart = Step[i].idxStart;
			while ( idxStart < Step[i].idxEnd )
			{
				while ( (idxEnd < Step[i].idxEnd) && ((timeSync[idxEnd] - timeSync[idxStart]) < TIME_OUT_STANDS) )
				{
					idxEnd++;
					// this code cuts a stand with maximum allowed duration
					// starting from the start of buffer or from the end of previous stand
				}
				// now the tmpStep (which contains a stand here) parameters are filled with Start and End indexes
				// and Start-End times , and everything else is default (zeros, or big number for DTW)
				tmpStep[idx].idxStart = idxStart;
				tmpStep[idx].idxEnd = idxEnd;
				tmpStep[idx].idxMax = (int16_t)floor((tmpStep[idx].idxStart+tmpStep[idx].idxEnd)/2.);
				tmpStep[idx].timeStart = timeSync[tmpStep[idx].idxStart];
				tmpStep[idx].timeEnd = timeSync[tmpStep[idx].idxEnd];
				tmpStep[idx].timeMax = timeSync[tmpStep[idx].idxMax];
				tmpStep[idx].pEvent = 0;
				tmpStep[idx].accMax = 0.;
				tmpStep[idx].accMin = 0.;
				tmpStep[idx].DTWideal = 0.;
				tmpStep[idx].DTWnext = 0.;
				tmpStep[idx].DTWprev = 0.;
				tmpStep[idx].DTWskip = 0.;
				tmpStep[idx].DTWcombined = 100.;
				tmpStep[idx].idxMin = 0;
				tmpStep[idx].isWalking = 0;
				tmpStep[idx].length = 0.;
				tmpStep[idx].timeMin = 0.;
				idx++;
				idxStart = idxEnd + 1; // previous Step[] now contains a max allowed stand and we continue cutting
			}
		}
		else // not a long stand or a step, we copy it as is to tmpStep
		{
			CopyStepToStep(&tmpStep[idx], &Step[i]);
			idx++;
		}
	}
	for ( i = 0; i < idx; i++ )
		CopyStepToStep(&Step[i], &tmpStep[i]);
	*cntSteps = idx;
	// now we only have steps or stands with normal length in Step[] array
}
//=========================================================================
/// place stands in virtual steps array
///
/// @param[in]		Steps             - pointer to steps 
/// @param[in]		cntSteps          - pointer to counter of steps
/// @param[in]      timeSync          - pointer to uniform time scale
/// @param[in/out]  cntTime           - pointer to virtual steps 
/// @param[in/out]  ntVirtSteps		  - pointer to counter of virtual steps
//=========================================================================
static void placeStandsInVirtualSteps(tStepParams* Step, 
										int8_t* cntSteps, 
										double* timeSync, 
										tStepParams* VirtStep, 
										int8_t* cntVirtSteps,
										int16_t idxStart,
										int16_t cntTime,
										int8_t mode)
{
	int16_t new_idxStart = idxStart;

		
	if ( *cntSteps > 0 ) // we only need period after the last real step
	{
		new_idxStart = Step[*cntSteps - 1].idxEnd + 1;
	}

	// no step candidates are in buffer
	// 1 stand interval

	int16_t idxStartFirstStep = 0;

	if ( *cntVirtSteps > 0 ) 
	{
		idxStartFirstStep = VirtStep[0].idxStart;
	}

	// stands are inserted only if there is a delay between the end of last real step and start of 1st virtual
	if ( ( new_idxStart < cntTime ) && ( new_idxStart < idxStartFirstStep ) )
	{
		placeStandsInStepsArray(VirtStep, cntVirtSteps, timeSync, cntTime, idxStartFirstStep, new_idxStart, 1);

		// Then stand interval is divided into 0.5 second intervals in this function
		divideStandsInStepsArray(VirtStep, cntVirtSteps, timeSync, mode);

	}
}
//=========================================================================
/// copy step to step
///
/// @param[in]  stepDst - pointer to steps inforamtion
/// @param[out] stepSrc - pointer to steps inforamtion
//=========================================================================
void CopyStepToStep(tStepParams* stepDst, tStepParams* stepSrc)
{
  stepDst->accMax = stepSrc->accMax;
  stepDst->accMin = stepSrc->accMin;
  stepDst->DTWideal = stepSrc->DTWideal;
  stepDst->DTWnext = stepSrc->DTWnext;
  stepDst->DTWprev = stepSrc->DTWprev;
  stepDst->DTWskip = stepSrc->DTWskip;
  stepDst->DTWcombined = stepSrc->DTWcombined;
  stepDst->idxEnd = stepSrc->idxEnd;
  stepDst->idxMax = stepSrc->idxMax;
  stepDst->idxMin = stepSrc->idxMin;
  stepDst->idxStart = stepSrc->idxStart;
  stepDst->isWalking = stepSrc->isWalking;
  stepDst->length = stepSrc->length;
  stepDst->pEvent = stepSrc->pEvent;
  stepDst->timeEnd = stepSrc->timeEnd;
  stepDst->timeMax = stepSrc->timeMax;
  stepDst->timeMin = stepSrc->timeMin;
  stepDst->timeStart = stepSrc->timeStart;
}
