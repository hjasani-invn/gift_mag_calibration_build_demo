/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  filtering sensor data module
 *   @file                   FilteringSensorData.cpp
 *   @author                 M. Zhokhova
 *   @date                   11.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#include <math.h>
#include <cassert>
#include "PDR.h"
#include "FIRfilter.h"

static int8_t getSampleRate(double* time, int16_t cntRawData, float* sampleRate);
static int8_t filteringData(tFilter* Filter, tSensData* sensData, float sampleRate, 
                            int8_t initProcess, tMagnitudeAcc* magnitudeData,
                            eSensorType SensorType);

//==============================================================
/// Initialize filtering sensor data: accelerometers, gyroscopes 
/// and magnetic compass
///
/// @param[in]     PDR  - pointer to instance
/// @param[in/out] Data - pointer to data
/// @return Zero if successful
//==============================================================
int8_t InitFilteringSensorData(tPDR* PDR, tData* Data)
{
  float sampleRate = 0.;

  // accelerometer data filtering
  if( getSampleRate(Data->accData.rawData.t, Data->accData.rawData.cntData, &sampleRate) )
    return 1;
  if( filteringData(&PDR->filterAcc, &Data->accData, sampleRate, FILTERING_INIT, &Data->magnitudeData, est_Accelerometer) )
    return 1;

  // gyr data filtering
  if( getSampleRate(Data->gyrData.rawData.t, Data->gyrData.rawData.cntData, &sampleRate) )
    return 1;
  if( filteringData(&PDR->filterGyr, &Data->gyrData, sampleRate, FILTERING_INIT, 0, est_Gyro) )
    return 1;

  // magnetic data filtering
  if( getSampleRate(Data->magnData.rawData.t, Data->magnData.rawData.cntData, &sampleRate) )
    return 1;
  if( filteringData(&PDR->filterMagn, &Data->magnData, sampleRate, FILTERING_INIT, 0, est_Magnetometer) )
    return 1;

  Data->accData.rawData.cntData = 0;
  Data->gyrData.rawData.cntData = 0;
  Data->magnData.rawData.cntData = 0;

  return 0;
}

//==============================================================
/// Filtering sensor data: accelerometers, gyroscopes and
/// magnetic compass
///
/// @param[in]     PDR  - pointer to instance
/// @param[in/out] Data - pointer to data
//==============================================================
void FilteringSensorData(tPDR* PDR, tData* Data)
{
  // accelerometer data filtering
  filteringData(&PDR->filterAcc, &Data->accData, Data->accData.smpRate, FILTERING_PROCESS, &Data->magnitudeData, est_Accelerometer);
  // gyr data filtering
  filteringData(&PDR->filterGyr, &Data->gyrData, Data->gyrData.smpRate, FILTERING_PROCESS, 0, est_Gyro);
  // magnetic data filtering
  filteringData(&PDR->filterMagn, &Data->magnData, Data->magnData.smpRate, FILTERING_PROCESS, 0, est_Magnetometer);

  Data->accData.rawData.cntData = 0;
  Data->gyrData.rawData.cntData = 0;
  Data->magnData.rawData.cntData = 0;

  return;
}

//==============================================================
/// Calculation of sample rate
///
/// @param[in]  time        - pointer to input data
/// @param[in]  cntRawData  - counter of raw data
/// @param[out] sampleRate  - pointer to sample rate
/// @return Zero if successful
//==============================================================
static int8_t getSampleRate(double* time,
                            int16_t cntRawData, 
                            float* sampleRate)
{
  int16_t i;
  double mean_dt;
  float smpRate = 0.;
  double dt[RAW_FLT_DATA];

 /* if (cntRawData < 25)  // wait 25 samples for sample rate calculation
    return 1;*/

  mean_dt = 0.;
  for( i = 0; i < (cntRawData-1); i++ )
  {
    dt[i] = time[i+1] - time[i];
    mean_dt += dt[i];
  }
  mean_dt /= (cntRawData-1);

  // chek stability
  if( (mean_dt > 0.001) && (mean_dt <= 0.25) )
  { // calculate sample rate
    smpRate = (float)(1./mean_dt);
    if( smpRate > 100. )
      *sampleRate = (float)floor(smpRate/20 + 0.5)*20;
    else if( smpRate > 40. )
      *sampleRate  = (float)floor(smpRate/10 + 0.5)*10;
    else if( smpRate > 10. )
      *sampleRate  = (float)floor(smpRate/4 + 0.5)*4;
    else
      *sampleRate  = (float)floor(smpRate/2 + 0.5)*2;
  }
  else // Imposible value. Sensor data period
    return 1;

  return 0;
}

// 10 Hz // float version
/*const double B5_HalfPlusOne[] = { 0.,
                                  0.167675,  0.664650};*/
// 10 Hz // double version
const double B5_HalfPlusOne[] = { 0.,
                                  0.16767497456406691, 0.66465005087186613};

// 16Hz // float version
/*const double B9_HalfPlusOne[] = { 0.,
                                 -0.011175,  0.061947,  0.242812,  0.412833};*/

// 16Hz // double version
const double B9_HalfPlusOne[] =
  { 0.,
   -0.011175107722185241, 0.061946719788814643, 0.2428118696149193, 0.41283303663690268};

// 24 Hz // double version
const double B13_HalfPlusOne[] =
    { 0,
     -0.0083607920357942917, 3.620039249160773e-18, 0.041803960178971457,
      0.11823945489201247,   0.20901980089485728,   0.27859515213990604};
//  { 0,
//    0.01510993784354,  0.04088719717188, 0.07554968921772, 0.1156464575358, 0.1564685295353, 0.1926763773916};

// 48 Hz // float version
/*const double B25_HalfPlusOne[] = { 0.,
                                  -0.002501, -0.004211, -0.003798, 1.823064e-18,
                                   0.008138,  0.021053,  0.038509, 0.059546, 
                                   0.082520,  0.105263,  0.125330, 0.140301};*/
// 48 Hz // double version
const double B25_HalfPlusOne[] =
  { 0,
   -0.002500597399990887, -0.0042105236623636213, -0.0037978649760741236, 1.8230642326733135e-18,
    0.0081382820915874132, 0.021052618311818112,   0.038509199959859672,  0.059545796680074682,
    0.082519714199699307,  0.10526309155909054,    0.12532954421044609,   0.14030147805170559};

// 100 Hz // double version FIR Widow Barret, Fs = 100, Fñ = 3
const double B51_HalfPlusOne[] =
  { 0,
   -0.00058598043185937774, -0.0011575414892665017, -0.0016484112707158031, -0.0019879496589394648, -0.0021038488836543985,
   -0.0019250323508070966,  -0.0013846483997462296, -0.0004230493089319635,  0.0010093565815133298,  0.002949490168891272,
    0.005419329195253569,    0.0084240734719951606,  0.011950829803527284,   0.015967882226016833,   0.020424594005678042,
    0.025251966601241242,    0.030363858207246231,   0.03565884133727646,    0.041022655989096664,   0.046331193069846018,
    0.051453922736235058,    0.056257664855532394,   0.060610584571113103,   0.064386285510307936,   0.067467866926300399};

// 100 Hz // double version FIR Widow Barret, Fs = 100, Fñ = 1
/*const double B51_1_HalfPlusOne[] =
  { 0,
    0.0011327539801418178,   0.0023500045920469568,  0.0036487301668381737,  0.0050255347144615844,  0.0064766585311982865,
    0.0079979903208592818,   0.0095850807876257556,  0.011233157652524316,   0.012937142039729848,   0.014691666173290553,
    0.016491092319493018,    0.01832953290495171,    0.020200871735637463,   0.022098786237473658,   0.024016770634845069,
    0.025948159979400611,    0.02788615493790355,    0.029823847244606405,   0.031754245720715994,   0.033670302760979334,
    0.035564941185274195,    0.037431081351338311,   0.039261668423425246,   0.041049699690741022,   0.042788251828995705};*/

// 200 Hz // double version, Fc = 3
const double B101_HalfPlusOne[] = 
  { 0.,
   -0.00014550431406396591, -0.00029311019169434499, -0.0004389662431090998,  -0.00057900774388739871, -0.00070899313254047642,
   -0.00082454313707625135, -0.0009211822005249366,  -0.00099438184951248761, -0.0010396056271014099,  -0.0010523551914987138,
   -0.0010282171660654193,  -0.00096291031352782489, -0.00085233259853301853, -0.00069260769782203489, -0.00048013051639137307,
   -0.0002116112711199975,   0.00011588228953896072,  0.00050488495009389487,  0.00095749678204031235,  0.0014753489737892368,
    0.0020595731003778444,   0.0027107741707947156,   0.0034290077613937839,   0.0042137615114360965,   0.0050639412215490011,
    0.0059778617581192446,   0.006953242926653694,    0.007987210435285828,    0.009076302026233371,    0.010216478808493941,
    0.01140314177978612,     0.012631153480096481,    0.013894864673579298,    0.015188145910376824,    0.016504423775586616,
    0.017836721589494266,    0.019177704281705028,    0.020519727122322785,    0.021854887956202333,    0.023175082551884954,
    0.024472062645440648,    0.025737496231381222,    0.026963029628348359,    0.028140350826661182,    0.0292612536082381,
    0.030317701917064382,    0.031301893950398701,    0.032206325437403198,    0.033023851572905619,    0.033747747079585333};

// 200 Hz // double version FIR Widow Barret, Fs = 200, Fñ = 1
const double B51_1_HalfPlusOne[] =
  { 0,
    0.00147762356450785, 0.00297906641879413, 0.00450296622639014, 0.00604793378186807, 0.00761255433382575, 
	0.00919538893502779, 0.0107949758183958,  0.0124098317975141,  0.0140384536902909,  0.0156793197643929,  
	0.0173308912030466,  0.0189916135897806,  0.0206599184106607,  0.0223342245725537,  0.0240129399359373,  
	0.0256944628607591,  0.0273771837638319,  0.0290594866862424,  0.0307397508692366,  0.0324163523370380,  
	0.0340876654850459,  0.0357520646718532,  0.0374079258135202,  0.0390536279785363,  0.0406875549819003};


static int SetFilter ( float sampleRate,
                       eSensorType SensorType,
                       double* &ptrPulseResp,
                       uint16_t &length,
                       int8_t &evenOddFIR,
                       float &smpRate,
                       int16_t &index_delay_flt)
{
  int res = 0;
  smpRate = (float)(floor(sampleRate/4.)*4.); // round sample rate 

  // FIR Window (Bartlett) Lowpass filter: 
  if((smpRate == 200.) || (smpRate == 180.) || (smpRate == 220.))// Fs = 200, N = 100 (B = 101)
  {
    ptrPulseResp = (double *)B51_1_HalfPlusOne; // B101_HalfPlusOne; // 
    length = FIR50_DL; // FIR100_DL; // 
    evenOddFIR = 0;
    smpRate = 200.;
    index_delay_flt = (int16_t)(smpRate/2)/4; // (int16_t)(smpRate/2)/2;
  }
  else if((smpRate == 100) || (smpRate == 120.) || (smpRate == 90.)) // Fs = 100, N = 50 (B = 51)
  {
    ptrPulseResp = (double *)B51_HalfPlusOne;
    length = FIR50_DL;
    evenOddFIR = 0;
    index_delay_flt = (int16_t)(smpRate/2)/2;
  }
  else if(((smpRate == 44.) || (smpRate == 48.) || (smpRate == 52.)) && 
          (SensorType == est_Magnetometer))// Fs = 24, N = 12 (B = 13)
  { // for magnetometer
    ptrPulseResp = (double *)B13_HalfPlusOne;
    length = FIR12_DL;
    evenOddFIR = 0;
    smpRate = 50.;
    index_delay_flt = (int16_t)(smpRate/2)/4;
  }
  else if((smpRate == 44.) || (smpRate == 48.) || (smpRate == 52.) || (smpRate == 60.))// Fs = 48, N = 24 (B = 25)
  { // for accelerometer & gyro
    ptrPulseResp = (double *)B25_HalfPlusOne;
    length = FIR24_DL;
    evenOddFIR = 0;
    index_delay_flt = (int16_t)(smpRate/2)/2;
  }
  else if( smpRate == 24. ) // Fs = 24, N = 12 (B = 13)
  {
    ptrPulseResp = (double *)B13_HalfPlusOne;
    length = FIR12_DL;
    evenOddFIR = 0;
    index_delay_flt = (int16_t)(smpRate/2)/2;
  }
  else if( smpRate == 16. ) // Fs = 16, N = 8 (B = 9)
  {
    ptrPulseResp = (double *)B9_HalfPlusOne;
    length = FIR8_DL;
    evenOddFIR = 0;
    index_delay_flt = (int16_t)(smpRate/2)/2;
  }
  else if( smpRate == 8. ) // Fs = 8, N = 4 (B = 5)
  {
    ptrPulseResp = (double *)B5_HalfPlusOne;
    length = FIR4_DL;
    evenOddFIR = 0;
    index_delay_flt = (int16_t)(smpRate/2)/2;
  }
  else if(smpRate == 20.)
  {
    ptrPulseResp = (double *)B9_HalfPlusOne;
    length = FIR8_DL;
    evenOddFIR = 0;
    index_delay_flt = (int16_t)(smpRate/2)/2;
  }
  else // Error! Imposible value. 
    res = 1;

  return res;
}

//===============================================================================
/// Filtering data
///
/// @param[in]      Filter        - pointer to filter structure
/// @param[in/out]  sensData      - pointer to sensors data
/// @param[in]      sampleRate    - sample rate
/// @param[in]      initProcess   - 0 - if init, 1 - if process
/// @param[iun/out] magnitudeData - pointer to magnitude data
/// @return Zero if successful
//===============================================================================
static int8_t filteringData(tFilter* Filter,
                            tSensData* sensData, 
                            float sampleRate, 
                            int8_t initProcess,
                            tMagnitudeAcc* magnitudeData,
                            eSensorType SensorType)
{
  int16_t i, index_delay_flt;
  int8_t j; 

  assert((sensData->rawData.cntData >= 0) && (sensData->rawData.cntData <= RAW_FLT_DATA));
  assert((sensData->fltData.cntData >= 0) && (sensData->fltData.cntData <= RAW_FLT_DATA));

  if( initProcess == FILTERING_INIT )
  {
    float smpRate;
    double* ptrPulseResp;
    uint16_t length;
    int8_t evenOddFIR;

    if (SetFilter ( sampleRate, SensorType, ptrPulseResp, length, evenOddFIR, smpRate, index_delay_flt))
      return 1; // can't set filter

    // init filter
    for( j = 0; j < 3; j++ )
    {
      InitFIRfilter(&Filter->FIR[j], ptrPulseResp, length, Filter->buffDelayLine[j], evenOddFIR);
    }
    if( SensorType == est_Accelerometer )
    {
      InitFIRfilter(&Filter->FIR[3], ptrPulseResp, length, Filter->buffDelayLine[3], evenOddFIR);
    }

    sensData->index_delay_flt = index_delay_flt;
    Filter->delay = sensData->index_delay_flt<<1;
    Filter->delay_t = sensData->index_delay_flt/smpRate; 
    Filter->cntMinStep = (int16_t)ceil(MIN_STEP_DURATION*sampleRate);

    sensData->smpRate = sampleRate;
    sensData->smpRateFIR = smpRate;

    sensData->fltData.cntData = 0;
  }
    
    for( i = 0; i < sensData->rawData.cntData; i++ )
    {
      for( j = 0; j < 3; j++ )
      {
        sensData->fltData.data[j][sensData->fltData.cntData] = PutFIRfilter(&Filter->FIR[j], sensData->rawData.data[j][i]);
        //sensData->fltData.data[j][ sensData->fltData.cntData] -= sensData->bias[j]; // !!!utilyse sensor bias
      }
      if( SensorType == est_Accelerometer ) // !!! magnitude data is not containse sensor bias; todo: calculates magnitude after synchronization
      { //
        magnitudeData->magnitudeRaw[i] = sqrt(sensData->rawData.data[0][i]*sensData->rawData.data[0][i] + 
                                              sensData->rawData.data[1][i]*sensData->rawData.data[1][i] + 
                                              sensData->rawData.data[2][i]*sensData->rawData.data[2][i]);
        magnitudeData->magnitudeFlt[sensData->fltData.cntData] = PutFIRfilter(&Filter->FIR[3], magnitudeData->magnitudeRaw[i]);
      }

      if (sensData->fltData.cntData > 0)
      {
        int64_t t_cur = (int64_t)(sensData->fltData.t[sensData->fltData.cntData-1]*1000.);
        int64_t t_next = (int64_t)((sensData->rawData.t[i] - Filter->delay_t)*1000.);
        if (t_cur >= t_next)
          continue; // skip dublicate time
        // NOTE: We use data with equal time stamp in FIR (see code above) due to mobile OS specific, 
        //       when usualy timestamp is invalidated due to CPU load, but sensor data data is correct.
      }

      sensData->fltData.t[sensData->fltData.cntData] = sensData->rawData.t[i] - Filter->delay_t;

      if (Filter->delay > 0)
        Filter->delay--;  // skip filter start interval
      else 
        sensData->fltData.cntData++;

      assert((sensData->fltData.cntData >= 0) && (sensData->fltData.cntData <= RAW_FLT_DATA));
    }

if (sensData->rawData.cntData < 0 )      assert(0);
if (sensData->rawData.cntData > RAW_FLT_DATA )      assert(0);

  return 0;
}
