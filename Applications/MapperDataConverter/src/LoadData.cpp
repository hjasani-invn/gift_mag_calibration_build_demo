/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  load data from file
 *   @file                   LoadData.cpp
 *   @author                 M. Zhokhova
 *   @date                   26.08.2013
 *   @version                1.0
 */
/*****************************************************************************/

#include <string.h>
#include <stdio.h>
#include <io.h> 
#include <direct.h>
#include <ctype.h>
#include "LoadData.h"
#include "pdr_rt/config.h"

#include <iostream>
#include <fstream>


int OpenSensorFile( tLoadData* loadData, std::string file_name, int idx_sensor)
{
  //char fname[MAX_LENGTH_STR];
  //strcpy_s(fname, MAX_LENGTH_STR, file_info->name);
  if( fopen_s(&loadData->fSens[idx_sensor], file_name.c_str(), "rt") != 0 )
  {
    printf("Error: can't open sensor %d data file\n", idx_sensor);
    return 1;
  }
  fgets(loadData->buff[idx_sensor], MAX_LENGTH_STR, loadData->fSens[idx_sensor]);
  while (!isdigit(loadData->buff[idx_sensor][0])) // to finde first data line
  {
    fgets(loadData->buff[idx_sensor], MAX_LENGTH_STR, loadData->fSens[idx_sensor]);
  }
  return 0;
}

//=========================================================================
/// Generate zero data
///
/// @param[in/out] rawData - pointer to data
/// @param[in] - cntRow
/// @param[in] - cntLine
//=========================================================================
void GenerateZeroData( tRawDataFile* rawData, int cntRow, int cntLine)
{
  int i,j;
  for (i = 0; i<cntLine; i++)
    for (j = 0; j<cntRow; j++)
      rawData->data[j][i] = 0.0;
  rawData->cntLine = (int16_t)cntLine;
}

#if NEW_DATA_FORMAT == 0
//=========================================================================
/// Definition names of data files
///
/// @param[in]  path      - pointer to a file folder
/// @param[out] loadData  - to data structure
/// @return Zero if successful
//=========================================================================
int8_t GetDataFileNames(const char* in_path, tLoadData* loadData)
{
  _finddata64i32_t file_info;
  //char fname[MAX_LENGTH_STR];
  char* out_path;

  out_path = _getcwd(NULL, 0);

  if( _chdir(in_path) == 0 )
  {
#if REAL_TIME_EMULATION
    if(( _findfirst64i32("acc_from*.*", &file_info) == -1 ))
#else
    if(( _findfirst64i32("acc_2*.*", &file_info) == -1 ) &&
       ( _findfirst64i32("*_0.csv", &file_info) == -1 ))
#endif
    {
      printf("Error: no accelerometer data file found\n");
      return 1;
    }
    else
      if (OpenSensorFile( loadData, &file_info, IDX_ACC))
        return 1;
#if REAL_TIME_EMULATION
    if(( _findfirst64i32("gyr_from*.*", &file_info) == -1 ))
#else
    if(( _findfirst64i32("gyr_2*.*", &file_info) == -1 ) &&
       ( _findfirst64i32("*_4.csv", &file_info) == -1 ))
#endif
    {
      printf("Error: no gyro data file found\n");
      return 1;
    }
    else
      if (OpenSensorFile( loadData, &file_info, IDX_GYRO))
        return 1;
#if (RAW_DATA_SUPPORT)
    if(( _findfirst64i32("mguncalibr_*.*", &file_info) == -1 ) &&
       ( _findfirst64i32("mgraw_*.*", &file_info) == -1 ))
    {
      printf("Error: no magnetic data file found\n");
      return 1;
    }
    else
      if (OpenSensorFile( loadData, &file_info, IDX_MAGN))
        return 1;
    if( _findfirst64i32("mgbias_*.*", &file_info) == -1 )
    {
      printf("Error: no magnetic bias data file found\n");
      return 1;
    }
    else
      if (OpenSensorFile( loadData, &file_info, IDX_MAG_BIAS))
        return 1;
#else
#if REAL_TIME_EMULATION
    if(( _findfirst64i32("mg_from*.*", &file_info) == -1 ))
#else
    if(( _findfirst64i32("mg_2*.*", &file_info) == -1 ) &&
       ( _findfirst64i32("*_1.csv", &file_info) == -1 ) &&
       ( _findfirst64i32("mgcalibr_2*.*", &file_info) == -1 ))
#endif
      {
        printf("Error: no gyro data file found\n");
        return 1;
      }
    else
      if (OpenSensorFile( loadData, &file_info, IDX_MAGN))
        return 1;

#endif
  }
  else
  {
    printf("Error: can't find specified sensor data path\n");
    return 1;
  }
  _chdir(out_path);
  return 0;
}

#if (REAL_TIME_EMULATION && (RAW_DATA_SUPPORT == 0))
//==============================================================================
/// Load data from file
///
/// @param[in]  LoadData - pointer to instance
/// @param[in]  nSens    - number of sensor
/// @param[out] rawData  - pointer to data
/// @return Zero if successful, 1 - error, 2 - end of file
//==============================================================================
static int8_t readData(tLoadData* loadData, int8_t nSens, tRawDataFile* rawData)
{
  int8_t res;
  int16_t i;
  int32_t idxBlock0, idxBlock;

  //idxBlock0 = idxBlock = -1;
  idxBlock = 0;
  idxBlock0 = loadData->LastBlockID[nSens-1] + 1;
  i = 0;
  rawData->cntLine = 0;

  do
  {
    res = sscanf_s(loadData->buff[nSens-1], "%lld, %d, %lf, %lf, %lf", &rawData->ts[i], &idxBlock, &rawData->data[0][i], 
      &rawData->data[1][i], &rawData->data[2][i]);
    if( res != 5 )
    {
      printf("Error data in file with the data from the sensors\n");
      fclose(loadData->fSens[nSens-1]);
      if (i)
      {
        rawData->cntLine = i;
        return 2;
      }
      else
      {
        return 1;
      }
    }
    //i++;
    //if( idxBlock0 == -1 )
    //  idxBlock0 = idxBlock;
    if( idxBlock == idxBlock0)
    {
      i++;
    }
    else if (idxBlock > idxBlock0)
    {
      loadData->LastBlockID[nSens-1] = idxBlock0;
      rawData->cntLine = i;
      return 0;
    }
    // else if (idxBlock > idxBlock0)
    // skip durrent data
  }
  while( fgets(loadData->buff[nSens-1], MAX_LENGTH_STR, loadData->fSens[nSens-1]) != NULL );

  if( i == 0 )
  {
  printf("Warning: the file with the data from the sensors empty\n");
  return 1;
  }
  rawData->cntLine = i;
  return 2;
}
#else
//==============================================================================
/// Load data from file
///
/// @param[in]  LoadData - pointer to instance
/// @param[in]  nSens    - number of sensor
/// @param[out] rawData  - pointer to data
/// @return Zero if successful, 1 - error, 2 - end of file
//==============================================================================
static int8_t readData(tLoadData* loadData, int8_t nSens, tRawDataFile* rawData)
{
  int res, initTime;
  int16_t i;
  char buff[10];
  double t;

  i = 0;
  initTime = 0;
  do
  {
    if (!strlen(loadData->buff[nSens-1]))
      continue; // skip empty strings
    res = sscanf_s(loadData->buff[nSens-1], "%lld, %lf, %lf, %lf, %lf", &rawData->ts[i], &t, &rawData->data[0][i], 
      &rawData->data[1][i], &rawData->data[2][i]);
    if( res != 5 )
    {
      printf("Error data in file of sensors %d\n",nSens);
      printf("%s\n",loadData->buff[nSens-1]);
      fclose(loadData->fSens[nSens-1]);
      if (i)
      {
        rawData->cntLine = i;
        return 2;
      }
      else
      {
        return 1;
      }
    }
    if (rawData->ts[i] > 1e14) // support of old time format with nanoseconds
    {
       rawData->ts[i] = (int64_t)(rawData->ts[i] * 1e-6);
    }
    i++;
    if( (nSens == 1) && (initTime == 0) )
    {
      loadData->timeEnd = rawData->ts[i-1] + MIN_STEP_DATA;
      initTime = 1;
    }
    //if( rawData->ts[i-1] >= loadData->timeEnd )
    if( rawData->ts[i-1] > loadData->timeEnd )
    {
      //rawData->cntLine = i;
      rawData->cntLine = i-1;
      return 0;
    }
  }while( fgets(loadData->buff[nSens-1], MAX_LENGTH_STR, loadData->fSens[nSens-1]) != NULL );

  if( i == 0 )
  {
    printf("Error: the file with the data from the sensors empty\n");
    return 1;
  }
  rawData->cntLine = i;
  return 2;
}
#endif // REAL_TIME_EMULATION

//=========================================================================
/// Load data from files
///
/// @param[in/out] LoadData - poiner to data for loading data from file
/// @return Zero if successful
//=========================================================================
int8_t LoadDataProcess(tLoadData* LoadData)
{
  int8_t res;

  // load data from the accelerometers
  res = readData(LoadData, 1, &LoadData->accData);
  printf("Time of loaded data: %lf\n", (LoadData->timeEnd/TIME_SCALE));
  if( res == 1 )
    return 1;

  // load gyroscopes data
  if( readData(LoadData, 2, &LoadData->gyrData) == 1 )
    return 1;
  // load magnetic data
  if( readData(LoadData, 3, &LoadData->magnData) == 1 )
    return 1;
#if (RAW_DATA_SUPPORT)
  // load magnetic bias
  if( readData(LoadData, 4, &LoadData->magnBias) == 1 )
    return 1;
#else 
  GenerateZeroData( &LoadData->magnBias, 3, LoadData->magnData.cntLine);
#endif

  if( res == 2)
    return 2;
  return 0;
} 
#endif

#if NEW_DATA_FORMAT == 1
bool FindFile(std::string dir, std::string mask, std::string sensor_name, std::string &file_name)
{
  std::string filter = dir;
  //filter += mask;
  filter = dir + "/" + mask;
  _finddata64i32_t file_info;
  if(( _findfirst64i32(filter.c_str(), &file_info) == -1 ))
  {
      printf("Error: no ""%s"" file found\n", (filter.c_str()));
        return false;
  }
  else
  {
      file_name = dir + "/" + file_info.name;
    return true;
  }
}


//=========================================================================
/// Definition names of data files
///
/// @param[in]  path      - pointer to a file folder
/// @param[out] loadData  - to data structure
/// @return Zero if successful
//=========================================================================
int8_t GetDataFileNames(const char* in_path, tLoadData* loadData)
{
  std::string file_name, dir;
  bool res = true;
  dir = in_path;
    
#if REAL_TIME_EMULATION
  if (FindFile(dir, "acc_from_rt*.*", "acc", file_name))
#else
  if (FindFile(dir, "acc_2*.*", "accelerometer", file_name))
#endif
  {
    if (!OpenSensorFile( loadData, file_name, IDX_ACC))
      res = res && false;
  }

#if REAL_TIME_EMULATION
  if (FindFile(dir, "gyr_from_rt*.*", "gyro", file_name))
#else
  //if (FindFile(dir, "gyr_2*.*", "gyro", file_name))
	if (FindFile(dir, "gyrraw*.*", "gyro", file_name))
#endif
  {
    if (!OpenSensorFile( loadData, file_name, IDX_GYRO))
      res = res && false;
  }

#if REAL_TIME_EMULATION
  if (FindFile(dir, "mg_from_rt*.*", "magnetic", file_name))
#else
  if (FindFile(dir, "mgraw_2*.*", "magnetic", file_name))
#endif
  {
    if (!OpenSensorFile( loadData, file_name, IDX_MAGN))
      res = res && false;
  }

  return res;
}


//==============================================================================
/// Load data from file
///
/// @param[in]  LoadData - pointer to instance
/// @param[in]  nSens    - number of sensor
/// @param[out] rawData  - pointer to data
/// @return Zero if successful, 1 - error, 2 - end of file
//==============================================================================
static int8_t readData(tLoadData* loadData, int8_t nSens, tRawDataFile* rawData)
#if (REAL_TIME_EMULATION)
{
  int8_t res;
  int16_t i;
  int32_t idxBlock0, idxBlock;

  idxBlock = 0;
  idxBlock0 = loadData->LastBlockID[nSens-1] + 1;
  i = 0;
  rawData->cntLine = 0;

  do
  {
    res = sscanf_s(loadData->buff[nSens-1], "%lld, %d, %lf, %lf, %lf,  %lf, %lf, %lf", &rawData->ts[i], &idxBlock, 
                                                                                      &rawData->data[0][i],
                                                                                      &rawData->data[1][i],
                                                                                      &rawData->data[2][i],
                                                                                      &rawData->data_bias[0][i],
                                                                                      &rawData->data_bias[1][i],
                                                                                      &rawData->data_bias[2][i]);
    if ( res == 5 )
      rawData->data_bias[0][i] = rawData->data_bias[1][i] = rawData->data_bias[2][i] =.0;

    if(( res != 5 ) && ( res != 8))
    {
      //printf("Invalid data in sensor file\n");
      printf("data in sensor file is finished\n");
      fclose(loadData->fSens[nSens - 1]);
      return 1;  // invalid data
      //return 0;  // valid data
    }
    if( idxBlock == idxBlock0)
    {
      i++;
    }
    else if (idxBlock > idxBlock0)
    {
      loadData->LastBlockID[nSens-1] = idxBlock0;
      rawData->cntLine = i;
      return 0;
    }

    if ((nSens == 1))
    {
        loadData->timeEnd = rawData->ts[i - 1] + MIN_STEP_DATA;
    }
    // else if (idxBlock > idxBlock0)
    // skip current data
  }
  while( fgets(loadData->buff[nSens-1], MAX_LENGTH_STR, loadData->fSens[nSens-1]) != NULL );

  return 1; // end of file
}
#else
{
  int res, initTime;
  int16_t i;
  char buff[10];
  double t;

  i = 0;
  initTime = 0;
  do
  {
    if (!strlen(loadData->buff[nSens-1]))
      continue; // skip empty strings
    res = sscanf_s(loadData->buff[nSens-1], "%lld, %lf, %lf, %lf, %lf,  %lf, %lf, %lf", &rawData->ts[i], &t, 
                                                                                      &rawData->data[0][i],
                                                                                      &rawData->data[1][i],
                                                                                      &rawData->data[2][i],
                                                                                      &rawData->data_bias[0][i],
                                                                                      &rawData->data_bias[1][i],
                                                                                      &rawData->data_bias[2][i]);
    if ( res == 5 )
      rawData->data_bias[0][i] = rawData->data_bias[1][i] = rawData->data_bias[2][i];

    if(( res != 5 ) && ( res != 8))
    {
      printf("Invalid data in sensor file\n");
      fclose(loadData->fSens[nSens-1]);
      return 1;  // invalid data
    }

		if ( res == 8 )
		{		 //removing biases from gyro data
			rawData->data[0][i] -= rawData->data_bias[0][i];
			rawData->data[1][i] -= rawData->data_bias[1][i];
			rawData->data[2][i] -= rawData->data_bias[2][i];
		}

    if (rawData->ts[i] > 1e14) // support nanoseconds timestamps
    {
       rawData->ts[i] = (int64_t)(rawData->ts[i] * 1e-6);
    }
    i++;
    if( (nSens == 1) && (initTime == 0) )
    {
      loadData->timeEnd = rawData->ts[i-1] + MIN_STEP_DATA;
      initTime = 1;
    }
    //if( rawData->ts[i-1] >= loadData->timeEnd )
    if( rawData->ts[i-1] > loadData->timeEnd )
    {
      //rawData->cntLine = i;
      rawData->cntLine = i-1;
      return 0;
    }
  }while( fgets(loadData->buff[nSens-1], MAX_LENGTH_STR, loadData->fSens[nSens-1]) != NULL );
  return 1;
}
#endif

//=========================================================================
/// Load data from files
///
/// @param[in/out] LoadData - poiner to data for loading data from file
/// @return Zero if successful
//=========================================================================
int8_t LoadDataProcess(tLoadData* LoadData)
{
  int8_t res;
  static int counter = 0;

  // load data from the accelerometers
  if ((res = readData(LoadData, 1, &LoadData->accData)) == 1)
    return 1;

  // load gyroscopes data
  if ((res = readData(LoadData, 2, &LoadData->gyrData)) == 1 )
    return 1;
  
  // load magnetic data
  if ((res = readData(LoadData, 3, &LoadData->magnData)) == 1 )
    return 1;

  if ( !(++counter % 10) )
    printf("Time of loaded data: %lf\n", (LoadData->timeEnd/TIME_SCALE));

  return res;
} 

#endif
