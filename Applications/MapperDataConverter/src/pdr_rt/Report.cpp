/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  Report & log functions
 *   @file                   Report.cpp
 *   @author                 D Churikov
 *   @date                   30.08.2013
 *   @version                1.0
 */
/*****************************************************************************/

#include "config.h"
#include "PDR.h"
#include "Init.h"
#include "Report.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef WIN32
#include <direct.h>
#define _USE_MATH_DEFINES
//char *out_path;
#endif

#ifdef __ANDROID__
#   include <android/log.h>
#   define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "IndoorDemoMap", __VA_ARGS__))
#else
#   define LOGI(...)
#endif

#define MAX_LENGTH_STR 1000

#include <math.h>

//===================================================================================
#define RPT_LOG             0
#define RPT_STEPS           1
#define RPT_PDR             2
#define RPT_ASENSOR_LOG     3
#define RPT_GSENSOR_LOG     4
#define RPT_MSENSOR_LOG     5
#define RPT_A_FIR_LOG       6
#define RPT_G_FIR_LOG       7
#define RPT_M_FIR_LOG       8
#define RPT_ATT_DEBUG1      9
#define RPT_NMEA_LOG        10
#define RPT_FF_LOG          11
//#define REPORT_COUNT        12

//===================================================================================
int8_t ReportPDR::report_init_flag = 0;
FILE*  ReportPDR::fOutArray[REPORT_COUNT];

bool  ReportPDR::report_enabled = false;

std::string ReportPDR::out_path = ".\\";

std::string ReportPDR::log_file_name;
std::string ReportPDR::step_report_file_name;
std::string ReportPDR::prd_report_file_name;
std::string ReportPDR::a_sensor_log_name;
std::string ReportPDR::g_sensor_log_name;
std::string ReportPDR::m_sensor_log_name;
std::string ReportPDR::a_fir_log_name;
std::string ReportPDR::g_fir_log_name;
std::string ReportPDR::m_fir_log_name;
std::string ReportPDR::att_dbg_log_name1;
std::string ReportPDR::nmea_log;
std::string ReportPDR::ff_log_name;

//===================================================================================
/// The function inits reports for our data
void ReportPDR::InitReports()
{
    if( report_enabled )
        if( !report_init_flag )
        {
            memset( fOutArray, 0, sizeof( fOutArray ) );
            report_init_flag = 1;
        }
}

void ReportPDR::EnableReports( bool val )
{
    report_enabled = val;
    //LOGI("ReportPDR::EnableReports: %d", report_enabled);
}
//===================================================================================
/// The function set output path for reports
char ReportPDR::SetOutputPath( const char *out_path0 )
{
    if( out_path0 && strlen( out_path0 ) )
    {
        out_path = out_path0;
        return true;
    }
    else return false;
}

//===================================================================================
/// The function creates report file names
void ReportPDR::InitReportNames( const char* file_name_marker )
{
    std::string file_folder;
    std::string os_prefix;
    std::string marker;
    std::string in_folder;
    std::string out_folder;

#ifdef WIN32
    os_prefix = "c-rt_";
    marker = "_";    marker += file_name_marker;
    file_folder = "";
#elif defined(__APPLE__)
    os_prefix = "i_";
    marker = "_";    marker += file_name_marker;
    file_folder = out_path;
#else
    os_prefix = "a_";
    marker = "_";    marker += file_name_marker;
    file_folder = out_path;
    in_folder = "";
    out_folder = "";
#endif

    if( !file_folder.empty() )
    {
        std::basic_string <char>::const_reference crefStr1 = out_path.at( out_path.size() - 1 );

        if( crefStr1 != '/' )
            file_folder = file_folder + "/";
    }

    log_file_name         = file_folder + out_folder +  os_prefix + "log"       + marker + ".txt";
    step_report_file_name = file_folder + out_folder +  os_prefix + "steps"     + marker + ".txt";
    prd_report_file_name  = file_folder + out_folder +  os_prefix + "pdr"       + marker + ".txt";

    a_sensor_log_name     = file_folder + in_folder +    "acc_from_rt"        + marker + ".log";
    g_sensor_log_name     = file_folder + in_folder +    "gyr_from_rt"        + marker + ".log";
    m_sensor_log_name     = file_folder + in_folder +     "mg_from_rt"        + marker + ".log";

    a_fir_log_name        = file_folder + out_folder +  os_prefix + "fir_acc"   + marker + ".log";
    g_fir_log_name        = file_folder + out_folder +  os_prefix + "fir_gyr"   + marker + ".log";
    m_fir_log_name        = file_folder + out_folder +  os_prefix + "fir_mg"    + marker + ".log";

    att_dbg_log_name1     = file_folder + out_folder +  os_prefix + "att_dbg1"  + marker + ".log";

    nmea_log              = file_folder + out_folder +      "nmea"              + marker + ".log";
    ff_log_name           = file_folder + out_folder +      "fusion_filter"     + marker + ".log";
}

//===================================================================================
/// The function calls the reports with finish flag and closes all file pointers
void ReportPDR::DestroyReports()
{
  int i;

  if( report_enabled )
    if( report_init_flag )
    {
      for( i = 0; i < REPORT_COUNT; i++ )
      {
        //close reports on destroy
#if REOPEN_REPORTS_EVERY_CALL == 0 
        if( fOutArray[i] )
        {
          fclose( ( FILE* )fOutArray[i] );
          fOutArray[i] = 0;
        }
#endif
      }
      report_init_flag = 0;
    }
}

//===================================================================================
/// The function flush all report files
void ReportPDR::FlushReports()
{
    int i;

    if( report_enabled )
        if( report_init_flag )
        {
            for( i = 0; i < REPORT_COUNT; i++ )
            {
                if( fOutArray[i] )
                {
                    fflush( ( FILE* )fOutArray[i] );
                }
            }
        }
}

//=======================================================================
/// The function output string in log file
/// The function automaticaly creates new file during first call
/// The file name is generated automaticaly
///
/// @param[in] logstr - string to output
//=======================================================================
void ReportPDR::LogReport( const char* format, ... )
{
    if( !report_enabled ) return;

#if LOG_REPORT_ON// && REPORTS_ON
    FILE* &fOut = fOutArray[RPT_LOG];

    if( !report_init_flag ) InitReports();

    ReopenReportFile( fOut, log_file_name );

    if( fOut != NULL )
    {
        va_list ptr;
        va_start( ptr, format );
        vfprintf( fOut, format, ptr );
        fflush( fOut );
        va_end( ptr );
      #if REOPEN_REPORTS_EVERY_CALL == 1
        fclose( fOut );
      #endif
    }

#endif // LOG_REPORT_ON  && REPORTS_ON
}

//=======================================================================
/// The function output step report in the file
/// The function automaticaly creates new file during first call
/// The file name is generated automaticaly
///
/// @param[in] CallCounter - counter of PDR call
/// @param[in] Steps - tStepsInfo structure instance
//=======================================================================
void ReportPDR::StepReport( int32_t CallCounter, void *vSteps )
{
    //LOGI("ReportPDR::StepReport: %d", report_enabled);
    if( !report_enabled ) return;

#if STEPS_REPORT_ON //&& REPORTS_ON
    tStepsInfo *Steps = ( tStepsInfo* ) vSteps;
    int8_t i;
    FILE* &fOut = fOutArray[RPT_STEPS];
    bool bPrintHeader = ( fOut == 0 );

    if( !report_init_flag )    InitReports();

    ReopenReportFile( fOut, step_report_file_name );

    // output data
    if( fOut != NULL )
    {
        if( bPrintHeader )
            fprintf( fOut, "Lines format: \nsys_time[sec] call_count p_event  length[m]  heading[deg] d_heading[deg]\n" );

        for( i = 0; i < Steps->cntPfSteps; i++ )
        {
            if( fOut != NULL )
            {
#ifndef PDR_DEBUG
                // fprintf(fOut, "%.3lf %5d %4.2lf %7.2lf %7.2lf %7.2lf %4d %7.2lf %7.2lf\n",
                fprintf( fOut, "%.3lf %5d %8.2lf %12.6lf %12.6lf %12.6lf %4d %12.6lf %12.6lf\n",
                         Steps->pfData[i].timeStart,
                         CallCounter,
                         Steps->pfData[i].pEvent,
                         Steps->pfData[i].length,
                         Steps->pfData[i].heading * 180. / M_PI,
                         Steps->pfData[i].headingInc * 180 / M_PI,
                         Steps->pfData[i].fAttitudeState,
                         Steps->pfData[i].pitch * 180 / M_PI,
                         Steps->pfData[i].roll * 180 / M_PI );
#else
                /*        fprintf(fOut, "%.3lf %5d %5.3lf %7.3lf %7.2lf %7.2lf %12.9lf %12.9lf\n",
                                  Steps->pfData[i].timeStart,
                                  CallCounter,
                                  Steps->pfData[i].pEvent,
                                  Steps->pfData[i].length,
                                  Steps->pfData[i].heading*180./M_PI,
                                  Steps->pfData[i].headingInc*180/M_PI,
                                  Steps->pfData[i].DTWcombined,
                                  Steps->pfData[i].acc_mean_g);
                */
                fprintf( fOut, "%.3lf %5d %5.3lf %7.3lf %7.2lf %7.2lf\n",
                         Steps->pfData[i].timeStart,
                         CallCounter,
                         Steps->pfData[i].pEvent,
                         Steps->pfData[i].length,
                         Steps->pfData[i].heading * 180. / M_PI,
                         Steps->pfData[i].headingInc * 180 / M_PI );
#endif
                //fflush( fOut );
            }
        }
      #if REOPEN_REPORTS_EVERY_CALL == 1
        fclose( fOut );
      #endif
    }

#endif //STEPS_REPORT_ON && REPORTS_ON
}

//=======================================================================
/// The function output general PDR report in file
/// The function automaticaly creates new file during first call
/// The file name is generated automaticaly
///
/// @param[in] pPdr - tPDR structure instance
/// @param[in] pData - tData structure instance
/// @param[in] pUserParams - tUserParams structure instance
/// @param[in] Steps - tStepsInfo structure array
/// @param[in] fSave - flag to save the report file
//=======================================================================
void ReportPDR::PDRReport( void *vPdr,
                           void *vData,
                           void * vUserParams,
                           void * vSteps,
                           int8_t fSave )
{
    if( !report_enabled ) return;

#if PDR_REPORT_ON //&& REPORTS_ON
    static int32_t step_count = 0;
    static int32_t total_interval_count = 0;
    tPDR *pPdr = ( tPDR * )vPdr;
    tData *pData = ( tData * )vData;
    tStepsInfo *Steps = ( tStepsInfo * )vSteps;
    tUserParams *pUserParams = ( tUserParams * )vUserParams;
    int32_t i;

    {
        // to avoid worning
        int tmp = pPdr->isStart;
        tmp = 0;
    }

    FILE* &fOut = fOutArray[RPT_PDR];

    if( !report_init_flag )    InitReports();

    ReopenReportFile( fOut, prd_report_file_name );

#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable: 6011)
#endif

    // step by step calculation
    if( ( Steps != 0 ) || ( fSave == 0 ) )
        for( i = 0; i < Steps->cntPfSteps; i++ )
        {
            total_interval_count++;

            if( Steps->pfData[i].pEvent > 0 )
                step_count++;
        }

#ifdef _MSC_VER
#pragma warning (pop)
#endif

    // output data
    if( ( fSave ) && ( fOut != NULL ) )
    {
        fprintf( fOut, "\nUser parameters:  height=%dcm, sex: %s\n",
                 pUserParams->height,
                 ( pUserParams->sex == es_Female ) ? "female" : "male" );

        fprintf( fOut, "\nInput Data:\n" );
        fprintf( fOut, "  acc sensor:      round sample rate=%.1fHz,  FIR sample rate=%.1fHz\n",
                 pData->accData.smpRate,    pData->accData.smpRateFIR );
        fprintf( fOut, "  gyro sensor:     round sample rate=%.1fHz,  FIR sample rate=%.1fHz\n",
                 pData->gyrData.smpRate,    pData->gyrData.smpRateFIR );
        fprintf( fOut, "  magnetic sensor: round sample rate=%.1fHz,  FIR sample rate=%.1fHz\n",
                 pData->magnData.smpRate,    pData->magnData.smpRateFIR );

        fprintf( fOut, "\nTotal intervals: %d, steps: %d, stands: %d\n",
                 total_interval_count,
                 step_count,
                 total_interval_count - step_count );
      #if REOPEN_REPORTS_EVERY_CALL == 1
        fclose( fOut );
      #endif
    }

#endif //PDR_REPORT_ON && REPORTS_ON
}

//=======================================================================
/// The function logs data from specified sensor
///
/// @param[in] SensorType - type of sensor to log
/// @param[in] CallCounter - counter of PDR call
/// @param[in] st - system time of loged data
/// @param[in] data_0 - sensor data 0
/// @param[in] data_1 - sensor data 1
/// @param[in] data_2 - sensor data 2
//=======================================================================
void ReportPDR::SensorLog( eSensorType SensorType, int32_t CallCounter,
                           double st, double data_0, double data_1, double data_2 )
{
    if( !report_enabled ) return;

#if SENSOR_LOGS_ON //&& REPORTS_ON
    int8_t rpt_id;
    char *fName = ( char * )"";

    // select sensor log
    if( SensorType == est_Accelerometer )
    {
        rpt_id = RPT_ASENSOR_LOG;
        fName = ( char * )a_sensor_log_name.c_str();
    }
    else if( SensorType == est_Gyro )
    {
        rpt_id = RPT_GSENSOR_LOG;
        fName = ( char * )g_sensor_log_name.c_str();
    }
    else if( SensorType == est_Magnetometer )
    {
        rpt_id = RPT_MSENSOR_LOG;
        fName = ( char * )m_sensor_log_name.c_str();
    }
    else
        return;

    FILE* &fOut = fOutArray[rpt_id];

    if( !report_init_flag )      InitReports();

    ReopenReportFile( fOut, fName );

    // output data
    if( fOut != NULL )
    {
        fprintf( fOut, "%lld, %d, %6.3f, %6.3f, %6.3f\n",
                 ( int64_t )( st * 1000 ),
                 CallCounter,
                 data_0,
                 data_1,
                 data_2 );
      #if REOPEN_REPORTS_EVERY_CALL == 1
        fclose( fOut );
      #endif
    }

#endif //SENSOR_LOGS_ON && REPORTS_ON
}

//=======================================================================
/// The function logs data from specified sensor
///
/// @param[in] SensorType - type of sensor to log
/// @param[in] CallCounter - counter of PDR call
/// @param[in] st - system time of loged data
/// @param[in] data_0 - sensor data 0
/// @param[in] data_1 - sensor data 1
/// @param[in] data_2 - sensor data 2
//=======================================================================
void ReportPDR::SensorLogEx( eSensorType SensorType, int32_t CallCounter,
                             double st, double data_0, double data_1, double data_2,
                                        double bias_0, double bias_1, double bias_2)
{
    if( !report_enabled ) return;

#if SENSOR_LOGS_ON //&& REPORTS_ON
    int8_t rpt_id;
    char *fName = ( char * )"";

    // select sensor log
    if( SensorType == est_Accelerometer )
    {
        rpt_id = RPT_ASENSOR_LOG;
        fName = ( char * )a_sensor_log_name.c_str();
    }
    else if( SensorType == est_Gyro )
    {
        rpt_id = RPT_GSENSOR_LOG;
        fName = ( char * )g_sensor_log_name.c_str();
    }
    else if( SensorType == est_Magnetometer )
    {
        rpt_id = RPT_MSENSOR_LOG;
        fName = ( char * )m_sensor_log_name.c_str();
    }
    else
        return;

    FILE* &fOut = fOutArray[rpt_id];

    if( !report_init_flag )      InitReports();

    ReopenReportFile( fOut, fName );

    // output data
    if( fOut != NULL )
    {
        fprintf( fOut, "%lld, %d, %9.6f, %9.6f, %9.6f  %9.6f, %9.6f, %9.6f\n",
                 ( int64_t )( st * 1000 ), CallCounter,
                                  data_0,data_1,data_2,
                                  bias_0,bias_1,bias_2);
      #if REOPEN_REPORTS_EVERY_CALL == 1
        fclose( fOut );
      #endif

    }

#endif //SENSOR_LOGS_ON && REPORTS_ON
}


//=======================================================================
/// The function logs data FIR after FIR from specified sensor
///
/// @param[in] SensorType - type of sensor to log
/// @param[in] st - system time of loged data
/// @param[in] data_0 - sensor data 0
/// @param[in] data_1 - sensor data 1
/// @param[in] data_2 - sensor data 2
//=======================================================================
void ReportPDR::FIRDataLog( eSensorType SensorType,
                            double st, double data_0, double data_1, double data_2 )
{
    if( !report_enabled ) return;

#if FIR_DATA_LOGS_ON //&& REPORTS_ON
    int8_t rpt_id;
    char *fName;

    // select sensor log
    if( SensorType == est_Accelerometer )
    {
        rpt_id = RPT_A_FIR_LOG;
        fName = ( char* )a_fir_log_name.c_str();
    }
    else if( SensorType == est_Gyro )
    {
        rpt_id = RPT_G_FIR_LOG;
        fName = ( char* )g_fir_log_name.c_str();
    }
    else if( SensorType == est_Magnetometer )
    {
        rpt_id = RPT_M_FIR_LOG;
        fName = ( char* )m_fir_log_name.c_str();
    }

    FILE* &fOut = fOutArray[rpt_id];

    if( !report_init_flag )      InitReports();

    ReopenReportFile( fOut, fName );

    // output data
    if( fOut != NULL )
    {
        fprintf( fOut, "%.3f,%15.12f,%15.12f,%15.12f\n",
                 st,   data_0, data_1, data_2 );
      #if REOPEN_REPORTS_EVERY_CALL == 1
        fclose( fOut );
      #endif

    }

#else // to avoid warning
    st = data_0 = data_1 = data_2 = SensorType;
#endif //FIR_DATA_LOGS_ON && REPORTS_ON
}

//=======================================================================
/// The function output step report on the screen
///
/// @param[in] Steps - tStepsInfo structure instance
//=======================================================================
void ReportPDR::ScreenStepReport( void *vSteps )
{
    if( !report_enabled ) return;

#ifdef WIN32
    int8_t i;
    tStepsInfo *Steps = ( tStepsInfo * )vSteps;

    // print data
    for( i = 0; i < Steps->cntPfSteps; i++ )
    {
        if( Steps->pfData[i].pEvent <= 0 )
        {
            printf( "Stand: t=%.3f p=%.3f l=%.3fm heading=%.2fdeg d_heading=%.2fdeg\n\r",
                    Steps->pfData[i].timeStart,
                    Steps->pfData[i].pEvent,
                    Steps->pfData[i].length,
                    Steps->pfData[i].heading,
                    Steps->pfData[i].headingInc );
        }
        else
        {
            printf( "Step: t=%.3f p=%.3f l=%.3fm heading=%.2fdeg d_heading=%.2fdeg\r\n",
                    Steps->pfData[i].timeStart,
                    Steps->pfData[i].pEvent,
                    Steps->pfData[i].length,
                    Steps->pfData[i].heading,
                    Steps->pfData[i].headingInc );
        }
    }

#endif
}

//=======================================================================
/// The function output attitude debug reports
///
//=======================================================================
void ReportPDR::AttDebugReport( void *vStep, double time, double *meanAcc, double *meanMagn )
{
    if( !report_enabled ) return;

    tStepParams *pStep = ( tStepParams * )vStep;
#if ATT_DEBUG_LOGS_ON //&& REPORTS_ON
    int8_t i;
    FILE* &fOut = fOutArray[RPT_ATT_DEBUG1];

    if( !report_init_flag )      InitReports();

    ReopenReportFile( fOut, att_dbg_log_name1 );

    // output data
    if( fOut != NULL )
    {
        //for( i = 0; i < Steps->cntPfSteps; i++ )
        {
            if( fOut != NULL )
            {
                fprintf( fOut, "%.6lf %.6lf  %.15lf %.15lf %.15lf   %.3lf %.3lf %.3lf\n",
                         pStep->timeStart,
                         time,
                         meanAcc[0], meanAcc[1], meanAcc[2],
                         meanMagn[0], meanMagn[1], meanMagn[2] );
              #if REOPEN_REPORTS_EVERY_CALL == 1
                fclose( fOut );
              #endif
            }
        }
    }

#else // to avoid warning
    time = pStep->accMax;
    time = meanAcc[0];
    time = meanMagn[0];
#endif //ATT_DEBUG_LOGS_ON && REPORTS_ON
}

//=======================================================================
/// The function output attitude debug reports
///
//=======================================================================
void ReportPDR::NmeaLog( const char* format, ... )
{
    if( !report_enabled ) return;

#if NMEA_LOG_ON //&& REPORTS_ON
    FILE* &fOut = fOutArray[RPT_NMEA_LOG];

    if( !report_init_flag )    InitReports();

    ReopenReportFile( fOut, nmea_log );

    if( fOut != NULL )
    {
        va_list ptr;
        va_start( ptr, format );
        vfprintf( fOut, format, ptr );
        fflush( fOut );
        va_end( ptr );
      #if REOPEN_REPORTS_EVERY_CALL == 1
        fclose( fOut );
      #endif

    }

#endif // FF_LOG_ON  && REPORTS_ON
}

//=======================================================================
/// The function outputs fusion filter log
///
//=======================================================================
void  ReportPDR::FusionFilterLog( const char* format, ... )
{
    if( !report_enabled ) return;

#if FF_LOG_ON //&& REPORTS_ON
    FILE* &fOut = fOutArray[RPT_FF_LOG];

    if( !report_init_flag )    InitReports();

    ReopenReportFile( fOut, ff_log_name );

    if( fOut != NULL )
    {
        va_list ptr;
        va_start( ptr, format );
        vfprintf( fOut, format, ptr );
        fflush( fOut );
        va_end( ptr );
      #if REOPEN_REPORTS_EVERY_CALL == 1
        fclose( fOut );
      #endif

    }

#endif // FF_LOG_ON  && REPORTS_ON
}

//-----------------------------------------------------------------------------
void  ReportPDR::ReopenReportFile( FILE* &fOut, std::string FileName )
{
    if( !report_enabled ) return;

    std::string open_spec;

    if( fOut == 0 ) // open new session
        open_spec = "wt";
    else  // reopen session
        open_spec = "at";

  #if REOPEN_REPORTS_EVERY_CALL == 0
  if( fOut == 0 ) // open report once
  #endif
  {
    #ifdef WIN32
    {
        // pc-model
        #pragma warning (push)
        #pragma warning (disable: 6031)
        #pragma warning (disable: 6387)
        char fName[MAX_LENGTH_STR];

        _mkdir( out_path.c_str() ); // create destination folder

        // generate filename
        strcpy_s( fName, MAX_LENGTH_STR, out_path.c_str() );

        if( ( fName[strlen( out_path.c_str() ) - 1] != '\\' ) && ( fName[strlen( out_path.c_str() ) - 1] != '/' ) )
            strcat_s( fName, MAX_LENGTH_STR, "\\" );

        strcat_s( fName, MAX_LENGTH_STR, FileName.c_str() );

        //open file
        if( fopen_s( &fOut, fName, open_spec.c_str() ) != 0 )
            printf( "Error. Can`t open %s\n", fName );

        #pragma warning (pop)
    }
    #else
    {
        // android
        fOut = fopen( FileName.c_str(), open_spec.c_str() );

        if( fOut == NULL )
            printf( "%s\n", FileName.c_str() );
    }
    #endif
  }
}
