/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  Report header file
 *   @file                   Report.h
 *   @author                 D Churikov
 *   @date                   30.08.2013
 *   @version                1.0
 */
/*****************************************************************************/

#ifndef _REPORT_H_
#define _REPORT_H_

#include "config.h"
#include <string>
#include <stdio.h>

// report controll
#define REPORTS_ON            0

#define LOG_REPORT_ON         0
#define STEPS_REPORT_ON       0
#define PDR_REPORT_ON         0
#define SENSOR_LOGS_ON        0
#ifdef PDR_DEBUG
#define FIR_DATA_LOGS_ON      1
#define ATT_DEBUG_LOGS_ON     1
#else
#define FIR_DATA_LOGS_ON      0
#define ATT_DEBUG_LOGS_ON     0
#endif

#define NMEA_LOG_ON          1
#define FF_LOG_ON            1

#define REPORT_COUNT        12

// additional debug reports (out of repotr class)
#define SYNC_DATA_LOGS_ON     0

#define REOPEN_REPORTS_EVERY_CALL 0

//extern char *out_path;

//===================================================================================
// Specific types definition
//===================================================================================

class ReportPDR
{
public:
  static void InitReportNames( const char* file_name_marker);
  static char SetOutputPath(const char *out_path0);
  static void InitReports();
  static void DestroyReports();
  static void FlushReports();
  static void LogReport(const char* format, ...);
  static void StepReport(int32_t CallCounter,void *vSteps);
  static void PDRReport(void *vPdr, void *vData, void *vUserParams, void *vSteps, int8_t fSave);
  static void SensorLog(eSensorType SensorType, int32_t CallCounter, double st,
                        double data_0, double data_1, double data_2);
  static void SensorLogEx( eSensorType SensorType, int32_t CallCounter,
                           double st, double data_0, double data_1, double data_2,
                           double bias_0, double bias_1, double bias_2);
  static void FIRDataLog(eSensorType SensorType, double st, double data_0, double data_1, double data_2);
  static void ScreenStepReport(void *vSteps);
  static void AttDebugReport(void *vStep, double time, double *meanAcc, double *meanMagn);
  static void NmeaLog(const char* format, ...);
  static void FusionFilterLog(const char* format, ...);
  static void EnableReports(bool val);

private:
   ReportPDR() {};
  ~ReportPDR() {};

  static void  ReopenReportFile(FILE* &fOut, std::string FileName);

  static std::string log_file_name;
  static std::string step_report_file_name;
  static std::string prd_report_file_name;
  static std::string a_sensor_log_name;
  static std::string g_sensor_log_name;
  static std::string m_sensor_log_name;
  static std::string a_fir_log_name;
  static std::string g_fir_log_name;
  static std::string m_fir_log_name;
  static std::string att_dbg_log_name1;
  static std::string nmea_log;
  static std::string ff_log_name;

  static std::string out_path;

  static int8_t report_init_flag;
  static FILE* fOutArray[REPORT_COUNT];

  static bool report_enabled;
};


#endif
