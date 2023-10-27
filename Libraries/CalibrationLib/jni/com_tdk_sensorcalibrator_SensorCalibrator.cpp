/**
* \copyright       Copyright (C) TDK, 2021
* \project         SensorCalibrator
* \brief           JNI interface class
* \file            com_tdk_sensorcalibrator_SensorCalibrator.cpp
* \author          V. Pentukhov
* \date            01.05.2021
*/
#include <memory>
#include <deque>
#include <queue>
#include <stdint.h>
#include <stdlib.h>
#include <fstream>

#include "com_tdk_sensorcalibrator_SensorCalibrator.h"
#include "SensorsCalibrator.h"


//#ifdef __ANDROID__
//#   include <android/log.h>
//#	ifdef LOGI
//#		undef LOGI
//#	endif
//#   define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "FusionFilter", __VA_ARGS__))
//#else
#   define LOGI(...)
//#endif

static Calibration::SensorsCalibrator*pCalibr;
static Calibration::SensorCalibrationParams* pClbParams;
static std::ofstream pLog;

/**
     * Start magnetic calibration
 */
 void JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_startCalibration(JNIEnv *pEnv, jobject pThis)
 {
        pCalibr = new Calibration::SensorsCalibrator();
        pClbParams = new Calibration::SensorCalibrationParams();
 }

 /**
	  * Clear data of all sensors
  */
 void JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_ClearSensorsData(JNIEnv* pEnv, jobject pThis)
 {
	 pCalibr->ClearSensorsData();
 }

 /**
	 * Adds data of accelerometer sensor
         * \param[in] jtime -   sample time stamp, [sec] 
         * \param[in] jx - array of sensor data X
         * \param[in] jy - array of sensor data Y
         * \param[in] jz - array of sensor data Z
         * \param[in] jtemperature - array of sensor temperature, [C]
	 * \param[in] jLen - number of sensors
	 * \return total accumulated sample count
 */
 jint JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_AddAccelData(
	 JNIEnv* pEnv, jobject pThis,
	 jdoubleArray jtime,
	 jdoubleArray jx,
	 jdoubleArray jy,
	 jdoubleArray jz,
	 jdoubleArray jtemperature,
	 jint jLen
 )
 {
	 int n = jLen;

	 double* time;
	 double* x;
	 double* y;
	 double* z;
	 double* temperature;

	 Calibration::SensorDataSample* sensorData;

	 time = new double[n];
	 x = new double[n];
	 y = new double[n];
	 z = new double[n];
	 temperature = new double[n];
	 sensorData = new  Calibration::SensorDataSample[n];

	 pEnv->GetDoubleArrayRegion(jtime, 0, jLen, time);
	 pEnv->GetDoubleArrayRegion(jx, 0, jLen, x);
	 pEnv->GetDoubleArrayRegion(jy, 0, jLen, y);
	 pEnv->GetDoubleArrayRegion(jz, 0, jLen, z);
	 pEnv->GetDoubleArrayRegion(jtemperature, 0, jLen, temperature);

	 for (int i = 0; i < n; i++)
	 {
		 sensorData[i].time = time[i];
		 sensorData[i].x = x[i];
		 sensorData[i].y = y[i];
		 sensorData[i].z = z[i];
		 sensorData[i].temperature = temperature[i];
	 }
	 uint32_t m = pCalibr->AddAccelData(sensorData, n);

	 delete[] time;
	 delete[] x;
	 delete[] y;
	 delete[] z;
	 delete[] temperature;
	 delete[] sensorData;

	 return m;
 }

 /**
	 * Adds data of of gyro sensor
         * \param[in] jtime -   sample time stamp, [sec] 
         * \param[in] jx - array of sensor data X
         * \param[in] jy - array of sensor data Y
         * \param[in] jz - array of sensor data Z
         * \param[in] jtemperature - array of sensor temperature, [C]
	 * \param[in] jLen - number of sensors
	 * \return total accumulated sample count
 */
 jint JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_AddGyroData(
	 JNIEnv* pEnv, jobject pThis,
	 jdoubleArray jtime,
	 jdoubleArray jx,
	 jdoubleArray jy,
	 jdoubleArray jz,
	 jdoubleArray jtemperature,
	 jint jLen
 )
 {
	 int n = jLen;

	 double* time;
	 double* x;
	 double* y;
	 double* z;
	 double* temperature;

	 Calibration::SensorDataSample* sensorData;

	 time = new double[n];
	 x = new double[n];
	 y = new double[n];
	 z = new double[n];
	 temperature = new double[n];
	 sensorData = new  Calibration::SensorDataSample[n];

	 pEnv->GetDoubleArrayRegion(jtime, 0, jLen, time);
	 pEnv->GetDoubleArrayRegion(jx, 0, jLen, x);
	 pEnv->GetDoubleArrayRegion(jy, 0, jLen, y);
	 pEnv->GetDoubleArrayRegion(jz, 0, jLen, z);
	 pEnv->GetDoubleArrayRegion(jtemperature, 0, jLen, temperature);

	 for (int i = 0; i < n; i++)
	 {
		 sensorData[i].time = time[i];
		 sensorData[i].x = x[i];
		 sensorData[i].y = y[i];
		 sensorData[i].z = z[i];
		 sensorData[i].temperature = temperature[i];
	 }
	 uint32_t m = pCalibr->AddGyroData(sensorData, n);

	 delete[] time;
	 delete[] x;
	 delete[] y;
	 delete[] z;
	 delete[] temperature;
	 delete[] sensorData;
	 
	 return m;
 }

 /**
	 * Adds data of magnetic sensor
         * \param[in] jtime -   sample time stamp, [sec] 
         * \param[in] jx - array of sensor data X
         * \param[in] jy - array of sensor data Y
         * \param[in] jz - array of sensor data Z
         * \param[in] jtemperature - array of sensor temperature, [C]
	 * \param[in] jLen - number of sensors
	 * \return total accumulated sample count
 */
 jint JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_AddMagneticData(
	 JNIEnv* pEnv, jobject pThis,
	 jdoubleArray jtime,
	 jdoubleArray jx,
	 jdoubleArray jy,
	 jdoubleArray jz,
	 jdoubleArray jtemperature,
	 jint jLen
 )
 {
	 int n = jLen;

	 double* time;
	 double* x;
	 double* y;
	 double* z;
	 double* temperature;

	 Calibration::SensorDataSample* sensorData;

	 time = new double[n];
	 x = new double[n];
	 y = new double[n];
	 z = new double[n];
	 temperature = new double[n];
	 sensorData = new  Calibration::SensorDataSample[n];

	 pEnv->GetDoubleArrayRegion(jtime, 0, jLen, time);
	 pEnv->GetDoubleArrayRegion(jx, 0, jLen, x);
	 pEnv->GetDoubleArrayRegion(jy, 0, jLen, y);
	 pEnv->GetDoubleArrayRegion(jz, 0, jLen, z);
	 pEnv->GetDoubleArrayRegion(jtemperature, 0, jLen, temperature);

	 for (int i = 0; i < n; i++)
	 {
		 sensorData[i].time = time[i];
		 sensorData[i].x = x[i];
		 sensorData[i].y = y[i];
		 sensorData[i].z = z[i];
		 sensorData[i].temperature = temperature[i];
	 }
	 uint32_t m = pCalibr->AddMagneticData(sensorData, n);

	 delete[] time;
	 delete[] x;
	 delete[] y;
	 delete[] z;
	 delete[] temperature;
	 delete[] sensorData;
	 
	 return m;
 }

 /**
	 * Estimates accel calibration params with accumulated data
	 * \param[out] calibrationParams - calculated calibration parameters
	 * \return calibration success status
 */
 jint JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_EstimateAccelCalibrationParams(
	 JNIEnv* pEnv, jobject pThis,
	 jobject calibrationParams,
     jdoubleArray jCM,
     jdoubleArray jcovB
 )
 {
	 Calibration::ReturnStatus status = pCalibr->EstimateAccelCalibrationParams(*pClbParams);

	 /* get the class */
	 jclass classCalibrationParams = pEnv->GetObjectClass(calibrationParams);
	 /* get the field ID */
	 jfieldID id_time = pEnv->GetFieldID(classCalibrationParams, "time", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_time, pClbParams->time); ///< calibration time stamp, [sec]

	 /* get the field ID */
	 jfieldID id_bx = pEnv->GetFieldID(classCalibrationParams, "bx", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_bx, pClbParams->bx); ///< bias X

	 /* get the field ID */
	 jfieldID id_by = pEnv->GetFieldID(classCalibrationParams, "by", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_by, pClbParams->by); ///< bias Y

	 /* get the field ID */
	 jfieldID id_bz = pEnv->GetFieldID(classCalibrationParams, "bz", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_bz, pClbParams->bz); ///< bias Z

	 /* get the field ID */
	 jfieldID id_temperature = pEnv->GetFieldID(classCalibrationParams, "temperature", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_temperature, pClbParams->temperature); ///< sensor temperature through calibration, [C]

	 /* get the field ID */
	 jfieldID id_DOP = pEnv->GetFieldID(classCalibrationParams, "DOP", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_DOP, pClbParams->DOP); /// estimated dilution of precision during calibration

         /* get the field ID */
	 jfieldID id_calibrationLevel = pEnv->GetFieldID(classCalibrationParams, "calibrationLevel", "C");
	 /* set the field value */
	 pEnv->SetCharField(calibrationParams, id_calibrationLevel, pClbParams->calibrationLevel); ///< calibration time stamp, [sec]

	 //jdoubleArray CM;
	 pEnv->SetDoubleArrayRegion(jCM, 0, 9, (double*)pClbParams->CM);
	 /* get the field ID */
	 //jfieldID id_CM = pEnv->GetFieldID(classCalibrationParams, "CM", "D");
	 /* set the field value */
	 //pEnv->SetObjectField(calibrationParams, id_CM, CM); ///< calibration time stamp, [sec]

	 //jdoubleArray covB;
	 pEnv->SetDoubleArrayRegion(jcovB, 0, 9, (double*)pClbParams->covB);
	 /* get the field ID */
	 //jfieldID id_covB = pEnv->GetFieldID(classCalibrationParams, "covB", "D");
	 /* set the field value */
	 //pEnv->SetObjectField(calibrationParams, id_covB, covB); ///< calibration time stamp, [sec]

	 return (jint)status;
 }


 /**
	 * Estimates magnetic calibration params with accumulated data
	 * \param[out] calibrationParams - calculated calibration parameters
	 * \return calibration success status
 */
 jint JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_EstimateMagneticCalibrationParams(
	 JNIEnv* pEnv, jobject pThis,
	 jobject calibrationParams,
     jdoubleArray jCM,
     jdoubleArray jcovB
 )
 {
	 Calibration::ReturnStatus status = pCalibr->EstimateMagneticCalibrationParams(*pClbParams);

	 /* get the class */
	 jclass classCalibrationParams = pEnv->GetObjectClass(calibrationParams);
	 /* get the field ID */
	 jfieldID id_time = pEnv->GetFieldID(classCalibrationParams, "time", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_time, pClbParams->time); ///< calibration time stamp, [sec]

	 /* get the field ID */
	 jfieldID id_bx = pEnv->GetFieldID(classCalibrationParams, "bx", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_bx, pClbParams->bx); ///< bias X

	 /* get the field ID */
	 jfieldID id_by = pEnv->GetFieldID(classCalibrationParams, "by", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_by, pClbParams->by); ///< bias Y

	 /* get the field ID */
	 jfieldID id_bz = pEnv->GetFieldID(classCalibrationParams, "bz", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_bz, pClbParams->bz); ///< bias Z

	 /* get the field ID */
	 jfieldID id_temperature = pEnv->GetFieldID(classCalibrationParams, "temperature", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_temperature, pClbParams->temperature); ///< sensor temperature through calibration, [C]

	 /* get the field ID */
	 jfieldID id_DOP = pEnv->GetFieldID(classCalibrationParams, "DOP", "D");
	 /* set the field value */
	 pEnv->SetDoubleField(calibrationParams, id_DOP, pClbParams->DOP); /// estimated dilution of precision during calibration

	 /* get the field ID */
	 jfieldID id_calibrationLevel = pEnv->GetFieldID(classCalibrationParams, "calibrationLevel", "C");
	 /* set the field value */
	 pEnv->SetCharField(calibrationParams, id_calibrationLevel, pClbParams->calibrationLevel); ///< calibration time stamp, [sec]

	 //jdoubleArray CM;
	 pEnv->SetDoubleArrayRegion(jCM, 0, 9, (double *)pClbParams->CM);
	 /* get the field ID */
	 ///jfieldID id_CM = pEnv->GetFieldID(classCalibrationParams, "CM", "D");
	 /* set the field value */
	 //pEnv->SetObjectField(calibrationParams, id_CM, CM); ///< calibration time stamp, [sec]

	 //jdoubleArray covB;
	 pEnv->SetDoubleArrayRegion(jcovB, 0, 9, (double*)pClbParams->covB);
	 /* get the field ID */
	 //jfieldID id_covB = pEnv->GetFieldID(classCalibrationParams, "covB", "D");
	 /* set the field value */
	 //pEnv->SetObjectField(calibrationParams, id_covB, covB); ///< calibration time stamp, [sec]

	 return (jint)status;
 }

 /**
	 * Open output stream for printing input magnetic and accumulated data
	 * \param[in] fname - name of output file
	 * \return success status
 */
 bool JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_OpenPrintData(
	 JNIEnv* pEnv, jobject pThis,
	 jstring jfname)
 {
         const char *cstr = pEnv->GetStringUTFChars(jfname, NULL);
         std::string str = std::string(cstr);
         pEnv->ReleaseStringUTFChars(jfname, cstr);
         pLog.open(str);
         return true;
 }

 /**
	 * Print input magnetic and accumulated data
	 * \return success status
 */
 bool JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_PrintData(
	 JNIEnv* pEnv, jobject pThis)
 {
         return pCalibr->PrintData(pLog);
 }

 /**
	 * Close output stream for printing input magnetic and accumulated data
	 * \return success status
 */
 bool JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_ClosePrintData(
	 JNIEnv* pEnv, jobject pThis)
 {
	 pLog.close();
         return true;
 }


