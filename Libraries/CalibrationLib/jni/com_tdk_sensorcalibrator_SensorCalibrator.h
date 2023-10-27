/**
* \copyright       Copyright (C) TDK, 2021
* \project         SensorCalibrator
* \brief           JNI interface class
* \file            com_tdk_sensorcalibrator_SensorCalibrator.h
* \author          V. Pentukhov
* \date            01.05.2021
*/
#include <jni.h>
#include <cstdint>

#ifndef _Included_com_tdk_sensorcalibrator_SensorCalibrator
#define _Included_com_tdk_sensorcalibrator_SensorCalibrator

#ifdef __cplusplus
extern "C" {
#endif

 /**
     * Start magnetic calibration
 */
 void JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_startCalibration(JNIEnv *pEnv, jobject pThis);	

 /**
	 * Clear data of all sensors
 */
 void JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_ClearSensorsData(JNIEnv* pEnv, jobject pThis);

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
 );

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
	 );

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
     );

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
 );

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
     );

 /**
	 * Open output stream for printing input magnetic and accumulated data
	 * \param[in] fname - name of output file
	 * \return success status
 */
 bool JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_OpenPrintData(
	 JNIEnv* pEnv, jobject pThis,
	 jstring jfname);

 /**
	 * Print input magnetic and accumulated data
	 * \return success status
 */
 bool JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_PrintData(
	 JNIEnv* pEnv, jobject pThis);


 /**
	 * Close output stream for printing input magnetic and accumulated data
	 * \return success status
 */
 bool JNICALL Java_com_tdk_sensorcalibrator_SensorCalibrator_ClosePrintData(
	 JNIEnv* pEnv, jobject pThis);


#ifdef __cplusplus
}
#endif
#endif
