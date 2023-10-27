/*!
* \copyright   Copyright  TDK, 2021
* \project                SensorCalibrator
* \brief                  Sensor calibration
* \file                   SensorCalibrator.java
* \author                 V. Pentukhov
* \date                   06.05.2021
*/

package com.tdk.sensorcalibrator;

import android.util.Log;

public class SensorCalibrator 
{
	
	static 
        {
	    try 
            {
	        System.loadLibrary("CalibrationLib");
	    } catch (UnsatisfiedLinkError e) 
            {
	      Log.d("CalibrationLib","Native code library failed to load.\n" + e);
	    }
	}

	public SensorCalibrator()
	{
	}

     /**
         * Start magnetic calibration
     */
    public native void startCalibration();	
    
     /**
    	 * Clear data of all sensors
     */
    public native void ClearSensorsData();
    
     /**
    	 * Adds data of accelerometer sensor
         * \param[in] time -   sample time stamp, [sec] 
         * \param[in] x - array of sensor data X
         * \param[in] y - array of sensor data Y
         * \param[in] z - array of sensor data Z
         * \param[in] temperature - array of sensor temperature, [C]
	 * \param[in] Len - number of sensors
	 * \return total accumulated sample count
     */
    public native int AddAccelData(
    	 double[] time,
    	 double[] x,
    	 double[] y,
    	 double[] z,
    	 double[] temperature,
    	 int Len
     );
    
     /**
    	 * Adds data of of gyro sensor
         * \param[in] time -   sample time stamp, [sec] 
         * \param[in] x - array of sensor data X
         * \param[in] y - array of sensor data Y
         * \param[in] z - array of sensor data Z
         * \param[in] temperature - array of sensor temperature, [C]
	 * \param[in] Len - number of sensors
	 * \return total accumulated sample count
     */
    public native int AddGyroData(
    	 double[] time,
    	 double[] x,
    	 double[] y,
    	 double[] z,
    	 double[] temperature,
    	 int Len
    	 );
    
     /**
    	 * Adds data of magnetic sensor
         * \param[in] time -   sample time stamp, [sec] 
         * \param[in] x - array of sensor data X
         * \param[in] y - array of sensor data Y
         * \param[in] z - array of sensor data Z
         * \param[in] temperature - array of sensor temperature, [C]
	 * \param[in] Len - number of sensors
	 * \return total accumulated sample count
     */
    public native int AddMagneticData(
    	 double[] time,
    	 double[] x,
    	 double[] y,
    	 double[] z,
    	 double[] temperature,
    	 int Len
         );
    
     /**
    	 * Estimates accel calibration params with accumulated data
    	 * \param[out] calibrationParams - calculated calibration parameters
    	 * \return calibration success status
     */
    public native int EstimateAccelCalibrationParams(
	 SensorCalibrationParams calibrationParams,
	 double[] CM,
	 double[] covB
     );
    
     /**
    	 * Estimates magnetic calibration params with accumulated data
    	 * \param[out] calibrationParams - calculated calibration parameters
    	 * \return calibration success status
     */
    public native int EstimateMagneticCalibrationParams(
	 SensorCalibrationParams calibrationParams,
	 double[] CM,
	 double[] covB
     );
    
	/**
	 * Open output stream for printing input magnetic and accumulated data
	 * \param[in] fname - name of output file
	 * \return success status
	 */
	public native boolean OpenPrintData(String fname);

	/**
	 * Print input magnetic and accumulated data
	 * \return success status
	 */
	public native boolean PrintData();


	/**
	 * Close output stream for printing input magnetic and accumulated data
	 * \return success status
	 */
	public native boolean ClosePrintData();

}

