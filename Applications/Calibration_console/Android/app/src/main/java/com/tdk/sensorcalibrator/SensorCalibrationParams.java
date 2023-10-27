/*!
*  \copyright   Copyright TDK, 2021
* \project                SensorCalibrator
* \brief                  Sensor calibration
* \file                   SensorCalibrationParams.java
* \author                 V. Pentukhov
* \date                   06.05.2021
*/

package com.tdk.sensorcalibrator;

public class SensorCalibrationParams
{
   public  double time; ///< calibration time stamp, [sec] 
   public  double bx;  ///< bias X
   public  double by;  ///< bias Y
   public  double bz;  ///< bias Z
   public  double[] CM; // must has size 9 eqvivalent [3][3]  ///< elipticity and scale matrix
   public  double temperature;  ///< sensor temperature through calibration, [C]	
   public  char calibrationLevel; ///< calibration accuracy level
   public  double[] covB; // must has size 9 eqvivalent [3][3] ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
   public  double DOP; /// estimated dilution of precision during calibration        
}

