#pragma once
#ifndef INTERFACE_SENSORS_CALIBRATOR
#define INTERFACE_SENSORS_CALIBRATOR
#include <iostream>


namespace Calibration
{
    enum class ReturnStatus
    {
        STATUS_SUCCESS = 0,            /**< operation success */
        STATUS_UNKNOWN_ERROR = -1,      /**< undefined error occurs due operation */        
        STATUS_UNREALISTIC_SCALE_FACTOR = -2,      /**< scale factor is very likely to be wrong */
        STATUS_HIGH_DOP = -3,  /**< high dilution of precision detected in input data */        
        STATUS_LOW_CALIBRATION_ACCURACY = -4,  /**< low calibration accuracy detected */
        STATUS_UNSTABLE_TEMPERATURE = -5,      /**< high temperature changes detected during calibration */
        STATUS_TOO_LONG_CALIBRATION_TIME = -6,  /**< calibration time exceeds optimal time */
        STATUS_INSUFFICIENT_DATA = -7,      /**< insufficient data was provided */
        STATUS_ERRORS_IN_INPUT_DATA = -8   /**< input data has errors */
    };

    struct SensorDataSample
    {
        double time; ///< sample time stamp, [sec] 
        double x; ///< sensor data X
        double y; ///< sensor data Y
        double z; ///< sensor data Z
        double temperature;  ///< sensor temperature, [C]
    };

    struct SensorCalibrationParams
    {
        double time; ///< calibration time stamp, [sec] 
        double bx;  ///< bias X
        double by;  ///< bias Y
        double bz;  ///< bias Z
        double CM[3][3];  ///< ellipticity and scale matrix
        double temperature;  ///< sensor temperature through calibration, [C]	
        int8_t calibrationLevel; ///< calibration accuracy level
        double covB[3][3]; ///< calibration accuracy metric, standard sensor data magnitude deviation after calibration
        double DOP; /// estimated dilution of precision during calibration
    };

    class ISensorsCalibrator
    {
    public:
        ISensorsCalibrator() {};
        virtual ~ISensorsCalibrator() {};

        /**
            * Clear data of all sensors
        */
        virtual void ClearSensorsData() = 0;

        /**
            * Adds data of accelerometer sensor
            * \param[in] pDataSamples - pointer to sensor data
            * \param[in] n - number of sensors
            * \return total accumulated sample count
        */
        virtual uint32_t AddAccelData(const SensorDataSample *pDataSamples, const uint32_t n) = 0;

        /**
            * Adds data of of gyro sensor
            * \param[in] pDataSamples - pointer to sensor data
            * \param[in] n - number of sensors
            * \return total accumulated sample count
        */
        virtual uint32_t AddGyroData(const SensorDataSample *pDataSamples, const uint32_t n) = 0;

        /**
            * Adds data of magnetic sensor
            * \param[in] pDataSamples - pointer to sensor data
            * \param[in] n - number of sensors
            * \return total accumulated sample count
        */
        virtual uint32_t AddMagneticData(const SensorDataSample *pDataSamples, const uint32_t n) = 0;

        /**
            * Estimates accel calibration params with accumulated data
            * \param[out] ClbParams - calculated calibration parameters
            * \return calibration success status
        */
        virtual ReturnStatus EstimateAccelCalibrationParams(SensorCalibrationParams& ClbParams) = 0;

        /**
            * Estimates magnetic calibration params with accumulated data
            * \param[out] ClbParams - calculated calibration parameters
            * \return calibration success status
        */
        virtual ReturnStatus EstimateMagneticCalibrationParams(SensorCalibrationParams& ClbParams) = 0;

        /**
            * Estimates magnetic calibration params with accumulated data
            * \param[in] pLog - pointer to output stream
            * \return success status
        */
        virtual bool PrintData(std::ostream& pLog) = 0;
    };

}


#endif