#pragma once
#ifndef SENSORS_CALIBRATOR
#define SENSORS_CALIBRATOR

#include <eigen/LU>
#include <eigen/Geometry>
#include <eigen/Core>
#include <eigen/Dense>
#include "ISensorsCalibrator.h"
#include <vector>
#include <algorithm>
#include <math.h>

namespace Calibration
{
    class SensorsCalibrator : public ISensorsCalibrator
    {
    public:
        SensorsCalibrator() 
        {
            average_temperature = 0.0;
            dop_x = 100.0;
            dop_y = 100.0;
            dop_z = 100.0;
            calibration_status = Calibration::ReturnStatus::STATUS_SUCCESS;
        };

        ~SensorsCalibrator() {};

        /**
            * Clear data of all sensors
        */
        void ClearSensorsData();

        /**
            * Adds data of accelerometer sensor
            * \param[in] pDataSamples - pointer to sensor data
            * \param[in] n - number of sensors
            * \return - total accumulated sample count
        */
        uint32_t AddAccelData(const SensorDataSample* pDataSamples, const uint32_t n);

        /**
            * Adds data of of gyro sensor
            * \param[in] pDataSamples - pointer to sensor data
            * \param[in] n - number of sensors
            * \return - total accumulated sample count
        */
        uint32_t AddGyroData(const SensorDataSample* pDataSamples, const uint32_t n);

        /**
            * Adds data of magnetic sensor
            * \param[in] pDataSamples - pointer to sensor data
            * \param[in] n - number of sensors
            * \return - total accumulated sample count
        */
        uint32_t AddMagneticData(const SensorDataSample* pDataSamples, const uint32_t n);

        /**
            * Estimates accel calibration params with accumulated data
            * \param[out] ClbParams - calculated calibration parameters
            * \return - calibration success status
        */
        ReturnStatus EstimateAccelCalibrationParams(SensorCalibrationParams& ClbParams);

        /**
            * Estimates magnetic calibration params with accumulated data
            * \param[out] ClbParams - calculated calibration parameters
            * \return - calibration success status
        */
        ReturnStatus EstimateMagneticCalibrationParams(SensorCalibrationParams& ClbParams);

        /**
            * Estimates magnetic calibration params with accumulated data
            * \param[in] pLog - pointer to output stream
            * \return - success status
        */
        bool PrintData(std::ostream& pLog);

    private:
        /** Least Square Eliposid Fitting calibration function
        * \param[in] calibrationData - vector of measurements
        * \param[out] calibrationParams - 3x3 matrix for soft iron/non-orthgonality/gain compensation, 3x1 hard iron vector (magnetometer/accelerometer bias)
        * \return - success status
        */
        Calibration::ReturnStatus LSEF_calibration(std::vector<SensorDataSample>& dataSelectedForCalibration, SensorCalibrationParams& calibrationParams);

        /** Eliposid Fitting function
        * \param[in] S11 - a matrix used for calculations
        * \param[in] S22_1 - a matrix used for calculations
        * \param[in] S12 - a matrix used for calculations
        * \param[in] q - a value used to limit the number of iterations
        * \param[out] a - an output vector
        * \param[out] Q - an output matrix
        * \param[out] cnd - output result (-1 if failed on first iteration)
        */
        void elfit(Eigen::MatrixXd &S11, Eigen::MatrixXd &S22_1, Eigen::MatrixXd &S12, double q, Eigen::VectorXd &a, Eigen::Matrix3d &Q, int &cnd);
        
        /**  Function for estimating bias uncertainty after calibration
        * \param[in] calibratedData - a vector of input data with calibration correction applied to it
        * \return - estimated bias uncertainty value 
        */
        double estimateBiasUncertainty(std::vector<SensorDataSample>& calibratedData);

        /**  Function for estimating DOP after calibration
        * \param[in] calibratedData - a vector of input data with calibration correction applied to it
        * \param[out] DOP_x - x component of DOP
        * \param[out] DOP_y - y component of DOP
        * \param[out] DOP_z - z component of DOP
        * \return - estimated DOP value
        */
        double estimateDOP(std::vector<SensorDataSample>& calibratedData, double& DOP_x, double& DOP_y, double& DOP_z);

        /**  Function for estimating temperature variations before calibration
        * \param[in] sensorData - a vector of input data 
        * \return - estimated temperature SD value
        */
        double estimateTemperatureVariations(std::vector<SensorDataSample>& sensorData);

        /**  Function for estimating sensor data divergency before calibration
        * \param[in] sensorData - a vector of input data
        * \return - calibration duration (in seconds)
        */
        double estimateSensorDataDivergency(std::vector<SensorDataSample>& sensorData);

        /**  Function for selecting a reduced vector of sensor data for calibration taken when sensors are not moving
        * \param[in] sensorData - a vector of input data
        * \param[in] averagingInterval - averaging interval used for filtering data
        * \param[in] threshold - value used to find stationary time intervals (it is different for magnetometer and accelerometer)
        * \param[out] dataSelectedForCalibration - a reduced vector of sensor data for calibration taken when sensors are not moving
        * \return - success status
        */
        bool selectSensorDataForCalibration(std::vector<SensorDataSample>& sensorData, double averagingInterval, double threshold, std::vector<SensorDataSample>& dataSelectedForCalibration);

        /**  Function for finding an specific element in a vector of timestamps
        * \param[in] ti - a given timestamp
        * \param[in] v - a vector of timestamps
        * \return - index of element in vector which occurs right after the given timestamp
        */
        size_t findRightNeighbour(double ti, std::vector<double>& v);

        /**  Function for finding an specific element in a vector of timestamps
        * \param[in] ti - a given timestamp
        * \param[in] v - a vector of timestamps
        * \return - index of element in vector which occurs right before the given timestamp
        */
        size_t findLeftNeighbour(double ti, std::vector<double>& v);

        /**  Function for calculating standard deviation for a vector of doubles
        * \param[in] v - a vector of doubles
        * \return - standard deviation
        */
        double calculateVectorSD(std::vector<double>& v);

        /**  Function for calculating mean value for a vector of doubles
        * \param[in] v - a vector of doubles
        * \return mean value of vector
        */
        double calculateVectorMean(std::vector<double>& v);
        
        /**  Function for calculating standard deviation for a vector of SensorDataSample
        * \param[in] sensorData - a vector of SensorDataSample
        * \param[out] sd_x - standard deviation for X component
        * \param[out] sd_y - standard deviation for Y component
        * \param[out] sd_z - standard deviation for Z component
        */
        void calculate_SensorDataSample_vector_SD(std::vector<SensorDataSample>& sensorData, double &sd_x, double& sd_y, double& sd_z);

        /**  Function for filtering a vector of doubles using a sliding window
        * \param[in] v - a vector of doubles
        * \param[in] span - window size (in samples)
        * \param[out] v_flt - filtered vector of doubles
        */
        void meanSlideWindow_Filter(std::vector<double>& v, uint32_t span, std::vector<double>& v_flt);

        /**  Function for updating return status
        * \param[in] return_status - new return status candidate
        */
        void updateReturnStatus(Calibration::ReturnStatus return_status);

        /**  Function for calculating calibration level for accelerometer based on bias uncertainty value
        * \param[in] biasUncertainty -  bias uncertainty
        * \return - calibration level
        */
        int8_t calculateAccelerometerCalibrationLevel(double biasUncertainty);

        /**  Function for calculating calibration level for magnetometer based on bias uncertainty value
        * \param[in] biasUncertainty -  bias uncertainty
        * \return - calibration level
        */
        int8_t calculateMagnetometerCalibrationLevel(double biasUncertainty);
        
        std::vector<SensorDataSample> rawMagneticData;
        std::vector<SensorDataSample> rawGyroData;
        std::vector<SensorDataSample> rawAccelData;

        std::vector<SensorDataSample> calibratedMagneticData;
        std::vector<SensorDataSample> calibratedGyroData;
        std::vector<SensorDataSample> calibratedAccelData;

        Calibration::ReturnStatus calibration_status;

        const uint32_t min_data_count = 10;
        const uint32_t optimal_data_count = 200;
        const size_t min_averaging_sample_number = 15;
        const double max_temp_variation = 5.0; // [celsius]
        const double max_calibration_time = 1200.0; // [seconds]
        const double max_DOP = 0.3;
        const double max_axis_DOP = 0.2;
        const double max_SD_axis_DOP = 0.1;
        const double max_allowed_scale_factor = 1.3;
        const double min_allowed_scale_factor = 0.7;
        const double magnetometer_low_calib_accuracy_limit = 30.0; // 30.0 mG (equals to 3.0 uT)
        const double accelerometer_low_calib_accuracy_limit = 0.1; // 0.1 m/sec^2
        double average_temperature;
        double dop_x;
        double dop_y;
        double dop_z;
    };
}

#endif