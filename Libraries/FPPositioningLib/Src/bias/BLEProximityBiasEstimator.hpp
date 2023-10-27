/**
* \copyright       Copyright (C) TDK, LLC., 2018
* \brief           BLE Proximity bias estimating
* \file            BLEProximityBiasEstimator.hpp
* \addtogroup      bias
* \author          V. Pentiukhov
* \date            20.01.2019
*/

#ifndef _BLEPROXIMITYBIAS_ESTIMATOR_HPP
#define _BLEPROXIMITYBIAS_ESTIMATOR_HPP

#include "ble_proximity_if.hpp"

// interface class for BiasEstimator
class IBLEProximityBiasEstimator
{
public:
    virtual bool  estimateBias(const std::map < BSSID, BLE_position> &ble_position_map, const Fppe::BleScanResult &measurement, std::vector <Location_and_time> &positions, unsigned long  &biasCounter, double &bias, double &biasSigma, std::ostream *log) = 0;
    virtual void initializeBias(double bias, double bias_sigma = 0.) = 0;
};

// Bias estimator implementation
class BLEProximityBiasEstimator : public IBLEProximityBiasEstimator
{
public:
    bool  estimateBias(const std::map < BSSID, BLE_position> &ble_position_map, const Fppe::BleScanResult &measurement, std::vector <Location_and_time> &positions, unsigned long  &biasCounter, double &bias, double &biasSigma, std::ostream *log);
    void initializeBias(double bias, double bias_sigma);
private:
    double previousBias = 0;
    double currentBias = 0;
    double biasSigma = 0;
    std::vector<double> biases;
    int filter_step = 1;
};

// Bias estimator implementation
class BleProximityBiasEstimator_EKF : public IBLEProximityBiasEstimator
{
public:
    bool  estimateBias(const std::map < BSSID, BLE_position> &ble_position_map, const Fppe::BleScanResult &measurement, std::vector <Location_and_time> &positions, unsigned long  &biasCounter, double &bias, double &biasSigma, std::ostream *log);
    void initializeBias(double bias, double bias_sigma);
private:
    double t = 0;
    double bias = 0;
    double bias_cov = 0;
};

#endif // _WIFIBLEBIAS_ESTIMATOR_HPP
 