/**
* \copyright       Copyright (C) TDK, LLC., 2018
* \brief           WiFi and BLE bias estimating
* \file            WiFiBLEBiasEstimator.hpp
* \addtogroup      bias
* \author          V. Pentiukhov
* \date            17.12.2018
*/

#ifndef _WIFIBLEBIAS_ESTIMATOR_HPP
#define _WIFIBLEBIAS_ESTIMATOR_HPP

#include "wifi_if.hpp"

// interface class for BiasEstimator
class IBiasEstimator
{
public:

    virtual bool  estimateBias(WiFi *wifi, const WiFi_Measurement &measurement, const WiFi_Location &location, int &cnt, unsigned long  &biasCounter, double &biasRSSI, std::ostream *log) = 0;

};

// Bias estimator implementation
class WiFiBLEBiasEstimator : public IBiasEstimator
{
public:

    bool  estimateBias(WiFi *wifi, const WiFi_Measurement &measurement, const WiFi_Location &location, int &cnt, unsigned long  &biasCounter, double &biasRSSI, std::ostream *log);
};

#endif // BLEPROXIMITYBIAS_ESTIMATOR_HPP
