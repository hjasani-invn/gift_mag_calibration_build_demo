/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Get BLE ploximity location
* \defgroup        ble_proximity
* \file            ble_proximity_loc.cpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/

#include "ble_proximity_loc.hpp"
#include <cmath>
#include "median.h"
#ifdef __ANDROID__
#   include <android/log.h>
#	ifdef LOGI
#		undef LOGI
#	endif
#   define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "Wi-Fi loc", __VA_ARGS__))
#else
#   define LOGI(...)
#endif

const double default_bias_sigma = 5;

BLE_Proximity_Locator::BLE_Proximity_Locator(const char* const pBLEProxMap, const size_t bleProxFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int rejectThreshold)
{
    ble_proximity_db = new (std::nothrow) BLE_Proximity_DB(N, M, rejectThreshold);
    initialized = (nullptr != ble_proximity_db);
    initialized = initialized ? ble_proximity_db->readFormatBle5(pBLEProxMap, bleProxFileSizeInBytes, converter) : initialized;

    biasBLEProximityEstimator = new (std::nothrow) BleProximityBiasEstimator_EKF();
    initialized &= (nullptr != biasBLEProximityEstimator);
    biasEstEnabled = false;
    biasCounter = 0;
    biasRssi = 0;
    biasRssiSigma = default_bias_sigma;
}

BLE_Proximity_Locator::~BLE_Proximity_Locator()
{
    delete ble_proximity_db;
    delete biasBLEProximityEstimator;
}

void BLE_Proximity_Locator::setBLEProximityLogic(int N, int M)
{
    ble_proximity_db->setBLEProximityLogic(N, M);
}

void BLE_Proximity_Locator::setSingleCutoffThreshold(const int reject_threshold)
{
    ble_proximity_db->setSingleCutoffThreshold(reject_threshold);
}

void BLE_Proximity_Locator::setMultipleCutoffThreshold(const int reject_threshold)
{
    ble_proximity_db->setMultipleCutoffThreshold(reject_threshold);
}

BLE_position BLE_Proximity_Locator::GetLocation(const Fppe::BleScanResult  &ble_scan, uint64_t time_stamp) const
{
    std::vector<Fppe::BleMeasCalibrated > ble_scan_calibrated;

    for (auto meas : ble_scan.scanBle)
    {
        Fppe::BleMeasCalibrated mes_calibrated;
        mes_calibrated = meas;
        mes_calibrated.bias = biasRssi;
        mes_calibrated.bias_sigms = std::max(biasRssiSigma, 1.);
        mes_calibrated.rssi_sigma = (meas.rssi > -80) ? 2. : 5.; // just guessed values for low and high RSSI levels
        ble_scan_calibrated.push_back(mes_calibrated);
    }

    BLE_position  result = ble_proximity_db->GetLocation(ble_scan_calibrated, time_stamp);
    return result;
}

void BLE_Proximity_Locator::GetLocation(const Fppe::BleScanResult  &ble_scan, uint64_t time_stamp, std::vector<WiFi_Location> &LocationList) const
{
    std::vector<Fppe::BleMeasCalibrated > ble_scan_calibrated;

    for (auto meas : ble_scan.scanBle)
    {
        Fppe::BleMeasCalibrated mes_calibrated;
        mes_calibrated = meas;
        mes_calibrated.bias = biasRssi;
        mes_calibrated.bias_sigms = std::max(biasRssiSigma, 1.);
        mes_calibrated.rssi_sigma = (meas.rssi > -80) ? 2. : 5.; // just guessed values for low and high RSSI levels
        ble_scan_calibrated.push_back(mes_calibrated);
    }
    ble_proximity_db->GetLocation(ble_scan_calibrated, time_stamp, LocationList);
}

std::map < BSSID, BLE_position> BLE_Proximity_Locator::GetBLEProximityMap() const
{
    return ble_proximity_db->GetBLEProximityMap();
}

bool  BLE_Proximity_Locator::estimateBias(const Fppe::BleScanResult &measurement, std::vector <Location_and_time> &positions, std::ostream *log)
{
    bool result(false);
    if (biasEstEnabled)
    {
        const std::map < BSSID, BLE_position> ble_position_map = GetBLEProximityMap();
        result = biasBLEProximityEstimator->estimateBias(ble_position_map, measurement, positions, biasCounter, biasRssi, biasRssiSigma, log);
    }
    return result;
}

void BLE_Proximity_Locator::setBias(double bias)
{
    biasRssi = bias;
    biasEstEnabled = true;
    biasBLEProximityEstimator->initializeBias(bias);
}

void BLE_Proximity_Locator::setBiasSigma(double bias_sigma)
{
	biasRssiSigma = bias_sigma;
}

bool BLE_Proximity_Locator::getBias(double &bias, double &bias_sigma)
{
    bias = biasRssi;
    bias_sigma = biasRssiSigma;
    return biasEstEnabled;
}
