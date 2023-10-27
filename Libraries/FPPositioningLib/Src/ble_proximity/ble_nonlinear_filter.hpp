/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Nonlinear filter to reduce BLE levels fluctuations
* \defgroup        ble_proximity
* \file            ble_nonlinear_filter.hpp
* \ingroup         ble_proximity
* \author          V Pentukhov, D Churikov
* \date            07.09.2018
*/

#ifndef BLE_NONLINEAR_FILTER_H
#define BLE_NONLINEAR_FILTER_H

#include "Fppe.hpp"
#include "wifi_data.hpp"
#include "CoordinateConverter.h"

#include <map>
#include <algorithm>


class BleNonlinearFilter
{
public:
    BleNonlinearFilter(BSSID hash, double coef, int64_t  timestamp_interval)
    {
       ble_hash = hash;
       decline_coef = coef;
       this->timestamp_interval = timestamp_interval;
    }
    ~BleNonlinearFilter() { }
    double  process(int64_t timestamp, double x);
     BSSID  get_hash();
private:
        BSSID    ble_hash;
        double   decline_coef;
        double   x_s;
        double   y_pref;
        int64_t  prev_timestamp = 0;
        int64_t  timestamp_interval = 0;
        uint64_t counter = 0;
};

class PeakDetector
{
public:
    PeakDetector(BSSID hash, double coef, int64_t  max_data_gap) :
        is_active(false), x_s(0.), timestamp(0), ble_hash(hash), decline_coef_(coef), max_data_gap_(max_data_gap) {}

    ~PeakDetector() { }

    void set_max_data_gap(int64_t  max_data_gap) { this->max_data_gap_ = max_data_gap; };
    
    void set_decline_coef(double  decline_coef) { this->decline_coef_ = decline_coef; };

    BSSID  get_hash() { return ble_hash; }

    double get_data() { return x_s; }

    bool predict(int64_t timestamp)
    {
        is_active = is_active && ((timestamp - this->timestamp) <= max_data_gap_);
        x_s = (is_active) ? (x_s + x_s * decline_coef_) : x_s;
        return is_active;
    };

    bool  update(int64_t timestamp, double x)
    {
        this->timestamp = timestamp;
        x_s = (is_active) ? std::max(x_s, x) : x;
        return is_active = true; // set is_active in true and return
    };

private:
    BSSID    ble_hash;
    double   decline_coef_;
    int64_t  max_data_gap_;
    double   x_s;
    int64_t  timestamp;
    bool is_active;
};

#endif // BLE_NONLINEAR_FILTER_H
