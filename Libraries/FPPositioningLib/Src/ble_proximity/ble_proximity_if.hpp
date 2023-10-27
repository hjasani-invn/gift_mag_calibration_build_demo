/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Abstract class for BLE proximity
* \defgroup        ble_proximity
* \file            ble_proximity_if.hpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/

#ifndef BLE_PROXIMITY_IF_H
#define BLE_PROXIMITY_IF_H


#include <vector>
#include <string>
#include <map>
#include <stdint.h>

#include "Fppe.hpp"
#include "wifi_data.hpp"
#include "ble_position.hpp"

struct Location_and_time
{
    WiFi_Location location;
    int64_t t;
};

class IBLEProximity
{
    public:

        virtual  void setBLEProximityLogic(int N, int M) = 0;

        virtual  void setSingleCutoffThreshold(const int reject_threshold) = 0;

        virtual  void setMultipleCutoffThreshold(const int reject_threshold) = 0;

        /**
        * estimates the location
        * \param[in] measurement measurement vector
        * \param[in] K the number of points used for interpolation
        * \return estimated location
        */
        virtual  BLE_position GetLocation(const Fppe::BleScanResult  &measurement, uint64_t time_stamp) const = 0;

        virtual  void GetLocation(const Fppe::BleScanResult  &measurement, uint64_t time_stamp, std::vector<WiFi_Location> &LocationList) const = 0;
        /**
        * \return initialization status (true if ok)
        */
        virtual  bool status() = 0;

        virtual ~IBLEProximity() {};

        /**
        * estimates the bias for the location
        * \param[out] estimated bias
        * \param[out] number of used measurements
        */
        virtual  bool  estimateBias(const Fppe::BleScanResult &measurement, std::vector <Location_and_time> &positions, std::ostream *log) = 0;

        /**
        * sets new bias value and enables bias estimation
        * \param[in] new bias to be set
        */
		virtual  void  setBias(double bias) = 0;

		/**
		* sets new bias sigma value and enables bias estimation
		* \param[in] new bias to be set
		*/
		virtual  void  setBiasSigma(double bias_sigma) = 0;

        /**
        * gets current bias value and bias estimation enable flag
        * \param[out] current bias value
        * \param[out] current bias st.dev
        * \return biaas estimation enable
        */
        virtual  bool  getBias(double &bias, double &bias_sigma) = 0;

        virtual std::map < BSSID, BLE_position> GetBLEProximityMap() const = 0;

        virtual size_t size() = 0;

};

#endif // BLE_PROXIMITY_IF_H
