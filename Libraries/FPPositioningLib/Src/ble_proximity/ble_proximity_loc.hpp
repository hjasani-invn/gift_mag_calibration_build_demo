/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Get BLE ploximity location
* \defgroup        ble_proximity
* \file            ble_proximity_loc.hpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/

#ifndef BLE_PROXIMITY_LOC_H
#define BLE_PROXIMITY_LOC_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <list>

#include "CoordinateConverter.h"

#include "ble_proximity_if.hpp"
#include "ble_proximity_db.hpp"
#include "BLEProximityBiasEstimator.hpp"

/**
* \ingroup         BLEProximity
*/
class BLE_Proximity_Locator : public IBLEProximity
{
    public:
        /**
        * Fingerprinting object constructor, implements BLE Proximity interface
        * \param[in] db_name fingerprint data 
        * \param[in] db_name fingerprint data size
        */
        BLE_Proximity_Locator(const char* const pBLEProxMap, const size_t bleProxFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int rejectThreshold);

        void setBLEProximityLogic(int N, int M);

        void setSingleCutoffThreshold(const int reject_threshold);

        void setMultipleCutoffThreshold(const int reject_threshold);

        BLE_position GetLocation(const Fppe::BleScanResult  &measurement, uint64_t time_stamp) const;

        void GetLocation(const Fppe::BleScanResult  &measurement, uint64_t time_stamp, std::vector<WiFi_Location> &LocationList) const;

        std::map < BSSID, BLE_position> GetBLEProximityMap() const;

        /**
        * estimates the bias for the location
        * \param[out] estimated bias
        * \param[out] number of used measurements
        */
        bool  estimateBias(const Fppe::BleScanResult &measurement, std::vector <Location_and_time> &positions, std::ostream *log);
        
        /**
        * sets new bias value and enables bias estimation
        * \param[in] new bias to be set
        */
		void  setBias(double bias);

		/**
		* sets new bias sigma value 
		* \param[in] new bias sigma to be set
		*/
		void  setBiasSigma(double bias_sigma);

        /**
        * gets current bias value and bias estimation enable flag
        * \param[out] current bias value
        * \param[out] current bias st.dev value
        * \return biaas estimation enable
        */
        bool  getBias(double &bias, double &bias_sigma);

        bool status()
        {
            return initialized;
        };

        size_t size() 
        {
            return ble_proximity_db->size();
        }

        /** destructior */
        virtual ~BLE_Proximity_Locator();

        /**
        * estimates rssi probability based on Parsen-window method, using gaussian kernel
        * \param[in] fp rssi histogram
        * \param[in] rssi evaluation point
        * \param[in] kernel_sig gaussian kernel parameter
        * \return probability of the rssi level
        */
        //static tProb hist2prob( const WiFi_DB::APData &fp, int rssi, tProb kernel_sig );

    private:
        bool initialized;
        BLE_Proximity_DB *ble_proximity_db;
        IBLEProximityBiasEstimator *biasBLEProximityEstimator;
        double biasRssi;
        double biasRssiSigma;
        bool   biasEstEnabled;
        unsigned long   biasCounter;

};
#endif // BLE_PROXIMITY_LOC_H
