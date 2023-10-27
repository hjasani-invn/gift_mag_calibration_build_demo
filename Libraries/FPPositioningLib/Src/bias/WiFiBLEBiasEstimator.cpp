/**
* \copyright       Copyright (C) TDK, LLC., 2018
* \brief           WiFi, BLE and Proximity bias estimating
* \file            WiFiBLEBiasEstimator.cpp
* \addtogroup      bias
* \author          V. Pentiukhov
* \date            17.12.2018
*/

#include <cmath>
#include <algorithm>
//#include <iostream> for std::cout
#include "WiFiBLEBiasEstimator.hpp"

bool  WiFiBLEBiasEstimator::estimateBias(WiFi *wifi, const WiFi_Measurement &measurement, const WiFi_Location &location, int &cnt, unsigned long  &biasCounter, double &biasRSSI, std::ostream *log)
{
    std::map<BSSID, RSSI> wifi_fp;
    bool success = false;
    double biasGain;
    wifi->getBiasGain(biasGain);
    //double biasRssiThreshold = wifi->getBiasRssiThreshold();
    //std::cout << "biasRssiThreshold = " << biasRssiThreshold << std::endl;
    //std::cout << "location = " << location.x << "   " << location.y << std::endl;
    double biasRssiThreshold = wifi->getBiasRssiThreshold();
    {
        success = wifi->getPositionData(wifi_fp, location, true, log);
    }
    //*log << "getPositionData success = " << success << std::endl;
    //*log << "getPositionData wifi_fp.size = " << wifi_fp.size() << std::endl;
    //std::cout << "getPositionData success = " << success << std::endl;
    //std::cout << "getPositionData wifi_fp.size = " << wifi_fp.size() << std::endl;
    if (success)
    {
        int N = 0;
        double deltaRSSI = 0;

        for (WiFi_Measurement::const_iterator it = measurement.begin(); it != measurement.end(); ++it)
        {
            std::map<BSSID, RSSI>::iterator ap_it = wifi_fp.find((*it).bssid);

            //std::cout << std::endl << "bssid = " << (*it).bssid;
            //std::cout << "  measRSSI = " << (*it).rssi;
            //std::cout << "  check 1 = " << int(((*it).rssi >= biasRssiThreshold));
            //std::cout << "  check 2 = " << int((ap_it != wifi_fp.end()));

            if (((*it).rssi >= biasRssiThreshold) && (ap_it != wifi_fp.end()))
            {
                double d = (*ap_it).second - (*it).rssi;
                deltaRSSI += d;
                N++;
                //std::cout << "  mapRSSI = " << (*ap_it).second;
            }
        }
        cnt = N;

        //std::cout << std::endl;
        //std::cout << std::endl;

        double real_gain;
        if (N >= 3)
        {
            biasCounter += 1;
            real_gain = location.metric * std::max(biasGain, 1. / static_cast<double>(biasCounter));
            deltaRSSI /= N;
            success = true;
        }
        else
        {
            real_gain = 0;
            success = false;
        }

        // filter of bias
        biasRSSI += real_gain * ((deltaRSSI - biasRSSI));

        //std::cout << "biasRssiThreshold = " << biasRssiThreshold << "  N = " << N << "  deltaRSSI = " << deltaRSSI << std::endl;
        //std::cout << "biasCounter = " << biasCounter << "  biasGain = " << real_gain << "  RSSI bias = " << biasRSSI << std::endl;
        //*log << "dbg: RSSI bias = " << biasRSSI << std::endl;
    }

    return success;

}
