#include <iostream>
#include <fstream>
#include <stdio.h>
#include <regex>
#include <cstdint>
#include <sstream>
// WARNING: dirent.h is not provided with visual studio
#include "dirent.h"

#include"wifi_data_parser.hpp"
#include "Fpbl.hpp"

bool wifi_data_parser::parse_wifi_file(std::string file_path)
    {
        std::ifstream infile( file_path );
        std::string line;
        const char delim = ',';

        uint64_t timestamp;
        int16_t  rssi;

        uint64_t wifiMeasurementNumberPrev = 0;
        WiFiMeasurement wifiMeasurement;
        WiFiScanResult wifiScan;
        std::string macStr;

        uint64_t wifiMeasurementNumber = 0;

        while ( std::getline( infile, line ) )
        {
            std::stringstream buf( line );
            std::string token;
            std::vector<std::string> result;

            while ( std::getline( buf, token, delim ) )
            {
                result.push_back( token );
            }

            std::stringstream ss;
            ss << result[0];
            ss >> timestamp;
            wifiMeasurement.timestamp = timestamp;
            ss.str( "" );
            ss.clear();

            // NOTE: value number [1] is skipped, unused
            ss << result[1];
            ss >> wifiMeasurementNumber;
            ss.str( "" );
            ss.clear();

           // if (timestamp == 59060)
           //     std::cout << timestamp << std::endl;

            ss << result[2];
            ss >> macStr;
            wifiMeasurement.mac = string_to_mac( macStr );
            ss.str( "" );
            ss.clear();

            ss << result[4];
            ss >> wifiMeasurement.frequency;
            ss.str( "" );
            ss.clear();

            ss << result[6];
            ss >> rssi;
            wifiMeasurement.rssi = ( int8_t ) rssi;
            ss.str( "" );
            ss.clear();

            if (result.size() >= 8)
            {
                ss << result[7];
                ss >> wifiMeasurement.timestamp;
                ss.str("");
                ss.clear();
            }
            else
                wifiMeasurement.timestamp = wifiScan.timestamp;
            // wifi data parsing done

            if ( wifiMeasurementNumber != wifiMeasurementNumberPrev )
            {
                if (wifiScan.scanWiFi.size() > 0)
                {
                    Fpbl::ReturnStatus status = processWiFiMeasurement(wifiScan.timestamp, wifiScan);
                }
                wifiScan.scanWiFi.clear();
                wifiScan.timestamp = timestamp;

                wifiMeasurementNumberPrev = wifiMeasurementNumber;
            }
            //if ((wifiScan.timestamp - wifiMeasurement.timestamp) < 500)
                wifiScan.scanWiFi.push_back(wifiMeasurement);
            //else
            //    std::cout << wifiScan.timestamp << "     " << wifiMeasurement.timestamp << std::endl;
 

        }

        if ( wifiScan.scanWiFi.size() > 0 )
            Fpbl::ReturnStatus status = processWiFiMeasurement(wifiScan.timestamp, wifiScan);

        return true;
    }

/** Puts single path wifi measurements into processing
* \param[in] timestamp unix time in [us]
* \param[in] wifi  WiFi scan result
* \return status code*/
Fpbl::ReturnStatus wifi_data_parser::processWiFiMeasurement(const uint64_t &timestamp, const WiFiScanResult &wifiScan)
{
    mWiFiScanResultTimestamps.push_back(timestamp);
    mWiFiScanResults.push_back(wifiScan);

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

uint64_t wifi_data_parser::getWiFiMeasurementTimestamp(const uint32_t number)
{
    return mWiFiScanResultTimestamps[number];
}
WiFiScanResult wifi_data_parser::getWiFiMeasurement(const uint32_t number)
{
    return mWiFiScanResults[number];
}

    BSSID wifi_data_parser::string_to_mac(std::string const &s)
    {
        uint64_t last = -1;
        uint64_t band = 0;
        uint64_t mac = 0;
        uint64_t a[8];
        int rc = 7;

        int cnt = std::count(s.begin(), s.end(), ':');


        const char delim = ':';

        std::stringstream buf(s);
        std::string token;
        std::vector<std::string> result;

        while (std::getline(buf, token, delim))
        {
            result.push_back(token);
        }

        uint16_t size = result.size();

        if (size != 8 && size != 7 && size != 6)
            return 0;


        for (int i = 0; i < 8; i++)
        {
            a[i] = 0;
        }

        mac = 0;

        for (int i = 0; i < size; i++)
        {
            std::stringstream ss;
            ss << result[size - 1 - i];
            ss >> std::hex >> a[i];
            ss.str("");
            ss.clear();

            mac |= a[i] << i * 8;
        }

        return mac;
    }
