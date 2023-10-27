#include <iostream>
#include <fstream>
#include <stdio.h>
#include <regex>
#include <cstdint>
#include <sstream>
// WARNING: dirent.h is not provided with visual studio
#include "dirent.h"
#include <stdint.h>

#include"ble_data_parser.hpp"
#include "Fpbl.hpp"

bool use_ble_exception = true;
const int ble_exception_count = 1;
uint16_t ble_exception_list[ble_exception_count][2] =
{
    0, 0
};

bool ble_data_parser::parse_ble_file(std::string file_path)
    {
        std::ifstream infile( file_path );
        std::string line = "";
        const char delim = ',';

        //std::getline(infile, line);

        uint64_t timestamp;
        int16_t  rssi;
        uint16_t  major, minor;
        int16_t txPower;

        uint64_t bleMeasurementNumberPrev = 0;
        BleMeasurement bleMeasurement;
        BleScanResult bleScan;
        std::string macStr;

        uint64_t bleMeasurementNumber = 0;

        long strCounter = 0;

        while ( std::getline( infile, line ) )
        {
            strCounter++;

            std::stringstream buf( line );
            std::string token;
            std::vector<std::string> result;

            while ( std::getline( buf, token, delim ) )
            {
                result.push_back( token );
            }

            // remove double timestamp
            result.erase(result.begin() + 1);

            //std::cout << result[7].length() << std :: endl;
            if (result[7].length() != 37)  // UUID length plus last zero symbol
               continue;

            std::stringstream ss;
            ss << result[0];
            ss >> timestamp;
            bleMeasurement.timestamp = timestamp;
            ss.str( "" );
            ss.clear();

            // NOTE: value number [1] is skipped, unused
            ss << result[1];
            ss >> bleMeasurementNumber;
            ss.str( "" );
            ss.clear();

            ss << result[2];
            ss >> macStr;
            bleMeasurement.hasMAC = false;

            if ( &macStr == NULL || macStr == "" )
            {
                bleMeasurement.mac = 0;
            }
            else
            {
                bleMeasurement.mac = string_to_mac( macStr );

                if ( bleMeasurement.mac != 0 )
                    bleMeasurement.hasMAC = true;
            }

            ss.str( "" );
            ss.clear();

            ss << result[4];
            ss >> bleMeasurement.frequency;
            ss.str( "" );
            ss.clear();

            ss << result[6];
            ss >> rssi;
            bleMeasurement.rssi = ( int8_t ) rssi;
            ss.str( "" );
            ss.clear();

            // UUID
            std::string uuid = result[7];
            string_to_ble(uuid, bleMeasurement.uuid);

            ss << result[8];
            ss >> major;
            bleMeasurement.major = major;
            ss.str( "" );
            ss.clear();

            ss << result[9];
            ss >> minor;
            bleMeasurement.minor = minor;
            ss.str( "" );
            ss.clear();

            // BLE hash function instead real mac address - it is disabled now in converter - it have to be replaced to FPBL
#if 0
            bleMeasurement.mac = ( ( uint64_t ) major << 16 ) + minor;
#elif 0
            bleMeasurement.mac = 0;
            //High 4 bytes of UUID plus major plus minor
            bleMeasurement.mac = ((uint64_t)bleMeasurement.uuid[0] << 56);
            bleMeasurement.mac += ((uint64_t)bleMeasurement.uuid[1] << 48);
            bleMeasurement.mac += ((uint64_t)bleMeasurement.uuid[2] << 40);
            bleMeasurement.mac += ((uint64_t)bleMeasurement.uuid[3] << 32);

            bleMeasurement.mac += ((uint64_t)major << 16) + minor;
#endif
            ss << result[10];
            ss >> txPower;
            bleMeasurement.txPower = ( int8_t ) txPower;
            ss.str( "" );
            ss.clear();
            // wifi data parsing done

            if ( bleMeasurementNumber != bleMeasurementNumberPrev )
            {
                if ( bleScan.scanBle.size() > 0 )
                    Fpbl::ReturnStatus status = processBleMeasurement( bleScan.timestamp, bleScan );

                bleScan.scanBle.clear();
                bleScan.timestamp = timestamp;

                bleMeasurementNumberPrev = bleMeasurementNumber;
            }

            bool process_ble_message = (bleMeasurement.major != 0) && (bleMeasurement.minor != 0);
            //process_ble_message &= (bleMeasurement.rssi > -100);
            if (use_ble_exception && process_ble_message)
            {
                for (int i = 0; i < ble_exception_count; i++)
                {
                    process_ble_message &= !((bleMeasurement.major == ble_exception_list[i][0]) && (bleMeasurement.minor == ble_exception_list[i][1]));
                }
            }

            if (process_ble_message)
            {
                bleScan.scanBle.push_back(bleMeasurement);
            }
            else
            {
                //std::cout << std::endl;
                std::cout << "reject ble exception:" << bleMeasurement.major << ", " << bleMeasurement.minor << "," << (int)bleMeasurement.rssi << std::endl;
            }
        }

        if ( bleScan.scanBle.size() > 0 )
            Fpbl::ReturnStatus status = processBleMeasurement(bleScan.timestamp, bleScan);

        return true;
    }

    /** Puts single path ble measurements into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] ble  BLE scan result
    * \return status code*/
    Fpbl::ReturnStatus ble_data_parser::processBleMeasurement(const uint64_t &timestamp, const BleScanResult &bleScan)
    {
        mBleScanResultTimestamps.push_back(timestamp);
        mBleScanResults.push_back(bleScan);

        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    uint64_t ble_data_parser::getBleMeasurementTimestamp(const uint32_t number)
    {
        return mBleScanResultTimestamps[number];
    }

    BleScanResult ble_data_parser::getBleMeasurement(const uint32_t number)
    {
        return mBleScanResults[number];
    }

    BSSID ble_data_parser::string_to_mac(std::string const &s)
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

    void ble_data_parser::string_to_ble(std::string const &s, uint8_t *ble)
    {
        uint32_t part1 = 0;
        uint16_t part2 = 0;
        uint16_t part3 = 0;
        uint16_t part4 = 0;
        uint64_t part5 = 0;

        uint8_t  tmp[16];

        memset(ble, 0, 16);
        memset(tmp, 0, 16);
        const char delim = '-';

        std::stringstream buf(s);
        std::string token;
        std::vector<std::string> result;

        while (std::getline(buf, token, delim))
        {
            result.push_back(token);
        }

        if (result.size() != 5)
            return;

        std::stringstream ss;

        ss << result[0];
        ss >> std::hex >> part1;
        ss.str("");
        ss.clear();

        *(uint32_t *)(tmp + 12) = part1;

        ss << result[1];
        ss >> std::hex >> part2;
        ss.str("");
        ss.clear();

        *(uint16_t *)(tmp + 10) = part2;

        ss << result[2];
        ss >> std::hex >> part3;
        ss.str("");
        ss.clear();

        *(uint16_t *)(tmp + 8) = part3;

        ss << result[3];
        ss >> std::hex >> part4;
        ss.str("");
        ss.clear();

        *(uint16_t *)(tmp + 6) = part4;

        ss << result[4];
        ss >> std::hex >> part5;
        ss.str("");
        ss.clear();

        *(uint64_t *)tmp |= part5;

        // big endian
        for (int i = 0; i < 16; i++)
        {
            //ble[15 - i] =  (tmp[i] & 0xF) << 4;
            //ble[15 - i] |= (tmp[i] >> 4) & 0xF;
            ble[15 - i] = tmp[i];
        }
        /*
        std::cout << std::hex << *(uint64_t *)(tmp + 8) << *(uint64_t *)tmp << "      ";
        std::cout << *(uint64_t *)(ble + 8) << *(uint64_t *)ble << "      ";
        std::cout << std::dec << std::endl;
        */
    }
