/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Data base for BLE proximity
* \defgroup        ble_proximity
* \file            ble_proximity_db.cpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/

#include <fstream>
#include <sstream>
#include <streambuf>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "ble_proximity_db.hpp"
#include "BleHash.h"

// todo: place this class in separate file (see realisation of this clas in files: FppeImp.hpp, ble_proximity_db.cpp, wifi_db_hist.cpp)
template <typename char_type> 
class iostreambuf : public std::basic_streambuf<char_type, std::char_traits<char_type> >
{
public:
    iostreambuf(char_type* buffer, std::streamsize bufferLength)
    {
        // Sets the values of the pointers defining the put area. Specifically, after the call pbase() == pbeg, pptr() == pbeg, epptr() == pend
        // pbeg - pointer to the new beginning of the put area
        // pend - pointer to the new end of the put area
        std::basic_streambuf<char_type, std::char_traits<char_type> >::setp(buffer, buffer + bufferLength);

        //Sets the values of the pointers defining the get area.Specifically, after the call eback() == gbeg, gptr() == gcurr, egptr() == gend
        //  gbeg - pointer to the new beginning of the get area
        //	gcurr - pointer to the new current character(get pointer) in the get area
        //	gend - pointer to the new end of the get area
        std::basic_streambuf<char_type, std::char_traits<char_type> >::setg(buffer, buffer, buffer + bufferLength);
    }
};

BLE_Proximity_DB::BLE_Proximity_DB(int N, int M, int rejThresh)
{
    SingleCutoffThreshold = rejThresh;
    MultipleCutoffThreshold = rejThresh;
    ble_data = new BLE_Accumulation(N, M);
}

BLE_Proximity_DB::~BLE_Proximity_DB()
{
    delete ble_data;
}

void BLE_Proximity_DB::setBLEProximityLogic(int N, int M)
{
    ble_data->set_BLE_Accumulation(N, M);
}

void BLE_Proximity_DB::setSingleCutoffThreshold(const int reject_threshold)
{
    SingleCutoffThreshold = reject_threshold;
}

void BLE_Proximity_DB::setMultipleCutoffThreshold(const int reject_threshold)
{
    MultipleCutoffThreshold = reject_threshold;
}


size_t BLE_Proximity_DB::size() const
{
    return ble_position_map.size();
}

bool BLE_Proximity_DB::GetDbItem(BSSID  ble_hash, BLE_position &db_item) const
{
    bool IsItemPresentInDB = true;
    try
    {
        db_item = ble_position_map.at(ble_hash);
    }
    catch (std::out_of_range)
    {
        IsItemPresentInDB = false;
    }
    return IsItemPresentInDB;
}

#if 0
bool BLE_Proximity_DB::readFormatBle5(const char* const pBleMap, const size_t bleFileSizeInBytes, const GeoLocConverter2D &converter)
{
    const double default_blp_height = 2;
    const int    default_blp_type = 0;
    Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
    size_t szHeader(0);
    // read and check header
    bool header_is_ok = true;
    const char delim = ',';

    uint8_t uuid[16];
    uint16_t major;
    uint16_t minor;
    int16_t txPower;
    BLE_position ble_location;
    BSSID  ble_hash;  // use as mac address
    double lat, lon, floor;
    double blp_height = default_blp_height;
    int blp_type = default_blp_type;

    int status = 0;
    std::string line;
    const int line_length = 1000;
    char line_buf[line_length];
    int countBLE = 0;

    iostreambuf<char> iostreamBuffer((char *)pBleMap, bleFileSizeInBytes);
    std::iostream stringstr(&iostreamBuffer);

    ble_position_map.clear();

    while (!stringstr.eof())
    {
        countBLE++;

        stringstr.get(line_buf, line_length - 1, '\n');
        stringstr.ignore(7777, '\n');
        line = std::string(line_buf);
        if (line == "")
            continue;

        std::string line1 = line;
        line1.erase(line1.length() - 1);
        
        line = line + "\n";

        // line parsing
        // line example - 236717FF-2415-4619-8E83-D5C8A51E4BB1,FF,2A,10,11.5,-70
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;

            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }
            if (result.size() < 7)
                continue;

            std::stringstream ss;

            // UUID
            string_to_ble(result[0], uuid);

            // major
            ss << result[1];            ss >> major;            ss.str("");            ss.clear();

            // minor
            ss << result[2];            ss >> minor;            ss.str("");            ss.clear();

            // BLE location
            ss << result[3];            ss >> lat;            ss.str("");            ss.clear();
            ss << result[4];            ss >> lon;            ss.str("");            ss.clear();
            ss << result[5];            ss >> floor;            ss.str("");            ss.clear();

            // transmiter power
            ss << result[6];            ss >> txPower;            ss.str("");            ss.clear();

            // floor height
            if (result.size() >= 8)
            {
                ss << result[7];            ss >> blp_height;            ss.str("");            ss.clear();
            }
            else
                blp_height = default_blp_height;

            // blp type
            if (result.size() >= 9)
            {
                ss << result[8];            ss >> blp_type;            ss.str("");            ss.clear();
            }
            else
            {
                blp_type = default_blp_type;
            }

            double x, y;
            converter.Geo2Local(lat, lon, &x, &y);

            ble_location.loc.set(x, y, floor, 1, true);
            //std::cout << x << "    " << y << std::endl;
            ble_location.txPower = txPower;
            ble_location.blp_height = blp_height;
            ble_location.beaconType = static_cast<BleBeaconType> (blp_type);

            ble_hash = getBleHash(major, minor, uuid);

            ble_position_map.insert(std::pair< BSSID, BLE_position >(ble_hash, ble_location));
        }
    }
    return (ble_position_map.size() > 0);
}
#else 
bool BLE_Proximity_DB::readFormatBle5(const char* const pBleMap, const size_t bleFileSizeInBytes, const GeoLocConverter2D &converter)
{
    const double default_blp_height = 2;
    const int    default_blp_type = 0;
    const char delim = ',';


    int status = 0;
    std::string line;
    const int line_length = 1000;
    char line_buf[line_length];

    iostreambuf<char> iostreamBuffer((char *)pBleMap, bleFileSizeInBytes);
    std::iostream stringstr(&iostreamBuffer);

    ble_position_map.clear();

    while (!stringstr.eof())
    {
        stringstr.get(line_buf, line_length - 1, '\n');
        stringstr.ignore(7777, '\n');
        line = std::string(line_buf);
        if (line == "")
            continue;

        std::string line1 = line;
        line1.erase(line1.length() - 1);

        line = line + "\n";

        // line parsing
        {
            uint8_t uuid[16];
            uint16_t beacon_major = 0;
            uint16_t beacon_minor = 0;
            int16_t txPower(0);
			int16_t txPower_correction(0);
            double lat(0), lon(0), floor(0);
            double blp_height = default_blp_height;
            int blp_type = default_blp_type;

            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;

            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }
            if (result.size() < 7)
                continue;

            std::stringstream ss;

            // UUID
            string_to_ble(result[0], uuid);

            // major
            try
            {
                beacon_major = static_cast<uint16_t>(std::stoul(result[1]));
                beacon_minor = static_cast<uint16_t>(std::stoul(result[2]));

                // BLE location
                lat = std::stod(result[3]);
                lon = std::stod(result[4]);
                floor = std::stoi(result[5]);

                txPower = std::stoi(result[6]);

                // floor height
                blp_height = default_blp_height; 
                if (result.size() >= 8)
                {
                    blp_height = std::stod(result[7]);
                }
                
                // blp type
                blp_type = default_blp_type;
                if (result.size() >= 9)
                {
                    blp_type = std::stoi(result[8]);
                }

                if (result.size() >= 10)
                {
                    txPower_correction = std::stoi(result[9]);
                }
            }
            catch (...)
            {
                continue;
            }

            double x(0), y(0);
            
            if (converter.Geo2Local(lat, lon, &x, &y))
            {
                BLE_position ble_location;
                ble_location.loc.set(x, y, floor, 1, true);
                ble_location.txPower = txPower;
                ble_location.txPower_correction = txPower_correction;
                ble_location.blp_height = blp_height;
                ble_location.beaconType = static_cast<BleBeaconType> (blp_type);

                BSSID ble_hash = getBleHash(beacon_major, beacon_minor, uuid);

                ble_position_map.insert(std::pair< BSSID, BLE_position >(ble_hash, ble_location));
            }
        }
    }
    return (ble_position_map.size() > 0);
}
#endif

//BLE_position BLE_Proximity_DB::GetLocation(const Fppe::BleScanResult  &measurement, uint64_t time_stamp) const {}
BLE_position BLE_Proximity_DB::GetLocation(const std::vector<Fppe::BleMeasCalibrated>  &measurement, uint64_t time_stamp) const
{
    BLE_position location = {};
    int count = 0;
    
    if (&measurement != NULL)
    {
        std::vector<Fppe::BleMeasCalibrated> ble_meas = measurement;
        
        // bias compensation
        for (auto it = ble_meas.begin(); it != ble_meas.end(); ++it)
        {
            it->rssi -= (RSSI)round(it->bias);
        }
        
        // sort from the most strong to the most weak
        std::sort(ble_meas.begin(), ble_meas.end(),
            [](const Fppe::BleMeasCalibrated &mea1, const Fppe::BleMeasCalibrated &mea2)
        { 
            if ((mea1.rssi - mea1.txPower) > (mea2.rssi - mea2.txPower))
                return true;
            else
                if ((mea1.rssi - mea1.txPower) == (mea2.rssi - mea2.txPower))
                    if (getBleHash(mea1.major, mea1.minor, mea1.uuid) < getBleHash(mea2.major, mea2.minor, mea2.uuid))
                        return true;
            return false;
        });

        // select BLE from DB in processed measurement packet
        for (auto it = ble_meas.begin(); it != ble_meas.end(); ++it)
        {
            BSSID  ble_hash = getBleHash(it->major, it->minor, it->uuid);
            
            if (GetDbItem(ble_hash, location) == false)  
                continue;
            
            
            if (location.loc.valid)
            {
                count++;
                location.loc.valid = false;// reset flag before taking it again
                if (it->rssi < (it->txPower + SingleCutoffThreshold))   // rejection by RSSI threshold
                {
                    ble_data->reset();
                }
                else
                {
                    if (ble_data->putCurrentBLEHash(ble_hash, time_stamp))
                    {
                        location.loc = ble_data->GetLocation(ble_position_map, time_stamp);
                    }
                }
                double expected_distance = pow(10, ((double)((int)it->txPower) - (int)it->rssi) / 20.); // ring
                double pos_uncertainty = pow(10, ((double)((int)it->txPower) - (int)it->rssi + it->rssi_sigma + it->bias_sigms) / 20.);
                //double pos_uncertainty = 5;
                location.distance = location.loc.metric = expected_distance;
                location.loc.p = 1;
                location.loc.rms_xy = pos_uncertainty;
                location.loc.h = location.blp_height;
                location.rxPower = it->rssi;
                location.major = getBleMajor(ble_hash);
                location.minor = getBleMinor(ble_hash);
                break;
            }
        }
    }
    return location;
}

//BLE_position BLE_Proximity_DB::GetLocation(const std::vector<Fppe::BleMeasCalibrated>  &measurement, uint64_t time_stamp) const
void BLE_Proximity_DB::GetLocation(const std::vector<Fppe::BleMeasCalibrated>  &measurement, uint64_t time_stamp, std::vector<WiFi_Location> &LocationList) const
{
    const int LocationListMax = 3;

    LocationList.clear();

    if ((&measurement != NULL) && (measurement.size() > 0))
    {
        std::vector<Fppe::BleMeasCalibrated> ble_meas = measurement;

        // bias compensation
        for (auto it = ble_meas.begin(); it != ble_meas.end(); ++it)
        {
            it->rssi -= (RSSI)round(it->bias);
        }

        // sort from the most strong to the most weak
        std::sort(ble_meas.begin(), ble_meas.end(),
            [](const Fppe::BleMeasCalibrated &mea1, const Fppe::BleMeasCalibrated &mea2)
        {
            if ((mea1.rssi - mea1.txPower) > (mea2.rssi - mea2.txPower))
                return true;
            else
                if ((mea1.rssi - mea1.txPower) == (mea2.rssi - mea2.txPower))
                    if (getBleHash(mea1.major, mea1.minor, mea1.uuid) < getBleHash(mea2.major, mea2.minor, mea2.uuid))
                        return true;
            return false;
        });

        // select BLE from DB in processed measurement packet
        for (auto it = ble_meas.begin(); it != ble_meas.end(); ++it)
        {
            BLE_position location = {};
            BSSID  ble_hash = getBleHash(it->major, it->minor, it->uuid);
            try
            {
                location = ble_position_map.at(ble_hash);
            }
            catch (std::out_of_range)
            {
                continue;
            }
            if (location.loc.valid)
            {
                if ((std::abs(it->rssi) - std::abs(it->txPower)) < std::abs(MultipleCutoffThreshold))
                {
                    //double expected_distance = pow(10, ((double)((int)it->rssi - (int)location.txPower)) / 20.); // ring
                    //location.loc.metric = expected_distance;
                    //location.loc.rms_xy = 0; // is not provided now
                    //location.loc.h = location.blp_height;
                    //LocationList.push_back(location.loc);
                    
                    double expected_distance = pow(10, ((double)((int)it->txPower) - (int)it->rssi) / 20.); // ring
                    double pos_uncertainty = pow(10, ((double)((int)it->txPower) - (int)it->rssi + it->rssi_sigma + it->bias_sigms) / 20.);
                    //double pos_uncertainty = pow(10, (it->rssi_sigma + it->bias_sigms) / 20.);
                    location.distance = location.loc.metric = expected_distance;
                    location.loc.p = 1;
                    location.loc.rms_xy = pos_uncertainty;
                    location.loc.h = location.blp_height;
                    location.rxPower = it->rssi;
                    location.major = getBleMajor(ble_hash);
                    location.minor = getBleMinor(ble_hash);
                    LocationList.push_back(location.loc);
                }
            }
        }
    }
    if (LocationList.size() > LocationListMax)
    {
        LocationList.resize(LocationListMax);
    }

    return ;
}

std::map < BSSID, BLE_position> BLE_Proximity_DB::GetBLEProximityMap() const
{
    return ble_position_map;
}

void BLE_Proximity_DB::string_to_ble(std::string const &s, uint8_t *ble)
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
        ble[15 - i] = tmp[i];
    }
    /*
    std::cout << std::hex << *(uint64_t *)(tmp + 8) << *(uint64_t *)tmp << "      ";
    std::cout << *(uint64_t *)(ble + 8) << *(uint64_t *)ble << "      ";
    std::cout << std::dec << std::endl;
    */
}
