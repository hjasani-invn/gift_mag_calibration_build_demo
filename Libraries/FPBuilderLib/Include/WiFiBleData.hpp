#ifndef WIFI_BLE_DATA_HPP
#define WIFI_BLE_DATA_HPP
#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include <ostream>
#include <iomanip>


typedef uint64_t BSSID; /**< mac address in decimal form */

struct _BSSID
{
    uint64_t value; /**< mac address in decimal form */

    _BSSID(uint64_t bssid) : value(bssid){};
    
    operator uint64_t() const { return value; };

    friend std::ostream& operator <<(std::ostream& os, const _BSSID& bssid)
    {
        std::ios_base::fmtflags fmt_state = os.flags(std::ios::hex | std::ios::uppercase);
        char fill_char = os.fill('0');
        unsigned char *bytes = (unsigned char *)&bssid.value;
        os <<         std::setw(2) << (unsigned int)bytes[6]
            << ":" << std::setw(2) << (unsigned int)bytes[5]
            << ":" << std::setw(2) << (unsigned int)bytes[4]
            << ":" << std::setw(2) << (unsigned int)bytes[3]
            << ":" << std::setw(2) << (unsigned int)bytes[2]
            << ":" << std::setw(2) << (unsigned int)bytes[1]
            << ":" << std::setw(2) << (unsigned int)bytes[0];
        fill_char = os.fill(fill_char);
        fmt_state = os.flags(fmt_state);
        return os;
    }
};


struct _UUID
{
    uint16_t bytes[16];  /**< uuid values represented in bytes*/

    _UUID(const uint8_t uuid[16])
    {
        for (int i = 0; i < 16; i++)
        {
            this->bytes[i] = uuid[i];
        }
    };


    friend std::ostream& operator <<(std::ostream& os, const _UUID& uuid)
    {
        std::ios_base::fmtflags fmt_state = os.flags(std::ios::hex);
        char fill_char = os.fill('0');
       // Warning: UUID is printed as 128-bit number in BIG endian format,
       //          according to the eddystone definition: https://github.com/google/eddystone/tree/master/eddystone-uid
        os << std::setw(2) << (unsigned int)uuid.bytes[0]
            << std::setw(2) << (unsigned int)uuid.bytes[1]
            << std::setw(2) << (unsigned int)uuid.bytes[2]
            << std::setw(2) << (unsigned int)uuid.bytes[3]
            << "-"
            << std::setw(2) << (unsigned int)uuid.bytes[4]
            << std::setw(2) << (unsigned int)uuid.bytes[5]
            << "-"
            << std::setw(2) << (unsigned int)uuid.bytes[6]
            << std::setw(2) << (unsigned int)uuid.bytes[7]
            << "-"
            << std::setw(2) << (unsigned int)uuid.bytes[8]
            << std::setw(2) << (unsigned int)uuid.bytes[9]
            << "-"
            << std::setw(2) << (unsigned int)uuid.bytes[10]
            << std::setw(2) << (unsigned int)uuid.bytes[11]
            << std::setw(2) << (unsigned int)uuid.bytes[12]
            << std::setw(2) << (unsigned int)uuid.bytes[13]
            << std::setw(2) << (unsigned int)uuid.bytes[14]
            << std::setw(2) << (unsigned int)uuid.bytes[15];
        fill_char = os.fill(fill_char);
        fmt_state = os.flags(fmt_state);
        return os;
    }
};

/** common rssi measurement*/
struct RSSIMeasurement
{
    int64_t timestamp;  /**< unix time [us] */
    BSSID mac;          /**< MAC addres in decimal form */
    int8_t rssi;        /**< RSSI value [dbm] */
    uint16_t frequency; /**< central channel frequency [MHz] */
};

inline bool compare_RSSIMeasurement_by_mac(const RSSIMeasurement& meas1, const RSSIMeasurement& meas2)
{
    return (meas1.mac < meas2.mac);
};


/** WiFi measurement */
//typedef RSSIMeasurement WiFiMeasurement;
struct WiFiMeasurement : public RSSIMeasurement
{
#if 0
    bool operator < (const WiFiMeasurement &wifiMeasurement)
    {
        if (this->mac < wifiMeasurement.mac)
            return true;
        else 
            return false;
    }
#endif

    friend std::ostream& operator <<(std::ostream& os, WiFiMeasurement& WiFiMeas)
    {
        WiFiMeas.print(os, 0);
        return os;
    }

    void print(std::basic_ostream<char> &out, uint64_t packet_number)
    {
        out << " " << this->timestamp;
        out << ", " << packet_number;
        out << ", " << (_BSSID) this->mac;
        out << ", " << "N/A";
        out << ", " << this->frequency;
        out << ", " << (int)this->rssi;
        out << ", " << (int)this->rssi;
        out << std::endl;
    }
    
};

/** BLE measurement (iBeacon) */
struct BleMeasurement : public RSSIMeasurement
{
    uint16_t major;     /**< iBeacon major number */
    uint16_t minor;     /**< iBeacon minor number */
    uint8_t uuid[16];   /**< iBeacon uuid 128 bit value */
    int8_t txPower;     /**< iBeacon tx power level [dbm] on 1m distance */
    bool hasMAC;        /**< mac address avaliability flag*/

    friend std::ostream& operator <<(std::ostream& os, BleMeasurement& bleMeas)
    {
        bleMeas.print(os, 0);
        return os;
    }

    void print(std::basic_ostream<char> &out, uint64_t packet_number)
    {
        out << " " << this->timestamp;
        out << ", " << packet_number;
        out << ", " << (_BSSID) this->mac;
        out << ", " << "N/A";
        out << ", " << this->frequency;
        out << ", " << (int)this->rssi;
        out << ", " << (int)this->rssi;
        out << ", " << (_UUID) this->uuid;
        out << ", " << this->major;
        out << ", " << this->minor;
        out << ", " << (int)this->txPower;
        out << ", " << (int)this->hasMAC;
        out << std::endl;
    }
};

/** WiFi scan result*/
struct WiFiScanResult
{
    int64_t  timestamp; /**< UNIX time in [ms] */
    uint32_t dataset_id;
    std::vector<WiFiMeasurement> scanWiFi; /**< WiFi observation */

    friend std::ostream& operator << (std::ostream& os, WiFiScanResult& WiFiResult)
    {
        WiFiResult.print(os, 0);
        return os;
    }

    void print(std::basic_ostream<char> &out, uint64_t packet_number)
    {
        for (std::vector <WiFiMeasurement>::iterator WiFiMeas = scanWiFi.begin(); WiFiMeas != scanWiFi.end(); WiFiMeas++)
        {
            WiFiMeas->print(out, packet_number);
        }
    }
};

/** BLE scan result*/
struct BleScanResult
{
    int64_t timestamp; /**< UNIX time in [ms] */
    uint32_t dataset_id;
    std::vector<BleMeasurement> scanBle; /**< BLE observation */

    friend std::ostream& operator << (std::ostream& os, BleScanResult& BleResult)
    {
        BleResult.print(os, 0);
        return os;
    }

    void print(std::basic_ostream<char> &out, uint64_t packet_number)
    {
        for (std::vector <BleMeasurement>::iterator bleMeas = scanBle.begin(); bleMeas != scanBle.end(); bleMeas++)
        {
            bleMeas->print(out, packet_number);
        }
    }
};

#endif //WIFI_BLE_DATA_HPP
