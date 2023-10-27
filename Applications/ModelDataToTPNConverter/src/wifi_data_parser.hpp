#ifndef WIFI_DATA_PARSER_HPP
#define WIFI_DATA_PARSER_HPP

#include <stdint.h>
#include <string>
#include <vector>

#include "Fpbl.hpp"

class wifi_data_parser
{
public:
    wifi_data_parser()
    {
        mWiFiScanResultTimestamps.clear();
        mWiFiScanResults.clear();
    };
    ~wifi_data_parser(){};
  //	wifi_data_parser(uint8_t* tpn_packet, uint16_t tpn_packet_size);
  bool  parse_wifi_file(std::string file_path);

  /** Puts single path wifi measurements into processing
  * \param[in] timestamp unix time in [us]
  * \param[in] wifi  WiFi scan result
  * \return status code*/
  Fpbl::ReturnStatus processWiFiMeasurement(const uint64_t &timestamp, const WiFiScanResult &wifiScan);

  size_t  getWiFiMeasurementSize()
  {
      return mWiFiScanResultTimestamps.size();
  }

  uint64_t getWiFiMeasurementTimestamp(const uint32_t number);
  WiFiScanResult getWiFiMeasurement(const uint32_t number);

  bool set_packet_data(uint8_t* tpn_packet, uint16_t tpn_packet_size);
	bool is_state_correct();


private:
	uint8_t* get_entity(uint16_t id);

private:
	uint16_t sz;
	uint8_t* data;

  std::vector<uint64_t> mWiFiScanResultTimestamps;
  std::vector<WiFiScanResult> mWiFiScanResults;

  uint64_t string_to_mac(std::string const &s);

};

#endif // WIFI_DATA_PARSER_HPP
