#ifndef BLE_DATA_PARSER_HPP
#define BLE_DATA_PARSER_HPP

#include <stdint.h>
#include <string>
#include <vector>

#include "Fpbl.hpp"

class ble_data_parser
{
public:
    ble_data_parser() 
    {
        mBleScanResultTimestamps.clear();
        mBleScanResults.clear();
    };
    ~ble_data_parser() {};
  
  //	ble_data_parser(uint8_t* tpn_packet, uint16_t tpn_packet_size);
  bool parse_ble_file(std::string file_path);

  /** Puts single path ble measurements into processing
  * \param[in] timestamp unix time in [us]
  * \param[in] ble  BLE scan result
  * \return status code*/
  Fpbl::ReturnStatus processBleMeasurement(const uint64_t &timestamp, const BleScanResult &bleScan);

  size_t  getBleMeasurementSize()
  {
      return mBleScanResultTimestamps.size();
  }

  uint64_t getBleMeasurementTimestamp(const uint32_t number);
  BleScanResult getBleMeasurement(const uint32_t number);

	bool set_packet_data(uint8_t* tpn_packet, uint16_t tpn_packet_size);
	bool is_state_correct();


private:
	uint8_t* get_entity(uint16_t id);

private:
	uint16_t sz;
	uint8_t* data;

  std::vector<uint64_t> mBleScanResultTimestamps;
  std::vector<BleScanResult> mBleScanResults;

  uint64_t string_to_mac(std::string const &s);
  void ble_data_parser::string_to_ble(std::string const &s, uint8_t *ble);

};

#endif // BLE_DATA_PARSER_HPP
