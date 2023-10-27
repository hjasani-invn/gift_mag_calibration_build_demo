
#define _USE_MATH_DEFINES
#include <fstream>
#include <iostream>
#include <cmath>

#include "model_data_reader.hpp"
#include "tpn_packet_creator.hpp"
#include "CoordinateConverter.h"
#include "CmdReader.hpp"
using namespace std;

#include "tpn_data_struct.hpp"
#include "tpn_adapter_from_model.hpp"
#include "tpn_entity_creator.hpp"

#include"wifi_data_parser.hpp"
#include"ble_data_parser.hpp"


int main( int argc, const char *argv[], char *envp[] )
{
    model_data_reader model_data_reader;
    model_output_data mod_output;

    std::string input_model_file_name;
    std::string input_wifi_file_name;
    std::string input_ble_file_name;
    std::string output_tpn_file_name;

    setOptionFromCmd( argc, argv, "--input_model", &input_model_file_name );
    setOptionFromCmd(argc, argv, "--input_wifi", &input_wifi_file_name);
    setOptionFromCmd(argc, argv, "--input_ble", &input_ble_file_name);
    setOptionFromCmd(argc, argv, "--output_tpn", &output_tpn_file_name);

    std::cout << std::dec << "input model file name = " << input_model_file_name << std::endl;

    std::ifstream ifs( input_model_file_name, std::ios::in  );
    std::ofstream ofs( output_tpn_file_name, std::ios::out | std::ios::binary );

    model_data_reader.setStream( &ifs );

	uint32_t epoch_number = 0;
  uint32_t wifi_number = 0;
  uint32_t ble_number = 0;

  WiFiScanResult wifiScan;
  BleScanResult  bleScan;
  uint64_t  timestampWiFi = 0;
  uint64_t  timestampBle = 0;
  uint64_t  timestampTpn = 0;
  uint64_t  timestampTpnPrev = 0;

  const uint64_t tpnTimeLength = 20; // s

	tpn_packet_creator packet_creator;
	tpn_entity_creator entity_creator;

	tpn_data data_for_tpn;

	uint8_t* p_tpn_packet = NULL;
	uint16_t tpn_packet_size;
	std::vector <entity_id_size_data> entities;
	entity_id_size_data entity_time;
	entity_id_size_data entity_position;
	entity_id_size_data entity_position_std;
	entity_id_size_data entity_attitude;
	entity_id_size_data entity_attitude_std;
	entity_id_size_data entity_attitude_mag;
	entity_id_size_data entity_attitude_orien;
	entity_id_size_data entity_grp_o7;
	entity_id_size_data entity_dh;
	entity_id_size_data entity_dbg;
	entity_id_size_data entity_heading_misalignment;
	entity_id_size_data entity_floor_number;
	entity_id_size_data entity_stride_information;
  entity_id_size_data entity_wifi_scan;
  entity_id_size_data entity_ble_scan;

  tpn_entity_wifi_scan_t tpn_entity_wifi_scan;
  tpn_entity_ble_scan_t  tpn_entity_ble_scan;

  wifi_data_parser  wifi_parser;
  wifi_parser.parse_wifi_file(input_wifi_file_name);

  ble_data_parser  ble_parser;
  ble_parser.parse_ble_file(input_ble_file_name);

  size_t  wifi_size = wifi_parser.getWiFiMeasurementSize();
  size_t  ble_size = ble_parser.getBleMeasurementSize();
  bool    new_wifi_time = false;
  bool    new_ble_time = false;
  std::cout << std::dec << "wifi size = " << wifi_size << std::endl;
  std::cout << std::dec << "ble size = " << ble_size << std::endl;

  if (wifi_number < wifi_size)
      timestampWiFi = wifi_parser.getWiFiMeasurementTimestamp(wifi_number);

  if (ble_number < ble_size)
      timestampBle = ble_parser.getBleMeasurementTimestamp(ble_number);
  
  while (model_data_reader.read_next_line(mod_output))
	{
		epoch_number++;

		tpn_adapter_from_model(mod_output, data_for_tpn);

		packet_creator.set_epoch_number(epoch_number);
		entities.clear();

    timestampTpn = (uint64_t)floor(data_for_tpn.tpn_entity_time.timetag_ * 1000 + 0.5);

		entity_creator.create_entity_time(data_for_tpn.tpn_entity_time, entity_time);
		entities.push_back(entity_time);

		entity_creator.create_entity_position(data_for_tpn.tpn_entity_position, entity_position);
		entities.push_back(entity_position);

		entity_creator.create_entity_position_std(data_for_tpn.tpn_entity_position_std, entity_position_std);
		entities.push_back(entity_position_std);
		
		entity_creator.create_entity_attitude(data_for_tpn.tpn_entity_attitude, entity_attitude);
		entities.push_back(entity_attitude);
		entity_creator.create_entity_attitude_standard_deviation(data_for_tpn.tpn_entity_attitude_standard_deviation, entity_attitude_std);
		entities.push_back(entity_attitude_std);
 
		entity_creator.create_entity_magnetic_data(data_for_tpn.tpn_entity_magnetometer_data, entity_attitude_mag);
		entities.push_back(entity_attitude_mag);
		entity_creator.create_entity_orientation_pitch_d(data_for_tpn.tpn_entity_orientation_pitch_d, entity_attitude_orien);
		entities.push_back(entity_attitude_orien);
		entity_creator.create_entity_grp_07(data_for_tpn.tpn_entity_grp_07, entity_grp_o7);
		entities.push_back(entity_grp_o7);
		entity_creator.create_entity_device_heading(data_for_tpn.tpn_entity_device_heading, entity_dh);
		entities.push_back(entity_dh);
		entity_creator.create_entity_internal_dbg(data_for_tpn.tpn_entity_internal_dbg, entity_dbg);
		entities.push_back(entity_dbg);
	    entity_creator.create_entity_heading_misalignment(data_for_tpn.tpn_entity_heading_misalignment, entity_heading_misalignment);
		entities.push_back(entity_heading_misalignment);
		entity_creator.create_entity_floor_number(data_for_tpn.tpn_entity_floor_number, entity_floor_number);
		entities.push_back(entity_floor_number);

		entity_creator.create_entity_stride_information(data_for_tpn.tpn_entity_stride_information, entity_stride_information);
		entities.push_back(entity_stride_information);

    // WiFi
        if (wifi_number < wifi_size)
        {
            if (timestampWiFi < timestampTpnPrev)
            {
                wifi_number++;
                new_wifi_time = true;
                timestampWiFi = wifi_parser.getWiFiMeasurementTimestamp(wifi_number);
            }

    if ( (timestampTpnPrev > 0) && (timestampWiFi >= timestampTpnPrev) && (timestampWiFi <= timestampTpn))
    {
            new_wifi_time = false;
            wifiScan = wifi_parser.getWiFiMeasurement(wifi_number);
            int t = wifi_parser.getWiFiMeasurementTimestamp(wifi_number);
            for (auto it = wifiScan.scanWiFi.begin(); it != wifiScan.scanWiFi.end(); it++)
            {
                tpn_entity_wifi_scan.frequency = it->frequency;
                tpn_entity_wifi_scan.mac = it->mac;
                tpn_entity_wifi_scan.rssi = it->rssi;
                tpn_entity_wifi_scan.timestamp = it->timestamp;

                entity_creator.create_entity_wifi_scan(tpn_entity_wifi_scan, entity_wifi_scan);
                entities.push_back(entity_wifi_scan);
            }
            wifi_number++;
                timestampWiFi = wifi_parser.getWiFiMeasurementTimestamp(wifi_number);
    }


        }
#if 0
    if ( (timestampTpnPrev > 0) && (timestampWiFi >= timestampTpnPrev) && (timestampWiFi <= timestampTpn))
    {
        if (wifi_number < wifi_size)
        {
            new_wifi_time = false;
            wifiScan = wifi_parser.getWiFiMeasurement(wifi_number);
            int t = wifi_parser.getWiFiMeasurementTimestamp(wifi_number);
            std::cout << "wifiScan :"<< wifi_number << "    " << t << "\n";
            for (auto it = wifiScan.scanWiFi.begin(); it != wifiScan.scanWiFi.end(); it++)
            {
                tpn_entity_wifi_scan.frequency = it->frequency;
                tpn_entity_wifi_scan.mac = it->mac;
                tpn_entity_wifi_scan.rssi = it->rssi;
                tpn_entity_wifi_scan.timestamp = it->timestamp;

                entity_creator.create_entity_wifi_scan(tpn_entity_wifi_scan, entity_wifi_scan);
                entities.push_back(entity_wifi_scan);
            }
            wifi_number++;
        }
    }
    else
    {
        if (wifi_number < wifi_size)
        {
            if (timestampWiFi < timestampTpnPrev)
            {
                wifi_number++;
                int t = wifi_parser.getWiFiMeasurementTimestamp(wifi_number);
                std::cout << "wifiScan_1 :"<< wifi_number << "    " << t << "\n";
                if (wifi_number < wifi_size)
                {
                    new_wifi_time = true;
                    timestampWiFi = wifi_parser.getWiFiMeasurementTimestamp(wifi_number);
                }
            }
        }
    }
#endif
    // Ble
    if ((timestampTpnPrev > 0) && (timestampBle >= timestampTpnPrev) && (timestampBle <= timestampTpn))
    {
        while ((ble_number < ble_size) && (timestampBle <= timestampTpn))
        {
            new_ble_time = false;
            bleScan = ble_parser.getBleMeasurement(ble_number);
            for (auto it = bleScan.scanBle.begin(); it != bleScan.scanBle.end(); it++)
            {
                memset(&tpn_entity_ble_scan, 0, sizeof(tpn_entity_ble_scan));
                tpn_entity_ble_scan.timestamp = it->timestamp;      /// unix time [us]
                tpn_entity_ble_scan.mac = it->mac;            /// MAC addres in decimal form 
                tpn_entity_ble_scan.has_MAC = it->hasMAC;        /// mac address avaliability flag
                tpn_entity_ble_scan.frequency = it->frequency;      /// central channel frequency [MHz]
                tpn_entity_ble_scan.major = it->major;          /// iBeacon major number
                tpn_entity_ble_scan.minor = it->minor;          /// iBeacon minor number

                tpn_entity_ble_scan.uuid_h = *(uint64_t *)((it->uuid) + 8);         /// iBeacon uuid 64 hight bytes
                tpn_entity_ble_scan.uuid_l = *(uint64_t *)(it->uuid);         /// iBeacon uuid 64 low bytes

                //std::cout << std::hex << tpn_entity_ble_scan.uuid_h << std::hex << tpn_entity_ble_scan.uuid_l << std::endl;

                tpn_entity_ble_scan.tx_power = it->txPower;       /// iBeacon tx power level [dbm] on 1m distance
                tpn_entity_ble_scan.rssi = it->rssi;

                entity_creator.create_entity_ble_scan(tpn_entity_ble_scan, entity_ble_scan);
                entities.push_back(entity_ble_scan);
            }
            ///////////
#if 0
            if (ble_number < ble_size - 1)
            {
                timestampBle = ble_parser.getBleMeasurementTimestamp(ble_number + 1);
                if (timestampBle < timestampTpn)
                {
                    ble_number++;
                }
            }
#else
            ble_number++;
            if (ble_number < ble_size)
            {
                new_ble_time = true;
                timestampBle = ble_parser.getBleMeasurementTimestamp(ble_number);
            }
#endif
            else
                break;
            ///////////
        }
    }
    else
    {
        if (ble_number < ble_size)
        {
            if (timestampBle < timestampTpnPrev)
            {
                timestampBle = ble_parser.getBleMeasurementTimestamp(ble_number);
            }
            while ((timestampBle < timestampTpnPrev) && (ble_number < ble_size))
            {
                ble_number++;
                if (ble_number < ble_size)
                {
                    new_ble_time = true;
                    timestampBle = ble_parser.getBleMeasurementTimestamp(ble_number);
                }
            }
        }
    }
    packet_creator.create_packet(entities);
    packet_creator.get_packet_data(&p_tpn_packet, tpn_packet_size);

    timestampTpnPrev = timestampTpn;

    ofs.write((const char*)p_tpn_packet, tpn_packet_size);
    ofs.flush();
  }

  std::cout << "wifi_number = " << wifi_number << std::endl;
  std::cout << "ble_number = " << ble_number << std::endl;
#if 0
  // WiFi, Ble after magnetic
  bool wifi_next = true;
  bool ble_next = true;

  while ((wifi_number < wifi_size) || (ble_number < ble_size))
  {
      if ( (wifi_number < wifi_size) && wifi_next )
      {
          if (new_wifi_time)
          {
              new_wifi_time = false;
              wifiScan = wifi_parser.getWiFiMeasurement(wifi_number);
          }
          else
          {
              wifi_number++;
              if (wifi_number < wifi_size)
              {
                  timestampWiFi = wifi_parser.getWiFiMeasurementTimestamp(wifi_number);
                  wifiScan = wifi_parser.getWiFiMeasurement(wifi_number);
              }
          }
      }
      wifi_next = false;
      if ((ble_number < ble_size ) && ble_next)
      {
          if (new_ble_time)
          {
              new_ble_time = false;
          }
          else
          {
              ble_number++;
              if (ble_number < ble_size)
              {
                  timestampBle = ble_parser.getBleMeasurementTimestamp(ble_number);
              }
          }
      }
      ble_next = false;

      epoch_number++;

      packet_creator.set_epoch_number(epoch_number);
      entities.clear();

      data_for_tpn.tpn_entity_time.timetag_ += (double)tpnTimeLength/1000;

      timestampTpn += tpnTimeLength;

      data_for_tpn.tpn_entity_magnetometer_data.raw_data_available_ = false;

      entity_creator.create_entity_time(data_for_tpn.tpn_entity_time, entity_time);
      entities.push_back(entity_time);

      entity_creator.create_entity_position(data_for_tpn.tpn_entity_position, entity_position);
      entities.push_back(entity_position);

      entity_creator.create_entity_position_std(data_for_tpn.tpn_entity_position_std, entity_position_std);
      entities.push_back(entity_position_std);

      entity_creator.create_entity_attitude(data_for_tpn.tpn_entity_attitude, entity_attitude);
      entities.push_back(entity_attitude);
      entity_creator.create_entity_attitude_standard_deviation(data_for_tpn.tpn_entity_attitude_standard_deviation, entity_attitude_std);
      entities.push_back(entity_attitude_std);

      entity_creator.create_entity_magnetic_data(data_for_tpn.tpn_entity_magnetometer_data, entity_attitude_mag);
      entities.push_back(entity_attitude_mag);
      entity_creator.create_entity_orientation_pitch_d(data_for_tpn.tpn_entity_orientation_pitch_d, entity_attitude_orien);
      entities.push_back(entity_attitude_orien);
      entity_creator.create_entity_grp_07(data_for_tpn.tpn_entity_grp_07, entity_grp_o7);
      entities.push_back(entity_grp_o7);
      entity_creator.create_entity_device_heading(data_for_tpn.tpn_entity_device_heading, entity_dh);
      entities.push_back(entity_dh);
      entity_creator.create_entity_internal_dbg(data_for_tpn.tpn_entity_internal_dbg, entity_dbg);
      entities.push_back(entity_dbg);
      entity_creator.create_entity_heading_misalignment(data_for_tpn.tpn_entity_heading_misalignment, entity_heading_misalignment);
      entities.push_back(entity_heading_misalignment);
      entity_creator.create_entity_floor_number(data_for_tpn.tpn_entity_floor_number, entity_floor_number);
      entities.push_back(entity_floor_number);

      entity_creator.create_entity_stride_information(data_for_tpn.tpn_entity_stride_information, entity_stride_information);
      entities.push_back(entity_stride_information);


      if ((timestampTpnPrev > 0) && (timestampWiFi >= timestampTpnPrev) && (timestampWiFi < timestampTpn))
      {
          wifi_next = true;

          if (wifi_number < wifi_size)
          {
              for (auto it = wifiScan.scanWiFi.begin(); it != wifiScan.scanWiFi.end(); it++)
              {
                  tpn_entity_wifi_scan.frequency = it->frequency;
                  tpn_entity_wifi_scan.mac = it->mac;
                  tpn_entity_wifi_scan.rssi = it->rssi;
                  tpn_entity_wifi_scan.timestamp = it->timestamp;

                  entity_creator.create_entity_wifi_scan(tpn_entity_wifi_scan, entity_wifi_scan);
                  entities.push_back(entity_wifi_scan);
              }
          }
      }

      // Ble
      if ((timestampTpnPrev > 0) && (timestampBle >= timestampTpnPrev) && (timestampBle <= timestampTpn))
      {
          ble_next = true;
          while ((ble_number < ble_size) && (timestampBle <= timestampTpn))
          {
              bleScan = ble_parser.getBleMeasurement(ble_number);
              for (auto it = bleScan.scanBle.begin(); it != bleScan.scanBle.end(); it++)
              {
                  memset(&tpn_entity_ble_scan, 0, sizeof(tpn_entity_ble_scan));
                  tpn_entity_ble_scan.timestamp = it->timestamp;      /// unix time [us]
                  if (ble_number > 1490)
                      std::cout << std::dec << ble_number << "        ble timestamp5 = " << it->timestamp << std::endl;
                  if (ble_number == 1498)
                      std::cout << "//////////////" << std::endl;
                  tpn_entity_ble_scan.mac = it->mac;            /// MAC addres in decimal form 
                  tpn_entity_ble_scan.has_MAC = it->hasMAC;        /// mac address avaliability flag
                  tpn_entity_ble_scan.frequency = it->frequency;      /// central channel frequency [MHz]
                  tpn_entity_ble_scan.major = it->major;          /// iBeacon major number
                  tpn_entity_ble_scan.minor = it->minor;          /// iBeacon minor number

                  //std::cout << std::hex << *(uint64_t *)((it->uuid) + 8) << std::hex << *(uint64_t *)(it->uuid) << std::endl;

                  tpn_entity_ble_scan.uuid_h = *(uint64_t *)((it->uuid) + 8);         /// iBeacon uuid 64 hight bytes
                  tpn_entity_ble_scan.uuid_l = *(uint64_t *)(it->uuid);         /// iBeacon uuid 64 low bytes

                  //std::cout << std::hex << tpn_entity_ble_scan.uuid_h << std::hex << tpn_entity_ble_scan.uuid_l << std::endl;

                  tpn_entity_ble_scan.tx_power = it->txPower;       /// iBeacon tx power level [dbm] on 1m distance
                  tpn_entity_ble_scan.rssi = it->rssi;

                  entity_creator.create_entity_ble_scan(tpn_entity_ble_scan, entity_ble_scan);
                  entities.push_back(entity_ble_scan);
              }
              ///////////
              ble_number++;
              if (ble_number < ble_size)
              {
                  new_ble_time = true;
                  timestampBle = ble_parser.getBleMeasurementTimestamp(ble_number);
              }
              else
                  break;
              ///////////
          }
      }

      packet_creator.create_packet(entities);
      packet_creator.get_packet_data(&p_tpn_packet, tpn_packet_size);

      timestampTpnPrev = timestampTpn;

      ofs.write((const char*)p_tpn_packet, tpn_packet_size);
      ofs.flush();
  }
#endif

    ofs.close();
    ifs.close();
    return 0;
}
