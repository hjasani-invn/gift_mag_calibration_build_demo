#ifndef IINPUTDATAREADER_HPP
#define IINPUTDATAREADER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <vector>
#include <regex>
#include <cstdint>
#include <sstream>
// WARNING: dirent.h is not provided with visual studio
#include "dirent.h"

#include "Fppe.hpp"

namespace FPPositionConsole
{
    class IInputDataReader
    {
        public:
            virtual ~IInputDataReader() {};

            virtual Fppe::ReturnStatus setInputDataFolder( const std::string &input_data_folder ) = 0;

            virtual Fppe::ReturnStatus setInputIncMagDataFile( const std::string &file_mask ) = 0;

            virtual Fppe::ReturnStatus setInputWiFiDataFile( const std::string &file_mask ) = 0;

            virtual Fppe::ReturnStatus setInputBleDataFile( const std::string &file_mask ) = 0;

            virtual Fppe::ReturnStatus setInputFrameworkDataFile(const std::string &file_mask) = 0;

            virtual Fppe::ReturnStatus setInputCollaborationDataFile(const std::string &file_mask) = 0;

            virtual Fppe::ReturnStatus getInputIncMagData(TpnOutput &tpnOutput, Fppe::WiFiScanResult &wifi_scan_result, Fppe::BleScanResult &ble_scan_result) = 0;

            virtual Fppe::ReturnStatus getInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result ) = 0;

            virtual Fppe::ReturnStatus getInputBleData( Fppe::BleScanResult &ble_scan_result ) = 0;

            virtual Fppe::ReturnStatus getInputFrameworkData(Fppe::Position &frame_work_position) = 0;

            virtual Fppe::ReturnStatus getInputCollaborationData(std::vector <Fppe::CollaborationData> &collaboration_position) = 0;
    };
}

#endif // IINPUTDATAREADER_HPP