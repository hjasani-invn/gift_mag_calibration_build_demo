#ifndef INPUTDATAREADER_HPP
#define INPUTDATAREADER_HPP

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

#include "IInputDataReader.hpp"
#include "tpn_data_reader.hpp"
#include "tpn_packet_parser.hpp"

namespace FPPositionConsole
{
    class InputDataReader : public IInputDataReader
    {
        public:
            InputDataReader();

            virtual ~InputDataReader();

            virtual Fppe::ReturnStatus setInputDataFolder( const std::string &input_data_folder );

            virtual Fppe::ReturnStatus setInputIncMagDataFile( const std::string &file_mask );

            virtual Fppe::ReturnStatus setInputWiFiDataFile( const std::string &file_mask );

            virtual Fppe::ReturnStatus setInputFrameworkDataFile(const std::string &file_mask);

            virtual Fppe::ReturnStatus setInputCollaborationDataFile(const std::string &file_mask);

            virtual Fppe::ReturnStatus setInputBleDataFile( const std::string &file_mask );

            virtual Fppe::ReturnStatus getInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result );

            virtual Fppe::ReturnStatus getInputBleData( Fppe::BleScanResult &ble_scan_result );

            virtual Fppe::ReturnStatus getInputFrameworkData(Fppe::Position &frame_work_position);

            virtual Fppe::ReturnStatus getInputCollaborationData(std::vector <Fppe::CollaborationData> &collaboration_position);

        private:

            std::string input_data_folder;

            std::ifstream inputPosMagDataStream;
            std::ifstream inputWiFiDataStream;
            std::ifstream inputBleDataStream;
            std::ifstream inputFrameworkDataStream;

            std::vector<std::string> fileslist;

            uint64_t wifiMeasurementNumberPrev;
            Fppe::WiFiMeasurement wifiMeasurement;

            uint64_t bleMeasurementNumberPrev;
            Fppe::BleMeasurement bleMeasurement;

            uint64_t frameworkMeasurementNumberPrev;

            Fppe::ReturnStatus setInputDataFile(std::ifstream &inputDataStream, const std::string &file_mask, std::ios_base::openmode mode = std::ios_base::in);

            friend class TpnInputDataReader;
    };

    class TpnInputDataReader : public InputDataReader
    {
    public:
        TpnInputDataReader() :packet_count(0), p2_s(0), p2_c(0), p2_alpha(0.01)
        {
            stored_e_baro_data.height_ = 0;
            stored_e_baro_data.height_standard_deviation_ = -1.e6;// set negative dispersion as flag of data unavaliability
        }

        virtual Fppe::ReturnStatus setInputIncMagDataFile(const std::string &file_mask);
        virtual Fppe::ReturnStatus getInputIncMagData(TpnOutput &tpnOutput, Fppe::WiFiScanResult &wifi_scan_result, Fppe::BleScanResult &ble_scan_result);
    private:
        tpn_data_reader   tpn_reader;
        tpn_packet_parser tpn_parser;
        int packet_count;
        double p2_s;
        double p2_c;
        double p2_alpha;
        tpn_entity_barometer_data stored_e_baro_data;
    };
}

#endif // INPUTDATAREADER_HPP