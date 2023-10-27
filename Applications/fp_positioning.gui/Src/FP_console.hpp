#ifndef FP_CONSOLE_HPP
#define FP_CONSOLE_HPP

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
#include "VenueEx.hpp"
#include "IFPBasesReader.hpp"
#include "IInputDataReader.hpp"
#include "track.h"

namespace FPPositionConsole
{
    class FP_console
    {
        public:
            FP_console( std::string name, OutPut_for_image *Out, Track *track );
            ~FP_console();

            Fppe::ReturnStatus setVenueParams( const Venue &venue );

            Fppe::ReturnStatus setFPBasesFolder( const std::string &input_data_folder );

            Fppe::ReturnStatus getBaseName( std::string &base_name, const std::string &file_mask );

            Fppe::ReturnStatus initializeMFP( const std::string &mfp_db_name, const double &max_X, const double &max_Y, const double &cellSize, const int &minFloor, const int &maxFloor );
            void setUpdateMFP( bool enable ); /**< updater MFP control */

            Fppe::ReturnStatus initializeWiFi( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p );
            void setUpdateWiFi( bool enable ); /**< updater WiFi control */

            Fppe::ReturnStatus initializeBLE( const std::string &ble_db_name, const double &min_p );
            void setUpdateBLE( bool enable ); /**< updater BLE control */

            Fppe::ReturnStatus initializeBLEProximity(const char* const pBleMap, const size_t bleFileSizeInBytes);
            void setUpdateBLEProximity(bool enable); /**< updater BLE control */

            Fppe::ReturnStatus setInputDataFolder( const std::string &input_data_folder );
            Fppe::ReturnStatus setInputIncMagDataFile( const std::string &file_mask );
            Fppe::ReturnStatus setInputWiFiDataFile( const std::string &file_mask );
            Fppe::ReturnStatus setInputBleDataFile( const std::string &file_mask );

            Fppe::ReturnStatus setOutputLogsFolder( const std::string &output_logs_folder );
            Fppe::ReturnStatus setOutputMagLogFile( const std::string &output_maglog_file, OutPut_for_image* Out, const std::string &output_maglog_file_dbg = "");
            Fppe::ReturnStatus setOutputWiFiLogFile( const std::string &output_wifilog_file, OutPut_for_image* Out, const std::string &output_wifilog_file_dbg = "");
            Fppe::ReturnStatus setOutputBLELogFile( const std::string &output_blelog_file, OutPut_for_image* Out, const std::string &output_blelog_file_dbg = "");
            Fppe::ReturnStatus setOutputBLEProximityLogFile(const std::string &output_blelog_file, OutPut_for_image* Out, const std::string &output_blelog_file_dbg);
            Fppe::ReturnStatus setOutputMixedLogFile( const std::string &output_mixedlog_file, OutPut_for_image* Out, const std::string &output_mixedlog_file_dbg = "");
            Fppe::ReturnStatus FP_console::setTrackForMFPMode();
            Fppe::ReturnStatus FP_console::setTrackForMixedMode();

            Fppe::ReturnStatus closeOutputLogFiles();

            void setStartPosition(const Fppe::Position &position);

            Fppe::ReturnStatus getTpnOutData( TpnOutput &tpnData, Fppe::WiFiScanResult &wifi_scan_result, Fppe::BleScanResult &ble_scan_result);
            Fppe::ReturnStatus getInputBleData( Fppe::BleScanResult &ble_scan_result );
            Fppe::ReturnStatus getInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result );

            Fppe::ReturnStatus processInputIncMagData( Fppe::CoordinatesIncrement coordinatesIncrement, MagneticVector mag_vector );
            Fppe::ReturnStatus processTpnOutput( const TpnOutput &tpnData );
            Fppe::ReturnStatus processInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result );
            Fppe::ReturnStatus processInputBleData( Fppe::BleScanResult &ble_scan_result );
            Fppe::ReturnStatus processInputBleDataForProximity( Fppe::BleScanResult &ble_scan_result );

            Fppe::ReturnStatus readMagneticBiasFromFile( Fppe::MagneticCalibrationParam *bias_cov, const std::string &input_magbias_file );
            Fppe::ReturnStatus writeMagneticBiasToFile( Fppe::MagneticCalibrationParam bias_cov, const std::string &output_magbias_file );

            void getMagneticBias( Fppe::MagneticCalibrationParam *bias_cov );
            void setMagneticBias( const Fppe::MagneticCalibrationParam &bias_cov );

            OutPut_for_image* Out;

        private:
            std::string FPname;
            Venue venue;

            Fppe::FPEngine *fpEngine;

            Fppe::IPositionUpdate *pPosMagCbk;
            Fppe::IPositionUpdate *pPosWiFiCbk;
            Fppe::IPositionUpdate *pPosBleCbk;
            Fppe::IPositionUpdate *pPosBleProximityCbk;
            Fppe::IPositionUpdate *pPosMixedCbk;

            IFPBasesReader    *pFPBasesReader;
            IInputDataReader  *pInputDataReader;

            std::ifstream inputstream;
            std::ofstream pf_log;
            std::ofstream logs[10];

            Fppe::BSSID string_to_mac( std::string const &s );
            Track *mTrack;

    };
}

#endif // FP_CONSOLE_HPP
