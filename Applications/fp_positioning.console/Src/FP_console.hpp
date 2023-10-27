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
#include "Venue.h"
#include "IFPBasesReader.hpp"
#include "IInputDataReader.hpp"

#include "fpVenue.hpp"

// Console class
namespace FPPositionConsole
{
    struct StartPosition : Fppe::Position
    {
        StartPosition() : isTimeValid(false), isPositionValid(false), isAzimuthValid(false), isFloorValid(false)
            //,isPositionUncertaintyValid(false), isAzimuthUncertaintyValid(false), isFloorUncertaintyValid(false)
        {};

        bool isTimeValid;
        bool isPositionValid;
        //bool isPositionUncertaintyValid;
        bool isAzimuthValid;
        //bool isAzimuthUncertaintyValid;
        bool isFloorValid;
        //bool isFloorUncertaintyValid;
    };

    class FP_console
    {
        public:
            FP_console(const std::string name, const std::string log_folder );
            ~FP_console();

            VersionNumber getLibVersion();
            VersionNumberBase getLibVersion_C();

			void setRandomSeeds();
            void setUseBarometer(bool enable); 
            void setOsType(OperationSystemType os_type);
            
            Fppe::ReturnStatus setVenueParams(const BaseVenue &venue);

            Fppe::ReturnStatus setFPBasesFolder( const std::string &input_data_folder );

            Fppe::ReturnStatus getBaseName( std::string &base_name, const std::string &file_mask );
			Fppe::ReturnStatus initializeMapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes);

            Fppe::ReturnStatus initializeMFP(const char* const pMFPMap, const size_t mfpFileSizeInBytes, const double max_X, const double max_Y, const double cellSize, const int minFloor, const int maxFloor);
            Fppe::ReturnStatus initializeMFP(const char* const pMFPMap, const size_t mfpFileSizeInBytes); // new interface support
			FPHeaderBaseType getMfpInfo();
            void setUpdateMFP( bool enable ); /**< updater MFP control */
            void setUpdateMapMatching(bool enable); /**< updater Map Matching control */

            Fppe::ReturnStatus initializeWiFi( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p );
            Fppe::ReturnStatus initializeWiFi(const char* const pWiFiMap, const size_t wifiFileSizeInBytes);
            FPHeaderBaseType getWfpInfo();
            void setUpdateWiFi( bool enable ); /**< updater WiFi control */

            Fppe::ReturnStatus initializeBLE( const char* const pBleMap, const size_t bleFileSizeInBytes, const double min_p );
            Fppe::ReturnStatus initializeBLE(const char* const pBleMap, const size_t bleFileSizeInBytes);
            FPHeaderBaseType getBfpInfo();
            void setUpdateBLE( bool enable ); /**< updater BLE control */
            Fppe::ReturnStatus  getWiFiBias(double *bias);
            void setWiFiBias(const double  &bias, int64_t delta_t);
            Fppe::ReturnStatus  getBLEBias(double *bias);
            void  setBLEBias(const double  &bias, int64_t delta_t);

            Fppe::ReturnStatus initializeBLEProximity(const char* const pBleMap, const size_t bleFileSizeInBytes);
            FPHeaderBaseType getProximityDbInfo();
            void setUpdateBLEProximity(bool enable); /**< updater BLE control */

            void setUpdateFrameworkPos(bool enable); /**< updater Framework position control */
            void setUpdateCollaboration(bool enable); /**< updater Framework position control */

            Fppe::ReturnStatus setInputDataFolder( const std::string &input_data_folder );
            Fppe::ReturnStatus setInputIncMagDataFile( const std::string &file_mask );
            Fppe::ReturnStatus setInputWiFiDataFile( const std::string &file_mask );
            Fppe::ReturnStatus setInputBleDataFile( const std::string &file_mask );
            Fppe::ReturnStatus setInputFrameworkDataFile(const std::string &file_mask);
            Fppe::ReturnStatus setInputCollaborationDataFile(const std::string &file_mask);

            //Fppe::ReturnStatus setOutputLogsFolder( const std::string &output_logs_folder );
            Fppe::ReturnStatus setOutputMagLogFile(const std::string &output_maglog_file, const std::string &output_maglog_file_dbg = "", const std::string &output_particleslog_file_dbg = "");
            Fppe::ReturnStatus setOutputWiFiLogFile( const std::string &output_wifilog_file, const std::string &output_wifilog_file_dbg = "" );
            Fppe::ReturnStatus setOutputBLELogFile( const std::string &output_blelog_file, const std::string &output_blelog_file_dbg = "" );
            Fppe::ReturnStatus setOutputBLEProximityLogFile(const std::string &output_bleproximitylog_file, const std::string &output_bleproximitylog_file_dbg = "");
            Fppe::ReturnStatus setOutputMixedLogFile(const std::string &output_mixedlog_file, const std::string &output_mixedlog_file_dbg = "", const std::string &output_particleslog_file_dbg = "");
            Fppe::ReturnStatus setOutputVenueDetectionLogFile(const std::string &output_venue_detect_log_file_dbg = "");
            Fppe::ReturnStatus setOutputExtendedProximityLogFile(const std::string &output_extended_proximity_log_file, const std::string &output_extended_proximity_log_file_dbg = "");

            Fppe::ReturnStatus closeOutputLogFiles();

            void setStartPosition( const Fppe::Position &position );

            Fppe::ReturnStatus getTpnOutData(TpnOutput &tpnData, Fppe::WiFiScanResult &wifi_scan_result, Fppe::BleScanResult &ble_scan_result);
            Fppe::ReturnStatus getInputBleData( Fppe::BleScanResult &ble_scan_result );
            Fppe::ReturnStatus getInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result );
            Fppe::ReturnStatus getInputFrameworkData(Fppe::Position &framework_position);
            Fppe::ReturnStatus getInputCollaborationData(std::vector <Fppe::CollaborationData> &collaboration_position);

            Fppe::ReturnStatus processInputIncMagData( Fppe::CoordinatesIncrement coordinatesIncrement, MagneticVector mag_vector );
            Fppe::ReturnStatus processTpnOutput( const TpnOutput &tpnData );
            Fppe::ReturnStatus processInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result );
            Fppe::ReturnStatus processInputBleData( Fppe::BleScanResult &ble_scan_result );
            Fppe::ReturnStatus processInputFrameworkPosition(Fppe::Position &framework_position);
            Fppe::ReturnStatus processInputCollaboration(std::vector <Fppe::CollaborationData> &collaboration_position);

            Fppe::ReturnStatus readMagneticBiasFromFile( Fppe::MagneticCalibrationParam *bias_cov, const std::string &input_magbias_file );
            Fppe::ReturnStatus writeMagneticBiasToFile( Fppe::MagneticCalibrationParam bias_cov, const std::string &output_magbias_file );

            bool getMagneticBias( Fppe::MagneticCalibrationParam *bias_cov );
            void setMagneticBias( const Fppe::MagneticCalibrationParam &bias_cov );

            PlatformType getPlatformType();
            void setPlatformType(PlatformType platform_type);
            void setBlpPulling(ePullingType type, double pulling_distance, double pulling_sigma = 0.5);

            void setBlpDetectionEnable(bool enable);
            void setBlpPositioningPdFilterParams(
                int peak_detector_max_delay_ms_in_moving,
                double descending_factor_in_moving,
                int peak_detector_max_delay_ms_in_stop,
                double descending_factor_in_stop
            );
            void setBlpDetectionPdFilterParams(
                int peak_detector_max_delay_ms_in_moving,
                double descending_factor_in_moving,
                int peak_detector_max_delay_ms_in_stop,
                double descending_factor_in_stop
            );
            void setBlpPositioningLogicParams(int filter_length, int repeat_number,
                int cutoff);

            void setBlpDetectionLogicParams(int filter_length, int repeat_number,
                int cutoff);

            int getProximityBeaconsNumber(int16_t floor, BleBeaconType beacon_type);

            void setMagneticFilterEnable(bool enable);

            void restart();

        private:
            std::string FPname;
            //Venue venue;
            BaseVenue venue;

            Fppe::FPEngine *fpEngine;

            Fppe::IPositionUpdate *pPosMagCbk;
            Fppe::IPositionUpdate *pPosWiFiCbk;
            Fppe::IPositionUpdate *pPosBleCbk;
            Fppe::IPositionUpdate *pPosBleProximityCbk;
            Fppe::IPositionUpdate *pPosMixedCbk;
            Fppe::IVenueDetectionUpdate *pVenueDetectCbk;
            Fppe::IExtendedProximityUpdate *pExtendedProximityCbk;

            IFPBasesReader    *pFPBasesReader;
            IInputDataReader  *pInputDataReader;

            std::ifstream inputstream;
            std::ofstream pf_log;
            std::ofstream logs[10];

            Fppe::BSSID string_to_mac( std::string const &s );
        
            std::string log_folder;
    };
}

#endif // FP_CONSOLE_HPP
