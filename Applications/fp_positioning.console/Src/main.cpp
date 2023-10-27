// FP_builder.cpp : Defines the entry point for7 the console application.
//

#include <iostream>
#include <fstream>
#include <streambuf>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <vector>
#include <queue>
#include <regex>
//#include <io.h>

#include "FP_console.hpp"
#include "CmdReader.hpp"
#include "SettingsParser.hpp"

static void print_fpHeader(std::string caption,  const FPHeaderBaseType &fpHeader)
{
    std::cout << caption << std::endl;
    std::cout << std::hex << "fp_signature:" << fpHeader.fp_signature;
    std::cout << " " << (char)(0xff & (fpHeader.fp_signature >> 0));
    std::cout << (char)(0xff & (fpHeader.fp_signature >> 8));
    std::cout << (char)(0xff & (fpHeader.fp_signature >> 16));
    std::cout << (char)(0xff & (fpHeader.fp_signature >> 24)) << std::endl;
    std::cout << "fp_type:" << fpHeader.fp_type;
    std::cout << " " << (char)(0xff & (fpHeader.fp_type >> 0));
    std::cout << (char)(0xff & (fpHeader.fp_type >> 8));
    std::cout << (char)(0xff & (fpHeader.fp_type >> 16));
    std::cout << (char)(0xff & (fpHeader.fp_type >> 24)) << std::endl;
    std::cout << std::dec << "sz_header:" << fpHeader.sz_header << std::endl;
    std::cout << "sz_data:" << fpHeader.sz_data << std::endl;
    std::cout << std::hex << "crc:" << fpHeader.crc << std::endl;
    std::cout << "version:" << (int)fpHeader.fp_buider_version.major;
    std::cout << "." << (int)fpHeader.fp_buider_version.minor;
    std::cout << "." << (int)fpHeader.fp_buider_version.build;
    std::cout << "." << (int)fpHeader.fp_buider_version.releaseId << std::endl;
    std::cout << std::dec << "fp_build_number:" << fpHeader.fp_build_number << std::endl;
    std::cout << std::dec << "fp_build_time:";
    std::cout << " " << fpHeader.fp_build_time.tm_hour;
    std::cout << ":" << fpHeader.fp_build_time.tm_min;
    std::cout << ":" << fpHeader.fp_build_time.tm_sec;
    std::cout << " " << fpHeader.fp_build_time.tm_mday;
    std::cout << "." << fpHeader.fp_build_time.tm_mon;
    std::cout << "." << fpHeader.fp_build_time.tm_year + 1900;    /* years since 1900 */
    std::cout << " day of week:" << fpHeader.fp_build_time.tm_wday;    /* days since Sunday - [0,6] */
    std::cout << " day of year:" << fpHeader.fp_build_time.tm_yday;    /* days since January 1 - [0,365] */
    std::cout << " isdst:" << fpHeader.fp_build_time.tm_isdst;   /* daylight savings time flag */

    std::cout << std::endl;
}

int main( const int argc, const char** argv )
{

    //std::ofstream ofs;
    //ofs.open("gnss.log", std::ofstream::out | std::ofstream::trunc);
    //ofs.close();
    
    const std::string file_mask_for_mag_base = "(.*)(.mfp4)";
    const std::string file_mask_for_wifi_base = "(.*)(.wifi4)";
    const std::string file_mask_for_ble_base = "(.*)(.ble4)";
    const std::string file_mask_for_ble_proximity_base = "(.*)(.blp4)";
	const std::string file_mask_for_binary_map = "(.*)(.map)";

    const std::string file_mask_for_mag_bias = "(.*)(.mbias)";

    //const std::string file_mask_for_posmag_indata = "(RAMP_)(.*)";
    //const std::string file_mask_for_incmag_indata = "(tpp_output\.dat)";
    //const std::string file_mask_for_incmag_indata = "(.*)(NavOutRawData\.dat)";
    const std::string file_mask_for_incmag_indata = "(.*)(TppOutput\.dat)";
    //const std::string file_mask_for_incmag_indata = "(irl_out\.dat)";
    //const std::string file_mask_for_incmag_indata = "(nav\.dat)";
    //const std::string file_mask_for_incmag_indata = "(rtw_out.dat)";

    const std::string file_mask_for_wifi_indata = "(wifi_in)(.*)";
    const std::string file_mask_for_ble_indata = "(ble_in)(.*)";
    const std::string file_mask_for_framework_indata = "(.*)(framework)(.*)";
    //const std::string file_mask_for_framework_indata = "no_framework.dat";
    const std::string file_mask_collaboration_indata = "(.*)(collab_in)(.*)";

    const std::string file_for_mag_outlog = "mag_out.kml";
    const std::string file_for_wifi_outlog = "wifi_out.kml";
    const std::string file_for_ble_outlog = "ble_out.kml";
    const std::string file_for_ble_proximity_outlog = "ble_proximity_out.kml";
    const std::string file_for_extended_proximity_out = "extended_proximity_out.kml";
    const std::string file_for_mixed_outlog = "mixed_out.kml";

    const std::string file_for_mag_outlog_dbg = "mag_out_dbg.log";
    const std::string file_for_wifi_outlog_dbg = "wifi_out_dbg.log";
    const std::string file_for_ble_outlog_dbg = "ble_out_dbg.log";
    const std::string file_for_ble_proximity_outlog_dbg = "ble_proximity_out_dbg.log";
    const std::string file_for_extended_proximity_out_dbg = "extended_proximity_out.log";
    const std::string file_for_mixed_outlog_dbg = "mixed_out_dbg.log";
    const std::string file_for_venue_detection_outlog_dbg = "venue_detection_dbg.log";

#if 0 // enbale particle output for visualisation
    std::string file_for_particles_outlog_dbg = "particles_out_for_visual.log";
#else
    std::string file_for_particles_outlog_dbg = "";
#endif

    //Venue venue = {};
    BaseVenue venue = {};

    std::string arg;
    std::stringstream ss;
    std::string name;
    double grid_size = 0;
    bool   magEnable = false;
    bool   wifiEnable = false;
    bool   bleEnable = false;
    bool   bleProximityEnable = false;
    bool   mmEnable = false;
    bool   collaborationEnable = false;

    bool wifi_text_format = true;
    bool ble_text_format = true;

    double magneticCellsize = 1.0;
    double magneticFPsizes[3];
    Fppe::Position startPosition;

    bool success = true;

    bool data_load_status = true;

    std::string mag_bias_file = "";
    std::string fp_bases_folder;
    std::string in_data_folder;
    std::string out_log_folder;

    std::string settings_file; // must be in JSON format

    // command line parsing
    success &= setOptionFromCmd( argc, argv, "--settings", &settings_file );

    FILE *pSF = fopen(settings_file.c_str(), "rt");
    std::cout << "settings file: " << settings_file << std::endl;
    if (pSF == 0)
    {
        std::cout << "settings file is not found" << std::endl;
        std::cout << "exit program" << std::endl;
        exit(0);
    }

    startPosition.is_valid = false;
    RtfpplSettings rtfppl_settings;
    parseSettings( settings_file, // input JSON file	with settings
                   name, // output
                   fp_bases_folder,
                   in_data_folder,
                   out_log_folder,
                   venue,
                   magEnable,
                   wifiEnable,
                   bleEnable,
                   bleProximityEnable,
                   mmEnable,
                   collaborationEnable,
                   magneticCellsize,
                   magneticFPsizes
                 );

    
    
    //global debug settings from settings file
    success = parseDebugSettings(settings_file, startPosition, rtfppl_settings);
    std::cout << "debug settings from: " << settings_file << (success ? "" : " - Error") << std::endl;

    // specific debug settings from debug settings file; this file is expected to be located in dataset folder
    std::string debug_settings_file;
    if (setOptionFromCmd(argc, argv, "--debug_settings", &debug_settings_file))
    {
        success = parseDebugSettings(debug_settings_file, startPosition, rtfppl_settings);
        std::cout << "debug settings from: " << debug_settings_file << (success ? "" : " - Error") << std::endl;
        if (false == success)
        {
            std::cout << "exit program" << std::endl;
            exit(0);
        }
    }

    std::cout << "fp folder:: " << fp_bases_folder << std::endl;
    std::cout << "input folder: " << in_data_folder << std::endl;
    std::cout << "output folder: " << out_log_folder << std::endl;
    std::cout << "venue: " << name << std::endl;

#ifdef _WIN32
    if (out_log_folder.back() != '\\')
    {
        out_log_folder = out_log_folder + "\\";
    }
#if 0
    // create output directory
    std::cout << "create output directory (unless it exists)" << std::endl;
    std::string command = "md " + out_log_folder; // folder name must have "\\" instead "/"
    std::cout << command << std::endl;
    system(command.c_str());
#endif
#else
    if (out_log_folder.back() != '/')
    {
        out_log_folder = out_log_folder + "/";
    }
#endif

    FPPositionConsole::FP_console FPc(name, out_log_folder);
    Fppe::ReturnStatus status;
    
    //status = FPc.setVenueParams( venue ); // disabled to match Coursa Venue

    VersionNumber ver = FPc.getLibVersion();
    std::cout << "RTFPPL version: " << int(ver.major) << "." << int(ver.minor) << "." << int(ver.build) << "." << int(ver.releaseId) <<  std::endl;
    VersionNumberBase ver_c = FPc.getLibVersion_C();
    std::cout << "RTFPPL version (C): " << int(ver_c.major) << "." << int(ver_c.minor) << "." << int(ver_c.build) << "." << int(ver_c.releaseId) << std::endl;

    // magnetic bias
    FPc.setFPBasesFolder(in_data_folder);
    status = FPc.getBaseName( mag_bias_file, file_mask_for_mag_bias );

    if ( status == Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        Fppe::MagneticCalibrationParam bias_cov;
        FPc.readMagneticBiasFromFile( &bias_cov, mag_bias_file );

        if ( status == Fppe::ReturnStatus::STATUS_SUCCESS )
         FPc.setMagneticBias( bias_cov );
    }
    else
    {
        Fppe::MagneticCalibrationParam bias_cov1 = {};
        bias_cov1.timestamp = 0;
        bias_cov1.mX = 0;//200.772;
        bias_cov1.mY = 0;
        bias_cov1.mZ = 8;
        memset(bias_cov1.covarianceMatrix, 0, sizeof(bias_cov1.covarianceMatrix));
        bias_cov1.covarianceMatrix[0][0] = 5;// 250;
        bias_cov1.covarianceMatrix[1][1] = 5;// 250;
        bias_cov1.covarianceMatrix[2][2] = 5;// 250;
        //FPc.setMagneticBias( bias_cov1 );
    }

    // load WiFi
    FPc.setFPBasesFolder(fp_bases_folder);
    std::string fp_wifi_base_file;
    status = FPc.getBaseName( fp_wifi_base_file, file_mask_for_wifi_base );
    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "files with mask  " << file_mask_for_wifi_base << "   are not found" << std::endl;
        FPc.setUpdateWiFi( false );
    }
    else
    {
        struct stat file_buf;
        stat( fp_wifi_base_file.c_str(), &file_buf );
        long wifiFileSizeInBytes = file_buf.st_size;
        char *pWiFiMap = ( char * )malloc( wifiFileSizeInBytes );

        FILE *pF = fopen( fp_wifi_base_file.c_str(), "rb" );
        bool success = ( pF != 0 );

        //    _ASSERT( pF );
        if ( pF )
        {
            if ( fread( pWiFiMap, wifiFileSizeInBytes, 1, pF ) == 0 )
            {
                //_ASSERT( 0 );
                success = false;
            }

            fclose( pF );
        }

        if ( success )
        {
            status = FPc.initializeWiFi(pWiFiMap, wifiFileSizeInBytes);
            if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
            {
                std::cout << "wifi db initialisation error: ";
                std::cout << "file = " << fp_wifi_base_file.c_str();
                std::cout << "status = " << static_cast<int>(status);
                std::cout << std::endl;
            }
            else
            {
                std::cout << "wifi fingerprint " << fp_wifi_base_file << " have been successfuly loaded" << std::endl;
                print_fpHeader("wifi fingerprint db info:", FPc.getWfpInfo());
            }
            FPc.setUpdateWiFi( wifiEnable ); /**< updater WiFi control */
        }
        else
        {
            std::cout << "file  " << fp_wifi_base_file << "   can not be opened" << std::endl;
            FPc.setUpdateWiFi( false );
        }
        //FPc.setWiFiBias(0, false);
    }

    // load BLE
    std::string fp_ble_base_file;
    status = FPc.getBaseName( fp_ble_base_file, file_mask_for_ble_base );
    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "files with mask  " << file_mask_for_ble_base << "   are not found" << std::endl;
        FPc.setUpdateBLE( false );
    }
    else
    {
        struct stat file_buf;
        stat( fp_ble_base_file.c_str(), &file_buf );
        long bleFileSizeInBytes = file_buf.st_size;
        char *pBleMap = ( char * )malloc( bleFileSizeInBytes );

        FILE *pF = fopen( fp_ble_base_file.c_str(), "rb" );
        bool success = ( pF != 0 );

        //    _ASSERT( pF );
        if ( pF )
        {
            if ( fread( pBleMap, bleFileSizeInBytes, 1, pF ) == 0 )
            {
                //_ASSERT( 0 );
                success = false;
            }

            fclose( pF );
        }

        if ( success )
        {
            status = FPc.initializeBLE(pBleMap, bleFileSizeInBytes);

            if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
            {
                std::cout << "ble db initialisation error: ";
                std::cout << "file = " << fp_ble_base_file.c_str();
                std::cout << "status = " << static_cast<int>(status);
                std::cout << std::endl;
            }
            else
            {
                std::cout << "ble fingerprint " << fp_ble_base_file << " have been successfuly loaded" << std::endl;
                print_fpHeader("ble fingerprint db info:", FPc.getBfpInfo());
            }

            FPc.setUpdateBLE(bleEnable); /**< updater Ble control */
        }
        else
        {
            std::cout << "file " << fp_ble_base_file << "   can not be opened" << std::endl;
            FPc.setUpdateBLE( false );
        }
        //FPc.setBLEBias(0.01, true);
    }

    // load proximity
    std::string fp_ble_proximity_base_file;
    status = FPc.getBaseName(fp_ble_proximity_base_file, file_mask_for_ble_proximity_base);

    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "files with mask " << file_mask_for_ble_proximity_base << "   are not found" << std::endl;
        FPc.setUpdateBLEProximity(true);
    }
    else
    {
        struct stat file_buf;
        stat(fp_ble_proximity_base_file.c_str(), &file_buf);
        long bleFileSizeInBytes = file_buf.st_size;
        char *pBleMap = (char *)malloc(bleFileSizeInBytes + 1);

        FILE *pF = fopen(fp_ble_proximity_base_file.c_str(), "rb");
        bool success = (pF != 0);

        //    _ASSERT( pF );
        if (pF)
        {
            if (fread(pBleMap, bleFileSizeInBytes, 1, pF) == 0)
            {
                //_ASSERT( 0 );
                success = false;
            }
            pBleMap[bleFileSizeInBytes] = '\0';
            fclose(pF);
        }

        if (success)
        {
            //printf("%s\n", pBleMap);
            status = FPc.initializeBLEProximity(pBleMap, bleFileSizeInBytes);

            if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
            {
                std::cout << "ble proximity db initialisation error: ";
                std::cout << "file = " << fp_ble_proximity_base_file.c_str();
                std::cout << "status = " << static_cast<int>(status);
                std::cout << std::endl;
                FPc.setUpdateBLEProximity(false); /**< control of Proximity udater*/
            }
            else
            {
                std::cout << "ble proximity db " << fp_ble_proximity_base_file << " have been successfuly loaded" << std::endl;
                print_fpHeader("ble proximity db info:", FPc.getProximityDbInfo());
                FPc.setUpdateBLEProximity(bleProximityEnable); /**< control of Proximity udater*/
                FPc.setBlpDetectionEnable(bleProximityEnable);
                //FPc.setBlpPositioningPdFilterParams(2500, 0.04, 10000, 0.005);
                //FPc.setBlpDetectionPdFilterParams(100, 0.04, 10000, 0.005);
                //FPc.setBlpPositioningLogicParams(1, 1, -10);
                //FPc.setBlpDetectionLogicParams(1, 1, -10);
                
                
                //std::cout << "prox on 1: " << FPc.getProximityBeaconsNumber(1,bbt_proximity)
                //          << ", " << FPc.getProximityBeaconsNumber(1, bbt_restricted_assistance)
                //          << ", " << FPc.getProximityBeaconsNumber(1, bbt_unknown);
                //std::cout << "prox on 4: " << FPc.getProximityBeaconsNumber(4, bbt_proximity)
                //    << ", " << FPc.getProximityBeaconsNumber(4, bbt_restricted_assistance)
                //    << ", " << FPc.getProximityBeaconsNumber(4, bbt_unknown);

            }
        }
        else
        {
            std::cout << "file " << fp_ble_proximity_base_file << "   can not be opened" << std::endl;
            FPc.setUpdateBLEProximity(false);
        }
    }

    // load MFP
    std::string fp_magnetic_base_file;
    status = FPc.getBaseName(fp_magnetic_base_file, file_mask_for_mag_base);


    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "files with mask  " << file_mask_for_mag_base << "   are not found" << std::endl;
        FPc.setUpdateMFP(false); /**< updater MFP control */
    }
    else
    {
        struct stat file_buf;
        stat(fp_magnetic_base_file.c_str(), &file_buf);
        size_t mfpMapSizeInBytes = file_buf.st_size;
        char *pMfpMap = (char *)malloc(mfpMapSizeInBytes);

        FILE *pF = fopen(fp_magnetic_base_file.c_str(), "rb");
        bool success = (pF != 0);
        //    _ASSERT( pF );

        if (pF)
        {
            if (fread(pMfpMap, mfpMapSizeInBytes, 1, pF) == 0)
            {
                //_ASSERT( 0 );
                success = false;
            }

            fclose(pF);
        }

        if (success)
        {
            status = FPc.initializeMFP(pMfpMap, mfpMapSizeInBytes); // load mfp4 format
            if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
            {
                status = FPc.initializeMFP(pMfpMap, mfpMapSizeInBytes, venue.size_x, venue.size_y, magneticCellsize, 0, venue.floors_count - 1);
            }
            if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
            {
                std::cout << "mfp db initialisation error: ";
                std::cout << "file = " << fp_magnetic_base_file.c_str();
                std::cout << "   status = " << static_cast<int>(status);
                std::cout << std::endl;
                magEnable = false;
            }
            else
            {
                std::cout << "magnetic fingerprint " << fp_magnetic_base_file << " have been successfuly loaded" << std::endl;
                print_fpHeader("magnetic fingerprint db info:", FPc.getMfpInfo());
            }

            FPc.setUpdateMFP(magEnable); /**< updater MFP control */
        }
        else
        {
            std::cout << "file  " << fp_magnetic_base_file << "   can not be opened" << std::endl;
            FPc.setUpdateMFP(false); /**< updater MFP control */
        }
    }

    // load MM
    std::string map_file;
    status = FPc.getBaseName(map_file, file_mask_for_binary_map);

    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "files with mask  " << file_mask_for_binary_map << "   are not found" << std::endl;
        FPc.setUpdateMapMatching(false);
    }
    else
    {
        struct stat file_buf;
        stat(map_file.c_str(), &file_buf);
        size_t MapSizeInBytes = file_buf.st_size;

        uint8_t* pMap = (uint8_t*)malloc(MapSizeInBytes);

        FILE *pF = fopen(map_file.c_str(), "rb");
        bool success = (pF != 0);

        if (pF)
        {
            if (fread(pMap, MapSizeInBytes, 1, pF) == 0)
            {
                success = false;
                return -1;
            }

            fclose(pF);
        }

        if (success)
        {
            status = FPc.initializeMapMatching(pMap, MapSizeInBytes); // initializing map matching using binary map
            FPc.setUpdateMapMatching(mmEnable);
        }
    }


    // Input dataset
    status = FPc.setInputDataFolder( in_data_folder );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "Input data folder  " << "\"" << in_data_folder << "\"" << " can not be opened" << std::endl;
        data_load_status = false;
    }

    status = FPc.setInputIncMagDataFile( file_mask_for_incmag_indata );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "Input mag data file can not be opened" << std::endl;
        data_load_status = false;
    }

    status = FPc.setInputWiFiDataFile( file_mask_for_wifi_indata );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "Input WiFi data file can not be opened" << std::endl;
        wifi_text_format = false;
    }

    status = FPc.setInputBleDataFile( file_mask_for_ble_indata );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "Input BLE data file can not be opened" << std::endl;
        ble_text_format = false;
    }

    status = FPc.setInputFrameworkDataFile(file_mask_for_framework_indata);
    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "Input framework-pos data file can not be opened" << std::endl;
        FPc.setUpdateFrameworkPos(false);
    }
    else
    {
        FPc.setUpdateFrameworkPos(true);
    }

    status = FPc.setInputCollaborationDataFile(file_mask_collaboration_indata);
    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "Input ble-collaboration data file can not be opened" << std::endl;
        FPc.setUpdateCollaboration(false);
    }
    else
    {
        FPc.setUpdateCollaboration(collaborationEnable);
    }

    status = FPc.setOutputMagLogFile(out_log_folder + file_for_mag_outlog, out_log_folder + file_for_mag_outlog_dbg, out_log_folder + file_for_particles_outlog_dbg);

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "file  " << file_for_mag_outlog << "   can not be opened" << std::endl;
        data_load_status = false;
    }

    status = FPc.setOutputWiFiLogFile( out_log_folder + file_for_wifi_outlog, out_log_folder + file_for_wifi_outlog_dbg );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "file  " << file_for_wifi_outlog << "   can not be opened" << std::endl;
        data_load_status = false;
    }

    status = FPc.setOutputBLELogFile( out_log_folder + file_for_ble_outlog, out_log_folder + file_for_ble_outlog_dbg );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "file  " << file_for_ble_outlog << "   can not be opened" << std::endl;
        data_load_status = false;
    }

    status = FPc.setOutputBLEProximityLogFile(out_log_folder + file_for_ble_proximity_outlog, out_log_folder + file_for_ble_proximity_outlog_dbg);

    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "file  " << file_for_ble_proximity_outlog << "   can not be opened" << std::endl;
        data_load_status = false;
    }

    status = FPc.setOutputExtendedProximityLogFile(""/*out_log_folder + file_for_extended_proximity_out*/, out_log_folder + file_for_extended_proximity_out_dbg);

    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "file  " << file_for_ble_proximity_outlog << "   can not be opened" << std::endl;
        data_load_status = false;
    }


    status = FPc.setOutputMixedLogFile(out_log_folder + file_for_mixed_outlog, out_log_folder + file_for_mixed_outlog_dbg, out_log_folder + file_for_particles_outlog_dbg);

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        std::cout << "file  " << file_for_mixed_outlog << "   can not be opened" << std::endl;
        data_load_status = false;
    }

    status = FPc.setOutputVenueDetectionLogFile(out_log_folder + file_for_venue_detection_outlog_dbg);

    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "file  " << file_for_venue_detection_outlog_dbg << "   can not be opened" << std::endl;
        data_load_status = false;
    }

    if (data_load_status)
    {
    MagneticVector mag_vector;

    Fppe::WiFiScanResult wifi_scan_result;
    Fppe::BleScanResult ble_scan_result;
    int wifi_scan_counter = 0;
    int ble_scan_counter = 0;

    Fppe::Position framework_position = { 0 };
    std::queue<Fppe::Position> framework_pos_queue;

    std::vector <Fppe::CollaborationData> collaboration_position;
    std::queue<std::vector <Fppe::CollaborationData>> collaboration_queue;

    if (startPosition.is_valid)
        FPc.setStartPosition(startPosition);

    TpnOutput tpnOut = TpnOutput();
    int64_t clock_mfp = 0;


#if 0
	// WARNING: enabling this condition for testing only!
    startPosition.lattitude = 51.06508992 ;
    startPosition.longitude = -114.14485648;
    startPosition.covariance_lat_lon[0][0] = 25;
    startPosition.covariance_lat_lon[0][1] = 0.00000000000000000;
    startPosition.covariance_lat_lon[1][0] = 0.00000000000000000;
    startPosition.covariance_lat_lon[1][1] = 25;
    startPosition.altitude = 0;
    startPosition.azimuth = 13.4118;
    startPosition.azimuth_std = 20;
    startPosition.floor_number = 4;
    startPosition.floor_std = 0.1;
    startPosition.timestamp = 157807;//8959;
    startPosition.is_valid = true;
#else
    //startPosition.is_valid = false;
#endif

    // RTFPPL settings
    FPc.setPlatformType(rtfppl_settings.platform_type);
    std::cout << "PlatformType: " << FPc.getPlatformType() << std::endl;

    // special settings
    //FPc.setRandomSeeds();
    FPc.setMagneticFilterEnable(false);
    //.FPc.setBlpPulling(pt_nonePulling, 1.5); // uncomment to disable pulling
    //FPc.setBLEBias(10, 0);
    //FPc.setUseBarometer(false);


    while (FPc.getTpnOutData(tpnOut, wifi_scan_result, ble_scan_result) == Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        if (wifi_text_format == true)
        {
            FPc.getInputWiFiData(wifi_scan_result);
        }
        if (wifi_scan_result.scanWiFi.size() > 0)
        {
            wifi_scan_result.timestamp -= 50;   // patch to sync with Corsa Venue
            FPc.processInputWiFiData(wifi_scan_result);
        }

        if (ble_text_format == true)
        {
            FPc.getInputBleData(ble_scan_result);
        }
        if (ble_scan_result.scanBle.size() > 0)
        {
            ble_scan_result.timestamp -= 50;   // patch to sync with Corsa Venue
            FPc.processInputBleData(ble_scan_result);
        }

        // framework data reading
        // it is realized uneficiently
        // TODO: update this code with reading function when frmework is becoming more actual
        if (FPc.getInputFrameworkData(framework_position) == Fppe::ReturnStatus::STATUS_SUCCESS)
            //if (framework_position.covariance_lat_lon[0][0] < 100)   // WARNING: uncertainty checking is now realized in console
                framework_pos_queue.push(framework_position);
        if (framework_pos_queue.size() > 0)
            if (framework_pos_queue.front().timestamp <= (tpnOut.timestamp * 1000))
            {
                FPc.processInputFrameworkPosition(framework_pos_queue.front());
                framework_pos_queue.pop();
            }

        // collaboration data reading
        // it is realized uneficiently
        // TODO: update this code with reading function when collaboration is becoming more actual
        if (FPc.getInputCollaborationData(collaboration_position) == Fppe::ReturnStatus::STATUS_SUCCESS)
            collaboration_queue.push(collaboration_position);
        if (collaboration_queue.size() > 0)
            if ((collaboration_queue.front())[0].timestamp <= (tpnOut.timestamp * 1000))
            {
                FPc.processInputCollaboration((collaboration_queue.front()));
                collaboration_queue.pop();
            }
        
        if (startPosition.is_valid)
        {
#if 0 // 1- enable start position from tpn
            startPosition.lattitude = tpnOut.position.lattitude;
            startPosition.longitude = tpnOut.position.longitude;
            startPosition.azimuth = tpnOut.position.user_heading;
            startPosition.timestamp = tpnOut.timestamp*1e3;
#endif

            if (startPosition.timestamp <= 0)
            {
                if (tpnOut.position.navigation_phase >= 1)
                {
                    startPosition.timestamp = tpnOut.timestamp*1e3;
                    FPc.setStartPosition(startPosition);
                    startPosition.is_valid = false;
                }
            }
            else if (startPosition.timestamp <= tpnOut.timestamp*1e3)
            {
                FPc.setStartPosition(startPosition);
                startPosition.is_valid = false;
            }
            else 
            {
                continue;
            }

        }
        
        bool is_tpn_processed = (tpnOut.position.is_valid) && (tpnOut.attitude.is_valid) && (tpnOut.mag_meas.is_valid) && (tpnOut.position.navigation_phase >= 1);
#if 1
        //debug output
        std::cout << tpnOut.timestamp;
        std::cout << " tpn:" << (is_tpn_processed ? 1 : 0)
            << "("
            << (tpnOut.position.is_valid ? 1 : 0)
            << (tpnOut.attitude.is_valid ? 1 : 0)
            << (tpnOut.mag_meas.is_valid ? 1 : 0)
            << (tpnOut.pdr.is_valid ? 1 : 0)
            << int(tpnOut.position.navigation_phase)
            << ")";
        if (wifi_scan_result.scanWiFi.size() > 0)       std::cout << " wifi:" << wifi_scan_result.scanWiFi.size();
        if (ble_scan_result.scanBle.size() > 0)        std::cout << " ble:" << ble_scan_result.scanBle.size();
        std::cout << std::endl;
#endif
        if (is_tpn_processed)
        {
            static long tpn_counter = 0;
            tpn_counter++;
            if (tpn_counter > 1)  // patch to sync with Corsa Venue
            {
                //clock_mfp -= __rdtsc();
                FPc.processTpnOutput(tpnOut);
                //clock_mfp += __rdtsc();
            }
        }
    }
    
    Fppe::MagneticCalibrationParam bias_cov;
    bool magbiasresult = FPc.getMagneticBias(&bias_cov);

    if (mag_bias_file == "")
    {
#ifdef _WIN32
        if (fp_bases_folder.back() != '\\')
        {
            fp_bases_folder = fp_bases_folder + "\\";
        }
#else
        if ( fp_bases_folder.back() != '/' )
        {
            fp_bases_folder = fp_bases_folder + "/";
        }
#endif

        mag_bias_file = fp_bases_folder + "default.mbias";
    }

    //FPc.writeMagneticBiasToFile(bias_cov, mag_bias_file);

    FPc.closeOutputLogFiles();

    std::cout << "\n MFP clock:" << clock_mfp << std::endl;
    }

#ifdef WIN32
    Sleep(2000); // debbuging sleep delay
#endif

    return (data_load_status == true) ? 0 : 1;
}
