//
//  wrapperiOS.m
//  temp
//
//  Created by Vladimir Pentyukhov on 25/07/2017.
//  Copyright © 2017 Vladimir Pentyukhov. All rights reserved.
//

#import <Foundation/Foundation.h>

#include <iostream>
#include <string>
//#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <vector>
#include <queue>
#include <regex>
#include <string>
//#include <io.h>

#include "FP_console.hpp"
#include "CmdReader.hpp"
#include "SettingsParser.hpp"
#include <ctime>
#include <math.h>

#import "WrapperiOS.h"

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
    std::cout << std::dec << "fp_build_time: " << fpHeader.fp_build_time.tm_sec;
    std::cout << ":" << fpHeader.fp_build_time.tm_min;     /* minutes after the hour - [0,59] */
    std::cout << ":" << fpHeader.fp_build_time.tm_hour;    /* hours since midnight - [0,23] */
    std::cout << " " << fpHeader.fp_build_time.tm_mday;    /* day of the month - [1,31] */
    std::cout << "." << fpHeader.fp_build_time.tm_mon;     /* months since January - [0,11] */
    std::cout << "." << fpHeader.fp_build_time.tm_year;    /* years since 1900 */
    std::cout << " " << fpHeader.fp_build_time.tm_wday;    /* days since Sunday - [0,6] */
    std::cout << " " << fpHeader.fp_build_time.tm_yday;    /* days since January 1 - [0,365] */
    std::cout << " " << fpHeader.fp_build_time.tm_isdst;   /* daylight savings time flag */
    std::cout << std::endl;
}

@interface WrapperiOS ()
{
int  mapViewWidth;
int  mapViewHeight;

//CGPoint location;
//CGPoint previousLocation;

double mapWidth;                      // meters
double mapHeight;

double left;                       // meters
double down;                       // meters

double mmcellsize;
double mgcellsize;
double mapcellsize;

float magBias1;
float magBias2;
float magBias3;
long long magBiasTime;

int currentfloor;
double currentX;
double currentY;

int minfloor;
int maxfloor;

double  scaleFactor ;

double scaleStepperValueOld;

NSString *mapBase;
NSString *vizBase;

NSString *trkFile;
NSString *accuracyFile;


NSString *jsonName;
NSMutableArray *filesList;

NSMutableData		  *receivedData;

BOOL enableMap;
NSFileHandle *fileTrk ;
NSFileHandle *fileAccuracy ;
NSString *documentDir;

//UIBackgroundTaskIdentifier _backgroundRecordingID;
//NSTimer *backgroundTimer;

BOOL  navEngineIsWorked;

BOOL   fileIsClosed;
}
@end

int mainiOS( const int argc, const char** argv );

@implementation WrapperiOS

- (id)init
{
    self = [super init];
    if (self)
    {
        // Do any additional setup after loading the view, typically from a nib.
        NSArray  *docList = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        documentDir  = [docList objectAtIndex:0];
        documentDir = [documentDir stringByAppendingString:@"/"];
        NSLog(@"documentDir %@", documentDir);
        const char *documentDirFileName =  [documentDir cStringUsingEncoding:NSUTF8StringEncoding];
        DIR *dir;
        struct dirent *entry;
        
        dir = opendir(documentDirFileName);
        if (!dir) {
            perror("diropen");
            exit(1);
        };
        
        while ( (entry = readdir(dir)) != NULL)
        {
            printf("%lld - %s [%d] %d\n", entry->d_ino, entry->d_name, entry->d_type, entry->d_reclen);

            NSString * fileName = [NSString stringWithCString:entry->d_name encoding:NSUTF8StringEncoding];
            
            //NSRange range = [fileName rangeOfString:@".json" ] ;
            //if(range.length > 0)
            if([fileName hasSuffix: @".json" ] )
            {
                fileName = [fileName stringByReplacingOccurrencesOfString:@".json" withString:@""];
                [filesList addObject: fileName ];
            }
        }
        
        closedir(dir);

        //NSError *e = nil;
        char *command_line[2];
        command_line[0] = "--settings";
        command_line[1] = "venue.json";
        {
            const std::string file_mask_for_mag_base = "(.*)(.mfp4)";
            //const std::string file_mask_for_mag_base = "(.*)(.mfp3)";
            const std::string file_mask_for_wifi_base = "(.*)(.wifi4)";
            const std::string file_mask_for_ble_base = "(.*)(.ble4)";
            const std::string file_mask_for_ble_proximity_base = "(.*)(.blp4)";

            const std::string file_mask_for_mag_bias = "(.*)(.mbias)";
            
            //const std::string file_mask_for_posmag_indata = "(RAMP_)(.*)";
            //const std::string file_mask_for_incmag_indata = "(tpp_output\.dat)";
            const std::string file_mask_for_incmag_indata = "(.*)(TppOutput\.dat)";
            //const std::string file_mask_for_incmag_indata = "(nav\.dat)";
            //const std::string file_mask_for_incmag_indata = "(rtfppl_data.txt)";
            
            const std::string file_mask_for_wifi_indata = "(wifi_in)(.*)";
            const std::string file_mask_for_ble_indata = "(ble_in)(.*)";
            const std::string file_mask_for_framework_indata = "(.*)(framework)(.*)";
            //const std::string file_mask_for_framework_indata = "no_framework.dat";
            
            const std::string file_for_mag_outlog = "mag_out.kml";
            const std::string file_for_wifi_outlog = "wifi_out.kml";
            const std::string file_for_ble_outlog = "ble_out.kml";
            const std::string file_for_ble_proximity_outlog = "ble_proximity_out.kml";
            const std::string file_for_mixed_outlog = "mixed_out.kml";
            
            std::string file_for_mag_outlog_dbg;
            std::string file_for_wifi_outlog_dbg;
            std::string file_for_ble_outlog_dbg;
            std::string file_for_ble_proximity_outlog_dbg;
            std::string file_for_mixed_outlog_dbg;
            
            file_for_mag_outlog_dbg = "mag_out_dbg.log";
            file_for_wifi_outlog_dbg = "wifi_out_dbg.log";
            file_for_ble_outlog_dbg = "ble_out_dbg.log";
            file_for_ble_proximity_outlog_dbg = "ble_proximity_out_dbg.log";
            file_for_mixed_outlog_dbg = "mixed_out_dbg.log";
            BaseVenue venue = {};
            
            std::string arg;
            std::stringstream ss;
            std::string name;
            double grid_size;
            bool   magEnable;
            bool   wifiEnable;
            bool   bleEnable;
            bool   bleProximityEnable;
            
            bool wifi_text_format = true;
            bool ble_text_format = true;

            double magneticCellsize = 1.0;
            Fppe::Position startPosition;
            double magneticFPsizes[3];
            
            bool success = true;
            
            std::string mag_bias_file = "";
            std::string fp_bases_folder;
            std::string in_data_folder;
            std::string out_log_folder;
            
            std::string settings_file = std::string("--settings"); // must be in JSON format
            
            // command line parsing
            success &= setOptionFromCmd( 2, (const char **)command_line, settings_file, &settings_file );
            
            NSString *jsonFile  = [NSString stringWithCString:settings_file.c_str() encoding:NSUTF8StringEncoding];
            
            jsonFile  = [documentDir stringByAppendingString:jsonFile];
            
            const char *p_jsonFile = [jsonFile cStringUsingEncoding: NSUTF8StringEncoding];
            startPosition.is_valid = false;
            parseSettings( std::string (p_jsonFile), // input JSON file	with settings
                          name,					 // output
                          fp_bases_folder,
                          in_data_folder,
                          out_log_folder,
                          venue,
                          magEnable,
                          wifiEnable,
                          bleEnable,
                          bleProximityEnable,
                          magneticCellsize,
                          magneticFPsizes,
                          startPosition
                          );
            
            fp_bases_folder  =  std::string(documentDirFileName) + fp_bases_folder;
            in_data_folder  =  std::string(documentDirFileName) + in_data_folder;
            out_log_folder  =  std::string(documentDirFileName) + out_log_folder;
#ifdef _WIN32
            if ( out_log_folder.back() != '\\' )
            {
                out_log_folder = out_log_folder + "\\";
            }
#else
            if ( out_log_folder.back() != '/' )
            {
                out_log_folder = out_log_folder + "/";
            }
#endif
            FPPositionConsole::FP_console FPc( name, out_log_folder );
            Fppe::ReturnStatus status = FPc.setVenueParams( venue );
            
            FPc.setFPBasesFolder( fp_bases_folder );
            
            std::string fp_magnetic_base_file;
            status = FPc.getBaseName( fp_magnetic_base_file, file_mask_for_mag_base );
            
            if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                std::cout << "files with mask  " << file_mask_for_mag_base << "   are not found" << std::endl;
                FPc.setUpdateMFP( false ); /**< updater MFP control */
            }
            else
            {
                struct stat file_buf;
                stat( fp_magnetic_base_file.c_str(), &file_buf );
                size_t mfpMapSizeInBytes = file_buf.st_size;
                char *pMfpMap = ( char * )malloc( mfpMapSizeInBytes );
                
                FILE *pF = fopen( fp_magnetic_base_file.c_str(), "rb" );
                bool success = ( pF != 0 );
                //    _ASSERT( pF );
                
                if ( pF )
                {
                    if ( fread( pMfpMap, mfpMapSizeInBytes, 1, pF ) == 0 )
                    {
                        //_ASSERT( 0 );
                        success = false;
                    }
                    
                    fclose( pF );
                }
                
                if ( success )
                {
                    status = FPc.initializeMFP(pMfpMap, mfpMapSizeInBytes); // load mfp4 format
                    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
                    {
                        status = FPc.initializeMFP(pMfpMap, mfpMapSizeInBytes, venue.size_x, venue.size_y, magneticCellsize, 0, venue.floors_count - 1);
                        if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
                        {
                            std::cout << "mfp db initialisation error: ";
                            std::cout << "file = " << fp_magnetic_base_file.c_str();
                            std::cout << "status = " << static_cast<int>(status);
                            std::cout << std::endl;
                        }
                        else
                        {
                            std::cout << "magnetic fingerprint " << fp_magnetic_base_file << " have been successfuly loaded" << std::endl;
                            print_fpHeader("magnetic fingerprint db info:", FPc.getMfpInfo());
                        }
                    }
                    
                    FPc.setUpdateMFP( magEnable ); /**< updater MFP control */
                }
                else
                {
                    std::cout << "file  " << fp_magnetic_base_file << "   can not be opened" << std::endl;
                    FPc.setUpdateMFP( false ); /**< updater MFP control */
                }
            }
            
            status = FPc.getBaseName( mag_bias_file, file_mask_for_mag_bias );
            
            if ( status == Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                Fppe::MagneticCalibrationParam bias_cov;
                FPc.readMagneticBiasFromFile( &bias_cov, mag_bias_file );
                
                //if ( status == Fppe::ReturnStatus::STATUS_SUCCESS )
                //    FPc.setMagneticBias( bias_cov );
            }
#if 0            
            Fppe::MagneticCalibrationParam bias_cov1 = {};
            bias_cov1.timestamp = 0;
            bias_cov1.mX = 0;
            bias_cov1.mY = 0;
            bias_cov1.mZ = 0;
            memset(bias_cov1.covarianceMatrix, 0, sizeof(bias_cov1.covarianceMatrix));
            bias_cov1.covarianceMatrix[0][0] = 5;// 250;
            bias_cov1.covarianceMatrix[1][1] = 5;// 250;
            bias_cov1.covarianceMatrix[2][2] = 5;// 250;            
            //FPc.setMagneticBias( bias_cov1 );
#endif            
            
            double min_p = 0.001; //validity threshold
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
                    status = FPc.initializeWiFi( pWiFiMap, wifiFileSizeInBytes, min_p );
                    if(status == Fppe::ReturnStatus::STATUS_SUCCESS)
                        FPc.setUpdateWiFi( wifiEnable ); /**< updater WiFi control */
                    else
                        status = FPc.initializeWiFi( pWiFiMap, wifiFileSizeInBytes );
                    if(status == Fppe::ReturnStatus::STATUS_SUCCESS)
                        FPc.setUpdateWiFi( wifiEnable ); /**< updater WiFi control */
                    else
                         FPc.setUpdateWiFi( false ); /**< updater WiFi control */
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
            }
            
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
                    
                    status = FPc.initializeBLE( pBleMap, bleFileSizeInBytes, min_p );
                    if(status == Fppe::ReturnStatus::STATUS_SUCCESS)
                        FPc.setUpdateBLE( bleEnable ); /**< updater WiFi control */
                    else
                        status = FPc.initializeBLE( pBleMap, bleFileSizeInBytes );
                    if(status == Fppe::ReturnStatus::STATUS_SUCCESS)
                        FPc.setUpdateBLE( bleEnable ); /**< updater WiFi control */
                    else
                        FPc.setUpdateBLE( false ); /**< updater WiFi control */
                    
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
            }

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
                    }
                    else
                    {
                        std::cout << "ble proximity db " << fp_ble_proximity_base_file << " have been successfuly loaded" << std::endl;
                        print_fpHeader("ble proximity db info:", FPc.getProximityDbInfo());
                    }
                    
                    FPc.setUpdateBLEProximity(bleProximityEnable); /**< control of Proximity udater*/
                }
                else
                {
                    std::cout << "file " << fp_ble_proximity_base_file << "   can not be opened" << std::endl;
                    FPc.setUpdateBLEProximity(false);
                }
            }
            
            status = FPc.setInputDataFolder( in_data_folder );
            
            if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                std::cout << "Input data folder  " << "\"" << in_data_folder << "\"" << " can not be opened" << std::endl;
                return nullptr;
            }
            
            status = FPc.setInputIncMagDataFile( file_mask_for_incmag_indata );
            
            if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                std::cout << "Input mag data file can not be opened" << std::endl;
                return nullptr;
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
            
#ifdef _WIN32
            
            if ( out_log_folder.back() != '\\' )
            {
                out_log_folder = out_log_folder + "\\";
            }
            
#else
            
            if ( out_log_folder.back() != '/' )
            {
                out_log_folder = out_log_folder + "/";
            }
            
#endif
            
            status = FPc.setOutputMagLogFile( out_log_folder + file_for_mag_outlog, out_log_folder + file_for_mag_outlog_dbg );
            
            if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                std::cout << "file  " << file_for_mag_outlog << "   can not be opened" << std::endl;
                return nullptr;
            }
            
            status = FPc.setOutputWiFiLogFile( out_log_folder + file_for_wifi_outlog, out_log_folder + file_for_wifi_outlog_dbg );
            
            if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                std::cout << "file  " << file_for_wifi_outlog << "   can not be opened" << std::endl;
                return nullptr;
            }
            
            status = FPc.setOutputBLELogFile( out_log_folder + file_for_ble_outlog, out_log_folder + file_for_ble_outlog_dbg );
            
            if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                std::cout << "file  " << file_for_ble_outlog << "   can not be opened" << std::endl;
                return nullptr;
            }
            
            status = FPc.setOutputBLEProximityLogFile(out_log_folder + file_for_ble_proximity_outlog, out_log_folder + file_for_ble_proximity_outlog_dbg);
            
            if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
            {
                std::cout << "file  " << file_for_ble_proximity_outlog << "   can not be opened" << std::endl;
                return nullptr;
            }

            status = FPc.setOutputMixedLogFile( out_log_folder + file_for_mixed_outlog, out_log_folder + file_for_mixed_outlog_dbg );
            
            if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                std::cout << "file  " << file_for_mixed_outlog << "   can not be opened" << std::endl;
                return nullptr;
            }
            
            MagneticVector mag_vector;
            
            Fppe::WiFiScanResult wifi_scan_result;
            Fppe::BleScanResult ble_scan_result;
            Fppe::Position framework_position = {0};
            std::queue<Fppe::Position> framework_pos_queue;
            
            if ( startPosition.is_valid )
                FPc.setStartPosition( startPosition );
            
            TpnOutput tpnOut = TpnOutput();
            //int64_t clock_mfp = 0;
#if 0
            startPosition.lattitude = 35.65706730   ;
            startPosition.longitude = 140.02629960;
            startPosition.covariance_lat_lon[0][0] = 144.000000000000000;
            startPosition.covariance_lat_lon[0][1] = 0.00000000000000000;
            startPosition.covariance_lat_lon[1][0] = 0.00000000000000000;
            startPosition.covariance_lat_lon[1][1] = 144.000000000000000;
            startPosition.altitude = 1125.654541015625;
            startPosition.azimuth = 0;
            startPosition.azimuth_std = 30.000000000000000;
            startPosition.floor_number = 2;
            startPosition.floor_std = 1;
            startPosition.timestamp = 13500;//8959;
            startPosition.is_valid = true;
#else
            startPosition.is_valid = false;
#endif
            
            //FPc.setMagneticFilterEnable(true);
            FPc.setMagneticFilterEnable(false);
            
#if 0
            // redirect std::cout to file
            std::streambuf *bak;
            std::ofstream file;
            //file.open(out_log_folder + "console.txt");
            file.open("dev/null");
            bak = std::cout.rdbuf();  // save
            std::cout.rdbuf(file.rdbuf()); // redirect
            //std::cout << "1234567890";
            //std::cout.flush();
            //std::cout.rdbuf(bak); // restore
#endif
            
            double processing_time = 0;
            
            while ( FPc.getTpnOutData( tpnOut, wifi_scan_result, ble_scan_result ) == Fppe::ReturnStatus::STATUS_SUCCESS )
            {
                if ( wifi_text_format == true )
                {
                    FPc.getInputWiFiData( wifi_scan_result );
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

                if (FPc.getInputFrameworkData(framework_position) == Fppe::ReturnStatus::STATUS_SUCCESS)
                    if (framework_position.covariance_lat_lon[0][0] < 100)   // WARNING: uncertainty checking is now realized in console
                        framework_pos_queue.push(framework_position);
                
                if (framework_pos_queue.size() > 0)
                    if (framework_pos_queue.front().timestamp <= (tpnOut.timestamp*1000))
                    {
                        FPc.processInputFrameworkPosition(framework_pos_queue.front());
                        framework_pos_queue.pop();
                    }


                if (startPosition.is_valid)
                {
#if 0
                    startPosition.lattitude = tpnOut.position.lattitude;
                    startPosition.longitude = tpnOut.position.longitude;
                    startPosition.timestamp = tpnOut.timestamp*1e3;
#endif
                    if (startPosition.timestamp <= tpnOut.timestamp*1e3)
                    {
                        FPc.setStartPosition(startPosition);
                        startPosition.is_valid = false;
                    }
                    
                }
                
                bool is_tpn_processed = (tpnOut.position.is_valid) && (tpnOut.attitude.is_valid) && (tpnOut.mag_meas.is_valid) && (tpnOut.position.navigation_phase >= 1);
#if 1
                //debug output
                std::cout << "tpn time: " << tpnOut.timestamp;
                std::cout << " tpn: " << (is_tpn_processed ? 1 : 0)
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
#if 1
                if (is_tpn_processed)
                {
                    static long tpn_counter = 0;
                    tpn_counter++;
                    if (tpn_counter > 1)  // patch to sync with Corsa Venue
                    {
                        //clock_mfp -= rdtsc();
                        //FPc.processTpnOutput(tpnOut);
                        //clock_mfp += rdtsc();
#define BILLION  1000000000L;
                        
                        // Calculate time taken by a request
                        struct timespec requestStart, requestEnd;
                        clock_gettime(CLOCK_REALTIME, &requestStart);
                        FPc.processTpnOutput(tpnOut);
                        clock_gettime(CLOCK_REALTIME, &requestEnd);
                        
                        // Calculate time it took
                        processing_time += ( requestEnd.tv_sec - requestStart.tv_sec );
                        processing_time += ((double)( requestEnd.tv_nsec - requestStart.tv_nsec ))  / BILLION ;
                        //printf( "processing_time = %lf\n", processing_time );
                    }
                }
#else
                //clock_mfp -= __rdtsc();
                FPc.processTpnOutput(tpnOut);
                //clock_mfp += __rdtsc();
#endif
            }
            
            printf( "\nprocessing_time = %lf\n\n", processing_time );
            
            Fppe::MagneticCalibrationParam bias_cov;
            FPc.getMagneticBias( &bias_cov );
            
            if ( mag_bias_file == "" )
            {
#ifdef _WIN32
                
                if ( fp_bases_folder.back() != '\\' )
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
            
            FPc.writeMagneticBiasToFile( bias_cov, mag_bias_file );
            
            
            FPc.closeOutputLogFiles();
            
            //std::cout << "\n MFP clock:" << clock_mfp;
            
            return 0;
        }
    }
    return self;
}
@end
