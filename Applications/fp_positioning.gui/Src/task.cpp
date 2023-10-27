#include "task.h"
#include <vector>
#include <regex>
#include "CmdReader.hpp"
#include "SettingsParser.hpp"
#include "secondwindow.h"

Task::Task(std::string &JSON, OutPut_for_image *Out, Track *track, QWaitCondition *pause)
{
    this->mainJSON = JSON;
   // this->venue_x = venue_x;
   // this->venue_y = venue_y;
    this->floor_count = floor_count;
    this->fp_path = fp_path;
    this->track_path = track_path;
    this->out_path = out_path;
    this->Output_Struct = Out;
    mTrack = track;
    drawed = true;

    mPause = pause;

    mStop = false;
}

Task::~Task()
{
}

void Task::setPause(bool pause)
{
   isPaused = pause;
}

void  Task::setTrackForMode(int mode)
{
    mTrackForMode = mode;
}

void Task::stop()
{
    mStop = true;
}


void Task::doWork()
{

    //SecondWindow w2;
    //w2.show();


    const std::string file_mask_for_mag_base = "(.*)(.mfp3)";
    const std::string file_mask_for_wifi_base = "(.*)(.wifi3)";
    const std::string file_mask_for_ble_base = "(.*)(.ble3)";
    const std::string file_mask_for_ble_proximity_base = "(.*)(.ble5)";

    const std::string file_mask_for_mag_bias = "(.*)(.mbias)";

//    const std::string file_mask_for_incmag_indata = "(tpp_output\.dat)";
  //  const std::string file_mask_for_incmag_indata = "(.*)(.dat)";
    const std::string file_mask_for_incmag_indata = "(.*)(TppOutput\.dat)";

    const std::string file_mask_for_wifi_indata = "(wifi_debug)(.*)";
    const std::string file_mask_for_ble_indata = "(ble_)(.*)";

    const std::string file_for_mag_outlog = "mag_out.kml";
    const std::string file_for_wifi_outlog = "wifi_out.kml";
    const std::string file_for_ble_outlog = "ble_out.kml";
    const std::string file_for_ble_proximity_outlog = "ble_out_proximity.kml";
    const std::string file_for_mixed_outlog = "mixed_out.kml";

    std::string file_for_mag_outlog_dbg = "mag_out_dbg.log";
    std::string file_for_wifi_outlog_dbg = "wifi_out_dbg.log";
    std::string file_for_ble_outlog_dbg = "ble_out_dbg.log";
    std::string file_for_ble_proximity_outlog_dbg = "ble_out_proximity_dbg.log";
    std::string file_for_mixed_outlog_dbg = "mixed_out_dbg.log";

    VenueEx venue = {};

    double magneticCellsize = 1.0;
    double magneticFPsizes[3];
    Fppe::Position startPosition = {0};

    std::string name;
    bool   magEnable;
    bool   wifiEnable;
    bool   bleEnable;
    bool   bleProximityEnable;

    bool wifi_text_format = true;
    bool ble_text_format = true;
    bool program_exit = false;

    std::string mag_bias_file = "";
    std::string fp_bases_folder;
    std::string in_data_folder;
    std::string out_log_folder;

    std::string settings_file = this->mainJSON; // must be in JSON format

    startPosition.is_valid = false;
    parseSettings(settings_file, // input JSON file	with settings
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

    FPPositionConsole::FP_console FPc( name, this->Output_Struct, this->mTrack );
    Fppe::ReturnStatus status = FPc.setVenueParams( venue );


    // Folder for fingerprint data file

    status = FPc.setFPBasesFolder( fp_bases_folder );
    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Folder for fingerprint data files"),
                                QString::fromUtf8(fp_bases_folder.c_str()),
                                QString("cannot be found"),
                                false);
    }

    std::string fp_magnetic_base_file;
    FPc.getBaseName( fp_magnetic_base_file, file_mask_for_mag_base );

    if(magneticFPsizes[0] > 0 && magneticFPsizes[1] > 0)
        status = FPc.initializeMFP(fp_magnetic_base_file, magneticFPsizes[0], magneticFPsizes[1], magneticCellsize, 0, magneticFPsizes[2] - 1);
    else
        status = FPc.initializeMFP(fp_magnetic_base_file, venue.size_x, venue.size_y, magneticCellsize, 0, venue.floors_count - 1);

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Magnetic database file: "),
                                QString::fromUtf8(fp_magnetic_base_file.c_str()),
                                QString("cannot be found or opened\n or have incorrect size"),
                                false);
        FPc.setUpdateMFP( false ); /**< updater MFP control */
    }
    else
        FPc.setUpdateMFP( magEnable ); /**< updater MFP control */

    status = FPc.getBaseName( mag_bias_file, file_mask_for_mag_bias );

    if ( status == Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        Fppe::MagneticCalibrationParam bias_cov;
        FPc.readMagneticBiasFromFile( &bias_cov, mag_bias_file );

        //if ( status == Fppe::ReturnStatus::STATUS_SUCCESS )
        //    FPc.setMagneticBias( bias_cov );
    }
    //FPc.setUpdateMFP( magEnable ); /**< updater MFP control */

    std::string fp_wifi_base_file;
    FPc.getBaseName( fp_wifi_base_file, file_mask_for_wifi_base );
    double min_p = 0.001; //validity threshold
    status = FPc.getBaseName( fp_wifi_base_file, file_mask_for_wifi_base );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("WiFi database file with mask: "),
                                QString::fromUtf8(file_mask_for_wifi_base.c_str()),
                                QString("cannot be found or opened"),
                                false);
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
            FPc.setUpdateWiFi( wifiEnable ); /**< updater WiFi control */
        }
        else
        {
            emit this->file_error_message(QString("WiFi database file: "),
                                    QString::fromUtf8(fp_wifi_base_file.c_str()),
                                    QString("cannot be found or opened"),
                                    false);
            FPc.setUpdateWiFi( false );
        }
    }

    std::string fp_ble_base_file;
    FPc.getBaseName( fp_ble_base_file, file_mask_for_ble_base );
    status = FPc.initializeBLE( fp_ble_base_file, min_p );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("BLE database file: "),
                                QString::fromUtf8(fp_ble_base_file.c_str()),
                                QString("cannot be found or opened"),
                                false);
        FPc.setUpdateBLE( false ); /**< updater Ble control */
    }
    else
        FPc.setUpdateBLE( bleEnable ); /**< updater Ble control */


    std::string fp_ble_proximity_base_file;
    status = FPc.getBaseName(fp_ble_proximity_base_file, file_mask_for_ble_proximity_base);

    if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
    {
        std::cout << "files with mask  " << file_mask_for_ble_proximity_base << "   are not found" << std::endl;
        FPc.setUpdateBLEProximity(false);
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
            status = FPc.initializeBLEProximity(pBleMap, bleFileSizeInBytes);

            if (status != Fppe::ReturnStatus::STATUS_SUCCESS)
            {
                std::cout << "ble proximity db initialisation error: ";
                std::cout << "file = " << fp_ble_proximity_base_file.c_str();
                std::cout << "status = " << static_cast<int>(status);
                std::cout << std::endl;
            }

            FPc.setUpdateBLEProximity(bleProximityEnable); /**< updater Ble control */
        }
        else
        {
            std::cout << "file " << fp_ble_proximity_base_file << "   can not be opened" << std::endl;
            FPc.setUpdateBLEProximity(false);
        }
    }



    // Folder for input data file

    status = FPc.setInputDataFolder( in_data_folder );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Input folder: "),
                                QString::fromUtf8(in_data_folder.c_str()),
                                QString("cannot be found"),
                                true);
        program_exit = true;
    }

    status = FPc.setInputIncMagDataFile( file_mask_for_incmag_indata );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Input data file with mask: "),
                                QString::fromUtf8(file_mask_for_incmag_indata.c_str()),
                                QString("cannot be found or opened"),
                                true);
        program_exit = true;
    }

    status = FPc.setInputWiFiDataFile( file_mask_for_wifi_indata );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("WiFi data file with mask:"),
                                QString::fromUtf8(file_mask_for_wifi_indata.c_str()),
                                QString("cannot be found or opened"),
                                false);
        wifi_text_format = false;
    }

    status = FPc.setInputBleDataFile( file_mask_for_ble_indata );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("BLE data file with mask:"),
                                QString::fromUtf8(file_mask_for_ble_indata.c_str()),
                                QString("cannot be found or opened"),
                                false);
        ble_text_format = false;
    }

    // Output folder and logs
    
    if ( !QDir(out_log_folder.c_str()).exists() )
        QDir().mkdir(out_log_folder.c_str());;

    if ( out_log_folder.back() != '\\' )
    {
        out_log_folder = out_log_folder + "\\";
    }
#if 1
    status = FPc.setOutputMagLogFile( out_log_folder + file_for_mag_outlog, this->Output_Struct, out_log_folder + file_for_mag_outlog_dbg );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Output magnetic log file:"),
                                QString::fromUtf8(file_for_mag_outlog.c_str()),
                                QString("cannot be opened"),
                                false);
    }

    status = FPc.setOutputWiFiLogFile( out_log_folder + file_for_wifi_outlog, this->Output_Struct, out_log_folder + file_for_wifi_outlog_dbg );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Output WiFi log file:"),
                                QString::fromUtf8(file_for_wifi_outlog.c_str()),
                                QString("cannot be opened"),
                                false);
    }

    status = FPc.setOutputBLELogFile( out_log_folder + file_for_ble_outlog, this->Output_Struct, out_log_folder + file_for_ble_outlog_dbg );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Output BLE log file:"),
                                QString::fromUtf8(file_for_ble_outlog.c_str()),
                                QString("cannot be opened"),
                                false);
    }

    status = FPc.setOutputBLEProximityLogFile( out_log_folder + file_for_ble_proximity_outlog, this->Output_Struct, out_log_folder + file_for_ble_proximity_outlog_dbg );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Output BLE log file:"),
                                QString::fromUtf8(file_for_ble_outlog.c_str()),
                                QString("cannot be opened"),
                                false);
    }
#endif
    status = FPc.setOutputMixedLogFile( out_log_folder + file_for_mixed_outlog, this->Output_Struct, out_log_folder + file_for_mixed_outlog_dbg );

    if ( status != Fppe::ReturnStatus::STATUS_SUCCESS )
    {
        emit this->file_error_message(QString("Output mixed log file:"),
                                QString::fromUtf8(file_for_mixed_outlog.c_str()),
                                QString("cannot be opened"),
                                false);
    }

    // start of data process
    emit this->file_error_message_last(program_exit);
//    emit this->file_error_message_last(true);
    mPauseMutex.lock();
    mPause->wait(&mPauseMutex); // use mPause to do not create new QWaitCondition
    mPauseMutex.unlock();

    switch(mTrackForMode)
    {
        case 0:
        FPc.setTrackForMFPMode();
        break;
        case 1:
        FPc.setTrackForMixedMode();
        break;
    }

    MagneticVector mag_vector;

    Fppe::WiFiScanResult wifi_scan_result;
    Fppe::BleScanResult ble_scan_result;

    if (startPosition.is_valid)
        FPc.setStartPosition(startPosition);


    TpnOutput tpnOut;

    while ( !mStop && (FPc.getTpnOutData( tpnOut, wifi_scan_result, ble_scan_result ) == Fppe::ReturnStatus::STATUS_SUCCESS) )
    {
            if (wifi_text_format == true)
            {
                FPc.getInputWiFiData(wifi_scan_result);
            }
            if (wifi_scan_result.scanWiFi.size() > 0)
                FPc.processInputWiFiData(wifi_scan_result);

            if (ble_text_format == true)
            {
                FPc.getInputBleData(ble_scan_result);
            }
            if (ble_scan_result.scanBle.size() > 0)
            {
                FPc.processInputBleData(ble_scan_result);
                FPc.processInputBleDataForProximity(ble_scan_result);
            }


        FPc.processTpnOutput( tpnOut );

        if(isPaused)
        {
            mPauseMutex.lock();
            mPause->wait(&mPauseMutex);
            mPauseMutex.unlock();
        }
    }

    Fppe::MagneticCalibrationParam bias_cov;
    FPc.getMagneticBias( &bias_cov );

    if ( mag_bias_file == "" )
    {
        if ( fp_bases_folder.back() != '\\' )
        {
            fp_bases_folder = fp_bases_folder + "\\";
        }

        mag_bias_file = fp_bases_folder + "default.mbias";
    }

    FPc.writeMagneticBiasToFile( bias_cov, mag_bias_file );
    FPc.closeOutputLogFiles();

    std::cout << "workFinished";
    std::cout.flush();
    emit this->workFinished();

}

