#include <conio.h>
#include <iomanip>
#include <sys/stat.h>

#include "FP_console.hpp"
#include "FPBasesReader.hpp"
#include "InputDataReader.hpp"
#include "PositionUpdate.hpp"

namespace FPPositionConsole
{
    FP_console::FP_console( std::string name, OutPut_for_image* Out, Track *track )
    {
        FPname = name;

        fpEngine = NULL;

        pFPBasesReader = new FPBasesReader();

        pInputDataReader = new TpnInputDataReader();

        fpEngine = new  Fppe::FPEngine();

        fpEngine->restart(); //Temporary

        pPosMagCbk = NULL;
        pPosWiFiCbk = NULL;
        pPosBleCbk = NULL;
        pPosMixedCbk = NULL;

        this->Out = Out;
        mTrack = track;

        std::vector<std::string> log_descr;
        fpEngine->getLogDescription( &log_descr );

        for ( auto it = log_descr.cbegin(); it != log_descr.cend(); ++it )
        {
            //std::cout << *it << std::endl;
        }

        for (int i = 0; i < log_descr.size(); ++i)
        {
                logs[i].open( log_descr[i] + "_init.log", std::ios::out );
                fpEngine->setLogStream( i, logs[i] );
        }

    }

    FP_console::~FP_console()
    {
        delete fpEngine;
        delete pFPBasesReader;
        delete pInputDataReader;

        delete pPosMagCbk;
        delete pPosWiFiCbk;
        delete pPosBleCbk;
        delete pPosBleProximityCbk;
        delete pPosMixedCbk;
    }


    Fppe::ReturnStatus FP_console::setVenueParams( const Venue &ven )
    {
        venue = ven;
        Fppe::ReturnStatus status = fpEngine->setVenueParams( venue );
        return status;
    }

    Fppe::ReturnStatus FP_console::setFPBasesFolder( const std::string &input_data_folder )
    {
        Fppe::ReturnStatus status;

        status = pFPBasesReader->setFPBasesFolder( input_data_folder );

        return status;
    }

    Fppe::ReturnStatus FP_console::getBaseName( std::string &base_name, const std::string &file_mask )
    {
        Fppe::ReturnStatus status;

        status = pFPBasesReader->getBaseName( base_name, file_mask );

        return status;
    }

    Fppe::ReturnStatus FP_console::initializeMFP( const std::string &mfp_db_name, const double &max_X, const double &max_Y, const double &cellSize, const int &minFloor, const int &maxFloor )
    {
        //Fppe::ReturnStatus status = fpEngine->initializeMFP( mfp_db_name, max_X, max_Y, cellSize, minFloor, maxFloor );
        //return status;

        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;

        struct stat file_buf;
        stat( mfp_db_name.c_str(), &file_buf );
        size_t mfpMapSizeInBytes = file_buf.st_size;
        char *pMfpMap = ( char * )malloc( mfpMapSizeInBytes );

        FILE *pF = fopen( mfp_db_name.c_str(), "rb" );
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
            status = fpEngine->initializeMFP( pMfpMap, mfpMapSizeInBytes, max_X, max_Y, cellSize, minFloor, maxFloor );
            fpEngine->setUpdateMFP( true ); /**< updater MFP control */
        }
        else
        {
            std::cout << "file  " << mfp_db_name << "   can not be opened" << std::endl;
            fpEngine->setUpdateMFP( false ); /**< updater MFP control */
        }
        return status;

    }

    void FP_console::setUpdateMFP( bool enable ) /**< updater MFP control */
    {
        fpEngine->setUpdateMFP( enable );
    }

    Fppe::ReturnStatus FP_console::initializeWiFi( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p )
    {
        Fppe::ReturnStatus status = fpEngine->initializeWiFi( pWiFiMap,  wifiFileSizeInBytes, min_p );
        return status;
    }

    void FP_console::setUpdateWiFi( bool enable ) /**< updater WiFi control */
    {
        fpEngine->setUpdateWiFi( enable );
    }

    Fppe::ReturnStatus FP_console::initializeBLE( const std::string &ble_db_name, const double &min_p )
    {
        //Fppe::ReturnStatus status = fpEngine->initializeBLE( ble_db_name, min_p );
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        return status;
    }

    void FP_console::setUpdateBLE( bool enable ) /**< updater BLE control */
    {
        fpEngine->setUpdateBLE( enable );
    }

    Fppe::ReturnStatus FP_console::initializeBLEProximity(const char* const pBleMap, const size_t bleFileSizeInBytes)
    {
        Fppe::ReturnStatus status = fpEngine->initializeBLEProximity(pBleMap, bleFileSizeInBytes);
        return status;
    }

    void FP_console::setUpdateBLEProximity(bool enable) /**< updater BLE control */
    {
        fpEngine->setUpdateBLEProximity(enable);
    }


    Fppe::ReturnStatus FP_console::setInputDataFolder( const std::string &input_data_folder )
    {
        Fppe::ReturnStatus status;

        status = pInputDataReader->setInputDataFolder( input_data_folder );

        return status;
    }

    Fppe::ReturnStatus FP_console::setInputIncMagDataFile( const std::string &file_mask )
    {
        Fppe::ReturnStatus status;

        status = pInputDataReader->setInputIncMagDataFile( file_mask );

        return status;
    }

    Fppe::ReturnStatus FP_console::setInputWiFiDataFile( const std::string &file_mask )
    {
        Fppe::ReturnStatus status;

        status = pInputDataReader->setInputWiFiDataFile( file_mask );

        return status;
    }

    Fppe::ReturnStatus FP_console::setInputBleDataFile( const std::string &file_mask )
    {
        Fppe::ReturnStatus status;

        status = pInputDataReader->setInputBleDataFile( file_mask );

        return status;
    }

    Fppe::ReturnStatus FP_console::setOutputMagLogFile( const std::string &output_maglog_file, OutPut_for_image *Out, const std::string &output_maglog_file_dbg)
    {

        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosMagCbk;

        try
        {
            pPosMagCbk = new PositionUpdate( output_maglog_file, Out, output_maglog_file_dbg ); //exception if file was not opened
        }
        catch ( ... )
        {
            pPosMagCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackMFP( pPosMagCbk );
        //dynamic_cast<PositionUpdate *>(pPosMagCbk)->setTrack(mTrack);
        return status;
    }

    Fppe::ReturnStatus FP_console::setOutputWiFiLogFile( const std::string &output_wifilog_file, OutPut_for_image *Out, const std::string &output_wifilog_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosWiFiCbk;

        try
        {
            pPosWiFiCbk = new PositionUpdate( output_wifilog_file, Out, output_wifilog_file_dbg ); //exception if file was not opened
        }
        catch ( ... )
        {
            pPosWiFiCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackWiFi( pPosWiFiCbk );
        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::setOutputBLELogFile( const std::string &output_blelog_file, OutPut_for_image *Out, const std::string &output_blelog_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosBleCbk;

        try
        {
            pPosBleCbk = new PositionUpdate( output_blelog_file, Out, output_blelog_file_dbg ); //exception if file was not opened
        }
        catch ( ... )
        {
            pPosBleCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackBLE( pPosBleCbk );
        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::setOutputBLEProximityLogFile(const std::string &output_bleproximitylog_file, OutPut_for_image *Out, const std::string &output_bleproximitylog_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosBleProximityCbk;

        try
        {
            pPosBleProximityCbk = new PositionUpdate(output_bleproximitylog_file, Out, output_bleproximitylog_file_dbg); //exception if file was not opened
        }
        catch (...)
        {
            pPosBleCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackBLEProximity(pPosBleProximityCbk);
        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::setOutputMixedLogFile( const std::string &output_mixedlog_file, OutPut_for_image *Out, const std::string &output_mixedlog_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosMixedCbk;

        try
        {
            pPosMixedCbk = new PositionUpdate( output_mixedlog_file, Out, output_mixedlog_file_dbg ); //exception if file was not opened
        }
        catch ( ... )
        {
            pPosMixedCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackMixed( pPosMixedCbk );
//        dynamic_cast<PositionUpdate *>(pPosMixedCbk)->setTrack(mTrack);
        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::setTrackForMFPMode()
    {
        if(pPosMagCbk != NULL && mTrack != NULL)
        {
            dynamic_cast<PositionUpdate *>(pPosMagCbk)->setTrack(mTrack);
            return Fppe::ReturnStatus::STATUS_SUCCESS;
        }
        else
            return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
    }

    Fppe::ReturnStatus FP_console::setTrackForMixedMode()
    {
        if(pPosMagCbk != NULL && mTrack != NULL)
        {
            dynamic_cast<PositionUpdate *>(pPosMixedCbk)->setTrack(mTrack);
            return Fppe::ReturnStatus::STATUS_SUCCESS;
        }
        else
            return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
    }

    Fppe::ReturnStatus FP_console::closeOutputLogFiles()
    {
        fpEngine->setPositionCallbackMFP( NULL );
        delete pPosMagCbk;
        pPosMagCbk = NULL;

        fpEngine->setPositionCallbackWiFi( NULL );
        delete pPosWiFiCbk;
        pPosWiFiCbk = NULL;

        fpEngine->setPositionCallbackBLE( NULL );
        delete pPosBleCbk;
        pPosBleCbk = NULL;

        fpEngine->setPositionCallbackMixed( NULL );
        delete pPosMixedCbk;
        pPosMixedCbk = NULL;

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    void FP_console::setStartPosition(const Fppe::Position &position)
    {
        fpEngine->setStartPosition(position);
    }

    Fppe::ReturnStatus  FP_console::getTpnOutData(TpnOutput &tpnData, Fppe::WiFiScanResult &wifi_scan_result, Fppe::BleScanResult &ble_scan_result)
    {
        Fppe::ReturnStatus status;

        status = pInputDataReader->getInputIncMagData(tpnData, wifi_scan_result, ble_scan_result);

        return status;
    }

    Fppe::ReturnStatus FP_console::getInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result )
    {

        Fppe::ReturnStatus status;

        status = pInputDataReader->getInputWiFiData( wifi_scan_result );

        return status;
    }

    Fppe::ReturnStatus FP_console::getInputBleData( Fppe::BleScanResult &ble_scan_result )
    {

        Fppe::ReturnStatus status;

        status = pInputDataReader->getInputBleData( ble_scan_result );

        return status;
    }

    Fppe::ReturnStatus  FP_console::processInputIncMagData( Fppe::CoordinatesIncrement coordinatesIncrement, MagneticVector mag_vector )
    {
        fpEngine->processMFP( mag_vector, coordinatesIncrement.timestamp );

        fpEngine->processIncrements( coordinatesIncrement );

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus  FP_console::processTpnOutput( const TpnOutput &tpnData )
    {
        fpEngine->processTpnOutput( tpnData );

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::processInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result )
    {
        fpEngine->processWiFi( wifi_scan_result );

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::processInputBleData( Fppe::BleScanResult &ble_scan_result )
    {
        fpEngine->processBLE( ble_scan_result );

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::processInputBleDataForProximity( Fppe::BleScanResult &ble_scan_result )
    {
        fpEngine->processBLEForProximity( ble_scan_result );

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::readMagneticBiasFromFile( Fppe::MagneticCalibrationParam *bias_cov, const std::string &input_magbias_file )
    {

        std::string line;
        const char delim = ',';

        std::ifstream inputStream;

        inputStream.open( input_magbias_file );

        if ( inputStream.fail() )
        {
            inputStream.close();
            return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        inputStream >> bias_cov->timestamp;
        inputStream >> bias_cov->mX;
        inputStream >> bias_cov->mY;
        inputStream >> bias_cov->mZ;

        memset( bias_cov->covarianceMatrix, 0, sizeof( bias_cov->covarianceMatrix ) );

        inputStream >> bias_cov->covarianceMatrix[0][0];
        inputStream >> bias_cov->covarianceMatrix[1][1];
        inputStream >> bias_cov->covarianceMatrix[2][2];

        inputStream.close();

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::writeMagneticBiasToFile( Fppe::MagneticCalibrationParam bias_cov, const std::string &output_magbias_file )
    {

        std::ofstream outputStream;

        outputStream.open( output_magbias_file );

        if ( outputStream.fail() )
        {
            outputStream.close();
            return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        outputStream << bias_cov.timestamp << std::endl;
        outputStream << bias_cov.mX << std::endl;
        outputStream << bias_cov.mY << std::endl;
        outputStream << bias_cov.mZ << std::endl;

        outputStream << bias_cov.covarianceMatrix[0][0] << std::endl;
        outputStream << bias_cov.covarianceMatrix[1][1] << std::endl;
        outputStream << bias_cov.covarianceMatrix[2][2] << std::endl;

        outputStream.close();

        return Fppe::ReturnStatus::STATUS_SUCCESS;

    }

    void FP_console::getMagneticBias( Fppe::MagneticCalibrationParam *bias_cov )
    {
        fpEngine->getMagneticBias( bias_cov );
    }

    void FP_console::setMagneticBias( const Fppe::MagneticCalibrationParam &bias_cov )
    {
        fpEngine->setMagneticBias( bias_cov );
    }
} // namespace FPBuilderConsole
