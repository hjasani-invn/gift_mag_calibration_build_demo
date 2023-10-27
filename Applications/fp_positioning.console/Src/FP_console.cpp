#include <iomanip>
#include <memory.h>

#include "FP_console.hpp"
#include "FPBasesReader.hpp"
#include "InputDataReader.hpp"
#include "PositionUpdate.hpp"
#include "VenueDetectionUpdate.hpp"
#include <vector>
#include "CFPPE.h"

namespace FPPositionConsole
{
    FP_console::FP_console(const std::string name, const std::string log_folder )
    {
        FPname = name;

        memset(&venue, 0, sizeof(Venue));

        fpEngine = NULL;

        pFPBasesReader = new FPBasesReader();

        pInputDataReader = new TpnInputDataReader();

        fpEngine = new  Fppe::FPEngine();

        fpEngine->restart(); //Temporary

        pPosMagCbk = NULL;
        pPosWiFiCbk = NULL;
        pPosBleCbk = NULL;
        pPosBleProximityCbk = NULL;
        pPosMixedCbk = NULL;
        pVenueDetectCbk = NULL;
        pExtendedProximityCbk = nullptr;

        std::vector<std::string> log_descr;
        fpEngine->getLogDescription( &log_descr );

        this->log_folder = log_folder;
        
        for (unsigned int i = 0; i < log_descr.size(); ++i)
        {
                logs[i].open( log_folder + log_descr[i] + "_init.log", std::ios::out );
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
        delete pPosMixedCbk;
    }

    
    VersionNumber FP_console::getLibVersion()
    {
        VersionNumber ver = fpEngine->getVersionNumber();
        return ver;
    }

    VersionNumberBase FP_console::getLibVersion_C()
    {
        VersionNumberBase ver = FPEngine_getVersionNumber((CIFPEngine*)this);
        return ver;
    }

    void FP_console::setRandomSeeds()
    {
        fpEngine->setRandomSeeds();
    }

    /**
    * set use barometr
    * param[in] enable/disable
    */
    void FP_console::setUseBarometer(bool enable)
    {
        fpEngine->setUseBarometer(enable);
    }

    /**
    * set OS type
    * param[in] OS type
    */
    void FP_console::setOsType(OperationSystemType os_type)
    {
        fpEngine->setOsType(os_type);
    }

    Fppe::ReturnStatus FP_console::setVenueParams(const BaseVenue &ven)
    {
        venue = ven;
        Fppe::ReturnStatus status = fpEngine->setVenueParams(venue);
        return status;
    }

    Fppe::ReturnStatus FP_console::setFPBasesFolder(const std::string &input_data_folder)
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

    Fppe::ReturnStatus FP_console::initializeMFP( const char* const pMFPMap, const size_t mfpFileSizeInBytes, const double max_X, const double max_Y, const double cellSize, const int minFloor, const int maxFloor )
    {
        Fppe::ReturnStatus status = fpEngine->initializeMFP( pMFPMap, mfpFileSizeInBytes, max_X, max_Y, cellSize, minFloor, maxFloor );
        return status;
    }

	Fppe::ReturnStatus FP_console::initializeMapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes)
	{
		Fppe::ReturnStatus status = fpEngine->initializeMapMatching(pMap, mapFileSizeInBytes);
		return status;
	}

    Fppe::ReturnStatus FP_console::initializeMFP(const char* const pMFPMap, const size_t mfpFileSizeInBytes)
    {
        Fppe::ReturnStatus status = fpEngine->initializeMFP(pMFPMap, mfpFileSizeInBytes);
        return status;
    }

    FPHeaderBaseType FP_console::getMfpInfo()
    {
        FPHeaderBaseType fpHeader = fpEngine->getWfpInfo();
        return fpHeader;
    }

    void FP_console::setUpdateMFP( bool enable ) /**< updater MFP control */
    {
        fpEngine->setUpdateMFP( enable );
    }

    void FP_console::setUpdateMapMatching(bool enable) /**< updater Map Matching control */
    {
        fpEngine->setUpdateMapMatching(enable);
    }

    Fppe::ReturnStatus FP_console::initializeWiFi( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p )
    {
        Fppe::ReturnStatus status = fpEngine->initializeWiFi( pWiFiMap,  wifiFileSizeInBytes, min_p );
        return status;
    }

    Fppe::ReturnStatus FP_console::initializeWiFi(const char* const pWiFiMap, const size_t wifiFileSizeInBytes)
    {
        Fppe::ReturnStatus status = fpEngine->initializeWiFi(pWiFiMap, wifiFileSizeInBytes);
        return status;
    }

    FPHeaderBaseType FP_console::getWfpInfo()
    {
        FPHeaderBaseType fpHeader = fpEngine->getWfpInfo();
        return fpHeader;
    }

    void FP_console::setUpdateWiFi( bool enable ) /**< updater WiFi control */
    {
        fpEngine->setUpdateWiFi( enable );
    }

    Fppe::ReturnStatus  FP_console::getWiFiBias(double *bias)
    {
        return fpEngine->getWiFiBias(bias);
    }

    void FP_console::setWiFiBias(const double  &bias, int64_t delta_t)
    {
        fpEngine->setWiFiBias(bias, delta_t);
    }

    Fppe::ReturnStatus  FP_console::getBLEBias(double *bias)
    {
        return fpEngine->getBLEBias(bias);
    }

    void FP_console::setBLEBias(const double  &bias, int64_t delta_t)
    {
        fpEngine->setBLEBias(bias, delta_t);
    }
    
    void FP_console::setUpdateFrameworkPos(bool enable) /**< updater Framework position control */
    {
        fpEngine->setUpdateExternalPosition(enable);
    }

    void FP_console::setUpdateCollaboration(bool enable) /**< updater collaboration position control */
    {
        fpEngine->setUpdateCollaboration(enable);
    }

    Fppe::ReturnStatus FP_console::initializeBLE( const char* const pBleMap, const size_t bleFileSizeInBytes, const double min_p )
    {
        Fppe::ReturnStatus status = fpEngine->initializeBLE( pBleMap, bleFileSizeInBytes, min_p );
        return status;
    }

    Fppe::ReturnStatus FP_console::initializeBLE(const char* const pBleMap, const size_t bleFileSizeInBytes)
    {
        Fppe::ReturnStatus status = fpEngine->initializeBLE(pBleMap, bleFileSizeInBytes);
        return status;
    }

    FPHeaderBaseType FP_console::getBfpInfo()
    {
        FPHeaderBaseType fpHeader = fpEngine->getBfpInfo();
        return fpHeader;
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

    FPHeaderBaseType FP_console::getProximityDbInfo()
    {
        FPHeaderBaseType fpHeader = fpEngine->getProximityDbInfo();
        return fpHeader;
    }

    void FP_console::setUpdateBLEProximity(bool enable) /**< updater BLE control */
    {
        fpEngine->setUpdateBLEProximity(enable);
    }

    Fppe::ReturnStatus FP_console::setInputDataFolder(const std::string &input_data_folder)
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

    Fppe::ReturnStatus FP_console::setInputFrameworkDataFile(const std::string &file_mask)
    {
        Fppe::ReturnStatus status;

        status = pInputDataReader->setInputFrameworkDataFile(file_mask);

        return status;
    }

    Fppe::ReturnStatus FP_console::setInputCollaborationDataFile(const std::string &file_mask)
    {
        Fppe::ReturnStatus status;

        status = pInputDataReader->setInputCollaborationDataFile(file_mask);

        return status;
    }

    Fppe::ReturnStatus FP_console::setOutputMagLogFile(const std::string &output_maglog_file, const std::string &output_maglog_file_dbg, const std::string &output_particleslog_file_dbg)
    {

        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosMagCbk;

        try
        {
            pPosMagCbk = new PositionUpdate(FilterType::FROM_MAGNETIC_FILTER, output_maglog_file, output_maglog_file_dbg, output_particleslog_file_dbg); //exception if file was not opened
        }
        catch ( ... )
        {
            pPosMagCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackMFP( pPosMagCbk );
        return status;
    }

    Fppe::ReturnStatus FP_console::setOutputWiFiLogFile(const std::string &output_wifilog_file, const std::string &output_wifilog_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosWiFiCbk;

        try
        {
            pPosWiFiCbk = new PositionUpdate(FilterType::NONE_GENERAL, output_wifilog_file, output_wifilog_file_dbg); //exception if file was not opened
        }
        catch ( ... )
        {
            pPosWiFiCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackWiFi( pPosWiFiCbk );
        return status;
    }

    Fppe::ReturnStatus FP_console::setOutputBLELogFile(const std::string &output_blelog_file, const std::string &output_blelog_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosBleCbk;

        try
        {
            pPosBleCbk = new PositionUpdate(FilterType::NONE_GENERAL, output_blelog_file, output_blelog_file_dbg); //exception if file was not opened
        }
        catch ( ... )
        {
            pPosBleCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackBLE( pPosBleCbk );
        return status;
    }

    Fppe::ReturnStatus FP_console::setOutputBLEProximityLogFile(const std::string &output_log_file, const std::string &output_log_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosBleProximityCbk;

        try
        {
            pPosBleProximityCbk = new PositionUpdate(FilterType::NONE_GENERAL, output_log_file, output_log_file_dbg); //exception if file was not opened
        }
        catch (...)
        {
            pPosBleProximityCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackBLEProximity(pPosBleProximityCbk);
        return status;
    }

    Fppe::ReturnStatus FP_console::setOutputExtendedProximityLogFile(const std::string &output_log_file, const std::string &output_log_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pExtendedProximityCbk;

        try
        {
            pExtendedProximityCbk = new ExtendedProximityUpdate(FilterType::NONE_GENERAL, output_log_file, output_log_file_dbg); //exception if file was not opened
        }
        catch (...)
        {
            pExtendedProximityCbk = nullptr;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setExtendedProximityCallback(pExtendedProximityCbk);
        return status;
    }

    Fppe::ReturnStatus FP_console::setOutputMixedLogFile(const std::string &output_mixedlog_file, const std::string &output_mixedlog_file_dbg, const std::string &output_particleslog_file_dbg)
    {
        Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
        delete pPosMixedCbk;

        try
        {
            pPosMixedCbk = new PositionUpdate(FilterType::FROM_MIXED_FILTER, output_mixedlog_file, output_mixedlog_file_dbg, output_particleslog_file_dbg); //exception if file was not opened
        }
        catch ( ... )
        {
            pPosMixedCbk = NULL;
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        fpEngine->setPositionCallbackMixed( pPosMixedCbk );
        return status;
    }

	Fppe::ReturnStatus FP_console::setOutputVenueDetectionLogFile(const std::string &output_venue_detect_log_file_dbg)
	{
		Fppe::ReturnStatus status = Fppe::ReturnStatus::STATUS_SUCCESS;
		delete pVenueDetectCbk;

		try
		{
			pVenueDetectCbk = new VenueDetectionUpdate(output_venue_detect_log_file_dbg);
		}
		catch (...)
		{
			pVenueDetectCbk = NULL;
			status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
		}

		fpEngine->setVenueDetectionCallbackWiFi(pVenueDetectCbk);
    return status;
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

        fpEngine->setPositionCallbackBLEProximity(NULL);
        delete pPosBleProximityCbk;
        pPosBleProximityCbk = NULL;

        fpEngine->setPositionCallbackMixed(NULL);
        delete pPosMixedCbk;
        pPosMixedCbk = NULL;

        fpEngine->setVenueDetectionCallbackWiFi(NULL);
        delete pVenueDetectCbk;
        pVenueDetectCbk = NULL;
        
        fpEngine->setExtendedProximityCallback(NULL);
        delete pExtendedProximityCbk;
        pExtendedProximityCbk = NULL;

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    void FP_console::setStartPosition( const Fppe::Position &position )
    {
        fpEngine->setStartPosition( position );
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

    Fppe::ReturnStatus FP_console::getInputFrameworkData(Fppe::Position &framework_position)
    {

        Fppe::ReturnStatus status;

        status = pInputDataReader->getInputFrameworkData(framework_position);

        return status;
    }

    Fppe::ReturnStatus FP_console::getInputCollaborationData(std::vector <Fppe::CollaborationData> &collaboration_position)
    {

        Fppe::ReturnStatus status;

        status = pInputDataReader->getInputCollaborationData(collaboration_position);

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
        fpEngine->processTpnOutput( tpnData);

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

    Fppe::ReturnStatus FP_console::processInputFrameworkPosition(Fppe::Position &framework_position)
    {
        fpEngine->processExternalPosition(framework_position);

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FP_console::processInputCollaboration(std::vector <Fppe::CollaborationData> &collaboration_position)
    {
        fpEngine->processInputCollaboration(collaboration_position);

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

    bool FP_console::getMagneticBias( Fppe::MagneticCalibrationParam *bias_cov )
    {
        return fpEngine->getMagneticBias( bias_cov );
    }

    void FP_console::setMagneticBias( const Fppe::MagneticCalibrationParam &bias_cov )
    {
        fpEngine->setMagneticBias( bias_cov );
    }

    PlatformType FP_console::getPlatformType()
    {
        return fpEngine->getPlatformType();
    }

    void FP_console::setPlatformType(PlatformType platform_type)
    {
        fpEngine->setPlatformType(platform_type);
    }

    void FP_console::setBlpPulling(ePullingType type, double pulling_distance, double pulling_sigma)
    {
        fpEngine->setBlpPulling(type, pulling_distance, pulling_sigma);
    }
 
    void FP_console::setBlpDetectionEnable(bool enable)
    {
        fpEngine->setBlpDetectionEnable(enable);
    }
    
    void FP_console::setBlpPositioningPdFilterParams(
        int peak_detector_max_delay_ms_in_moving,
        double descending_factor_in_moving,
        int peak_detector_max_delay_ms_in_stop,
        double descending_factor_in_stop
    )
    {
        fpEngine->setBlpPositioningPdFilterParams(
            peak_detector_max_delay_ms_in_moving,
            descending_factor_in_moving,
            peak_detector_max_delay_ms_in_stop,
            descending_factor_in_stop);
    }

    void FP_console::setBlpDetectionPdFilterParams(
        int peak_detector_max_delay_ms_in_moving,
        double descending_factor_in_moving,
        int peak_detector_max_delay_ms_in_stop,
        double descending_factor_in_stop
    )
    {
        fpEngine->setBlpDetectionPdFilterParams(
            peak_detector_max_delay_ms_in_moving,
            descending_factor_in_moving,
            peak_detector_max_delay_ms_in_stop,
            descending_factor_in_stop);
    }

    void FP_console::setBlpPositioningLogicParams(int filter_length, int repeat_number,
        int cutoff)
    {
        fpEngine->setBlpPositioningLogicParams(filter_length, repeat_number, cutoff);
    }

    void FP_console::setBlpDetectionLogicParams(int filter_length, int repeat_number,
        int cutoff)
    {
        fpEngine->setBlpDetectionLogicParams(filter_length, repeat_number, cutoff);
    }

    int FP_console::getProximityBeaconsNumber(int16_t floor, BleBeaconType beacon_type)
    {
        return fpEngine->getProximityBeaconsNumber(floor, beacon_type);
    }

    void FP_console::setMagneticFilterEnable(bool enable)
    {
        fpEngine->setMagFilterEnabled(enable);
    }

    void FP_console::restart()
    {
        fpEngine->restart();
    }

} // namespace FPBuilderConsole
