#include "Fppe.hpp"
#include "FppeImp.hpp"
#include "version.h"

using namespace Fppe; //TODO delete this


FPEngine::FPEngine()
{
    mFPEngine = new FPEngineImpl();
}
FPEngine::~FPEngine()
{
    delete mFPEngine;
}


/** \return version info*/
VersionNumber FPEngine::getVersionNumber() const
{
    VersionNumber v = { FPPE_VERSION_MAJOR, FPPE_VERSION_MINOR, FPPE_VERSION_BUILD, static_cast<VersionNumberReleaseId>( FPPE_VERSION_RELEASE_ID ) };
    return v;
}


///**
//* resets internal object state
//* \param[in] filename output log filename
//*/
//void FPEngine::setLogFile( const std::string &filename )
//{
//    mFPEngine->setLogFile( filename );
//}
//
///**
//* enables logging
//* \param[in] enabled control flag
//*/
//void FPEngine::setLogEnabled( const bool &enabled )
//{
//    mFPEngine->setLogEnabled( enabled );
//}

/**
* resets internal object state, disables logging
*/
void FPEngine::restart()
{
    mFPEngine->restart();
}

/**
* main method, process inertial sensors data and initiates processing pipeline
* \param[in] increment position increments in local frame
*/
void FPEngine::processIncrements( const CoordinatesIncrement &increment )
{
    mFPEngine->processIncrements( increment );
}

void FPEngine::processTpnOutput( const TpnOutput &tpn_output)
{
    mFPEngine->processTpnOutput( tpn_output);
}

/**
* pushes WiFi measurement into the processing pipeline
* \param[in] scan_wifi WiFi measurements vector with timestamp
*/
void FPEngine::processWiFi( const WiFiScanResult &scan_wifi )
{
    mFPEngine->processWiFi( scan_wifi );
}

/**
* pushes BLE measurement into the processing pipeline
* \param[in] scan_ble BLE measurements vector with timestamp
*/
void FPEngine::processBLE( const BleScanResult &scan_ble )
{
    mFPEngine->processBLE( scan_ble );
}

/**
* pushes Magnetic measurement into the processing pipeline
* \param[in] mag_data magnetic vector with timestamp
*/
void FPEngine::processMFP( const MagneticVector &mag_data, int64_t timestamp )
{
    mFPEngine->processMFP( mag_data, timestamp );
}

/**
* pushes external location measurement into the processing pipeline
* \param[in] position external position
*/
void FPEngine::processExternalPosition( const Fppe::Position &position )
{
    mFPEngine->processExternalPosition( position );
}

/**
* pushes ble_collaboration into the processing pipeline
* \param[in] ble_collaboration position
*/
void FPEngine::processInputCollaboration(std::vector <Fppe::CollaborationData> &collaboration_position)
{
    mFPEngine->processInputCollaboration(collaboration_position);
}

int FPEngine::getProximityBeaconsNumber(const int16_t floor, const BleBeaconType beacon_type)
{
    return mFPEngine->getProximityBeaconsNumber(floor, beacon_type);
}


/**
* Sets main filter position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine::setPositionCallbackMixed( Fppe::IPositionUpdate *pPosCbk )
{
    mFPEngine->setPositionCallbackMixed( pPosCbk );
}

/**
* Sets WiFi only position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine::setPositionCallbackWiFi( Fppe::IPositionUpdate *pPosCbk )
{
    mFPEngine->setPositionCallbackWiFi( pPosCbk );
}

/**
* Sets BLE only position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine::setPositionCallbackBLE( Fppe::IPositionUpdate *pPosCbk )
{
    mFPEngine->setPositionCallbackBLE( pPosCbk );
}

/**
* Sets BLE proximity only position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine::setPositionCallbackBLEProximity( Fppe::IPositionUpdate *pPosCbk )
{
    mFPEngine->setPositionCallbackBLEProximity( pPosCbk );
}

/**/
void FPEngine::setExtendedProximityCallback(IExtendedProximityUpdate *pProximityCbk)
{
    mFPEngine->setExtendedProximityCallback(pProximityCbk);
}

/**
* Sets MFP only(mfp+pdr) position callback
* param[in] pPosCbk pointer to the callback implementation
*/
void FPEngine::setPositionCallbackMFP( Fppe::IPositionUpdate *pPosCbk )
{
    mFPEngine->setPositionCallbackMFP( pPosCbk );
}

/**
* Sets venue detection WiFi callback
* param[in] pVenueDetectCbk pointer to the callback implementation
*/
void FPEngine::setVenueDetectionCallbackWiFi(Fppe::IVenueDetectionUpdate *pVenueDetectCbk)
{
	mFPEngine->setVenueDetectionCallbackWiFi(pVenueDetectCbk);
}

/**
* initialize WiFi module and loads fingerprint from  the file
* \param[in] wifi_db_name fingerprint filename
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
ReturnStatus FPEngine::initializeWiFi( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p )
{
    ReturnStatus result = mFPEngine->initializeWiFI( pWiFiMap, wifiFileSizeInBytes, min_p );
    return result;
}

/**
* initialize WiFi module and loads fingerprint from  the array
* \param[in] pWiFiMap fingerprint pointer to array
* \param[in] wifiFileSizeInBytes fingerprint data size
* \return success status
*/
ReturnStatus FPEngine::initializeWiFi(const char* const pWiFiMap, const size_t wifiFileSizeInBytes)
{
    ReturnStatus result = mFPEngine->initializeWiFI(pWiFiMap, wifiFileSizeInBytes);
    return result;
}

/**
* initialize BLE module and loads fingerprint from the file
* \param[in] ble_db_name fingerprint filename
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
ReturnStatus FPEngine::initializeBLE( const char* const pBleMap, const size_t bleFileSizeInBytes, const double min_p )
{
    ReturnStatus result = mFPEngine->initializeBLE( pBleMap, bleFileSizeInBytes, min_p );
    return result;
}

/**
* initialize BLE module and loads fingerprint from the file
* \param[in] ble_db_name fingerprint filename
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
ReturnStatus FPEngine::initializeBLE(const char* const pBleMap, const size_t bleFileSizeInBytes)
{
    ReturnStatus result = mFPEngine->initializeBLE(pBleMap, bleFileSizeInBytes);
    return result;
}

/**
* initialize BLE proximity module and loads fingerprint from the file
* \param[in] ble_db_name fingerprint filename
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
ReturnStatus FPEngine::initializeBLEProximity(const char* const pBleMap, const size_t bleFileSizeInBytes)
{
    ReturnStatus result = mFPEngine->initializeBLEProximity(pBleMap, bleFileSizeInBytes);
    return result;
}

/**
* initialize MFP module and loads fingerprint from the file
* \param[in] mfp_db_name fingerprint filename
* \param[in] max_X defines fingerprint size on X axis [m] [TODO  to be moved into FP file]
* \param[in] max_Y defines fingerprint size on Y axis [m] [TODO  to be moved into FP file]
* \param[in] cellSize defines internal map discrete [m] [TODO  to be moved into FP file]
* \param[in] minFloor define minimum floor number [TODO  to be moved into FP file]
* \param[in] maxFloor define maximum floor number [TODO  to be moved into FP file]
* \return success status
*/
ReturnStatus FPEngine::initializeMFP( const char* const pMFPMap, const size_t mfpFileSizeInBytes, const double max_X, const double max_Y, const double cellSize, const int minFloor, const int maxFloor )
{
    ReturnStatus result = mFPEngine->initializeMFP( pMFPMap, mfpFileSizeInBytes, max_X, max_Y, cellSize, minFloor, maxFloor );
    return result;
}


ReturnStatus FPEngine::initializeMFP(const char* const pMFPMap, const size_t mfpFileSizeInBytes)
{
    ReturnStatus result = mFPEngine->initializeMFP(pMFPMap, mfpFileSizeInBytes);
    return result;
}

ReturnStatus FPEngine::initializeMapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes)
{
	ReturnStatus result = mFPEngine->initializeMapMatching(pMap, mapFileSizeInBytes);
	return result;
}


FPHeaderBaseType FPEngine::getWfpInfo()
{
    return mFPEngine->getWfpInfo();
}

FPHeaderBaseType FPEngine::getMfpInfo()
{
    return mFPEngine->getMfpInfo();
}

FPHeaderBaseType FPEngine::getBfpInfo()
{
    return mFPEngine->getBfpInfo();
}

FPHeaderBaseType FPEngine::getProximityDbInfo()
{
    return mFPEngine->getProximityDbInfo();
}

/**
* Defines venue local frame
* \param[in] venue  venue parameters with local frame origin
* \return success
*/
ReturnStatus FPEngine::setVenueParams( const Venue &venue )
{
    ReturnStatus result = mFPEngine->setVenueParams( venue );
    return result;
}

/**
* Defines venue local frame
* \param[in] venue  venue parameters with local frame origin
* \return success
*/
ReturnStatus FPEngine::setVenueParams(const BaseVenueType &venue)
{
    ReturnStatus result = mFPEngine->setVenueParams(venue);
    return result;
}

/** updater WiFi control
* \param[in] enable control flag
*/
void FPEngine::setUpdateWiFi( bool enable )
{
    mFPEngine->setUpdateWiFi( enable );
}

/** updater MFP control
*\ param[in] enable control flag
*/
void FPEngine::setUpdateMFP( bool enable )
{
    mFPEngine->setUpdateMFP( enable );
}

/** updater Map Matching control
*\ param[in] enable control flag
*/
void FPEngine::setUpdateMapMatching(bool enable) /**< updater Map Matching control */
{
    mFPEngine->setUpdateMapMatching(enable);
}

/** updater BLE control
* \param[in] enable control flag
*/
void FPEngine::setUpdateBLE( bool enable )
{
    mFPEngine->setUpdateBLE( enable );
}

/** updater BLE proximity control
* \param[in] enable control flag
*/
void FPEngine::setUpdateBLEProximity( bool enable )
{
    mFPEngine->setUpdateBLEProximity( enable );
}

/** updater External Pos control
* \param[in] enable set Pos update enable/disable
*/
void FPEngine::setUpdateExternalPosition( const bool enable )
{
    mFPEngine->setUpdateExternalPosition( enable );
}

/** updater BLE Pos control
* \param[in] enable set Pos update enable/disable
*/
void FPEngine::setUpdateCollaboration(const bool enable)
{
    mFPEngine->setUpdateCollaboration(enable);
}

/** Corrector fusion filter position control
* \param[in] enable set Pos update enable/disable
*/
void FPEngine::setCorrectFusionFilterPosition(const bool enable)
{
    mFPEngine->setCorrectFusionFilterPosition(enable);
}

/** Enables floor increment usage, switches to another motion model
* \param[in] enable control flag, if enabled uses increments in motion model, otherwise use deterministic floor model
*/
void FPEngine::setFloorIncrements( bool enable )
{
    mFPEngine->setFloorIncrements( enable );
}

/** Gets mg calibration params
* \param[out] bias_cov esimated magnetic bias with covariance
*/
bool FPEngine::getMagneticBias( MagneticCalibrationParam *bias_cov )
{
    return mFPEngine->getMagneticBias( bias_cov );
}

/** Gets mg calibration params
* \param[in] bias_cov initial magnetic bias with covariance
*/
void FPEngine::setMagneticBias( const MagneticCalibrationParam &bias_cov )
{
    mFPEngine->setMagneticBias( bias_cov );
}

/** Gets platform type
* \return platform type
*/
PlatformType FPEngine::getPlatformType()
{
    return mFPEngine->getPlatformType();
}

/** Sets platform type
* \param[in] platform type
*/
void FPEngine::setPlatformType(PlatformType platform_type)
{
    mFPEngine->setPlatformType(platform_type);
}

/** Sets BLP pulling type
* \param[in] pulling type
* \param[in] pulling distance
* \param[in] pulling sigma
*/
void FPEngine::setBlpPulling(ePullingType type, double pulling_distance, double pulling_sigma)
{
    mFPEngine->setBlpPulling(type, pulling_distance, pulling_sigma);
}

void FPEngine::setBlpDetectionEnable(bool enable)
{
    mFPEngine->setBlpDetectionEnable(enable);
}

void FPEngine::setBlpPositioningPdFilterParams(
    int peak_detector_max_delay_ms_in_moving,
    double descending_factor_in_moving,
    int peak_detector_max_delay_ms_in_stop,
    double descending_factor_in_stop
)
{
    mFPEngine->setBlpPositioningPdFilterParams(
        peak_detector_max_delay_ms_in_moving,
        descending_factor_in_moving,
        peak_detector_max_delay_ms_in_stop,
        descending_factor_in_stop);
}

void FPEngine::setBlpDetectionPdFilterParams(
    int peak_detector_max_delay_ms_in_moving,
    double descending_factor_in_moving,
    int peak_detector_max_delay_ms_in_stop,
    double descending_factor_in_stop
)
{
    mFPEngine->setBlpDetectionPdFilterParams(
        peak_detector_max_delay_ms_in_moving,
        descending_factor_in_moving,
        peak_detector_max_delay_ms_in_stop,
        descending_factor_in_stop);
}

void FPEngine::setBlpPositioningLogicParams(int filter_length, int repeat_number,
    int cutoff)
{
    mFPEngine->setBlpPositioningLogicParams(filter_length, repeat_number, cutoff);
}

void FPEngine::setBlpDetectionLogicParams(int filter_length, int repeat_number,
    int cutoff)
{
    mFPEngine->setBlpDetectionLogicParams(filter_length, repeat_number, cutoff);
}

/** WiFi calibration status [obsolete]
* param[out] bias rssi bias
* returns calibration enabled
*/
ReturnStatus FPEngine::getWiFiBias(double *bias)
{
    bool result = mFPEngine->getWiFiBias(bias);
    if (result)
        return ReturnStatus::STATUS_SUCCESS;
    else
        return ReturnStatus::STATUS_UNKNOWN_ERROR;
}
/** sets WiFi calibration [obsolete]
* param[in] bias initial rssi bias
* param[in] enabled enables calibration
*/
void FPEngine::setWiFiBias( const double  &bias, int64_t delta_t )
{
    mFPEngine->setWiFiBias( bias, delta_t );
}

/** BLE calibration status [obsolete]
* param[out] bias rssi bias
* returns calibration enabled
*/
ReturnStatus FPEngine::getBLEBias(double *bias)
{
    bool result = mFPEngine->getBLEBias(bias);
    if (result)
        return ReturnStatus::STATUS_SUCCESS;
    else
        return ReturnStatus::STATUS_UNKNOWN_ERROR;
}

void FPEngine::setBLEBias(const double  &bias, int64_t delta_t)
{
    mFPEngine->setBLEBias(bias, delta_t);
}

ReturnStatus FPEngine::getBLEBiasWithUncertainty(double *bias, double *bias_uncertainty)
{
	bool result = mFPEngine->getBLEBiasWithUncertainty(bias, bias_uncertainty);
	if (result)
		return ReturnStatus::STATUS_SUCCESS;
	else
		return ReturnStatus::STATUS_UNKNOWN_ERROR;
}

void FPEngine::setBLEBiasWithUncertainty(const double  &bias, const double &bias_uncertainty, int64_t bias_age)
{
	mFPEngine->setBLEBiasWithUncertainty(bias, bias_uncertainty, bias_age);
}

/**
* Sets fine known initial position
* \param[in] position initial position in global frame
*/
void FPEngine::setStartPosition( const Position &position )
{
    mFPEngine->setStartPosition( position );
}

void FPEngine::setRandomSeeds()
{
    mFPEngine->setRandomSeeds();
}

void FPEngine::setUseBarometer(bool enable)
{
    mFPEngine->setUseBarometer(enable);
}
            
void FPEngine::setOsType(OperationSystemType os_type)
{
    mFPEngine->setOsType(os_type);
}

bool FPEngine::setLogStream( const unsigned int &idx, std::ostream &os )
{
    return mFPEngine->setLogStream( idx, os );
}

void FPEngine::getLogDescription( std::vector<std::string> *log_descriptions )
{
    mFPEngine->getLogDescription( log_descriptions );
}


/**
* Control Mag PF
* param[in] enable disables/enables Mag filter
*/
void FPEngine::setMagFilterEnabled(bool enable)
{
    mFPEngine->setMagFilterEnabled(enable);
}

/**
* Control Mixed PF
* param[in] enable disables/enables Mixed filter
*/
void FPEngine::setMixedFilterEnabled(bool enable)
{
    mFPEngine->setMixedFilterEnabled(enable);
}

/**
* return Mag filter state
*/
bool FPEngine::getMagFilterEnabled()
{
    return mFPEngine->getMagFilterEnabled();
}

/**
* return Mixed filter state
*/
bool FPEngine::getMixedFilterEnabled()
{
    return mFPEngine->getMixedFilterEnabled();
}

/**
* return venue params
*/
BaseVenueType FPEngine::getVenueParams()
{
    return mFPEngine->getVenueParams();
}

