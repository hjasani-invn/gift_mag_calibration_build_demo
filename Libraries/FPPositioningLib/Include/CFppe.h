/**
* \file CFppe.hpp
* \brief Fingerprint Builder Library API
* \author Mostafa Elhoushi (melhoushi@invensense.com), dchurikov@invensense.com
* \version 1.0
* \copyright InvenSense, all rights reserved
* \date November 15, 2019
*/

/*! \C Wrapper for FPPE Library
* \section
* \image
*/

#ifndef C_FPPE_H
#define C_FPPE_H

#include <stdbool.h>
#include "fpHeader.h"
#include "Venue.h"
#include "CMagData.h"
#include "CTpnData.h"
#include "LibraryInfo.h"
#include "fpeDataTypes.h"


/* Guard C code in headers, while including them from C++ */
#ifdef  __cplusplus
#define FPPE_BEGIN_DECLS extern "C" {
#define FPPE_END_DECLS    }
#else
#define FPPE_BEGIN_DECLS
#define FPPE_END_DECLS
#endif


FPPE_BEGIN_DECLS

#ifdef _WIN32
#  define DLL_EXPORT __declspec( dllexport )
#else
#  define DLL_EXPORT
#endif

/** Fingerprint position engine */

/** build type */
typedef VersionNumberReleaseId CVersionNumberReleaseId;
//typedef enum CVersionNumberReleaseIdTag
//{
//   VERSION_ALPHA = 0,              /**< alpha version */
//    VERSION_BETA = 1,               /**< beta version */
//    VERSION_RELEASE_CANDIDATE = 2,  /**< pre release */
//    VERSION_RELEASE = 3             /**< release */
//} CVersionNumberReleaseId;*/


/** library build info */
typedef VersionNumberBase CVersionNumber;
//typedef struct CVersionNumberTag
//{
//    uint8_t major;  /**< major version number */
//    uint8_t minor;  /**< minor version number */
//    uint32_t build; /**< plain build number */
//    CVersionNumberReleaseId release_id; /**< build type */
//} CVersionNumber;

/**
    * operation status codes
    */
typedef enum CReturnStatusTag
{
    STATUS_SUCCESS = 0,            /**< operation success */
    STATUS_UNKNOWN_ERROR = -1      /**< undefined error occurs due operation */
} CReturnStatus;

/** input coordinates description (in WGS84)*/
typedef struct CPositionTag
{
    int64_t timestamp;     /**< unix time [ms] */
    double lattitude;       /**< lattitude [deg] [-90..+90]*/
    double longitude;       /**< longitude [deg] [-180..+180] */
    double azimuth;         /**< direction to north [deg] [-180..+180] */
    double altitude;        /**< altitude (sea level?) [m] */
    int16_t floor_number;   /**< discrete floor number [0..32767], must be positive or zero */
    double covariance_lat_lon[2][2];   /**< Lattitude/Longitude covariance matrix in column order [rad^2]\n
                                        *  cov(lat,lat), cov(lon,lat)\n
                                        *  cov(lat,lon), cov(lon,lon) */
    double azimuth_std;     /**< azimuth standard deviation [deg] */
    double floor_std;       /**< altitude standard deviation [floor] */
    bool is_valid;          /**< position validity flag */
} CPosition;

/** Attitude information (relative to ENU?) */
typedef struct CAttitudeTag
{
    double quaternion[4];   /**< orientation quaternion [q0, q1, q2, q3] */
    double covariance_quaternion[4][4]; /**< quaternion covariance matrix, column order\n
                                        * cov(q0, q0), cov(q1, q0), cov(q2, q0), cov(q3, q0)\n
                                        * cov(q0, q1), cov(q1, q1), cov(q2, q1), cov(q3, q1)\n
                                        * cov(q0, q2), cov(q1, q2), cov(q2, q2), cov(q3, q2)\n
                                        * cov(q0, q3), cov(q1, q3), cov(q2, q3), cov(q3, q3) */
	bool is_valid;           /**< orientation validity flag */
} CAttitude;

typedef uint64_t cvenue_id; /**<  unique venue identifier type */

/** Coordinates increment*/
typedef struct CCoordinatesIncrementTag
{
    int64_t timestamp;          /**< UNIX time [ms] */
    double d_x;                 /**< x-coordinate increment [m] */
    double d_y;                 /**< y-coordinate increment [m] */
    double d_floor;             /**< z-axis increment [floor] */
    double covariance_yx[2][2]; /**< increment covariance matrix in column order [m^2]\n
                                    *  cov(x,x), cov(y,x)\n
                                    *  cov(x,y), cov(y,y) */
    double d_floor_std;         /**< z-axis standard deviation [floor] */

    CAttitude attitude;         /**< device orientation relative start position */
    //bool is_step;             - temporary commented  /**< true if step detected */
    bool is_motion;             /**< true if it is moving */
    bool is_transit;            /**< true if it is transition with portal */

} CCoordinatesIncrement;

typedef CMagneticData CMagneticCalibrationParam; /**< magnetic calibration parameters (bias with cov matrix) */

typedef uint64_t CBSSID; /**< mac address in decimal form */

/** common rssi measurement*/
typedef struct CRSSIMeasurementTag
{
    int64_t timestamp;  /**< unix time [us] */
    CBSSID mac;          /**< MAC addres in decimal form */
    int8_t rssi;        /**< RSSI value [dbm] */
    uint16_t frequency; /**< central channel frequency [MHz] */
} CRSSIMeasurement;


/** WiFi measurement */
typedef CRSSIMeasurement CWiFiMeasurement;

/** BLE measurement (iBeacon) */
typedef struct CBleMeasurementTag
{
    int64_t timestamp;  /**< unix time [us] */
    CBSSID mac;          /**< MAC addres in decimal form */
    int8_t rssi;        /**< RSSI value [dbm] */
    uint16_t frequency; /**< central channel frequency [MHz] */

    uint16_t major;     /**< iBeacon major number */
    uint16_t minor;     /**< iBeacon minor number */
    uint8_t uuid[16];   /**< iBeacon uuid 128 bit value */
    int8_t txPower;     /**< iBeacon tx power level [dbm] on 1m distance */
    
    bool hasMAC;        /**< mac address avaliability flag*/

    uint8_t proximity;   /**< Android: not defined; iOS: proximity metrics*/
    uint8_t accuracy;    /**< Android: not defined; iOS: proximity accuracy , [decemeters]*/
} CBleMeasurement;

/** WiFi scan result*/
typedef struct CWiFiScanResultTag
{
    int64_t timestamp; /**< UNIX time in [ms] */
	CWiFiMeasurement* scanWiFi; /**< WiFi observation */
	uint16_t n_scans_wifi; /**<Number of WiFi measurements */
} CWiFiScanResult;

/** BLE scan result*/
typedef struct CBleScanResultTag
{
    int64_t timestamp; /**< UNIX time in [ms] */
    CBleMeasurement* scanBle; /**< BLE observation */
	uint16_t n_scans_ble; /**<Number of BLE measurements */
} CBleScanResult;

/** Particle is used for debug and visualization pirposes */
typedef struct CParticleTag
{
    double x;
    double y;
    double z;
    double w;
} CParticle;

/** proximity beacon data structure*/
typedef BleBeaconData CProximityBeaconData;

/**
* Callback interfaces
*/

/**
* Position calback with particle cloud 
* \param[in] X,Y coordinates in a local frame [m]
* \param[in] Floor floor level
* \param[in] H heading [rad]
* \param[in] Sig estimated position deviation
* \param[in] t timestamp [ms]
* \param[in] state pointer to the particles array
* \param[in] N particles count
*/
typedef void(*CIPositionUpdate_vars_ptr)(void*, double, double, double, double, double, double, const CParticle*, int);

/**
* Position calback 
*/
typedef void(*CIPositionUpdate_struct_ptr)(void*, const CPosition*);

/**
* Venue detection calback
*/
typedef void(*CIVenueDetectionUpdate_vars_ptr)(void*, bool, double);

/**
* Extended proximity callback
*/
typedef void(*CIExtendedProximityUpdate_struct_ptr)(void*, double, const CProximityBeaconData *);


typedef struct CIPositionUpdateTag
{
    CIPositionUpdate_vars_ptr update_by_vars;
    CIPositionUpdate_struct_ptr update_by_struct;

    void* object_handle; // optional pointer to arbitrary structure to be used by other functions
} CIPositionUpdate;

typedef struct CIVenueDetectionUpdateTag
{
    CIVenueDetectionUpdate_vars_ptr update_by_vars;

    void* object_handle; // optional pointer to arbitrary structure to be used by other functions
} CIVenueDetectionUpdate;

typedef struct CIExtendedProximityUpdateTag
{
    CIExtendedProximityUpdate_struct_ptr update_by_struct;

    void* object_handle; // optional pointer to arbitrary structure to be used by other functions
} CIExtendedProximityUpdate;

typedef struct CICollaborationDataTag
{
	int64_t timestamp;       /**< unix time [ms] */
	double  lattitude;       /**< lattitude [deg] [-90..+90]*/
	double  longitude;       /**< longitude [deg] [-180..+180] */
	double  cov_ne[2][2];    /**< local NE-coordinates covariance matrix [m^2]*/
	bool is_position_valid;  /**< position validity flag */

	double floor_number;     /**< discrete physical floor number*/
	double  floor_std;       /**< floor standard deviation [floor], the field is unavailable if it is negative */
	bool is_floor_valid;     /**< position validity flag */

	double  distance;             /**< distance to collaborative unit [m]  */
	double  distance_uncertainty; /**< distance uncertainty [m], the field is unavailable if it is negative */
	bool is_distance_valid;       /**< position validity flag */
	uint64_t BSSID;               /**< device ID */
} CICollaborationData;

typedef struct CIPositionCallbackObjectTag CIPositionCallbackObject;
DLL_EXPORT CIPositionCallbackObject* PositionCallbackObject_new(const CIPositionUpdate posCbk);
DLL_EXPORT void PositionCallbackObject_free(CIPositionCallbackObject *positionCallback);

typedef struct CIVenueDetectionCallbackObjectTag CIVenueDetectionCallbackObject;
DLL_EXPORT CIVenueDetectionCallbackObject* VenueDetectionCallbackObject_new(const CIVenueDetectionUpdate venueCbk);
DLL_EXPORT void VenueDetectionCallbackObject_free(CIVenueDetectionCallbackObject *venueDetectionCallback);

typedef struct CIExtendedProximityCallbackObjectTag CIExtendedProximityCallbackObject;
DLL_EXPORT CIExtendedProximityCallbackObject* ExtendedProximityCallbackObject_new(const CIExtendedProximityUpdate extProxCbk);
DLL_EXPORT void ExtendedProximityCallbackObject_free(CIExtendedProximityCallbackObject *extProxCbkObj);

typedef struct CILoggerTag CILogger;
DLL_EXPORT CILogger *CLogger_new(FILE *f);
DLL_EXPORT void CLogger_free(CILogger *logger);
/**
* FPEE Enging interface
*/

/** Forward declaration */
typedef struct CIFPEngineTag CIFPEngine;
/** default constructor*/
DLL_EXPORT CIFPEngine* FPEngine_new();
/** destructor*/
DLL_EXPORT void FPEngine_free(CIFPEngine* fpEngine);

/** \return version info*/
DLL_EXPORT CVersionNumber FPEngine_getVersionNumber(CIFPEngine* fpEngine);


/**
* Sets output for corresponding log
* param[in] pfEngine - PFEngine object
* param[in] logger - logger object
* param[in] id - log identifier
* return succes status
*/
DLL_EXPORT bool FPEngine_setLogger(CIFPEngine* fpEngine, CILogger* logger, const unsigned int id);

/**
* Sets output for corresponding log
* param[in] pfEngine - PFEngine object
* return logs count
*/
DLL_EXPORT int FPEngine_getLogsCount(CIFPEngine* fpEngine);

/**
* Return output for corresponding log
* param[in] pfEngine - PFEngine object
* param[out] log_descr - description
* param[in] size - description array size
* param[in] id - log id
* return number of witten characters include null-character
*/
DLL_EXPORT int FPEngine_getLogDescription(CIFPEngine* fpEngine, char* log_descr, size_t size, unsigned int id);

/**
* partially resets internal object state
*/
DLL_EXPORT void FPEngine_restart(CIFPEngine* fpEngine);

/**
* main method, process inertial sensors data and initiates processing pipeline
* used for debug purposes
* \param[in] increment position increments in local frame
*/
DLL_EXPORT void FPEngine_processIncrements(CIFPEngine* fpEngine, const CCoordinatesIncrement* increment);


/**
* main method, process inertial sensors data and initiates processing pipeline
* \param[in] tpn_output position and attitude information in tpn like format
*/
DLL_EXPORT void FPEngine_processTpnOutput(CIFPEngine* fpEngine, const CTpnOutput* tpn_output);

/**
* main method, process inertial sensors data and initiates processing pipeline
* \param[in] tpn_output position and attitude information in tpn like format
* \param[in] tpn_pos_extra additional position information
*/
DLL_EXPORT void FPEngine_processTpnOutputWithExtraInformation(CIFPEngine* fpEngine, const CTpnOutput* tpn_output, CTpnPositionExtra* tpn_pos_extra);

/**
* pushes WiFi measurement into the processing pipeline
* \param[in] scan_wifi WiFi measurements vector with timestamp
*/
DLL_EXPORT void FPEngine_processWiFi(CIFPEngine* fpEngine, const CWiFiScanResult* scan_wifi);

/**
* pushes BLE measurement into the processing pipeline
* \param[in] scan_ble BLE measurements vector with timestamp
*/
DLL_EXPORT void FPEngine_processBLE(CIFPEngine* fpEngine, const CBleScanResult* scan_ble);

/**
* pushes Magnetic measurement into the processing pipeline
* \param[in] mag_data magnetic vector with timestamp
*/
DLL_EXPORT void FPEngine_processMFP(CIFPEngine* fpEngine, const CMagneticVector* mag_data, int64_t timestamp);

/**
* pushes external location measurement into the processing pipeline
* \param[in] position external position
*/
DLL_EXPORT void FPEngine_processExternalPosition(CIFPEngine* fpEngine, const CPosition* position);

/**
* pushes collaboration data into the processing pipeline
* \param[in] collaboration_position - an array of collaboration positions
* \param[in] size - size of array of collaboration positions 
*/
DLL_EXPORT void FPEngine_processInputCollaboration(CIFPEngine* fpEngine, CICollaborationData* collaboration_position, size_t size);

/**
* calculates number of proximity beacons of specified type on specified floor in BLP-DB
* \param[in] floor - phisical floor number
* \param[in] beacon_type - beacon type
* \param[out] number of found beacons or -1 if BLP-DB is not initializaed or -2 if venue is not initialized
*/
DLL_EXPORT int FPEngine_getProximityBeaconsNumber(CIFPEngine* fpEngine, const int16_t floor, const BleBeaconType beacon_type);

/**
* Sets main filter position callback
* \param[in] pPosCbk pointer to the callback implementation
*/
DLL_EXPORT void FPEngine_setPositionCallbackMixed(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj);

/**
* Sets WiFi only position callback
* \param[in] pPosCbk pointer to the callback implementation
*/
DLL_EXPORT void FPEngine_setPositionCallbackWiFi(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj);

/**
* Sets BLE only position callback
* \param[in] pPosCbk pointer to the callback implementation
*/
DLL_EXPORT void FPEngine_setPositionCallbackBLE(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj);

/**
* Sets BLE proximity only position callback
* \param[in] pPosCbk pointer to the callback implementation
*/
DLL_EXPORT void FPEngine_setPositionCallbackBLEProximity(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj);

/**
* Sets MFP only(mfp+pdr) position callback
* \param[in] pPosCbk pointer to the callback implementation
*/
DLL_EXPORT void FPEngine_setPositionCallbackMFP(CIFPEngine* fpEngine, CIPositionCallbackObject *pPosCbkObj);

/**
* Sets extended proximity callback
* \param[in] CIExtendedProximityCallbackObject pointer to the callback implementation
*/
DLL_EXPORT void FPEngine_setExtendedProximityCallback(CIFPEngine* fpEngine, CIExtendedProximityCallbackObject *pExtProxCbkObj);

/**
* Sets WiFi venue detection position callback
* \param[in] pVenueWiFiCbkObj pointer to the callback implementation
*/
DLL_EXPORT void FPEngine_setVenueDetectionCallbackWiFi(CIFPEngine* fpEngine, CIVenueDetectionCallbackObject *pVenueWiFiCbkObj);

/**
* initialize WiFi module and loads fingerprint from a specified buffer
* this is old-interface, just wifi3 format is supported only
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] pWiFiMap - WiFi map buffer pointer
* \param[in] wifiFileSizeInBytes - WiFi map buffer size
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
DLL_EXPORT CReturnStatus FPEngine_initializeWiFI(CIFPEngine* fpEngine, const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p);

/**
* initialize WiFi module and loads fingerprint from specified buffer
* wifi3, wifi4 formats are supported
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] pWiFiMap - pointer of WiFi FP buffer (includeing header)
* \param[in] wifiFileSizeInBytes - WiFi FP buffer size (includeing header)
* \return success status
*/
DLL_EXPORT CReturnStatus FPEngine_initializeWiFI_Ex(CIFPEngine* fpEngine, const char* const pWiFiMap, const size_t wifiFileSizeInBytes);

/**
* initialize BLE module and loads fingerprint from a specified buffer
* this is old-interface, just ble3 format is supported only
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] pBleMap - BLE map buffer pointer
* \param[in] bleFileSizeInBytes - BLE map buffer size
* \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
* \return success status
*/
DLL_EXPORT CReturnStatus FPEngine_initializeBLE(CIFPEngine* fpEngine, const char* const pBleMap, const size_t bleFileSizeInBytes, const double min_p);

/**
* initialize BLE module and loads fingerprint from a specified buffer
* ble3, ble4 formats are supported
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] pBleMap - BLE map buffer pointer (includeing header)
* \param[in] bleFileSizeInBytes - BLE map buffer size (includeing header)
* \return success status
*/
DLL_EXPORT CReturnStatus FPEngine_initializeBLE_Ex(CIFPEngine* fpEngine, const char* const pBleMap, const size_t bleFileSizeInBytes);

/**
* initialize BLE proximity module and loads proximity DB from a specified buffer
* ble3, ble4 formats are supported
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] pBleMap - BLE map buffer pointer
* \param[in] bleFileSizeInBytes - BLE map buffer size
* \return success status
*/
DLL_EXPORT CReturnStatus FPEngine_initializeBLEProximity(CIFPEngine* fpEngine, const char* const pBleMap, const size_t bleFileSizeInBytes);

/**
* initialize MFP module and loads fingerprint from a specified buffer
* this is old-interface, mfp3 format is supported only
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] pMFPMap - MFP map buffer pointer
* \param[in] mfpFileSizeInBytes - MFP map buffer size
* \param[in] max_X defines fingerprint size on X axis [m] [TODO  to be moved into FP file]
* \param[in] max_Y defines fingerprint size on Y axis [m] [TODO  to be moved into FP file]
* \param[in] cellSize defines internal map discrete [m] [TODO  to be moved into FP file]
* \param[in] minFloor define minimum floor number [TODO  to be moved into FP file]
* \param[in] maxFloor define maximum floor number [TODO  to be moved into FP file]
* \return success status
*/
DLL_EXPORT CReturnStatus FPEngine_initializeMFP(CIFPEngine* fpEngine, const char* const pMFPMap, const size_t mfpFileSizeInBytes, const double max_X, const double max_Y, const double cellSize, const int minFloor, const int maxFloor);

/**
* initialize MFP module and loads fingerprint from a specified buffer
* mfp4 format is supported
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] pMFPMap - MFP FP buffer pointer (includeing header)
* \param[in] mfpFileSizeInBytes - MFP FP buffer size (includeing header)
* \return success status
*/
DLL_EXPORT CReturnStatus FPEngine_initializeMFP_Ex(CIFPEngine* fpEngine, const char* const pMFPMap, const size_t mfpFileSizeInBytes);

/**
* initialize Map Matching module and loads map from  the specified memory buffer
* \param[in] pMap map buffer
* \param[in] mapFileSizeInBytes size of the buffer in bytes
* \return success status
*/
DLL_EXPORT CReturnStatus FPEngine_initializeMapMatching(CIFPEngine* fpEngine, const uint8_t* const pMap, const size_t mapFileSizeInBytes);

/**
* Set venue parameters (local frame, size) - fp3 interface
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] venue  venue parameters - fp3 interface
* \return success
*/
DLL_EXPORT CReturnStatus FPEngine_setVenueParams(CIFPEngine* fpEngine, const Venue* venue);

/**
* Set venue parameters (local frame, size)
* \param[in] fpEngine - CIFPEngine instance pointer
* \param[in] venue  venue parameters 
* \return success
*/
DLL_EXPORT CReturnStatus FPEngine_setVenueParamsEx(CIFPEngine* fpEngine, const BaseVenueType* venue);

/**
* Get venue parameters
* \param[in] fpEngine - CIFPEngine instance pointer
* \return structure BaseVenueType wich contains current venue parameters
*/
DLL_EXPORT BaseVenueType FPEngine_getVenueParamsEx(CIFPEngine* fpEngine);

/**
* Get WiFi fingerprint info
*\param[in] fpEngine - CIFPEngine instance pointer
* \return structure FPHeaderBaseType with WiFi fingerprint information
*/
DLL_EXPORT FPHeaderBaseType FPEngine_getWfpInfo(CIFPEngine* fpEngine);

/**
* Get magnetic fingerprint info
*\param[in] fpEngine - CIFPEngine instance pointer
* \return structure FPHeaderBaseType with magnetic fingerprint information
*/
DLL_EXPORT FPHeaderBaseType FPEngine_getMfpInfo(CIFPEngine* fpEngine);

/**
* Get BLE fingerprint info
* \param[in] fpEngine - CIFPEngine instance pointer
* \return structure FPHeaderBaseType with BLE fingerprint information
*/
DLL_EXPORT FPHeaderBaseType FPEngine_getBfpInfo(CIFPEngine* fpEngine);

/**
* Get Bluetooth proximity DB info
* \param[in] fpEngine - CIFPEngine instance pointer
* \return Bluetooth proximity DB info
*/
DLL_EXPORT FPHeaderBaseType FPEngine_getProximityDbInfo(CIFPEngine* fpEngine);

/** updater WiFi control
* \param[in] enable set wiFi update enable/disable
*/
DLL_EXPORT void FPEngine_setUpdateWiFi(CIFPEngine* fpEngine, bool enable);

/** updater BLE control
* \param[in] enable set BLE update enable/disable
*/
DLL_EXPORT void FPEngine_setUpdateBLE(CIFPEngine* fpEngine, bool enable);

/** updater BLE proximity control
* \param[in] enable set BLE update enable/disable
*/
DLL_EXPORT void FPEngine_setUpdateBLEProximity(CIFPEngine* fpEngine, bool enable);

/** updater MFP control
* \param[in] enable set magnetic update enable/disable
*/
DLL_EXPORT void FPEngine_setUpdateMFP(CIFPEngine* fpEngine, bool enable);

/** updater Map Matching control
*\ param[in] enable control flag
*/
DLL_EXPORT void FPEngine_setUpdateMapMatching(CIFPEngine* fpEngine, bool enable); /**< updater Map Matching control */

/** updater External Pos control
* \param[in] enable set Pos update enable/disable
*/
DLL_EXPORT void FPEngine_setUpdateExternalPosition(CIFPEngine* fpEngine, bool enable);

/** updater Collaboration control
* \param[in] enable set Pos update enable/disable
*/
DLL_EXPORT void FPEngine_setUpdateCollaboration(CIFPEngine* fpEngine, bool enable);

/** Corrector fusion filter position control
* \param[in] enable set Pos update enable/disable
*/
DLL_EXPORT void FPEngine_setCorrectFusionFilterPosition(CIFPEngine* fpEngine, bool enable);

/** Enables floor increment usage, switches to another motion model */
DLL_EXPORT void FPEngine_setFloorIncrements(CIFPEngine* fpEngine, bool enable);

/** Gets mg calibration params
* \param[out] bias_cov esimated magnetic bias with covariance
*/
DLL_EXPORT bool FPEngine_getMagneticBias(CIFPEngine* fpEngine, CMagneticCalibrationParam *bias_cov);

/** Gets mg calibration params
* \param[in] bias_cov initial magnetic bias with covariance
*/
DLL_EXPORT void FPEngine_setMagneticBias(CIFPEngine* fpEngine, const CMagneticCalibrationParam* bias_cov);

/** Gets platform type
* \return platform type
*/
DLL_EXPORT PlatformType FPEngine_getPlatformType(CIFPEngine* fpEngine);

/** Sets platform type
* \param[in] platform type
*/
DLL_EXPORT void FPEngine_setPlatformType(CIFPEngine* fpEngine, PlatformType platform_type);

/** Sets BLP pulling type
* \param[in] pulling type
* \param[in] pulling distance
*/
DLL_EXPORT void FPEngine_setBlpPulling(CIFPEngine* fpEngine, ePullingType type, double pulling_distance);

DLL_EXPORT void FPEngine_setBlpDetectionEnable(CIFPEngine* fpEngine, bool enable);

DLL_EXPORT void FPEngine_setBlpPositioningPdFilterParams(CIFPEngine* fpEngine,
    int peak_detector_max_delay_ms_in_moving,
    double descending_factor_in_moving,
    int peak_detector_max_delay_ms_in_stop,
    double descending_factor_in_stop
);

DLL_EXPORT void FPEngine_setBlpDetectionPdFilterParams(CIFPEngine* fpEngine,
    int peak_detector_max_delay_ms_in_moving,
    double descending_factor_in_moving,
    int peak_detector_max_delay_ms_in_stop,
    double descending_factor_in_stop
);

DLL_EXPORT void FPEngine_setBlpPositioningLogicParams(CIFPEngine* fpEngine, 
    int filter_length, int repeat_number, int cutoff);

DLL_EXPORT void FPEngine_setBlpDetectionLogicParams(CIFPEngine* fpEngine, 
    int filter_length, int repeat_number, int cutoff);

/** gets Wi-Fi bias
* \param[out] bias rssi bias
* returns ReturnStatus::STATUS_SUCCESS if success, otherwise  error code
*/
DLL_EXPORT CReturnStatus FPEngine_getWiFiBias(CIFPEngine* fpEngine, double *bias);
/** sets WiFi bias
* param[in] bias initial rssi bias
* param[in] delta_t time since last saved bias
*/
DLL_EXPORT void FPEngine_setWiFiBias(CIFPEngine* fpEngine, const double*  bias, int64_t delta_t);

/** gets BLE bias
* \param[out] bias rssi bias
* returns ReturnStatus::STATUS_SUCCESS if success, otherwise  error code
*/
DLL_EXPORT CReturnStatus FPEngine_getBLEBias(CIFPEngine* fpEngine, double *bias);
/** sets BLE bias
* param[in] bias initial rssi bias
* param[in] delta_t time since last saved bias
*/
DLL_EXPORT void FPEngine_setBLEBias(CIFPEngine* fpEngine, const double*  bias, int64_t delta_t);

/** gets BLE bias and BLE bias uncertainty
* \param[out] bias - RSSI bias
* \param[out] bias_uncertainty - RSSI bias uncertainty
* returns ReturnStatus::STATUS_SUCCESS if success, otherwise  error code
*/
DLL_EXPORT CReturnStatus FPEngine_getBLEBiasWithUncertainty(CIFPEngine* fpEngine, double *bias, double *bias_uncertainty);

/** sets BLE bias and bias uncertainty
* param[in] bias - RSSI bias
* param[in] bias_uncertainty - RSSI bias uncertainty
* param[in] bias_age - time since last saved bias [ms]
*/
DLL_EXPORT void FPEngine_setBLEBiasWithUncertainty(CIFPEngine* fpEngine, const double*  bias, const double *bias_uncertainty, int64_t bias_age);

/**
* Sets fine known initial position
* \param[in] position initial position in global frame
*/
DLL_EXPORT void FPEngine_setStartPosition(CIFPEngine* fpEngine, const CPosition* position);

/**
* Sets random seeds for internal randomizers
*/
DLL_EXPORT void FPEngine_setRandomSeeds(CIFPEngine* fpEngine);

/**
* set use barometr
* param[in] enable/disable
*/
DLL_EXPORT   void FPEngine_setUseBarometer(CIFPEngine* fpEngine, bool enable);

/**
* set OS type
* param[in] OS type
*/
DLL_EXPORT   void FPEngine_setOsType(CIFPEngine* fpEngine, OperationSystemType os_type);

/**
* Control Mag PF
* \param[in] enable disables/enables Mag filter
*/
DLL_EXPORT void FPEngine_setMagFilterEnabled(CIFPEngine* fpEngine, bool enable);

/**
* Control Mixed PF
* \param[in] enable disables/enables Mixed filter
*/
DLL_EXPORT void FPEngine_setMixedFilterEnabled(CIFPEngine* fpEngine, bool enable);

/**
* Control Mag PF
* \return Mag filter state
*/
DLL_EXPORT bool FPEngine_getMagFilterEnabled(CIFPEngine* fpEngine);

/**
* Control Mixed PF
* \return Mixed filter state
*/
DLL_EXPORT bool FPEngine_getMixedFilterEnabled(CIFPEngine* fpEngine);


FPPE_END_DECLS

#endif //C_FPPE_H

