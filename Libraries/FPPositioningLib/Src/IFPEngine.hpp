#ifndef IFPENGINE_HPP
#define IFPENGINE_HPP
#include "Fppe.hpp"
#include "fpHeader.h"

class Fppe::IFPEngine
{
    public:
        IFPEngine()
        {
            ;    ///< default constructor
        }
        virtual ~IFPEngine()
        {
            ;    ///< destructor
        }

        /**
        * resets internal object state
        * \param[in] filename output log filename
        */
        virtual void setLogFile( const std::string &filename ) = 0;

        /**
        * enables logging
        * \param[in] enabled control flag
        */
        virtual void setLogEnabled( const bool &enabled ) = 0;

        /**
        * resets internal object state, disables logging
        */
        virtual void restart() = 0;

        /**
        * main method, process inertial sensors data and initiates processing pipeline
        * \param[in] increment position increments in local frame
        */
        virtual void processIncrements( const Fppe::CoordinatesIncrement &increment ) = 0;

        /**
        * main method, process inertial sensors data and initiates processing pipeline
        * \param[in] tpn_output position and attitude information it tpn like format
        */
        virtual void processTpnOutput( const TpnOutput &tpn_output) = 0;

        /**
        * pushes WiFi measurement into the processing pipeline
        * \param[in] scan_wifi WiFi measurements vector with timestamp
        */
        virtual void processWiFi( const Fppe::WiFiScanResult &scan_wifi ) = 0;

        /**
        * pushes BLE measurement into the processing pipeline
        * \param[in] scan_ble BLE measurements vector with timestamp
        */
        virtual void processBLE( const Fppe::BleScanResult &scan_ble ) = 0;

        /**
        * calculates number of proximity beacons of specified type on specified floor in BLP-DB
        * \param[in] floor - phisical floor number
        * \param[in] beacon_type - beacon type
        * \param[out] number of found beacons or -1 if BLP-DB is not initializaed or -2 if venue is not initialized
        */
        virtual int getProximityBeaconsNumber(const int16_t floor, const BleBeaconType beacon_type) = 0;

        /**
        * pushes Magnetic measurement into the processing pipeline
        * \param[in] mag_data magnetic vector with timestamp
        */
        virtual void processMFP( const MagneticVector &mag_data, int64_t timestamp ) = 0;

        /**
        * pushes external location measurement into the processing pipeline
        * \param[in] pos external position
        */
        virtual void processExternalPosition( const Fppe::Position &external_pos ) = 0;

        /**
       * pushes ble_collaboration into the processing pipeline
       * \param[in] ble_collaboration position
       */
        virtual void processInputCollaboration(std::vector <Fppe::CollaborationData> &collaboration_position) = 0;

        /**
        * Sets main filter position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        virtual void setPositionCallbackMixed( Fppe::IPositionUpdate *pPosCbk ) = 0;

        /**
        * Sets WiFi only position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        virtual void setPositionCallbackWiFi( Fppe::IPositionUpdate *pPosCbk ) = 0;

        /**
        * Sets BLE only position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        virtual void setPositionCallbackBLE( Fppe::IPositionUpdate *pPosCbk ) = 0;

        /**
        * Sets BLE proximity only position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        virtual void setPositionCallbackBLEProximity( Fppe::IPositionUpdate *pPosCbk ) = 0;

        /**
        * Sets BLE proximity position callback with extended data
        * param[in] pPosCbk pointer to the callback implementation
        */
        virtual void setExtendedProximityCallback(Fppe::IExtendedProximityUpdate *pProximityCbk) = 0;

        /**
        * Sets MFP only(mfp+pdr) position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        virtual void setPositionCallbackMFP( Fppe::IPositionUpdate *pPosCbk ) = 0;

        /**
        * Sets venue detection WiFi callback
        * param[in] pVenueDetectCbk pointer to the callback implementation
        */
        virtual void setVenueDetectionCallbackWiFi(Fppe::IVenueDetectionUpdate *pVenueDetectCbk) = 0;

        /**
        * initialize WiFi module and loads fingerprint from  the file
        * \param[in] wifi_db_name fingerprint filename
        * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
        * \return success status
        */
        virtual Fppe::ReturnStatus initializeWiFI( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p ) = 0;

        /**
        * initialize WiFi module and loads fingerprint from  the specified memory buffer
        * \param[in] pWiFiMap fingerprint buffer
        * \param[in] wifiFileSizeInBytes size of the buffer in bytes
        * \return success status
        */
        virtual Fppe::ReturnStatus initializeWiFI(const char* const pWiFiMap, const size_t wifiFileSizeInBytes) = 0; // new interface

        /**
        * get information about WFP DB
        * \return FPHeader structure
        */
        virtual FPHeaderBaseType getWfpInfo() = 0;

        /**
        * initialize BLE module and loads fingerprint from the file
        * \param[in] ble_db_name fingerprint filename
        * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
        * \return success status
        */
        virtual Fppe::ReturnStatus initializeBLE( const char* const pBleMap, const size_t bleFileSizeInBytes, const double min_p ) = 0;

        /**
        * initialize BLE module and loads fingerprint from  the specified memory buffer
        * \param[in] pBleMap fingerprint buffer
        * \param[in] bleFileSizeInBytes size of the buffer in bytes
        * \return success status
        */
        virtual Fppe::ReturnStatus initializeBLE(const char* const pBleMap, const size_t bleFileSizeInBytes) = 0; // new interface

        /**
        * initialize BLE proximity module and loads fingerprint from the file
        * \param[in] ble_db_name fingerprint filename
        * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
        * \return success status
        */
        virtual Fppe::ReturnStatus initializeBLEProximity( const char* const pBleMap, const size_t bleFileSizeInBytes) = 0;

        /**
        * get information about BFP DB
        * \return FPHeader structure
        */
        virtual FPHeaderBaseType getBfpInfo() = 0;

        /**
        * get information about Bluetooth proximity DB
        * \return FPHeader structure
        */
		virtual FPHeaderBaseType getProximityDbInfo() = 0;

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
        virtual Fppe::ReturnStatus initializeMFP( const char* const pMFPMap, const size_t mfpFileSizeInBytes, double max_X, double max_Y, double cellSize, int minFloor, int maxFloor ) = 0;

        /**
        * initialize MFP module and loads fingerprint from  the specified memory buffer
        * \param[in] pMFPMap fingerprint buffer
        * \param[in] mfpFileSizeInBytes size of the buffer in bytes
        * \return success status
        */
        virtual Fppe::ReturnStatus initializeMFP(const char* const pMFPMap, const size_t mfpFileSizeInBytes) = 0; // new interface

        /**
        * get information about MFP DB
        * \return FPHeader structure
        */
        virtual FPHeaderBaseType getMfpInfo() = 0;

		/**
			* initialize Map Matching module and loads map from the specified memory buffer
			* \param[in] pMap - pointer to array
			* \param[in] mapFileSizeInBytes - buffer size
			* \param[in] floor_shift - real number of lowest floor
			* \param[in] floor_zero_enable
			* \return success status
		*/
		virtual Fppe::ReturnStatus initializeMapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes) = 0;

        /**
        * Defines venue local frame
        * \param[in] venue  venue parameters with local frame origin
        * \return success
        */
        virtual Fppe::ReturnStatus setVenueParams( const Venue &venue ) = 0;
        
        virtual Fppe::ReturnStatus setVenueParams(const BaseVenueType &venue) = 0;

        /**
        * return venue params
        */
        virtual BaseVenueType getVenueParams() = 0;

        /**
        * set use barometr
        * param[in] enable/disable
        */
        virtual void setUseBarometer(bool enable) = 0;

        /**
        * set OS type
        * param[in] OS type
        */
        virtual void setOsType(OperationSystemType os_type) = 0;

        virtual void setUpdateWiFi( bool enable ) = 0; /**< updater WiFi control */
        virtual void setUpdateBLE( bool enable ) = 0;  /**< updater BLE control */
        virtual void setUpdateBLEProximity( bool enable ) = 0;  /**< updater BLE control */
        virtual void setUpdateMFP( bool enable ) = 0;  /**< updater MFP control */
        virtual void setUpdateMapMatching(bool enable) = 0; /**< updater Map Matching control */
        virtual void setUpdateExternalPosition( bool enable ) = 0;  /**< updater External Position control */
        virtual void setUpdateCollaboration( bool enable ) = 0;  /**< updater BLE Position control */
        virtual void setCorrectFusionFilterPosition(const bool enable) = 0;

        /** Enables floor increment usage, switches to another motion model */
        virtual void setFloorIncrements( bool enable ) = 0;

        /** Gets mg calibration params
        * \param[out] bias_cov esimated magnetic bias with covariance
        */
        virtual bool getMagneticBias( Fppe::MagneticCalibrationParam *bias_cov ) = 0;

        /** Gets mg calibration params
        * \param[in] bias_cov initial magnetic bias with covariance
        */
        virtual void setMagneticBias( const Fppe::MagneticCalibrationParam &bias_cov ) = 0;

        /** Gets platform type
        * \return platform type
        */
        virtual PlatformType getPlatformType() = 0;

        /** Sets platform type
        * \param[in] platform type
        */
        virtual void setPlatformType(PlatformType platform_type) = 0;

        /** Sets BLP pulling type
        * \param[in] pulling type
        * \param[in] pulling distance
        * \param[in] pulling sigma
        */
        virtual void setBlpPulling(ePullingType type, double pulling_distance, 
            double pulling_sigma) = 0;

        virtual void setBlpDetectionEnable(bool enable) = 0;

        virtual void setBlpPositioningPdFilterParams(
            int peak_detector_max_delay_ms_in_moving,
            double descending_factor_in_moving,
            int peak_detector_max_delay_ms_in_stop,
            double descending_factor_in_stop
        ) = 0;

        virtual void setBlpDetectionPdFilterParams(
            int peak_detector_max_delay_ms_in_moving,
            double descending_factor_in_moving,
            int peak_detector_max_delay_ms_in_stop,
            double descending_factor_in_stop
        ) = 0;

        virtual void setBlpPositioningLogicParams(int filter_length, int repeat_number, 
            int cutoff) = 0;

        virtual void setBlpDetectionLogicParams(int filter_length, int repeat_number,
            int cutoff) = 0;

        /** WiFi calibration status [obsolete]
        * param[out] bias rssi bias
        * returns calibration enabled
        */
        virtual bool getWiFiBias( double *bias ) = 0;
        /** sets WiFi calibration [obsolete]
        * param[in] bias initial rssi bias
        * param[in] delta_t time since last saved bias
        */
        virtual void setWiFiBias( const double  &bias, int64_t delta_t ) = 0;

        /** BLE calibration status [obsolete]
        * param[out] bias rssi bias
        * returns calibration enabled
        */
        virtual bool getBLEBias(double *bias) = 0;
        /** sets BLE calibration [obsolete]
        * param[in] bias initial rssi bias
        * param[in] delta_t time since last saved bias
        */
        virtual void setBLEBias( const double  &bias, int64_t delta_t ) = 0;

		virtual bool getBLEBiasWithUncertainty(double *bias, double *bias_uncertainty) = 0;

		virtual void setBLEBiasWithUncertainty(const double  &bias, const double &bias_uncertainty, int64_t delta_t) = 0;

        /**
        * Sets fine known initial position
        * \param[in] position initial position in global frame
        */
        virtual void setStartPosition( const Fppe::Position &position ) = 0;

        /**
        * Sets rndom seeds for internal randomizers
        */
        virtual void setRandomSeeds() = 0;

        /**
        * Sets output stream for corresponding log id
        * param[in] idx log index
        * param[in] os output stream
        * return succes status
        */
        virtual bool setLogStream( const unsigned int &idx, std::ostream &os ) = 0;

        /**
        * Return vector of logs  brief descriptions
        * param[out] log_descriptions pointer to descrpitions
        */
        virtual void getLogDescription( std::vector<std::string> *log_descriptions ) = 0;


        /**
        * Control Mag PF
        * param[in] enable disables/enables Mag filter
        */
        virtual void setMagFilterEnabled( bool enable ) = 0;

        /**
        * Control Mixed PF
        * param[in] enable disables/enables Mixed filter
        */
        virtual void setMixedFilterEnabled( bool enable ) = 0;

        /**
        * return Mag filter state
        */
        virtual bool getMagFilterEnabled() = 0;

        /**
        * return Mixed filter state
        */
        virtual bool getMixedFilterEnabled() = 0;
};

#endif
