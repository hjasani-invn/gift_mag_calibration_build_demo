/**
 * \file Fppe.hpp
 * \brief Fingerprint Builder Library API
 * \author Mikhail Frolov (mfrolov@invensense.com)
 * \version 0.1
 * \copyright InvenSense, all rights reserved
 * \date February 29, 2016
 */

/*! \mainpage FPPE Library
* \section intro_sec Module 2 interaction
* \image latex Module2.pdf "Interaction diagram" width=14cm
*/

#ifndef FPPE_HPP
#define FPPE_HPP
#include <string>
#include <iostream>
#include "LibraryInfo.hpp"
#include "Venue.h"
#include "MagData.hpp"
#include "TpnData.hpp"
#include "fpHeader.h"
#include "fpeDataTypes.h"


#include <stdint.h>
#include <vector>

#ifdef _WIN32
#  define DLL_EXPORT __declspec( dllexport )
#else
#  define DLL_EXPORT
#endif

/** Fingerprint position engine */
namespace Fppe
{
    /**
     * operation status codes
     */
    enum class ReturnStatus
    {
        STATUS_SUCCESS = 0,            /**< operation success */
        STATUS_UNKNOWN_ERROR = -1,      /**< undefined error occurs due operation */
        STATUS_WRONG_DATA = -2,      /**< wrong data was provided */
        STATUS_UNKNOWN_FORMAT = -3      /**< unknown data format */
    };

    /** input coordinates description (in WGS84)*/
    struct Position
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
    };

    /** Attitude information (relative to ENU?) */
    struct Attitude
    {
        double quaternion[4];   /**< orientation quaternion [q0, q1, q2, q3] */
        double covariance_quaternion[4][4]; /**< quaternion covariance matrix, column order\n
                                            * cov(q0, q0), cov(q1, q0), cov(q2, q0), cov(q3, q0)\n
                                            * cov(q0, q1), cov(q1, q1), cov(q2, q1), cov(q3, q1)\n
                                            * cov(q0, q2), cov(q1, q2), cov(q2, q2), cov(q3, q2)\n
                                            * cov(q0, q3), cov(q1, q3), cov(q2, q3), cov(q3, q3) */
        bool is_valid;           /**< orientation validity flag */
    };

    typedef uint64_t venue_id; /**<  unique venue identifier type */

    /** Coordinates increment*/
    struct CoordinatesIncrement
    {
        int64_t timestamp;          /**< UNIX time [ms] */
        double d_x;                 /**< x-coordinate increment [m] */
        double d_y;                 /**< y-coordinate increment [m] */
        double d_floor;             /**< z-axis increment [floor] */
        double covariance_yx[2][2]; /**< increment covariance matrix in column order [m^2]\n
                                        *  cov(x,x), cov(y,x)\n
                                        *  cov(x,y), cov(y,y) */
        double d_floor_std;         /**< z-axis standard deviation [floor] */

        Attitude attitude;          /**< device orientation relative start position */
        bool is_motion;             /**< true if it is moving */
        // bool is_step;               /**< true if step detected */
         bool is_transit;            /*true if escalator or elevator or stairs*/
        //double mis;
        //double mis_p1;
        //double mis_p2;
        double mis_std;             /**< misaligment std [rad] */
        //double mis_p1_std;
        //double mis_p2_std;
    };

    typedef MagneticData MagneticCalibrationParam; /**< magnetic calibration parameters (bias with cov matrix) */

    typedef uint64_t BSSID; /**< mac address in decimal form */

    /** common rssi measurement*/
    struct RSSIMeasurement
    {
        RSSIMeasurement() : timestamp(0), mac(0), rssi(0), frequency(0) {};

        int64_t timestamp;  /**< unix time [us] */
        BSSID mac;          /**< MAC addres in decimal form */
        int8_t rssi;        /**< RSSI value [dbm] */
        uint16_t frequency; /**< central channel frequency [MHz] */
    };

    /** rssi measurement accompanied with uncertainties and bias*/
    struct RssiMeasCalibrated : public RSSIMeasurement
    {
        RssiMeasCalibrated() : rssi_sigma(1.), bias(0), bias_sigms(1.) {};
        
        RssiMeasCalibrated &operator=(const RSSIMeasurement& rssi_meas)
        {
            *this = RssiMeasCalibrated();
            RSSIMeasurement::operator=(rssi_meas);
            return *this;
        }

        double rssi_sigma;    /**< rssi noise dispersion [dBm]*/
        double bias;          /**< rssi bias [dBm]*/
        double bias_sigms;    /**< rssi dispersion [dBm]*/
    };

    /** WiFi measurement */
    typedef RSSIMeasurement WiFiMeasurement;

    /** transmitted BLE beacon data (iBeacon) */
    struct BleData
    {
        BleData() : txPower(0), hasMAC(false), uuid{}, proximity(0), accuracy(0)
        {
            this->major = this->minor = 0; //! it is imposible to define major and minor as default constructor parameters in GNU
        };

        uint16_t major;     /**< iBeacon major number */
        uint16_t minor;     /**< iBeacon minor number */
        uint8_t uuid[16];   /**< iBeacon uuid 128 bit value */
        int8_t txPower;     /**< iBeacon tx power level [dbm] on 1m distance */

        bool hasMAC;        /**< mac address avaliability flag*/

        uint8_t proximity;   /**< Android: not defined; iOS: proximity metrics*/
        uint8_t accuracy;    /**< Android: not defined; iOS: proximity accuracy , [decemeters]*/
    };

    /** BLE measurement (iBeacon) */
    struct BleMeasurement : public RSSIMeasurement, BleData {    };

    /** BLE measurement (iBeacon)  accompanied with uncertainties and bias*/
    struct BleMeasCalibrated : public RssiMeasCalibrated, BleData
    {
        
        BleMeasCalibrated &operator=(const BleMeasurement& ble_rssi_meas)
        {
            this->RssiMeasCalibrated::operator=(ble_rssi_meas);
            this->BleData::operator=(ble_rssi_meas);
            return *this;
        }
    };

    /** WiFi scan result*/
    struct WiFiScanResult
    {
        int64_t timestamp; /**< UNIX time in [ms] */
        std::vector<WiFiMeasurement> scanWiFi; /**< WiFi observation */
    };

    /** BLE scan result*/
    struct BleScanResult
    {
        int64_t timestamp; /**< UNIX time in [ms] */
        std::vector<BleMeasurement> scanBle; /**< BLE observation */
    };

    /** Particle is used for debug and visualization pirposes */
    struct Particle
    {
        double x;
        double y;
        double z;
        double w;
		int lkh;
    };

    struct CollaborationData
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
    };

    /**
    * Position callback interface
    */
    class IPositionUpdate
    {
        public:
            virtual ~IPositionUpdate()
            {
                ;
            }

            /**
            * this prototype is used in PC model to visualize particle cloud
            * \param[in] X,Y coordinates in a local frame [m]
            * \param[in] Floor floor level
            * \param[in] H heading [rad]
            * \param[in] Sig estimated position deviation
            * \param[in] t timestamp [ms]
            * \param[in] state pointer to the particles array
            * \param[in] N particles count
            */
            virtual void update( double X,  double Y, double Floor, double H, double Sig, double t, const Particle *state, int N ) = 0;

            /**
            * This method is called when new position estimation avaliable
            * \param[in]  position navigation solution
            */
            virtual void update( const Position &position ) = 0;
    };

    /**
    * Venue detection callback interface
    */
    class IVenueDetectionUpdate
    {
    public:
        virtual ~IVenueDetectionUpdate()
        {
            ;
        }

        /**
        * this prototype is used in PC model to visualize particle cloud
        * \param[in] pos_inside_venue - true if position is inside venue, false if outside
        * \param[in] t timestamp [ms]
        */
        virtual void update(bool pos_inside_venue, double t) = 0;
    };

    
    /** proximity beacon data structure*/
    typedef BleBeaconData ProximityBeaconData;

    /**
    * Extended Proximity callback interface
    */
    class IExtendedProximityUpdate
    {
    public:
        virtual ~IExtendedProximityUpdate()
        {
            ;
        }

        /**
        * This method is called when new proximity position estimation avaliable
        * \param[in]  proximity beacon data
        */
        virtual void update(double t, const ProximityBeaconData &proximity_beacon_data) = 0;

        /** 
        * reserved *
        * This method is called when new proximity position estimation avaliable
        * \param[in]  list of proximity beacons data
        */
        //virtual void update(double t, const std::initializer_list< ProximityBeaconData> &proximity_beacon_data) = 0;

    };

    /** Forward declaration */
    class IFPEngine;
    /** Main RTFPPL class. Integrates different modules to obtain fused navigation solution */
    class DLL_EXPORT FPEngine
    {
        public:
            /** default constructor*/
            FPEngine();
            /** destructor*/
            ~FPEngine();

            /** \return version info*/
            VersionNumber getVersionNumber() const;

            ///**
            //* resets internal object state
            //* \param[in] filename output log filename
            //*/
            //void setLogFile( const std::string &filename );

            ///**
            //* enables logging
            //* \param[in] enabled control flag
            //*/
            //void setLogEnabled( const bool &enabled );

            /**
            * resets internal object state, disables logging
            */
            void restart();

            /**
            * main method, process inertial sensors data and initiates processing pipeline
            * used for debug purposes
            * \param[in] increment position increments in local frame
            */
            void processIncrements( const CoordinatesIncrement &increment );


            /**
            * main method, process inertial sensors data and initiates processing pipeline
            * \param[in] tpn_output position and attitude information it tpn like format
            */
            void processTpnOutput( const TpnOutput &tpn_output);

            /**
            * pushes WiFi measurement into the processing pipeline
            * \param[in] scan_wifi WiFi measurements vector with timestamp
            */
            void processWiFi( const WiFiScanResult &scan_wifi );

            /**
            * pushes BLE measurement into the processing pipeline
            * \param[in] scan_ble BLE measurements vector with timestamp
            */
            void processBLE( const BleScanResult &scan_ble );

            /**
            * pushes Magnetic measurement into the processing pipeline
            * \param[in] mag_data magnetic vector with timestamp
            */
            void processMFP( const MagneticVector &mag_data, int64_t timestamp );

            /**
            * pushes external location measurement into the processing pipeline
            * \param[in] position external position
            */
            void processExternalPosition( const Position &position );

            /**
            * pushes collaboration into the processing pipeline
            * \param[in] collaboration position
            */
            void processInputCollaboration(std::vector <Fppe::CollaborationData> &collaboration_position);

            /**
            * calculates number of proximity beacons of specified type on specified floor in BLP-DB
            * \param[in] floor - phisical floor number
            * \param[in] beacon_type - beacon type
            * \param[out] number of found beacons or -1 if BLP-DB is not initializaed or -2 if venue is not initialized
            */
            int getProximityBeaconsNumber(const int16_t floor, const BleBeaconType beacon_type);


            /**
            * Sets main filter position callback
            * \param[in] pPosCbk pointer to the callback implementation
            */
            void setPositionCallbackMixed( IPositionUpdate *pPosCbk );

            /**
            * Sets WiFi only position callback
            * \param[in] pPosCbk pointer to the callback implementation
            */
            void setPositionCallbackWiFi( IPositionUpdate *pPosCbk );

            /**
            * Sets BLE only position callback
            * \param[in] pPosCbk pointer to the callback implementation
            */
            void setPositionCallbackBLE( IPositionUpdate *pPosCbk );

            /**
            * Sets BLE proximity only position callback
            * \param[in] pPosCbk pointer to the callback implementation
            */
            void setPositionCallbackBLEProximity(IPositionUpdate *pPosCbk);

            /**
            * Sets BLE proximity position with extended data callback
            * \param[in] pPosCbk pointer to the callback implementation
            */
            void setExtendedProximityCallback(IExtendedProximityUpdate *pProximityCbk);

            /**
            * Sets MFP only(mfp+pdr) position callback
            * \param[in] pPosCbk pointer to the callback implementation
            */
            void setPositionCallbackMFP( IPositionUpdate *pPosCbk );

            /**
            * Sets WiFi venue detection callback
            * \param[in] pVenueDetectCbk pointer to the callback implementation
            */
            void setVenueDetectionCallbackWiFi(IVenueDetectionUpdate *pVenueDetectCbk);

            /**
            * initialize WiFi module and loads fingerprint from  the array
            * \param[in] pWiFiMap fingerprint pointer to array
            * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
            * \return success status
            */
            ReturnStatus initializeWiFi( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p );
            
            /**
            * initialize WiFi module and loads fingerprint from  the array
            * \param[in] pWiFiMap fingerprint pointer to array
            * \param[in] wifiFileSizeInBytes fingerprint data size
            * \return success status
            */
            ReturnStatus initializeWiFi(const char* const pWiFiMap, const size_t wifiFileSizeInBytes);

            /**
            * initialize BLE module and loads fingerprint from the array
            * \param[in] pBleMap fingerprint pointer to array
            * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
            * \return success status
            */
            ReturnStatus initializeBLE( const char* const pBleMap, const size_t bleFileSizeInBytes, const double min_p );

            /**
            * initialize BLE module and loads fingerprint from the file
            * \param[in] ble_db_name fingerprint filename
            * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
            * \return success status
            */
            ReturnStatus initializeBLE(const char* const pBleMap, const size_t bleFileSizeInBytes);

            /**
            * initialize BLE proximity module and loads fingerprint from the file
            * \param[in] ble_db_name fingerprint filename
            * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
            * \return success status
            */
            ReturnStatus initializeBLEProximity(const char* const pBleMap, const size_t bleFileSizeInBytes);

            /**
            * initialize MFP module and loads fingerprint from the array
            * \param[in] pMFPMap fingerprint pointer to array
            * \param[in] max_X defines fingerprint size on X axis [m] [TODO  to be moved into FP file]
            * \param[in] max_Y defines fingerprint size on Y axis [m] [TODO  to be moved into FP file]
            * \param[in] cellSize defines internal map discrete [m] [TODO  to be moved into FP file]
            * \param[in] minFloor define minimum floor number [TODO  to be moved into FP file]
            * \param[in] maxFloor define maximum floor number [TODO  to be moved into FP file]
            * \return success status
            */
            
            ReturnStatus initializeMFP( const char* const pMFPMap, const size_t mfpFileSizeInBytes, const double max_X, const double max_Y, const double cellSize, const int minFloor, const int maxFloor );
            /**
            * initialize MFP module and loads fingerprint from the specified memory buffer
            * \param[in] pMFPMap fingerprint pointer to array
            * \param[in] mfpFileSizeInBytes buffer size
            * \return success status
            */
            ReturnStatus initializeMFP(const char* const pMFPMap, const size_t mfpFileSizeInBytes);

			/**
			* initialize Map Matching module and loads map from the specified memory buffer
			* \param[in] pMap - pointer to array
			* \param[in] mapFileSizeInBytes - buffer size
			* \param[in] cellSize - size of map matching cell [m]
			* \param[in] floor_shift - real number of lowest floor
			* \param[in] floor_zero_enable 
			* \return success status
			*/
			ReturnStatus initializeMapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes);
            
            /**
            * get information about WFP DB
            * \return FPHeader structure
            */
            FPHeaderBaseType getWfpInfo();

            /**
            * get information about MFP DB
            * \return FPHeader structure
            */
            FPHeaderBaseType getMfpInfo();
            
            /**
            * get information about BFP DB
            * \return FPHeader structure
            */
            FPHeaderBaseType getBfpInfo();

            /**
            * get information about proximity DB
            * \return FPHeader structure
            */
            FPHeaderBaseType getProximityDbInfo();

            /**
            * Defines venue local frame
            * \param[in] venue  venue parameters with local frame origin
            * \return success
            */
            ReturnStatus setVenueParams( const Venue &venue );

            /**
            * Defines venue local frame
            * \param[in] venue  venue parameters with local frame origin
            * \return success
            */
            ReturnStatus setVenueParams(const BaseVenueType &venue); // new venue interface

            /** updater WiFi control
            * \param[in] enable set wiFi update enable/disable
            */
            void setUpdateWiFi( const bool enable );

            /** updater BLE control
            * \param[in] enable set BLE update enable/disable
            */
            void setUpdateBLE( const bool enable );

            /** updater BLE proximity control
            * \param[in] enable set BLE update enable/disable
            */
            void setUpdateBLEProximity( const bool enable );

            /** updater MFP control
            * \param[in] enable set magnetic update enable/disable
            */
            void setUpdateMFP( const bool enable );

            /** updater Map Matching control
            *\ param[in] enable control flag
            */
            void setUpdateMapMatching(bool enable); /**< updater Map Matching control */

            /** updater External Pos control
            * \param[in] enable set Pos update enable/disable
            */
            void setUpdateExternalPosition( const bool enable );

            /** updater Collaboration Pos control
            * \param[in] enable set Pos update enable/disable
            */
            void setUpdateCollaboration(const bool enable);

            /** Corrector fusion filter position control
            * \param[in] enable set Pos update enable/disable
            */
            void setCorrectFusionFilterPosition(bool enable);

            /** Enables floor increment usage, switches to another motion model */
            void setFloorIncrements( const bool enable );

            /** Gets mg calibration params
            * \param[out] bias_cov esimated magnetic bias with covariance
            */
            bool getMagneticBias( MagneticCalibrationParam *bias_cov );

            /** Sets mg calibration params
            * \param[in] bias_cov initial magnetic bias with covariance
            */
            void setMagneticBias( const MagneticCalibrationParam &bias_cov );

            /** Gets platform type
            * \return platform type
            */
            PlatformType getPlatformType();

            /** Sets platform type
            * \param[in] platform type
            */
            void setPlatformType(PlatformType platform_type);

            /** Sets BLP pulling type
            * \param[in] pulling type
            * \param[in] pulling distance
            * \param[in] pulling sigma
            */
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

            /** WiFi calibration status [obsolete]
            * \param[out] bias rssi bias
            * returns ReturnStatus::STATUS_SUCCESS if success, otherwise  error code
            */
            ReturnStatus getWiFiBias( double *bias );
            /** sets WiFi calibration [obsolete]
            * param[in] bias initial rssi bias
            * param[in] delta_t time since last saved bias
            */
            void setWiFiBias( const double  &bias, int64_t delta_t );

            ReturnStatus getBLEBias(double *bias);

            void setBLEBias(const double  &bias, int64_t delta_t );

			ReturnStatus getBLEBiasWithUncertainty(double *bias, double *bias_uncertainty);

			void setBLEBiasWithUncertainty(const double  &bias, const double &bias_uncertainty, int64_t bias_age);
					   
            /**
            * Sets fine known initial position
            * \param[in] position initial position in global frame
            */
            void setStartPosition( const Position &position );

            /**
            * Sets rndom seeds for internal randomizers
            */
            void setRandomSeeds();

            /**
            * Sets output stream for corresponding log id
            * param[in] idx log index
            * param[in] os output stream
            * return succes status
            */
            bool setLogStream( const unsigned int &idx, std::ostream &os );

            /**
            * Return vector of logs  brief descriptions
            * param[out] log_descriptions pointer to descrpitions
            */
            void getLogDescription( std::vector<std::string> *log_descriptions );

            /**
            * Control Mag PF
            * param[in] enable disables/enables Mag filter
            */
            void setMagFilterEnabled(bool enable);

            /**
            * Control Mixed PF
            * param[in] enable disables/enables Mixed filter
            */
            void setMixedFilterEnabled(bool enable);

            /**
            * return Mag filter state
            */
            bool getMagFilterEnabled();

            /**
            * return Mixed filter state
            */
            bool getMixedFilterEnabled();

            /**
            * return venue params
            */
            BaseVenueType getVenueParams();

            /**
            * set use barometr
            * param[in] enable/disable
            */
            void setUseBarometer(bool enable); 
            
            /**
            * set OS type
            * param[in] OS type
            */
            void setOsType(OperationSystemType os_type);

        private:
            FPEngine( const FPEngine & ); //disable copy constructor
            const FPEngine &operator=( const FPEngine & ); //disable copy
            friend class IFPEngine;
            IFPEngine *mFPEngine;
    };

}

#endif //FPPE_HPP

