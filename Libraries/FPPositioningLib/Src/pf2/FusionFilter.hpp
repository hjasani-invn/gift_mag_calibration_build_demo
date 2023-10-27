/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           The main class of integration different modules.
* Implements platform independent data processing to obtain navigation
* \ingroup        PF
* \file            FusionFilter.hpp
* \author          M.Frolov D.Churikov
* \date            28.11.2014
*/

#ifndef FUSION_FILTER_H
#define FUSION_FILTER_H

#include <stdint.h>
#include <queue>

#include "Fppe.hpp"
#include "PF.hpp"
#include "UpdateWiFi.hpp"
#include "UpdateBLEProximity.hpp"
#include "UpdateMFP3D.hpp"
#include "UpdateMM.hpp"
#include "UpdatePortals.hpp"
#include "UpdateRBFBias.hpp"
#include "UpdateExternalPos.hpp"
#include "UpdateCollaboration.hpp"
#include "UpdateMM.hpp"
#include "InitializerPF.hpp"
#include "InitializerMFP.hpp"
#include "InitializerNormal.hpp"
#include "InitializerInArea.hpp"
#include "InitializerInAreaMapMatching.hpp"
#include "IFFSettings.hpp"
#include "LikelihoodPos.hpp"
#include "Venue.h"
#include "CoordinateConverter.h"

#include "wifi_if.hpp"
#include "ble_proximity_if.hpp"
#include "wifi_helper.hpp"
#include "ble_proximity_helper.hpp"
#include "ble_nonlinear_filter.hpp"

#include "PredictTPN3D.hpp"
#include "PredictVdr3D.hpp"

#define ENABLE_MULTIPOINT_WIFI_INITIALIZER   0
#define ENABLE_MULTIPOINT_BLE_PROXIMITY  1
#define ENABLE_MULTIPOINT_WIFI_BIAS_ESTIMATION  1

/**
* Position callback interface
*/
class IPositionUpdateFF
{
    public:
        virtual ~IPositionUpdateFF()
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
        virtual void update( double X,  double Y, double Floor, double H, double Sig, double t, const PF::Particle *state, int N ) = 0;
};

/**
* Venue detection callback interface
*/
class IVenueDetectionUpdateFF
{
public:
    virtual ~IVenueDetectionUpdateFF()
    {
        ;
    }

virtual void update(bool pos_inside_venue, double t) = 0;
};

/**
* Extended proximity position callback interface
*/
class IExtendedProximityUpdateFF
{
public:
    virtual ~IExtendedProximityUpdateFF()
    {
        ;
    }
    virtual void update(double t, const BLE_position &beacon_pos) = 0;
};

//===================================================================================
/// Enumeration type of environment state
enum eEnvironmentState
{
    eES_Unknown = 0,
    eES_Indoor,
    eES_Outdoor,
    eES_Mixed
};

typedef std::pair<const std::string, std::ostream **> LogDescriptor;


/** IFFSettings class */
class FFPredictionProxy : public IFFData<PF::Float>
{
    public:

        FFPredictionProxy( const IFFData<PF::Float> &iFFData, const Prediction<PF::Float> &predictions ) : iFF( iFFData ), pred( predictions )
        {
        }
        const Prediction<PF::Float>& getPrediction() const
        {
            return pred;
        }
        const MagMeasurements<PF::Float>& getMagMeas() const
        {
            return iFF.getMagMeas();
        }

        const PositionWithUncertainties& getExternalPosition() const
        {
            return iFF.getExternalPosition();
        }

        tMFP * const & getMFP() const
        {
            return iFF.getMFP();
        }

        MapMatching * const & getMapMatching() const
        {
            return iFF.getMapMatching();
        }

    private:
        const IFFData<PF::Float> &iFF;
        const Prediction<PF::Float> &pred;
};

/** Main filter class. Integrates different modules to obtain fused navigation solution */
class FusionFilter : IFFData<PF::Float>
{
    public:
        FusionFilter(); ///< default constructor
        ~FusionFilter();

        /**
        * resets mixed filter if position reacquisition was triggered
        */
        void restart_for_reacquisition();
        /**
        * resets internal object state
        * \param[in] file_folder output log directory
        * \param[in] log_time  log filename suffix, if string is empty logging disabled
        */
        void restart( const std::string &file_folder, const std::string &log_time );

        void restart( const std::string &file_folder, const std::string &log_time, bool reset_mixed, bool reset_MFP );

        /**
        * resets internal object state, disables logging
        */
        void restart()
        {
            restart( std::string(), std::string() );
        }


        /**
        * main method, process inertial sensors data and initiates processing pipeline
        * \param[in] aAcc[3] three component array of pointers to the accelerometer data components [m/s^2]
        * \param[in] tsAcc array of accelerometer data timestamps [ms]
        * \param[in] AccLineCount accelerometer data count
        * \param[in] aGyro[3] three component array of pointers to the gyroscope data components [rad/s]
        * \param[in] tsGyro array of accelerometer data timestamps [ms]
        * \param[in] GyroLineCount gyroscope data count
        * \param[in] aMag[3] three component array of pointers to the magnetometer data components [uT]
        * \param[in] tsMag array of accelerometer data timestamps [ms]
        * \param[in] MagLineCount mag data count
        * \param[in] aMagBias[3] three component array of pointers to the known magnetometer biases [uT]
        * \param[in] tsMagBias array of mag biases timestamps [ms]
        * \param[in] MagBiasLineCount mag biases count
        */
        void process_sensors( double *aAcc[3],  int64_t *tsAcc,  int16_t AccLineCount,
                              double *aGyro[3], int64_t *tsGyro, int16_t GyroLineCount,
                              double *aMag[3],  int64_t *tsMag,  int16_t MagLineCount,
                              double *aMagBias[3],  int64_t *tsMagBias,  int16_t MagBiasLineCount );

        /**
        * Main procedure, process all data
        * \param[in] d_pos[3]  increments array {d_x, d_y, d_z}
        * \param[in] pos_cov[3][3]  increments covariance matrix,  d_z uncorrelated with d_x, d_y
        * \param[in] tsMagBias  unix time [ms]
        */
        void process_increments( const double mis_std, const double d_pos[3], const double pos_cov[3][3], const double quat[4], const double quat_cov[4][4], const bool quat_valid, const bool is_motion, const int64_t timestamp, const bool is_transit);

        /**
        * pushes WiFi measurement into the processing pipeline
        * \param[in] measurement WiFi measurements vector
        * \param[in] timestamp [ms]
        * \param[in] n measurements serial number
        */
        void process_wifi(const WiFi_Measurement  &measurement, int64_t san_timestamp, int n);

        /**
        * pushes BLE measurement into the processing pipeline
        * \param[in] measurement BLE measurements vector
        * \param[in] timestamp [ms]
        * \param[in] n measurements serial number
        */
        void process_beacons( const WiFi_Measurement  &measurement, int64_t san_timestamp, int n );

        /**
        * pushes ble proximity position into the processing pipeline
        * \param[in] pos external position [meters]
        */
        void process_beacons_proximity(const Fppe::BleScanResult &scan_ble, int64_t san_timestamp, int n);

        /**
        * calculates number of proximity beacons of specified type on specified floor in BLP-DB
        * \param[in] floor - pf-floor number
        * \param[in] beacon_type - beacon type
        * \param[out] number of found beacons or -1 if BLP-DB is not initializaed
        */
        int get_proximity_beacons_number(const int floor, const BleBeaconTypeTag beacon_type)const;

        /**
        * pushes Mag measurement into the processing pipeline
        * \param[in] measurement Mag 3-components vector
        * \param[in] timestamp [ms]
        */
        void process_mag( const std::vector<double>  &measurement, int64_t timestamp );


        /**
        * pushes Barometer measurement into the processing pipeline
        * \param[in] pressure in [hPa]
        * \param[in] timestamp in [ms]
        */
        void process_baro( const double pressure, const int64_t timestamp );


        /**
        * pushes external location measurement into the processing pipeline
        * \param[in] pos external position [meters]
        */
        void process_external_pos( const PositionWithUncertainties &pos );

        /**
        * pushes ble_collaboration into the processing pipeline
        * \param[in] ble_collaboration position
        */
        void process_collaboration(std::vector <Fppe::CollaborationData> &collaboration_position);

        //PositionWithUncertainties getProximityPos(const int64_t step_timestamp);
        
        /**
        * Sets main filter position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setCallbackWiFi( IPositionUpdateFF *pPosCbk )
        {
            pWiFiUpd = pPosCbk;
        }
        /**
        * Sets secondary filter position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setCallbackBle( IPositionUpdateFF *pPosCbk )
        {
            pBleUpd = pPosCbk;
        }

        void setCallbackBleProximity(IPositionUpdateFF *pPosCbk)
        {
            pBleProximityUpd = pPosCbk;
        }

        void setCallbackExtendedProximity(IExtendedProximityUpdateFF* pExProxCbk)
        {
            pExtendedProximityUpd = pExProxCbk;
        }

        /**
        * Sets Magnetic only filter position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setCallbackMFP( IPositionUpdateFF *pPosCbk )
        {
            pMfpUpd = pPosCbk;
        }

        /**
        * Sets Mixed filter position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setCallbackMixed( IPositionUpdateFF *pPosCbk )
        {
            pMixedUpd = pPosCbk;
        }

        /**
        * Sets Mixed filter position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setVenueCallbackWiFi(IVenueDetectionUpdateFF *pVenueWiFiCbk)
        {
            venueWiFiUpd = pVenueWiFiCbk;
        }

        /**
        * initialize WiFi module and loads fingerprint from  the file
        * \param[in] wifi_db_name fingerprint filename
        * \param[in] minProb validity metric threshold. TODO to be moved in fingerprint file
        * \return success status
        */
        bool initializeWiFi( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double minProb );
       
        /**
        * initialize BLE module and loads fingerprint from the file
        * \param[in] ble_db_name fingerprint filename
        * \param[in] minProb validity metric threshold. TODO to be moved in fingerprint file
        * \return success status
        */
        bool initializeBLE( const char* const pBleMap, const size_t bleFileSizeInBytes, const double minProb );

        /**
        * initialize BLE proximity module
        * \param[in/out] ppProxInstance - pointer to proximity instsance
        * \param[in] pProxMap - pointer to proximity map
        * \param[in] pProxMap - pointer to proximity map
        * \param[in] proxFileSizeInBytes - size of proximity map
        * \param[in] converter - is used to convert lat, lon to local X, Y coordinates for proximity beacons
        * \param[in] N - maximum number of latest accumulated proximity measurements accumulated
        * \param[in] M - minimum number of valid proximity measurements for proximity position to be used
        * \param[in] SingleCutoffThreshold - threshold for rejecting measurements with weak signal in single position
        * \param[in] MultipleCutoffThreshold) - threshold for rejecting measurements with weak signal in positions list
        * \return success status
        */
        bool initializeBLEProximity(IBLEProximity ** ppProxInstance, std::string ProxInstanceName, const char* const pProxMap, const size_t proxFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int SingleCutoffThreshold, const int MultipleCutoffThreshold);

        /**
        * initialize BLE proximity module usesd in positioning
        * \param[in] pProxMap - pointer to proximity map
        * \param[in] proxFileSizeInBytes - size of proximity map
        * \param[in] converter - is used to convert lat, lon to local X, Y coordinates for proximity beacons
        * \param[in] N - maximum number of latest accumulated proximity measurements accumulated
        * \param[in] M - minimum number of valid proximity measurements for proximity position to be used
        * \param[in] rejectThreshold - threshold for rejecting measurements with weak signal
        * \return success status
        */
        bool initializeBLEProximity_for_positioning(const char* const pProxMap, const size_t bleFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int rejectThreshold);

        /**
        * initialize BLE proximity module used in beacons detection
        * \param[in] pProxMap - pointer to proximity map
        * \param[in] proxFileSizeInBytes - size of proximity map
        * \param[in] converter - is used to convert lat, lon to local X, Y coordinates for proximity beacons
        * \param[in] N - maximum number of latest accumulated proximity measurements accumulated
        * \param[in] M - minimum number of valid proximity measurements for proximity position to be used
        * \param[in] rejectThreshold - threshold for rejecting measurements with weak signal
        * \return success status
        */
        bool initializeBLEProximity_for_detection(const char* const pProxMap, const size_t proxFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int rejectThreshold);
       
        /**
        /**
        * initialize BLE proximity module used for pf-initialization
        * \param[in] pProxMap - pointer to proximity map
        * \param[in] proxFileSizeInBytes - size of proximity map
        * \param[in] converter - is used to convert lat, lon to local X, Y coordinates for proximity beacons
        * \param[in] N - maximum number of latest accumulated proximity measurements accumulated
        * \param[in] M - minimum number of valid proximity measurements for proximity position to be used
        * \param[in] rejectThreshold - threshold for rejecting measurements with weak signal
        * \return success status
        */
        bool initializeBLEProximity_for_start(const char* const pProxMap, const size_t proxFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int rejectThreshold);
        
        /**
        * initialize MapMatching module and loads map
        * \param[in] pMap - pointer to map
        * \param[in] mapFileSizeInBytes - size of map
        * \param[in] cellSize defines internal map discrete [m]
        * \param[in] floor_shift - real number of lowest floor
        * \param[in] floor_zero_enable 
        * \return success status
        */
        bool initializeMapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes, double max_X, double max_Y, int16_t floor_shift, bool floor_zero_enable, const GeoLocConverter2D &converter);
        /**
        * initialize MFP module and loads fingerprint from the file
        * \param[in] mfp_script_name fingerprint filename
        * \param[in] max_X,max_Y defines fingerprint area [m] [TODO  to be moved into FP file]
        * \param[in] cellSize defines internal map discrete [m] [TODO  to be moved into FP file]
        * \param[in] minFloor,maxFloor floors range [TODO  to be moved into FP file]
        * \return success status
        */
        bool initializeMFP( const char* const pMFPMap, const size_t mfpFileSizeInBytes, double max_X, double max_Y, double cellSize, int minFloor, int maxFloor );

        /**
        * Resets collected MFP map (experimental)
        */
        void resetMapMfp();

        /**
        * Defines transformation from local frame to global coordinates
        * \param[in] lat,lon local frame origin [deg]
        * \param[in] alfa_lat,alfa_lon transforamtion params [deg]
        * \param[in] rot local frame rotation [deg]
        * \return success
        */
        bool setLocalFrameParams( double   lat0,
                                  double   lon0,
                                  double   alfa_lat,
                                  double   alfa_lon,
                                  double   rot );

        /// updater control @{
        void setUpdateCompass( bool enable );
        //void setUpdateMFPDiff( bool enable );
        void setUpdateWiFi( bool enable );
        void setUpdateBLE( bool enable );
        void setUpdateBLEProximity( bool enable );
        void setUpdateMM( bool enable );
        void setUpdatePortals(bool enable);
        void setUpdateMFP( bool enable );
        //void setUpdateMM3D( bool enable );
        void setUpdateExternalPosition( bool enable );
        void setUpdateCollaboration(bool enable);
        void setCorrectFusionFilterPosition(bool enable);
        ///@}

        /// Enables barometer usage, switches to another motion model
        void setUseBarometer( bool enable );

        /// enables WiFi heuristic control
        void setAdjustWfpParams( bool enable );
        /// enables MFP heuristic control
        void setAdjustMfpParams( bool enable );

        /** Gets mg calibration params
        * \param[out] time the last estimated bias timestamp
        * \param[out] pMagBias 3-component array of biases
        * \param[out] pMagBiasCov 3-components array of bias std
        */
        bool GetMagneticBiasAndCov( int64_t *time, double * pMagBias, double * pMagBiasCov);
        /**
        * Sets mag calibration params
        * \param[in] time last known bias timestamp [ms]
        * \param[in] pMagBias 3-component array of biases
        */
        void SetMagneticBias( int64_t time, double * pMagBias );
        /**
        * Sets mag bias std
        * \param[in] pMagBiasCov 3-components array of bias std
        */
        void SetMagneticBiasCov( double * pMagBiasCov );
        /** WiFi calibration status [obsolete]
        * param[out] bias rssi bias
        * returns calibration enabled
        */

        /** Gets platform type
        * \return platform type
        */
        PlatformType getPlatformType();

        /** Sets platform type
        * \param[in] platform_type platform type
        */
        void setPlatformType(PlatformType platform_type);

        /** Sets venue type
        * \param[in] venue_type platform type
        */
        void setVenueType(VenueType venue_type);

        /** Sets BLP pulling type
        * \param[in] type pulling type
        * \param[in] pulling_distance pulling distance
        * \param[in] pulling_sigma pulling sigma
        */
        void setBlpPulling(ePullingType type, double pulling_distance, double pulling_sigma);

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

        bool getWiFiBias( double &bias );
        /** sets WiFi calibration [obsolete]
        * param[in] bias initial rssi bias
        * param[in] delta_t - time since last calibration
        */
        void setWiFiBias( double  bias, int64_t delta_t );
        /** BLE calibration status [obsolete]
        * param[out] bias rssi bias
        * returns calibration enabled
        */
        bool getBFPBias( double &bias );
        /** sets BLE calibration [obsolete]
        * param[in] bias initial rssi bias
        * param[in] enabled enables calibration
        */
        void setBFPBias( double  bias, int64_t delta_t );

        bool getBLEProxBias(double &bias);

		bool getBLEBiasWithUncertainty(double &bias, double &bias_uncertainty);

        void setBLEProxBias(double  bias, int64_t delta_t );

		void setBLEBiasWithUncertainty(double bias, double bias_uncertainty, int64_t bias_age);

        bool getOutputBLEBias(double &bias);

        /** 
        * Return current position dispersion of mixed filer
        */
        double getMixedFilterSigma();

        /**
        * Return current initialization statua ofmixed filer
        */
        bool isMixedFilterInitialized();

        /**
        * Sets flags for soft or hard reset based on std_heading and sigma mix
        */
        void updateReacquisitionState( int64_t time, double std_heading, double sigma_mix, double level );

        /**/

        void setStartPositionWithUncertainties( double _x, double _y, double _z, double _heading, const double cov_xy[2][2], double std_z, double std_h, int64_t timestamp );

        void getLogDescriptors( std::vector<LogDescriptor> *log_descriptors );

        bool setLogStream( const unsigned int &idx, std::ostream &os );

        /**
        * Control Mag PF
        * param[in] enable disables/enables Mag filter
        */
        void setMFPFilterEnabled( bool enable );
        /**
        * Control Mixed PF
        * param[in] enable disables/enables Mixed filter
        */
        void setMixedFilterEnabled( bool enable );
        /**
        * return Mag filter state
        */
        bool getMagFilterEnabled();
        /**
        * return Mixed filter state
        */
        bool getMixedFilterEnabled();

        /**
        * Set percent of particles for update by likelihood in prediction model
        */
        void setLkhUpdateRatio(double upd_ratio);

        /**
        * Set minimal and maximal limits of particle number for mixed particle filter 
        * param[in] min_particle_count - minimal particle number limit
        * param[in] max_particle_count - maximal particle number limit
        */
        void setParticleCountForMixedFilter(int32_t min_particle_count, int32_t max_particle_count);

        /**
        * Set minimal and maximal limits of particle number for mfp-only particle filter
        * param[in] min_particle_count - minimal particle number limit
        * param[in] max_particle_count - maximal particle number limit
        */
        void setParticleCountForMfpFilter(int32_t min_particle_count, int32_t max_particle_count);

        /**
        * Set random seeds for internal filter randomizers
        */
        void setRandomSeeds();

        /**
        * Set type of OS (Android / iOS)
        */
        void setOsType(OperationSystemType type_os);

        void setFloorHeight(double floor_height)
        {
            this->floor_height = floor_height;
            predict_lks_mfp->set_floor_height(floor_height);
            predict_lks_mix->set_floor_height(floor_height);
            mfp_portals.setFloorHeight(floor_height);
        }

        void setTimeInterval(double time_interval)
        {
            this->time_interval = time_interval;
            predict_lks_mfp->set_time_interval(time_interval);
            predict_lks_mix->set_time_interval(time_interval);
            mfp_portals.set_time_interval(time_interval);
        }

        void setVerticalSpeed(double vertical_speed_low, double vertical_speed_high, PortalType portal_type)
        {
            switch (portal_type)
            {
            case k_Elevator:   
                this->vertical_speed_elevator_low = vertical_speed_low;
                this->vertical_speed_elevator_high = vertical_speed_high;
                break;
            case k_Escalator: 
                this->vertical_speed_escalator_low = vertical_speed_low;
                this->vertical_speed_escalator_high = vertical_speed_high;
                break;
            case k_Stairs: 
                this->vertical_speed_stairs_low = vertical_speed_low;
                this->vertical_speed_stairs_high = vertical_speed_high;
                break;
            }
            predict_lks_mfp->set_vertical_speed(vertical_speed_low, vertical_speed_high, portal_type);
            predict_lks_mix->set_vertical_speed(vertical_speed_low, vertical_speed_high, portal_type);
            mfp_portals.setVverticalSpeed(vertical_speed_low, vertical_speed_high, portal_type);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        /**
        * Set up prediction model for magnetic filter in depending on specified venue type and platform (carrier) type
        * param[in] venue_type - type of venue
        * param[in] platform_type - type of platform (carrier)
        * reurn status of prediction model setting up (true if new model have been set, otherwice false)
        */
        bool setPredictionModelForMagFilter(VenueType venue_type, PlatformType platform_type);
        /**
        * Set up prediction model for mixed filter in depending on specified venue type  and platform (carrier) type
        * param[in] venue_type - type of venue
        * param[in] platform_type - type of platform (carrier)
        * reurn status of prediction model setting up (true if new model have been set, otherwice false)
        */
        bool setPredictionModelForMixedFilter(VenueType venue_type, PlatformType platform_type);

    private:
        WiFi_Location estimateBLEPos( const int64_t step_timestamp );

        WiFi_Location estimateBLEProximityPos_for_start(const int64_t step_timestamp);
        void estimateBLEProximityPos(const int64_t step_timestamp, int64_t &ble_proximity_timestamp, std::vector<WiFi_Location > &LocationList);
        WiFi_Location estimateWiFiPos(const int64_t timestamp, const int64_t wifi_solution_delay_limit);
        WiFi_Location rejectWiFiSpike( const WiFi_Location &wifi_pos );
        PositionWithUncertainties getExternalPos( const int64_t step_timestamp );
        //bool iterationWiFiBiasEsimation( double &bias, const WiFi_Measurement &meas, const WiFi_Location &location, int &cnt );
        std::vector <Fppe::CollaborationData> getCollaborationPos(const int64_t step_timestamp);
        void estimateCollaborationPos(const int64_t filter_timestamp, int64_t &collaboration_timestamp, std::vector<Fppe::CollaborationData> &LocationList);

        bool getWiFiLocationList(const int64_t timestamp, int64_t &solution_timestamp, std::vector<WiFi_Location> &loc_list, int64_t max_delay);

        //void adjustPredictionParams();
        void resizePF();

        void openLogFiles( const std::string &file_folder, const std::string &log_time );
        void closeLogFiles();
        void initializeVariables();
        void initializeUpdaters();
        void initializeInitializers();
		void initializeInjectors();
        void initializeFilterMFP();
        void initializeFilterMixed();

        void process_beacons_proximity_for_positioning(const Fppe::BleScanResult &scan_ble, int64_t scan_timestamp, int n);
        void process_beacons_proximity_for_detection(const Fppe::BleScanResult &scan_ble, int64_t scan_timestamp, int n);

        bool integrate( const double mis_std, const double d_pos[3], const double pos_cov[3][3], const double quat[4], const double quat_cov[4][4], const bool quat_valid, const bool is_motion, const int64_t timestamp, const bool is_transit);
        const Prediction<>& getPrediction() const;
        const MagMeasurements<>& getMagMeas() const;
        tMFP * const & getMFP() const;
        MapMatching * const & getMapMatching() const;
        const PositionWithUncertainties& getExternalPosition() const;

        void controlMagFilter();
        void controlMixedFilter();
		
		void controlInjection();
		void controlBarometer();

        bool correctPositionWithMapMatching(PF::Particle &position, const PF::Particle &prev_position);

        void PrintParams();

        PF::Particle pos_mix_corrected_prev = {0};
        PF::Particle pos_mfp_corrected_prev = {0};
        bool position_mm_correction_enable;

        PF::IFilter *pMFP;
        PF::IFilter *pMixed;

        //PF::PredictTPN3D tpn_3d; /// work with floor increments
        //PF::PredictTPN3D tpn_2d; /// TODO uses fixed froor number
        PF::PredictTPN3D tpn_3d_start;

        PF::UpdateWiFi wifi_update_mix;
        PF::UpdateWiFi ble_update_mix;
        PF::UpdateBLEProximity ble_proximity_update_mix;
        PF::UpdateMFP3D  mfp3D_update;
        PF::UpdateMFP3D  mfp3D_update_mix;
        //PF::UpdateMFP3D  mfp3D_update_init;
        PF::UpdateMFP3D_Init  mfp3D_update_init;
        PF::UpdateRBFBias rbf_bias;
        PF::UpdateRBFBias rbf_bias_init;
        PF::UpdateExternalPosition pos_update;
        PF::UpdateBLECollaboration ble_collaboration_update;
        PF::UpdateMM  mm_update;
        PF::UpdatePortals1 mfp_portals;

        WiFi_Location wifi_pos;
        WiFi_Location ble_pos;
		PositionWithUncertainties proximity_pos;
#if ENABLE_MULTIPOINT_WIFI_INITIALIZER
        PF::InitializerInSeveralAreas_RBF *init_wifi;
#else
        PF::InitializerInArea_RBF *init_wifi;
#endif
        PF::InitializerInArea_RBF *init_ble;
        PF::InitializerInArea_RBF *init_proximity;
        PF::InitializerInAreaMapMatching_RBF *init_wifi_mm;
        PF::InitializerInAreaMapMatching_RBF *init_ble_mm;
        PF::InitializerInAreaMapMatching_RBF *init_proximity_mm;
        PF::InitializerInArea_RBF *init_collaboration;
        PF::InitializerInAreaMapMatching_RBF *init_collaboration_mm;

        PF::FilterInitializer *init_pf;
        PF::FilterInitializer *init_fine;
        PF::InitializerNormal_RBF *init_normal_mfp;
        PF::InitializerNormal_RBF *init_normal_mix;
        PF::InitializerMFP *init_mfp;
        PF::MagInitializerInArea *init_mfp_area;
		PF::InitializerInArea_RBF *init_mixed_area;
		PF::InitializerInAreaMapMatching_RBF *init_mixed_area_mm;
        PF::InitializerMfpPF *init_mfp_pf;
        PF::RBFInitializer *init_rbf;

#if ENABLE_MULTIPOINT_WIFI_INITIALIZER
		PF::InitializerInSeveralAreas_RBF *injector_wifi;
#else
		PF::InitializerInArea *injector_wifi;
#endif
		PF::InitializerInArea_RBF *injector_ble;
		PF::InitializerInArea_RBF *injector_proximity;
		PF::InitializerInAreaMapMatching_RBF *injector_wifi_mm;
		PF::InitializerInAreaMapMatching_RBF *injector_ble_mm;
		PF::InitializerInAreaMapMatching_RBF *injector_proximity_mm;
		PF::InitializerNormal_RBF *injector_normal_mix;
		PF::RBFInitializer *injector_rbf;
        PF::InitializerInArea_RBF *injector_collaboration;
        PF::InitializerInAreaMapMatching_RBF *injector_collaboration_mm;

        WiFi *wifi = 0;
        WiFi *ble = 0;
        IBLEProximity *ble_proximity = 0;
        IBLEProximity *ble_proximity_start = 0;
        IBLEProximity *ble_proximity_detection = 0;
        tMFP *mfp = 0;
        MapMatching *mm = 0;

        WiFi_Helper wifi_helper;
        WiFi_Helper ble_helper;
        BLE_Proximity_Helper ble_proximity_helper;
        BLE_Proximity_Helper ble_proximity_start_helper;
        BLE_Proximity_Helper ble_proximity_bias_helper;

        int64_t wifi_injection_count;
        int64_t wifi_timestamp;
        int64_t last_wifi_timestamp;
        int start_count_down;
        //int64_t ble_proximity_timestamp;
		int64_t ble_proximity_timestamp_start;
        int64_t ble_proximity_bias_timestamp;
        int64_t collaboration_timestamp;
        int64_t filter_timestamp; //current filter unix time [ms]

        int integration_time;  //filter perion length [ms]


        IPositionUpdateFF *pMfpUpd;
        IPositionUpdateFF *pMixedUpd;
        IPositionUpdateFF *pWiFiUpd;
        IPositionUpdateFF *pBleUpd;
        IPositionUpdateFF *pBleProximityUpd;
        IVenueDetectionUpdateFF *venueWiFiUpd;
        IExtendedProximityUpdateFF *pExtendedProximityUpd;

        int stepCounter;
        int wifi_spike_count;
        int pf_mix_max_plimit;
        int pf_mix_min_plimit;

        int pf_mfp_max_plimit;
        int pf_mfp_min_plimit;

        //TODO? extract to separate class
        std::ostream *pf_log;
        std::ostream *wifi_log;
        std::ostream *ble_log;
        std::ostream *ble_proximity_log;
        //std::ostream *waypoints_log;

        std::vector<LogDescriptor> logs;

        std::ofstream stub_pf_log;
        std::ofstream stub_wifi_log;
        std::ofstream stub_ble_log;
        std::ofstream stub_ble_proximity_log;
        //std::ofstream stub_waypoints_log;
        // TODO END
        
        double biasWiFiRSSI;
        double biasWiFiGain;
        bool   biasWiFiEstEnabled;
        //bool   biasWiFiEstSleeped;

        double biasBLERSSI;
        double biasBLEGain;
        bool   biasBLEEstEnabled;
        //bool   biasBLEEstSleeped;

        double biasBLEProxRSSI;
        bool   biasBLEProxEstEnabled;
        
        //bool wifi_new_meas;
        bool is_motion;
        eEnvironmentState updatersState;
        int env_count_down;
        int szpf_count_down;
        int wfpt_count_down;
        int mgpt_count_down;

        bool bWfpAdjustment;
        bool bMfpAdjustment;

        bool baroEnabled;
        bool pfMixedEnabled;

        bool hard_reset_enabled;
        bool hard_reset_required;
        bool heading_is_stable;
		double reacquisition_injection_percentage;
		double reacquisition_injection_percentage_for_change_floor;
        int64_t last_hard_reset_time;
		int64_t last_soft_injection_time;
        double reacquisition_block_in_stops;
        std::deque<double> level_queue;

        typedef std::pair<int64_t, std::vector<double>> MagMeas;
        typedef std::queue<MagMeas>  MagFIFO;
        MagFIFO magFifo;
        int sum_N;
        bool inTrackingMixed;
        PF::Particle mixed_pos;
        PF::Particle mixed_pos_std;
        double sigma_mfp, sigma_mix;

        MagMeasurements<> mag, mag_avg;
        //Eigen::Matrix<PF::Float, 3, 3> C_avg;
        Prediction<PF::Float> prediction;
        Prediction<PF::Float> prediction_mix;
        Prediction<PF::Float> prediction_mfp;
        std::vector<PF::UpdateSource *> pf_mix_updaters;
        std::queue<PositionWithUncertainties> pos_queue;
        PositionWithUncertainties external_pos;

        std::queue<std::vector <Fppe::CollaborationData>> collaboration_queue;

        PF::tPredictTPN3D *predict_lks_mfp;
        PF::tPredictTPN3D *predict_lks_mix;

        // prediction model instances
        PF::PredictTPN3DLkH_MutiPoint  predict_lks_multypoint_pf1; // lkh-multipoint for PDR
        PF::PredictTPN3DLkH_MutiPoint  predict_lks_multypoint_pf2; // lkh-multipoint for PDR

        PF::PredictVdr3D  predict_vdr_pf1; // prediction for VDR
        PF::PredictVdr3D  predict_vdr_pf2; // prediction for VDR

        //std::queue<PositionWithUncertainties> proximity_pos_queue;

        FFPredictionProxy i_pf_mix;
        FFPredictionProxy i_pf_mfp;

        std::map <BSSID, PeakDetector> proximity_meas_filter;
        std::map <BSSID, PeakDetector> proximity_meas_filter_for_detection;

        std::vector <Location_and_time> position_history;

        OperationSystemType os_type;
        PlatformType platform_type;
        VenueType venue_type;

        int peak_detector_max_delay_ms_for_positioning_in_moving;
        double descending_factor_for_positioning_in_moving;
        int peak_detector_max_delay_ms_for_positioning_in_stop;
        double descending_factor_for_positioning_in_stop;

        int peak_detector_max_delay_ms_for_detection_in_moving;
        double descending_factor_for_detection_in_moving;
        int peak_detector_max_delay_ms_for_detection_in_stop;
        double descending_factor_for_detection_in_stop;

        bool ble_proximity_detection_enable;
        bool auto_positioning_poximity_params_update_for_os_type;
        bool auto_detection_poximity_params_update_for_os_type;
        double floor_height;
        double time_interval;
        double vertical_speed_elevator_low;
        double vertical_speed_escalator_low;
        double vertical_speed_stairs_low;
        double vertical_speed_elevator_high;
        double vertical_speed_escalator_high;
        double vertical_speed_stairs_high;
        bool   isUseBarometer;
};

#endif //FUSION_FILTER_H

