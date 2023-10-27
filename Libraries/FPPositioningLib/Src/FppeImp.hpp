#ifndef FPPE_IMPL_HPP
#define FPPE_IMPL_HPP

#include <cassert>
#include <vector>
#include <algorithm>
#include <streambuf>
#include <chrono>

#include "IFPEngine.hpp"
#include "FusionFilter.hpp"
#include "BleHash.h"
#include "tpn_converter.hpp"
#include "CoordinateConverter.h"
#include "fpHeader.hpp"
#include "CRC.h"
#include "Venue.h"
#include "version.h"
#include "AltitudeFilter.hpp"

// patch: for floor params in fingerprints built by FPBL early v1.1.0
static void PatchVenueFloorParameters(BaseVenueType venue, VersionNumberBase buiderVersion, int16_t &venue_floor_shift, bool &venue_floor_zero_enable)
{
    int64_t version_hash_new_floor_setting = 1 * 1000 + 1;
    int64_t version_hash = buiderVersion.major * 1000 + buiderVersion.minor;
    if (version_hash < version_hash_new_floor_setting)
    {// sett correct values for fingerprints built with old FPBL version
        venue_floor_shift = 1;
        venue_floor_zero_enable = false;
    }
    else
    {// sett correct values for fingerprints built with old FPBL version
        venue_floor_shift = venue.floor_shift;
        venue_floor_zero_enable = venue.floor_zero_enable;
    }
}

class MisNoiseEstimator
{
    public:
        MisNoiseEstimator() : mis_T( 5.0 )
        {
            reset();
        }
        void reset()
        {
            call_counter = 0;
            mis_p2_c = 0;
            mis_p2_s = 0;
            mis_c = 0;
            mis_s = 0;
            mis_f_c = 0;
            mis_f_s = 0;
            mis_slope_smoothed = 0;
            mis_fifo_5s.resize( 20 );

            for ( size_t i = 0; i < mis_fifo_5s.size(); ++i )
            {
                mis_fifo_5s[i] = 0.0;
            }
        }

        double process( double mis_sm, double mis_p1, double mis_p2 )
        {
            call_counter++;
            double a = 1. / call_counter;
            double mis = mis_sm / 180 * M_PI;
            double p2 = mis_p2 / 180 * M_PI;
            a = std::max( a, 1. / ( mis_T * 20 ) );

            mis_p2_c += a * ( cos( p2 ) - mis_p2_c );
            mis_p2_s += a * ( sin( p2 ) - mis_p2_s );
            double p2_std = sqrt( -log( pow( mis_p2_c, 2 ) + pow( mis_p2_s, 2 ) ) ) * 180 / M_PI;

            mis_c += a * ( cos( mis ) - mis_c );
            mis_s += a * ( sin( mis ) - mis_s );
            double m_std = sqrt( -log( pow( mis_c, 2 ) + pow( mis_s, 2 ) ) ) * 180 / M_PI;

            double b = 1. / call_counter;
            b = std::max( b, 1. / ( 0.2 * 20 ) );
            mis_f_c += 0.2 * ( cos( mis ) - mis_f_c );
            mis_f_s += 0.2 * ( sin( mis ) - mis_f_s );
            double f_mis = atan2( mis_f_s, mis_f_c ) * 180 / M_PI;

            mis_fifo_5s[call_counter % mis_fifo_5s.size()] = f_mis;

            double slope_mis = mis_fifo_5s[call_counter % mis_fifo_5s.size()] - mis_fifo_5s[( call_counter + 1 ) % mis_fifo_5s.size()];
            slope_mis = ( slope_mis > 180 ) ? slope_mis - 360 : slope_mis;
            slope_mis = ( slope_mis < -180 ) ? slope_mis + 360 : slope_mis;
            slope_mis = std::abs( slope_mis );

            mis_slope_smoothed = std::max( mis_slope_smoothed, slope_mis );
            mis_slope_smoothed += 0.1 * ( slope_mis - mis_slope_smoothed );

            mis_slope_smoothed = m_std;
            double std_mis = ( p2_std > 50 ) ? std::max( m_std, slope_mis ) : 0;
            //std::cout << "fppe_inc.mis_std = " << fppe_inc.mis_std << std::endl;

            //std::cout << "est_std = " << std_mis
            //    << " p2 = " << p2_std
            //    << " mstd = " << m_std
            //    << " slope = " << slope_mis
            //    << " slope_s - " << mis_slope_smoothed << std::endl;
            //std_mis = 0;
            return std_mis;
        }
    private:

        int call_counter;
        const double mis_T;     // T - estimation [sec]
        double mis_p2_c;        // cos
        double mis_p2_s;        // sin
        double mis_c;           // cos
        double mis_s;           // sin
        double mis_f_c;         // cos
        double mis_f_s;         // sin
        double mis_slope_smoothed;        // smoothed slope
        std::vector<double> mis_fifo_5s;
};


// todo: place this class in separate file (see realisation of this clas in files: FppeImp.hpp, ble_proximity_db.cpp, wifi_db_hist.cpp)
template <typename char_type>
class iostreambuf : public std::basic_streambuf<char_type, std::char_traits<char_type> >
{
public:
    iostreambuf(char_type* buffer, std::streamsize bufferLength)
    {
        // Sets the values of the pointers defining the put area. Specifically, after the call pbase() == pbeg, pptr() == pbeg, epptr() == pend
        // pbeg - pointer to the new beginning of the put area
        // pend - pointer to the new end of the put area
        std::basic_streambuf<char_type, std::char_traits<char_type> >::setp(buffer, buffer + bufferLength);

        //Sets the values of the pointers defining the get area.Specifically, after the call eback() == gbeg, gptr() == gcurr, egptr() == gend
        //  gbeg - pointer to the new beginning of the get area
        //	gcurr - pointer to the new current character(get pointer) in the get area
        //	gend - pointer to the new end of the get area
        std::basic_streambuf<char_type, std::char_traits<char_type> >::setg(buffer, buffer, buffer + bufferLength);
    }
};


class FPEngineImpl : public Fppe::IFPEngine
{
    public:
        FPEngineImpl()
        {
            mFF = new FusionFilter();
            mCbkMfp = NULL;
            mCbkMix = NULL;
            mCbkBle = NULL;
            mCbkBleProximity = NULL;
            mCbkWiFi = NULL;
            mCbkVenueDetectWiFi = NULL;
            mCbkExtendedProximity = nullptr;
            double cov[2][2] = {};
            cov[0][0] = 100;
            cov[1][1] = 100;
            
            wifi_scan_counter = 0;
            ble_scan_counter = 0;
            ble_proximity_scan_counter = 0;
            wifi_last_scan_time = -std::numeric_limits<int>::min();
            
            previous_pos_valid = false;
            //mFF->setStartPositionWithUncertainties(15, 25, 0, 0, cov, 0, 30, 0);

            venue_floor_height = k_default_floor_hight; // default floor hight
            
            ffVenue = {0};

            mFF->getLogDescriptors( &logs );
            logs.push_back( LogDescriptor( "converter", TPN_converter.getLogDescriptor() ) );

            // engine version logging
            if (logs.size() > 0)
            {
                auto it = std::find_if(logs.begin(), logs.end(), [=](const LogDescriptor& a) {return a.first == "pf_log"; });
                VersionNumber v = { FPPE_VERSION_MAJOR, FPPE_VERSION_MINOR, FPPE_VERSION_BUILD, static_cast<VersionNumberReleaseId>(FPPE_VERSION_RELEASE_ID) };
                **(it->second) << "RTFPPL verision: " << v.major << "." << v.minor << "." << v.build << "." << v.releaseId << std::endl;
            }

            altitude_filter = new AltitudeFilter();
            setUseBarometer(true);
        }

        ~FPEngineImpl()
        {
            delete mFF;

            delete mCbkMfp;
            delete mCbkMix;
            delete mCbkWiFi;
            delete mCbkBle;
        }

        /**
        * resets internal object state
        * \param[in] filename output log filename
        */
        void setLogFile( const std::string &filename )
        {
            assert( 0 && "not implemented" );
        }

        /**
        * enables logging
        * \param[in] enabled control flag
        */
        void setLogEnabled( const bool &enabled )
        {
            assert( 0 && "not implemented" );
        }

        /**
        * resets internal object state, disables logging
        */
        void restart()
        {
            mFF->restart( "", "init" );
            TpnOutput dummy = {};
            //converter.set_pdr_data( dummy );
            previous_pos_valid = false;
        }

        /**
        * main method, process inertial sensors data and initiates processing pipeline
        * \param[in] increment position increments in local frame
        */
        void processIncrements( const Fppe::CoordinatesIncrement &increment )
        {

            double d_pos[3] = { increment.d_x, increment.d_y, increment.d_floor };
            double d_pos_cov[3][3] = {};
            d_pos_cov[0][0] = increment.covariance_yx[0][0];
            d_pos_cov[0][1] = increment.covariance_yx[0][1];
            d_pos_cov[1][0] = increment.covariance_yx[1][0];
            d_pos_cov[1][1] = increment.covariance_yx[1][1];

            d_pos_cov[2][2] = increment.d_floor_std;

            double q[4], q_cov[4][4];

            for ( int i = 0; i < 4; ++i )
            {
                q[i] = increment.attitude.quaternion[i];

                for ( int j = 0; j < 4; ++j )
                {
                    q_cov[i][j] = increment.attitude.covariance_quaternion[i][j];
                }
            }

            mFF->process_increments( increment.mis_std, d_pos, d_pos_cov, q, q_cov, increment.attitude.is_valid, increment.is_motion, increment.timestamp, increment.is_transit);
        }

        /**
        * main method, process inertial sensors data and initiates processing pipeline
        * \param[in] tpn_output position and attitude information it tpn like format
        */
        void processTpnOutput( const TpnOutput &tpn_data)
        {
            Fppe::CoordinatesIncrement fppe_inc = {};
            bool Ok_to_process = false;
            //static double sum_h = 0;
#if 0 // !!!DEBUG: android P sacn rate imitation
            static std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            static auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1.time_since_epoch());
            static std::chrono::duration<long, std::micro> int_usec = int_ms;
            std::srand(int_usec.count()); // update pseudo-random seed

#define START_TYPE 3

#if START_TYPE  == 1
            static double start_timestamp = 120. + std::rand() % 120;
#elif START_TYPE  == 2
            double start_timestamp = 120. + 41;
#elif START_TYPE  == 3
            static double start_timestamp = 120;
#else
            double start_timestamp = 0;
#endif
            
            if (tpn_data.timestamp < start_timestamp)
            {
                return;
            }
#endif

            FfPosition ff_position = TPN_converter.ConvertPositionData( tpn_data.timestamp, tpn_data.position );
            MffAttitude mff_attitude = TPN_converter.ConvertAttitudeData( tpn_data.timestamp, tpn_data.attitude );
            MagneticMeasurement mag = TPN_converter.ConvertMagneticData( tpn_data.timestamp, tpn_data.mag_meas );

            if ( previous_pos_valid )
            {
                volatile int64_t dtau = std::abs( previous_ff_position.timestamp - ff_position.timestamp );

                fppe_inc.timestamp = ff_position.timestamp;

                fppe_inc.d_x = ff_position.x - previous_ff_position.x;
                fppe_inc.d_y = ff_position.y - previous_ff_position.y;

                // 
                if (ffVenue.floors_count > 1)
                {
                    double floor_height = venue_floor_height;
                    if ((false == std::isnormal(floor_height)) || (floor_height <= 0.)) // floor hight correctness controll
                    {
                        floor_height = 1.*k_default_floor_hight;
                    }
                    double const k_max_dh = 10; // m
                    double dh = ff_position.altitude - previous_ff_position.altitude;
#if 1
                    dh = altitude_filter->process(dh);
#endif
                    if (!isUseBarometer)
                    {
                        dh = 0; // zero altitude data when no bado insalled
                    }
                    else
                    {
                        dh = (std::abs(dh) <= k_max_dh) ? dh : 0; // altitude outliers rejection
                    }
                    
                    fppe_inc.d_floor = dh / floor_height;

                    //fppe_inc.d_floor = ff_position.floor_number - previous_ff_position.floor_number;
                    fppe_inc.d_floor_std = 0.05 * dtau / 500;
                }
                else
                {
                    // disable multifloor model for mfp
                    fppe_inc.d_floor = fppe_inc.d_floor_std = 0.;
                }

                // is_motion flag calculation
                if (mFF->getPlatformType() == PlatformType::pftp_FORKLIFT)
                {
                    const double vdr_motion_threshold = 0.25; // m/sec
                    double vehicle_speed = std::sqrt(fppe_inc.d_x * fppe_inc.d_x + fppe_inc.d_y * fppe_inc.d_y) / (dtau*1e-3);
                    fppe_inc.is_motion = (vehicle_speed > vdr_motion_threshold) ? true : false ;
                }
                else if (mFF->getPlatformType() == PlatformType::pftp_PEDESTRIAN)
                {
                    //assume is_motion if data is invalid
                    fppe_inc.is_motion = (tpn_data.pdr.is_valid == false || tpn_data.pdr.stride_length > 0);

                    //additional logic for step (assuming step is true if user is on the escalator or stairs)
                    if (tpn_data.position.mode_of_transit == 3 || tpn_data.position.mode_of_transit == 4 || tpn_data.position.mode_of_transit == 5)
                    {
                        fppe_inc.is_motion = true;
                    }
                }
                else
                {
                    // is_motion is true by default
                    fppe_inc.is_motion = true;
                }

                //2 -elevator, 3-stairs, 4/5 - escalator walking/standing
                if (tpn_data.position.mode_of_transit == 2 || /*tpn_data.position.mode_of_transit == 3 ||*/ tpn_data.position.mode_of_transit == 4 || tpn_data.position.mode_of_transit == 5)
                    fppe_inc.is_transit = true;
                else
                    fppe_inc.is_transit = false;

                double dx_sigma = (mFF->getPlatformType() == PlatformType::pftp_FORKLIFT) ? 0.01 : 0.1; // decreased TPN position nois for fork lift
                double dy_sigma = dx_sigma;

                double var_l = pow( 0.2 * dtau, 2 );

                double mis_std = mis_noise_est.process( tpn_data.pdr.misalignment, tpn_data.pdr.misalignment_p1, tpn_data.pdr.misalignment_p2 );
                double l0_2 = 0;

                fppe_inc.mis_std = mis_std / 180 * M_PI;
                fppe_inc.covariance_yx[0][0] = std::max((dx_sigma * dx_sigma * dtau / 50 + l0_2), 0.000001 * dtau / 50);
                fppe_inc.covariance_yx[1][1] = std::max((dy_sigma * dy_sigma * dtau / 50 + l0_2), 0.000001 * dtau / 50);
                fppe_inc.covariance_yx[0][1] = fppe_inc.covariance_yx[1][0] = 0.0; // TODO: covariance xy calculation

                fppe_inc.attitude.is_valid = mff_attitude.is_valid;

                fppe_inc.attitude.quaternion[0] = mff_attitude.quaternion[0];
                fppe_inc.attitude.quaternion[1] = mff_attitude.quaternion[1];
                fppe_inc.attitude.quaternion[2] = mff_attitude.quaternion[2];
                fppe_inc.attitude.quaternion[3] = mff_attitude.quaternion[3];

                fppe_inc.attitude.covariance_quaternion[0][0] = 1e-3; // ! debug value
                fppe_inc.attitude.covariance_quaternion[1][1] = 1e-3; // ! debug value
                fppe_inc.attitude.covariance_quaternion[2][2] = 1e-3; // ! debug value
                fppe_inc.attitude.covariance_quaternion[3][3] = 1e-3; // ! debug value

                Ok_to_process = ( ff_position.is_valid && previous_pos_valid );

                const int64_t max_gap_ms = 2500;
                if ( dtau > max_gap_ms )
                {
                    fppe_inc.attitude.is_valid = false;
                    Ok_to_process = false;
                }
            }
            else  // previous position was not valid, checking if new is valid
            {
                if ( ff_position.is_valid )
                {
                    previous_pos_valid = true;
                }
            }

            if ( Ok_to_process ) //&& tpn_data.timestamp > 0) // ??? WiFi should work anyway !TODO Separate data validity
            {
                processMFP( mag, fppe_inc.timestamp );
                processIncrements( fppe_inc );
            }

            previous_ff_position = ff_position;

        }

        /**
        * pushes WiFi measurement into the processing pipeline
        * \param[in] scan_wifi WiFi measurements vector with timestamp
        */
        void processWiFi( const Fppe::WiFiScanResult &scan_wifi )
        {
            WiFi_Measurement meas;
            int64_t scan_time = scan_wifi.timestamp;
            ++wifi_scan_counter;

#if 0 // android P sacn rate imitation
            if (scan_time % 120000 > 20000)
            {
                return;
            }
#endif

            const int64_t k_max_wifi_meas_delay = 5000; // ms
            const int64_t k_min_wifi_meas_delay = 3000; // ms
            int64_t earliest_meas_time = std::max(wifi_last_scan_time, scan_time - k_max_wifi_meas_delay);
            earliest_meas_time = std::min(earliest_meas_time, scan_time - k_min_wifi_meas_delay); // patched to increase wifi data avalaibility on bad smartfones

            for ( std::vector<Fppe::WiFiMeasurement>::const_iterator it = scan_wifi.scanWiFi.cbegin(); it != scan_wifi.scanWiFi.cend(); ++it )
            {
                int64_t meas_time = it->timestamp;

                //bool is_utilized = (meas_time > earliest_meas_time) && (meas_time <= scan_time);
                bool is_utilized = (meas_time > earliest_meas_time) && (meas_time > 0);
                if (is_utilized)
                {
                    WiFi_ApInfo ap = { it->mac, it->rssi, it->frequency, meas_time };
                    meas.push_back(ap);
                    wifi_last_scan_time = scan_time;
                }

#if 0 // an additional wifi input data output
                static std::ofstream wifi_log("wifi_scans.csv");
                wifi_log << wifi_scan_counter;
                wifi_log << ", " << scan_time;
                wifi_log << ", " << meas_time;
                wifi_log << ", " << it->mac;
                wifi_log << ", " << int(it->rssi);
                wifi_log << ", " << it->frequency;
                wifi_log << ", " << int(is_utilized);
                wifi_log << std::endl;
                wifi_log.flush();
#endif
            }

            if ( meas.size() > 0 )
            {
                mFF->process_wifi(meas, scan_time, wifi_scan_counter);
            }
        }

        /**
        * pushes BLE measurement into the processing pipeline
        * \param[in] scan_ble BLE measurements vector with timestamp
        */
        void processBLE(const Fppe::BleScanResult &scan_ble)
        {
            bool typeOS = scan_ble.scanBle[0].hasMAC;
            mFF->setOsType(scan_ble.scanBle[0].hasMAC ? OperationSystemType::OS_ANDROID : OperationSystemType::OS_IOS);
            processBLEForFingerprint(scan_ble);
            processBLEForProximity(scan_ble);
        }

        /**
        * pushes BLE measurement into the processing pipeline
        * \param[in] scan_ble BLE measurements vector with timestamp
        */
        void processBLEForFingerprint(const Fppe::BleScanResult &scan_ble)
        {
            WiFi_Measurement meas;
            int64_t t = scan_ble.timestamp;
            ++ble_scan_counter;

            for ( std::vector<Fppe::BleMeasurement>::const_iterator it = scan_ble.scanBle.cbegin(); it != scan_ble.scanBle.cend(); ++it )
            {
                WiFi_ApInfo ap = { it->mac, it->rssi, it->frequency, it->timestamp, it->txPower };

                //if (it->hasMAC == false)
                {
                    ap.bssid = getBleHash( it->major, it->minor, it->uuid );
                }

                meas.push_back( ap );
            }

            if ( meas.size() > 0 )
            {
                mFF->process_beacons( meas, t, ble_scan_counter );
            }
        }

        /**
        * calculates number of proximity beacons of specified type on specified floor in BLP-DB
        * \param[in] floor - phisical floor number
        * \param[in] beacon_type - beacon type
        * \param[out] number of found beacons or -1 if BLP-DB is not initializaed or -2 if venue is not initialized
        */
        int getProximityBeaconsNumber(const int16_t floor, const BleBeaconType beacon_type) 
        {
            int result(-2);
            
            if (mCoordConverter.IsInitialized())
            {
                int16_t pf_floor;
                TPN_converter.SetCurrentRealFloor(floor);
                TPN_converter.GetCurrentLogicalFloor(&pf_floor);
                result = mFF->get_proximity_beacons_number(pf_floor, beacon_type);
            }
            return result;
        }

        /**
        * pushes BLE measurement into the processing pipeline
        * \param[in] scan_ble BLE measurements vector with timestamp
        */
        void processBLEForProximity( const Fppe::BleScanResult &scan_ble )
        {
            int64_t t = scan_ble.timestamp;
            ++ble_proximity_scan_counter;

            Fppe::BleScanResult ble = scan_ble;
           //for(auto it = ble.scanBle.begin(); it != ble.scanBle.end(); ++it)
           //   it->rssi += 10;

            if (scan_ble.scanBle.size() > 0)
            {
                //mFF->process_beacons_proximity(scan_ble, t, ble_proximity_scan_counter);
                mFF->process_beacons_proximity(ble, t, ble_proximity_scan_counter);
            }
        }

        /**
        * pushes Magnetic measurement into the processing pipeline
        * \param[in] mag_data magnetic vector with timestamp
        */
        void processMFP( const MagneticVector &mag_data, int64_t timestamp )
        {
            std::vector<double> vMag { mag_data.mX, mag_data.mY, mag_data.mZ };
            mFF->process_mag( vMag, timestamp );
        }

        /**
        * pushes external location measurement into the processing pipeline
        * \param[in] pos external position
        */
        void processExternalPosition( const Fppe::Position &external_pos )
        {
            PositionWithUncertainties local_pos = {};

            bool success = mCoordConverter.Geo2Local( external_pos.lattitude, external_pos.longitude, &local_pos.x, &local_pos.y );
            local_pos.heading = external_pos.azimuth * M_PI / 180; //TODO add alpha angle!!!
            local_pos.std_h = external_pos.azimuth_std * M_PI / 180; //TODO check!

            mCoordConverter.Geo2Local_cov_matrix( external_pos.covariance_lat_lon, local_pos.cov_xy );

            int16_t pf_floor;
            TPN_converter.SetCurrentRealFloor(external_pos.floor_number);
            TPN_converter.GetCurrentLogicalFloor(&pf_floor);

            local_pos.z = pf_floor;
            local_pos.std_z = external_pos.floor_std;
            local_pos.timestamp = external_pos.timestamp;
            local_pos.valid = true;

            mFF->process_external_pos( local_pos );
        }

        /**
        * pushes ble_collaboration into the processing pipeline
        * \param[in] ble_collaboration position
        */
        void processInputCollaboration(std::vector <Fppe::CollaborationData> &collaboration_position)
        {
            mFF->process_collaboration(collaboration_position);
        }

        /**
        * Sets main filter position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setPositionCallbackMixed( Fppe::IPositionUpdate *pPosCbk )
        {
            if ( mCbkMix != NULL ) delete mCbkMix;

            mCbkMix = new CallbackAdapter(mCoordConverter, TPN_converter, pPosCbk);
            mFF->setCallbackMixed( mCbkMix );

        }

        /**
        * Sets WiFi only position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setPositionCallbackWiFi( Fppe::IPositionUpdate *pPosCbk )
        {
            if ( mCbkWiFi != NULL ) delete mCbkWiFi;

            mCbkWiFi = new CallbackAdapter(mCoordConverter, TPN_converter, pPosCbk);
            mFF->setCallbackWiFi( mCbkWiFi );
        }

        /**
        * Sets BLE only position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setPositionCallbackBLE( Fppe::IPositionUpdate *pPosCbk )
        {
            if ( mCbkBle != NULL ) delete mCbkBle;

            mCbkBle = new CallbackAdapter(mCoordConverter, TPN_converter, pPosCbk);
            mFF->setCallbackBle( mCbkBle );
        }

        /**
        * Sets proximity position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setPositionCallbackBLEProximity(Fppe::IPositionUpdate *pPosCbk)
        {
            if (mCbkBleProximity != NULL) delete mCbkBleProximity;

            mCbkBleProximity = new CallbackAdapter(mCoordConverter, TPN_converter, pPosCbk);
            mFF->setCallbackBleProximity(mCbkBleProximity);
        }

        /**
        * Sets proximity position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setExtendedProximityCallback(Fppe::IExtendedProximityUpdate *pProximityCbk)
        {
            if (mCbkExtendedProximity != NULL) delete mCbkExtendedProximity;

            mCbkExtendedProximity = new ExtendedProximityCallbackAdapter(mCoordConverter, TPN_converter, pProximityCbk);
            mFF->setCallbackExtendedProximity(mCbkExtendedProximity);
        }

        /**
        * Sets MFP only(mfp+pdr) position callback
        * param[in] pPosCbk pointer to the callback implementation
        */
        void setPositionCallbackMFP( Fppe::IPositionUpdate *pPosCbk )
        {
            if ( mCbkMfp != NULL ) delete mCbkMfp;

            mCbkMfp = new CallbackAdapter(mCoordConverter, TPN_converter, pPosCbk);
            mFF->setCallbackMFP( mCbkMfp );
        }

        /**
        * Sets venue detection WiFi callback
        * param[in] pVenueDetectCbk pointer to the callback implementation
        */
        void setVenueDetectionCallbackWiFi(Fppe::IVenueDetectionUpdate *pVenueDetectCbk)
        {
            if (mCbkVenueDetectWiFi != NULL) delete mCbkVenueDetectWiFi;

            mCbkVenueDetectWiFi = new VenueDetectionCallbackAdapter(pVenueDetectCbk);
            mFF->setVenueCallbackWiFi(mCbkVenueDetectWiFi);
        }

        /**
        * initialize WiFi module and loads fingerprint from  the file
        * \param[in] wifi_db_name fingerprint filename
        * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
        * \return success status
        */
        Fppe::ReturnStatus initializeWiFI( const char* const pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p )
        {
            Fppe::ReturnStatus result = initializeWiFi3(pWiFiMap, wifiFileSizeInBytes, min_p);
            return result;
        }
        
        /**
        * initialize WiFi module and loads fingerprint from specified memory
        * this method suppots wifi3 formats
        * \param[in] pWiFiMap - pointer to WiFiMap
        * \param[in] wifiFileSizeInBytes - size of WiFiMap in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeWiFi3(const char* pWiFiMap, const size_t wifiFileSizeInBytes, const double min_p)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            size_t szHeader(0);

            // read and check header
            bool header_is_ok = true;
            {
                std::string line;
                const int line_length = 1000;
                char line_buf[line_length];

                iostreambuf<char> iostreamBuffer((char *)pWiFiMap, wifiFileSizeInBytes);
                std::iostream stringstr(&iostreamBuffer);

                stringstr.get(line_buf, line_length - 1, '\n');
                szHeader = strlen(line_buf);
                stringstr.ignore(7777, '\n');

                line = std::string(line_buf);
                line = line + "\n";

                if (strcmp(line.c_str(), "WIFI_DB VER 3\r\n") != 0)
                {
                    header_is_ok = false;
                }
            }
            result = (header_is_ok == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT;

            // check data: the format does not contains any information for data checking

            // load data
            if (header_is_ok)
            {
                bool success = mFF->initializeWiFi(pWiFiMap + szHeader, wifiFileSizeInBytes, min_p);
                result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
                wfpInfo = (success == true) ? fp3Header().initializeWfp3() : fp3Header();
            }

            return result;
        }

        /**
        * initialize WiFi module and loads fingerprint from specified memory
        * this method suppots wifi4 formats
        * \param[in] pWiFiMap - pointer to WiFiMap
        * \param[in] wifiFileSizeInBytes - size of WiFiMap in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeWiFi4(const char* pWiFiMap, const size_t wifiFileSizeInBytes)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            Wfp4Header fpHeader;

            // check input buffer, header and data
            bool header_is_ok(false), data_are_ok(false), success(false);

            if ((nullptr != pWiFiMap) && (wifiFileSizeInBytes >= fpHeader.getHeaderSizeInBytes()))
            {
                //parse header
                fpHeader.parse(pWiFiMap);

                // check header
                header_is_ok = fpHeader.check();
                result = (false == header_is_ok ) ? Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT : result;

                // check data : wifi4 format does not provide actual crc for data integrity checking now
                data_are_ok = wifiFileSizeInBytes > static_cast<size_t>(fpHeader.getHeaderSizeInBytes()); // data buffer has non zero size
                result = (false == data_are_ok) ? Fppe::ReturnStatus::STATUS_WRONG_DATA : result;
            }

            // db info logging
            std::ostream *pf_log = GetLogStream("pf_log");
            if (pf_log)
            {
                *pf_log << "WFP DB: " << (void *)pWiFiMap << ", " << wifiFileSizeInBytes << " DB_HEADER < ";
                if (header_is_ok)
                {
                    fpHeader.print(*pf_log);
                }
                *pf_log << " >";
                *pf_log << std::endl;
            }

            WFPVenue wfp_venue = fpHeader.getVenue();

            // patch: for floor params in fingerprints built by FPBL early v1.1.0
            PatchVenueFloorParameters((BaseVenueType)wfp_venue, fpHeader.getBuilderVersion(), wfp_venue.floor_shift, wfp_venue.floor_zero_enable);
            // end of patch

            // initialize
            if (header_is_ok && data_are_ok)
            {
                size_t szHeader = fpHeader.getHeaderSizeInBytes();
                size_t szData = wifiFileSizeInBytes - szHeader;
                
                success = mFF->initializeWiFi(pWiFiMap + szHeader, szData, (wfp_venue.minProbMetric != 0.0) ? wfp_venue.minProbMetric : k_default_wifi_min_prob_metric);
                result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_WRONG_DATA;
                wfpInfo = (success == true) ? fpHeader : (FPHeader)(fp3Header());
            }

            // update venue if it has not inited yet
            bool venue_isnt_inited = (ffVenue.size_x == 0) || (ffVenue.size_y == 0);
            if (header_is_ok && venue_isnt_inited) // set venue if DB-header is fine and venue has not been set befor, it is not requred that db-data is ok
            {
                setVenueParams(static_cast<BaseVenueType>(wfp_venue));
            }

            // db loading status logging
            if (0 != pf_log)
            {
                *pf_log << "WFP initialization status: " << (header_is_ok ? 1 : 0) << (data_are_ok ? 1 : 0) << (success ? 1 : 0);
                *pf_log << std::endl;
            }

            return result;
        }

        /**
        * initialize WiFi module and loads fingerprint from specified memory
        * this method suppots wifi3 and wifi4 formats
        * \param[in] pWiFiMap - pointer to WiFiMap
        * \param[in] wifiFileSizeInBytes - size of WiFiMap in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeWiFI(const char* const pWiFiMap, const size_t wifiFileSizeInBytes)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT;
            
            //try to initialize wifi4 format
            result = initializeWiFi4(pWiFiMap, wifiFileSizeInBytes);

            if (result == Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT)
            {
                result = initializeWiFi3(pWiFiMap, wifiFileSizeInBytes, k_default_wifi_min_prob_metric);
            }

            return result;
        }

        /**
        * initialize BLE module and loads fingerprint from the specified memory
        * \param[in] ble_db_name fingerprint filename
        * \param[in] min_p validity metric threshold. TODO to be moved in fingerprint file
        * \return success status
        */
        Fppe::ReturnStatus initializeBLE( const char* const pBleMap, const size_t bleMapSizeInBytes, const double min_p )
        {
            Fppe::ReturnStatus result = initializeBLE3(pBleMap, bleMapSizeInBytes, min_p);
            return result;
        }

        /**
        * initialize BLE module and loads fingerprint from specified memory
        * this method suppots ble3 formats
        * \param[in] pBleMap - pointer to BleMap
        * \param[in] bleMapSizeInBytes - size of BleMap in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeBLE3(const char* pBleMap, const size_t bleMapSizeInBytes, const double min_p)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            size_t szHeader(0);

            // read and check header
            bool header_is_ok = true;
            {
                std::string line;
                const int line_length = 1000;
                char line_buf[line_length];

                iostreambuf<char> iostreamBuffer((char *)pBleMap, bleMapSizeInBytes);
                std::iostream stringstr(&iostreamBuffer);

                stringstr.get(line_buf, line_length - 1, '\n');
                szHeader = strlen(line_buf);
                stringstr.ignore(7777, '\n');

                line = std::string(line_buf);
                line = line + "\n";

                if ((strcmp(line.c_str(), "BLE_DB VER 3\r\n") != 0) &&
                    (strcmp(line.c_str(), "WIFI_DB VER 3\r\n") != 0))
                {
                    header_is_ok = false;
                }
            }
            result = (header_is_ok == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT;

            // check data: the format does not contains any information for data checking

            // load data
            if (header_is_ok)
            {
                bool success = mFF->initializeBLE(pBleMap + szHeader, bleMapSizeInBytes, min_p);
                result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
                bfpInfo = (success == true) ? fp3Header().initializeBfp3() : fp3Header();
            }

            return result;
        }

        /**
        * initialize BLE module and loads fingerprint from specified memory
        * this method suppots ble4 formats
        * \param[in] pBleMap - pointer to BleMap
        * \param[in] bleMapSizeInBytes - size of BleMap in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeBLE4(const char* pBleMap, const size_t bleMapSizeInBytes)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            Bfp4Header fpHeader;


            // check input buffer, header and data
            bool header_is_ok(false), data_are_ok(false), success(false);

            if ((nullptr != pBleMap) && (bleMapSizeInBytes >= fpHeader.getHeaderSizeInBytes()))
            {
                //parse header
                fpHeader.parse(pBleMap);

                // check header
                header_is_ok = fpHeader.check();
                result = (false == header_is_ok) ? Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT : result;

                // check data: ble4 format does not provide actual crc for data integrity checking now
                data_are_ok = bleMapSizeInBytes > static_cast<size_t>(fpHeader.getHeaderSizeInBytes()); // data buffer has non zero size
                result = (false == data_are_ok) ? Fppe::ReturnStatus::STATUS_WRONG_DATA : result;
            }

            // db info logging
            std::ostream *pf_log = GetLogStream("pf_log");
            if (pf_log)
            {
                *pf_log << "BFP DB: " << (void *)pBleMap << ", " << bleMapSizeInBytes << " DB_HEADER < ";
                if (header_is_ok)
                {
                    fpHeader.print(*pf_log);
                }
                *pf_log << " >";
                *pf_log << std::endl;
            }

            BFPVenue bfp_venue = fpHeader.getVenue();

            // patch: for floor params in fingerprints built by FPBL early v1.1.0
            PatchVenueFloorParameters((BaseVenueType)bfp_venue, fpHeader.getBuilderVersion(), bfp_venue.floor_shift, bfp_venue.floor_zero_enable);
            // end of patch


            // initialize
            if (header_is_ok && data_are_ok)
            {
                size_t szHeader = fpHeader.getHeaderSizeInBytes();
                size_t szData = bleMapSizeInBytes - szHeader;

                success = mFF->initializeBLE(pBleMap + szHeader, szData, (bfp_venue.minProbMetric != 0.0) ? bfp_venue.minProbMetric : k_default_ble_min_prob_metric);
                result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_WRONG_DATA;
                bfpInfo = (success == true) ? fpHeader : (FPHeader)(fp3Header());
            }

                // update venue if it has not inited yet
            bool venue_isnt_inited = (ffVenue.size_x == 0) || (ffVenue.size_y == 0);
            if (header_is_ok && venue_isnt_inited) // set venue if DB-header is fine and venue has not been set befor, it is not requred that db-data is ok
            {
                setVenueParams(static_cast<BaseVenueType>(bfp_venue));
            }

            // db info logging
            std::vector<LogDescriptor> log_descriptors;
            mFF->getLogDescriptors(&log_descriptors);
            if (pf_log)
            {
                *pf_log << "BFP initialization status: " << (header_is_ok ? 1 : 0) << (data_are_ok ? 1 : 0) << (success ? 1 : 0);
                *pf_log << std::endl;
            }

            return result;
        }

        /**
        * initialize BLE module and loads fingerprint from  the specified memory buffer
        * \param[in] pBleMap fingerprint buffer
        * \param[in] bleFileSizeInBytes size of the buffer in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeBLE(const char* const pBleMap, const size_t bleFileSizeInBytes)// new interface
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT;

            //try to initialize wifi4 format
            result = initializeBLE4(pBleMap, bleFileSizeInBytes);

            if (result == Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT)
            {
                result = initializeBLE3(pBleMap, bleFileSizeInBytes, k_default_ble_min_prob_metric);
            }

            return result;
        }

        /**
        * initialize BLE proximity module and loads fingerprint from specified memory
        * this method suppots ble3 formats
        * \param[in] pBleMap - pointer to BleMap
        * \param[in] bleMapSizeInBytes - size of BleMap in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeBleProximity(const char* pBlProxDb, const size_t bleFileSizeInBytes)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT;

            //try to initialize format-4 first
            result = initializeBLPox4(pBlProxDb, bleFileSizeInBytes);

            if (result == Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT)
            {
                result = initializeBLPox3(pBlProxDb, bleFileSizeInBytes);
            }

            return result;
        }

        /**
        * initialize BLE proximity module from fp3 format
        * this method suppots ble3 formats
        * \param[in] pBlProxDb - pointer to BLE proximitry DB
        * \param[in] pBlProxDbSizeInBytes - size of BleMap in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeBLPox3(const char* pBlProxDb, const size_t bleFileSizeInBytes)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            size_t szHeader(0);

            // read and check header
            bool header_is_ok = true;
            {
                std::string line;
                const int line_length = 1000;
                char line_buf[line_length];

                iostreambuf<char> iostreamBuffer((char *)pBlProxDb, bleFileSizeInBytes);
                std::iostream stringstr(&iostreamBuffer);
                //std::iostream *stringstr = new std::iostream(&iostreamBuffer);

                stringstr.get(line_buf, line_length - 1, '\n');
                szHeader = strlen(line_buf) + 1;
                stringstr.ignore(7777, '\n');

                line = std::string(line_buf);
                line = line + "\n";

                if ((strcmp(line.c_str(), "BLE_DB VER 5\r\n") != 0) &&
                    (strcmp(line.c_str(), "BLP_DB VER 3\r\n") != 0))
                {
                    header_is_ok = false;
                }
            }

            //delete stringstr;

            result = (header_is_ok == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT;

            // check data: the format does not contains any information for data checking

            // load data
            if (header_is_ok)
            {
                bool success = mFF->initializeBLEProximity_for_positioning(pBlProxDb + szHeader, bleFileSizeInBytes - szHeader, mCoordConverter, k_default_filter_length, k_default_repeat_number, k_default_cutoff_threshold);
                success &= mFF->initializeBLEProximity_for_detection(pBlProxDb + szHeader, bleFileSizeInBytes - szHeader, mCoordConverter, k_default_filter_length, k_default_repeat_number, k_default_cutoff_threshold);
                success &= mFF->initializeBLEProximity_for_start(pBlProxDb + szHeader, bleFileSizeInBytes - szHeader, mCoordConverter, k_default_filter_length_in_start, k_default_repeat_number_in_start, k_default_cutoff_threshold_start);
            
                result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
                proximityInfo = (success == true) ? fp3Header().initializeBlp3() : fp3Header();
            }

            return result;
        }

        /**
        * initialize BLE proximity module from fp4 format
        * this method suppots ble3 formats
        * \param[in] pBleMap - pointer to BleMap
        * \param[in] bleMapSizeInBytes - size of BleMap in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeBLPox4(const char* pBlProxDb, const size_t bleFileSizeInBytes)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            Blp4Header fpHeader;

            // check input buffer, header and data
            bool header_is_ok(false), data_are_ok(false), success(false);
            if ((nullptr != pBlProxDb) && (bleFileSizeInBytes >= fpHeader.getHeaderSizeInBytes()))
            {
                //parse header
                fpHeader.parse(pBlProxDb);

                // check header
                header_is_ok = fpHeader.check();
                result = (false == header_is_ok) ? Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT : result;

                // check data: ble4 formap does not provide actual crc for data integrity checking now
                data_are_ok = bleFileSizeInBytes > static_cast<size_t>(fpHeader.getHeaderSizeInBytes()); // data buffer has non zero size
                result = (false == data_are_ok) ? Fppe::ReturnStatus::STATUS_WRONG_DATA : result;
            }

            // db info logging
            std::ostream *pf_log = GetLogStream("pf_log");
            if (pf_log)
            {
                *pf_log << "BLP DB: " << (void *)pBlProxDb << ", " << bleFileSizeInBytes << " DB_HEADER < ";
                if (header_is_ok)
                {
                    fpHeader.print(*pf_log);
                }
                *pf_log << " >";
                *pf_log << std::endl;
            }

            BProxVenue blp_venue = fpHeader.getVenue();

            // patch: for floor params in fingerprints built by FPBL early v1.1.0
            PatchVenueFloorParameters((BaseVenueType)blp_venue, fpHeader.getBuilderVersion(), blp_venue.floor_shift, blp_venue.floor_zero_enable);
            // end of patch

            // update venue if it has not been inited yet
            bool venue_isnt_inited = (ffVenue.size_x == 0) || (ffVenue.size_y == 0);
            if (header_is_ok && venue_isnt_inited) // set venue if DB-header is fine and venue has not been set befor, it is not requred that db-data is ok
            {
                setVenueParams(static_cast<BaseVenueType>(blp_venue));
            }

            // initialize
            if (header_is_ok && data_are_ok)
            {
                size_t szHeader = fpHeader.getHeaderSizeInBytes();
                size_t szData = bleFileSizeInBytes - szHeader;

                success = mFF->initializeBLEProximity_for_positioning(
                    pBlProxDb + szHeader,
                    szData,
                    mCoordConverter,
                    (blp_venue.filter_length != 0) ? blp_venue.filter_length : k_default_filter_length,
                    (blp_venue.repeat_number != 0) ? blp_venue.repeat_number : k_default_repeat_number,
                    (blp_venue.cutoff_threshold != 0) ? blp_venue.cutoff_threshold : k_default_cutoff_threshold);

                success = mFF->initializeBLEProximity_for_detection(
                    pBlProxDb + szHeader,
                    szData,
                    mCoordConverter,
                    (blp_venue.filter_length != 0) ? blp_venue.filter_length : k_default_filter_length,
                    (blp_venue.repeat_number != 0) ? blp_venue.repeat_number : k_default_repeat_number,
                    (blp_venue.cutoff_threshold != 0) ? blp_venue.cutoff_threshold : k_default_cutoff_threshold);

                success &= mFF->initializeBLEProximity_for_start(
                    pBlProxDb + szHeader, 
                    szData,
                    mCoordConverter,
                    k_default_repeat_number_in_start, 
                    k_default_filter_length_in_start, 
                    k_default_cutoff_threshold_start);
                result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_WRONG_DATA;
                proximityInfo = (success == true) ? fpHeader : (FPHeader)fp3Header();
            }

            // initialization results logging
            if (pf_log)
            {
                *pf_log << "BLP initialization status: " << (header_is_ok ? 1 : 0) << (data_are_ok ? 1 : 0) << (success ? 1 : 0);
                *pf_log << std::endl;
            }

            return result;
        }

        /**
        * initialize BLE proximity module and loads fingerprint from  the specified memory buffer
        * \param[in] pBleMap fingerprint buffer
        * \param[in] bleFileSizeInBytes size of the buffer in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeBLEProximity(const char* const pBleMap, const size_t bleFileSizeInBytes)// new interface
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT;

            //try to initialize wifi4 format
            result = initializeBleProximity(pBleMap, bleFileSizeInBytes);

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
        Fppe::ReturnStatus initializeMFP( const char* const pMFPMap, const size_t mfpFileSizeInBytes, double max_X, double max_Y, double cellSize, int minFloor, int maxFloor )
        {
            bool success = mFF->initializeMFP( pMFPMap, mfpFileSizeInBytes, max_X, max_Y, cellSize, minFloor, maxFloor );

            Fppe::ReturnStatus result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            mfpInfo = (success == true) ? fp3Header().initializeMfp3() : fp3Header();

            return result;
        }

        /**
        * initialize MFP module and loads fingerprint from  the specified memory buffer
        * \param[in] pMFPMap fingerprint buffer
        * \param[in] mfpFileSizeInBytes size of the buffer in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeMFP4(const char* const pMFPMap, const size_t mfpFileSizeInBytes)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            Mfp4Header fpHeader;

            // check input buffer, header and data
            bool header_is_ok(false), data_are_ok(false), success(false);

            if ((nullptr != pMFPMap) && (mfpFileSizeInBytes >= fpHeader.getHeaderSizeInBytes()))
            {
                //parse header
                fpHeader.parse(pMFPMap);

                // check header
                header_is_ok = fpHeader.check(mfpFileSizeInBytes);
                result = (false == header_is_ok) ? Fppe::ReturnStatus::STATUS_UNKNOWN_FORMAT : result;

                // check data 
                data_are_ok = mfpFileSizeInBytes > static_cast<size_t>(fpHeader.getHeaderSizeInBytes()); // data buffer has non zero size
                if (header_is_ok && data_are_ok)
                {
                    data_are_ok = fpHeader.checkCRC(pMFPMap + fpHeader.getHeaderSizeInBytes(), fpHeader.getDataSize());
                }
            }

            // db info logging
            std::ostream *pf_log = GetLogStream("pf_log");
            if (pf_log)
            {
                *pf_log << "MFP DB: " << (void *)pMFPMap << ", " << mfpFileSizeInBytes << " DB_HEADER < ";
                if (header_is_ok)
                {
                    fpHeader.print(*pf_log);
                }
                *pf_log << " >";
                *pf_log << std::endl;
            }

            MFPVenue mfp_venue = fpHeader.getVenue();

            // patch: for floor params in fingerprints built by FPBL early v1.1.0
            PatchVenueFloorParameters((BaseVenueType)mfp_venue, fpHeader.getBuilderVersion(), mfp_venue.floor_shift, mfp_venue.floor_zero_enable);
            // end of patch

            // initialize
            if (header_is_ok && data_are_ok)
            {
                int minFloor = 0;
                int maxFloor = mfp_venue.floors_count - 1;
                success = mFF->initializeMFP(pMFPMap + fpHeader.getHeaderSizeInBytes(), fpHeader.getDataSize(),
                    mfp_venue.size_x, mfp_venue.size_y, mfp_venue.cell_size, minFloor, maxFloor);
                result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_WRONG_DATA;
                mfpInfo = (success == true) ? fpHeader : (FPHeader)(fp3Header());
            }

            if (header_is_ok) // set/reset venue if DB-header has been loaded
            {
                setVenueParams(static_cast<BaseVenueType>(mfp_venue));
            }

            // db status logging
            if (0 != pf_log)
            {
                *pf_log << "MFP initialization status: " << (header_is_ok ? 1 : 0) << (data_are_ok ? 1 : 0) << (success ? 1 : 0);
                *pf_log << std::endl;
            }

            return result;
        }

        /**
        * initialize MFP module and loads fingerprint from  the specified memory buffer
        * \param[in] pMFPMap fingerprint buffer
        * \param[in] mfpFileSizeInBytes size of the buffer in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeMFP(const char* const pMFPMap, const size_t mfpFileSizeInBytes) // new interface
        {
            Fppe::ReturnStatus result = result = initializeMFP4(pMFPMap, mfpFileSizeInBytes);

            return result;
        }

        /**
        * initialize Map Matching module and loads map from  the specified memory buffer
        * \param[in] pMap map buffer
        * \param[in] mapFileSizeInBytes size of the buffer in bytes
        * \return success status
        */
        Fppe::ReturnStatus initializeMapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;

            std::ostream *pf_log = GetLogStream("pf_log");
            if (pf_log)
            {
                *pf_log << "MM Map: " << (void *)pMap << ", " << mapFileSizeInBytes << std::endl;
            }

            bool success = false;
            if ((nullptr != pMap) && (mapFileSizeInBytes > 0))
            {
                BaseVenueType venue = getVenueParams(); // we get map cell size from JSON file
                success = mFF->initializeMapMatching(pMap, mapFileSizeInBytes, venue.size_x, venue.size_y, venue.floor_shift, venue.floor_zero_enable, mCoordConverter);
                result = (success == true) ? Fppe::ReturnStatus::STATUS_SUCCESS : Fppe::ReturnStatus::STATUS_WRONG_DATA;
            }

            if (pf_log)
            {
                *pf_log << "MM initialization status: " << (success ? 1 : 0) << std::endl;
            }

            return result;
        }

        /**
        * get information about WFP DB
        * \return FPHeader structure
        */
        FPHeaderBaseType getWfpInfo()
        {
            return wfpInfo.getInstance();
        }

        /**
        * get information about MFP DB
        * \return FPHeader structure
        */
        FPHeaderBaseType getMfpInfo()
        {
            return mfpInfo.getInstance();
        }

        /**
        * get information about BFP DB
        * \return FPHeader structure
        */
        FPHeaderBaseType getBfpInfo()
        {
            return bfpInfo.getInstance();
        }

        /**
        * get information about proximity DB
        * \return FPHeader structure
        */
        FPHeaderBaseType getProximityDbInfo()
        {
            return proximityInfo.getInstance();
        }

        /**
        * Defines fusion filter venue local frame (old interface)
        * \param[in] venue  venue parameters with local frame origin
        * \return success
        */
        Fppe::ReturnStatus setVenueParams( const Venue &venue )
        {
            return setVenueParams(BaseVenue(venue));
        }

        /**
        * Defines venue local frame
        * \param[in] venue  venue parameters with local frame origin
        * \return success
        */
        Fppe::ReturnStatus setVenueParams(const BaseVenueType &venue)
        {
            Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;

            venue_floor_height = (venue.floors_count > 1) ? venue.floor_height : (5 * k_default_floor_hight);

            //bool succes = this->TPN_converter.SetVenueFrameParams(venue.origin_lattitude, venue.origin_longitude, venue.alfa, venue.beta, venue.origin_azimuth);
            bool succes = this->TPN_converter.SetVenueFrameParams(venue);
            succes &= mCoordConverter.SetFrameParams(venue.origin_lattitude, venue.origin_longitude, venue.alfa, venue.beta, venue.origin_azimuth);

            if (succes == true)
            {
                ffVenue = venue; // save venue params
                result = Fppe::ReturnStatus::STATUS_SUCCESS;
            }

            {   // venue info logging
                std::vector<LogDescriptor> log_descriptors;
                mFF->getLogDescriptors(&log_descriptors);
                if (log_descriptors.size() > 0)
                {
                    auto it = std::find_if(log_descriptors.begin(), log_descriptors.end(), [=](const LogDescriptor& a) {return a.first == "pf_log"; });
                    **(it->second) << "SetVenueParams: " << (succes) ? 1 : 0;
                    **(it->second) << ",, " << ffVenue.id;
                    **(it->second) << ", " << ffVenue.venue_type;
                    **(it->second) << ",, " << ffVenue.origin_lattitude;
                    **(it->second) << ", " << ffVenue.origin_longitude;
                    **(it->second) << ", " << ffVenue.origin_altitude;
                    **(it->second) << ", " << ffVenue.origin_azimuth;
                    **(it->second) << ", " << ffVenue.alfa;
                    **(it->second) << ", " << ffVenue.beta;
                    **(it->second) << ",, " << ffVenue.size_x;
                    **(it->second) << ", " << ffVenue.size_y;
                    **(it->second) << ",, " << ffVenue.floors_count;
                    **(it->second) << ", " << ffVenue.floor_shift;
                    **(it->second) << ", " << ffVenue.floor_height;
                    **(it->second) << ", " << (ffVenue.floor_zero_enable) ? 1:0;;
                    **(it->second) << std::endl;
                }
            }

            // set venue type
            mFF->setVenueType(venue.venue_type);

            mFF->setFloorHeight(venue_floor_height);
            mFF->setTimeInterval(0.5);
            //mFF->setVerticalSpeed(0.1, 1.0, k_Elevator);
            //mFF->setVerticalSpeed(0.1, 0.4, k_Escalator);
            //mFF->setVerticalSpeed(0.1, 0.3, k_Stairs);

            // set particle count detends on venue type
            int32_t min_particle_count(2000);
            switch (ffVenue.venue_type)
            {
            case kMallVenue:
            case kOfficeVenue:
            case kFactoryVenue:
                min_particle_count = (ffVenue.floors_count < 2) ? 1250 : 2000;
                break;
            case kAisleVenue:
            case kDefaultVenue:
            default:
                min_particle_count = (ffVenue.floors_count < 2) ? 1500 : 2000;
                break;
            }
            mFF->setParticleCountForMixedFilter(min_particle_count, int32_t(2.5 * min_particle_count));
            mFF->setParticleCountForMfpFilter(min_particle_count, int32_t(5 * min_particle_count));

            return result;
        }

        /**
        * Return venue structure which is used for fingerprint frame linking with GEO frame
        * \return BaseVenue structure
        */
        BaseVenueType getVenueParams()
        {
            return static_cast<BaseVenueType>(ffVenue);
        }

        void setUpdateWiFi( bool enable ) /**< updater WiFi control */
        {
            mFF->setUpdateWiFi( enable );
        }
        void setUpdateBLE( bool enable ) /**< updater BLE control */
        {
            mFF->setUpdateBLE( enable );
        }
        void setUpdateBLEProximity( bool enable ) /**< updater BLE control */
        {
            mFF->setUpdateBLEProximity( enable );
        }
        void setUpdateMFP( bool enable ) /**< updater MFP control */
        {
            mFF->setUpdateMFP( enable );
        }
        void setUpdateMapMatching(bool enable) /**< updater Map Matching control */
        {
            mFF->setUpdateMM(enable);
        }
        void setUpdateExternalPosition( bool enable )
        {
            mFF->setUpdateExternalPosition( enable );
        }
        void setUpdateCollaboration(bool enable)
        {
            mFF->setUpdateCollaboration(enable);
        }

        void setCorrectFusionFilterPosition(bool enable)
        {
            mFF->setCorrectFusionFilterPosition(enable);
        }

        /** Enables floor increment usage, switches to another motion model */
        void setFloorIncrements( bool enable )
        {
            assert( 0 && "not implemented" );
        }

        void setMagFilterEnabled( bool enable )
        {
            mFF->setMFPFilterEnabled( enable );
        }

        void setMixedFilterEnabled( bool enable )
        {
            mFF->setMixedFilterEnabled( enable );
        }

        bool getMagFilterEnabled()
        {
            return mFF->getMagFilterEnabled();
        }

        bool getMixedFilterEnabled()
        {
            return mFF->getMixedFilterEnabled();
        }

        /** Gets mg calibration params
        * \param[out] bias_cov esimated magnetic bias with covariance
        */
        bool getMagneticBias( Fppe::MagneticCalibrationParam *bias_cov )
        {
            int64_t t = 0;
            double bias[3] = {};
            double cov[3] = {};
            bool result = mFF->GetMagneticBiasAndCov(&t, bias, cov);
            
            bias_cov->timestamp = t;
            bias_cov->mX = bias[0];
            bias_cov->mY = bias[1];
            bias_cov->mZ = bias[2];
            memset( bias_cov->covarianceMatrix, 0, sizeof( bias_cov->covarianceMatrix ) );
            bias_cov->covarianceMatrix[0][0] = cov[0];
            bias_cov->covarianceMatrix[1][1] = cov[1];
            bias_cov->covarianceMatrix[2][2] = cov[2];

            return result;
        }

        /** Gets mg calibration params
        * \param[in] bias_cov initial magnetic bias with covariance
        */
        void setMagneticBias( const Fppe::MagneticCalibrationParam &bias_cov )
        {
            int64_t t = bias_cov.timestamp;

            double bias[3] = { bias_cov.mX, bias_cov.mY, bias_cov.mZ };
            double cov[3] = { bias_cov.covarianceMatrix[0][0], bias_cov.covarianceMatrix[1][1], bias_cov.covarianceMatrix[2][2] };

            mFF->SetMagneticBiasCov( cov );
            mFF->SetMagneticBias( t, bias );
        }

        /** Gets platform type
        * \return platform type
        */
        PlatformType getPlatformType()
        {
            return mFF->getPlatformType();
        }

        /** Sets platform type
        * \param[in] platform type
        */
        void setPlatformType(PlatformType platform_type)
        {
            mFF->setPlatformType(platform_type);
        }

        /** Sets BLP pulling type
        * \param[in] pulling type
        * \param[in] pulling distance
        * \param[in] pulling sigma
        */
        void setBlpPulling(ePullingType type, double pulling_distance, double pulling_sigma)
        {
            mFF->setBlpPulling(type, pulling_distance, pulling_sigma);
        }

        void setBlpDetectionEnable(bool enable)
        {
            mFF->setBlpDetectionEnable(enable);
        }

        void setBlpPositioningPdFilterParams(
            int peak_detector_max_delay_ms_in_moving,
            double descending_factor_in_moving,
            int peak_detector_max_delay_ms_in_stop,
            double descending_factor_in_stop
        )
        {
            mFF->setBlpPositioningPdFilterParams(
                peak_detector_max_delay_ms_in_moving,
                descending_factor_in_moving,
                peak_detector_max_delay_ms_in_stop,
                descending_factor_in_stop);
        }

        void setBlpDetectionPdFilterParams(
            int peak_detector_max_delay_ms_in_moving,
            double descending_factor_in_moving,
            int peak_detector_max_delay_ms_in_stop,
            double descending_factor_in_stop
        )
        {
            mFF->setBlpDetectionPdFilterParams(
                peak_detector_max_delay_ms_in_moving,
                descending_factor_in_moving,
                peak_detector_max_delay_ms_in_stop,
                descending_factor_in_stop);
        }

        void setBlpPositioningLogicParams(int filter_length, int repeat_number,
            int cutoff)
        {
            mFF->setBlpPositioningLogicParams(filter_length, repeat_number, cutoff);
        }

        void setBlpDetectionLogicParams(int filter_length, int repeat_number,
            int cutoff)
        {
            mFF->setBlpDetectionLogicParams(filter_length, repeat_number, cutoff);
        }

        /** WiFi calibration status [obsolete]
        * param[out] bias rssi bias
        * returns calibration enabled
        */
        bool getWiFiBias( double *bias )
        {
            return mFF->getWiFiBias(*bias);
        }
        /** sets WiFi calibration [obsolete]
        * param[in] bias initial rssi bias
        * param[in] delta_t time since last saved bias
        */
        void setWiFiBias( const double  &bias, int64_t delta_t )
        {
            mFF->setWiFiBias(bias, delta_t);
        }

        bool getBLEBias(double *bias)
        {
            return mFF->getOutputBLEBias(*bias);
        }

        void setBLEBias( const double  &bias, int64_t delta_t )
        {
            mFF->setBFPBias(bias, delta_t);
            mFF->setBLEProxBias(bias, delta_t);
        }

		bool getBLEBiasWithUncertainty(double *bias, double *bias_uncertainty)
		{
			return mFF->getBLEBiasWithUncertainty(*bias, *bias_uncertainty);
		}

		void setBLEBiasWithUncertainty(const double &bias, const double &bias_uncertainty, int64_t bias_age)
		{
			mFF->setBLEBiasWithUncertainty(bias, bias_uncertainty, bias_age);
		}

        /**
        * Sets fine known initial position
        * \param[in] position initial position in global frame
        */
        void setStartPosition( const Fppe::Position &position )
        {
            //TODO add return value
            double X, Y, H, H_std;
            bool success = mCoordConverter.Geo2Local( position.lattitude, position.longitude, &X, &Y );
            double covariance_local[2][2];
            success &= mCoordConverter.Geo2Local_cov_matrix(position.covariance_lat_lon, covariance_local);

            int16_t pf_floor;
            TPN_converter.SetCurrentRealFloor(position.floor_number);
            TPN_converter.GetCurrentLogicalFloor(&pf_floor);


            H = position.azimuth * M_PI / 180; // this angle does not require correction with alfa angle because this one is equal to gama (see Algorithm Description)
            //H_std = position.azimuth_std * M_PI / 180; // warning: position.azimuth_std can be used as frame missalignment angle dispersion
            H_std = M_PI; // warning: position.azimuth_std disabled to increase start robustness
            mFF->setStartPositionWithUncertainties(X, Y, pf_floor, H, covariance_local, position.floor_std, H_std, position.timestamp);
        }

        /**
        * Sets random seeds for internal randomizers
        */
        void setRandomSeeds()
        {
            mFF->setRandomSeeds();
        }

        /**
        * set use barometr
        * param[in] enable/disable
        */
        void setUseBarometer(bool enable)
        {
            isUseBarometer = enable;
            mFF->setUseBarometer(enable);
        }
        
        /**
        * set OS type
        * param[in] OS type
        */
        void setOsType(OperationSystemType os_type)
        {
            //mFF->setOsType(os_type);
        }

        void getLogDescription( std::vector<std::string> *log_descriptions )
        {
            if ( log_descriptions != NULL )
            {
                log_descriptions->resize( 0 );

                for ( auto it = logs.cbegin(); it != logs.cend(); ++it )
                {
                    log_descriptions->push_back( it->first );
                }
            }
        }


        bool setLogStream( const unsigned int &idx, std::ostream &os )
        {
            bool result = false;

            if ( logs.size() > idx )
            {
                *logs[idx].second = &os;
                result = true;
            }

            return result;
        }

    private:

        std::ostream * GetLogStream(std::string log_name)
        {
            std::vector<LogDescriptor> log_descriptors;
            mFF->getLogDescriptors(&log_descriptors);
            auto it = std::find_if(log_descriptors.begin(), log_descriptors.end(), [=](const LogDescriptor& a) {return a.first == log_name; });
            return (log_descriptors.end() != it) ? *it->second : 0;
        }


        // internal constants
        const double k_default_floor_hight = 5.; // m

        // position callback adapter
        class CallbackAdapter : public IPositionUpdateFF
        {
            public:
                CallbackAdapter(const GeoLocConverter2D &converter, TpnConverter &TPN_converter,  Fppe::IPositionUpdate *pPosCbk)
                {
                    pCbk = pPosCbk;
                    mN = 0;
                    pParticles = NULL;
                    pConverter = &converter;
                    pTpnConverter = &TPN_converter;
                }

                ~CallbackAdapter()
                {
                    delete[] pParticles;
                }
            private:
                void update( double X, double Y, double Floor, double H, double Sig, double t, const PF::Particle *state, int N )
                {
                    if ( pCbk != NULL )
                    {
                        if ( mN != N )
                        {
                            mN = N;
                            delete [] pParticles; //it is safe to delete nullptr
                            pParticles = new Fppe::Particle[mN];
                        }

                        for (int i = 0; (state != NULL) && (i < mN); i++)
                        {
                            pParticles[i].x = state[i].pos.x;
                            pParticles[i].y = state[i].pos.y;
                            pParticles[i].z = state[i].pos.level;
                            pParticles[i].w = state[i].w;
							pParticles[i].lkh = state[i].lkh;
                        }

                        // real floor calculation
                        pTpnConverter->SetCurrentLogicalFloor((int16_t)floor(Floor + 0.5));
                        int16_t real_floor;// = 3;
                        pTpnConverter->GetCurrentRealFloor(&real_floor);

						bool in_tracking = true;

						if (Sig < 0.0)
						{
							Sig *= -1;
							in_tracking = false;
						}

                        // calculation of bubble size for uncertainty area depicture
                        double bubble_size = Sig;

                        pCbk->update(X, Y, real_floor, H, bubble_size, t, pParticles, mN);

                        Fppe::Position position = {};

                        bool success = pConverter->Local2Geo( X, Y, &position.lattitude, &position.longitude );
                        position.timestamp = ( int64_t )floor( t + 0.5 );
                        position.is_valid = success && in_tracking;
                        position.floor_number = real_floor;
                        position.altitude = 0; // unknown
                        //TODO fill the other fields

                        double sumW = 0;
                        Eigen::Matrix<PF::Float, 2, 2> CovXY = Eigen::Matrix<PF::Float, 2, 2>::Zero();
                        Eigen::Matrix<PF::Float, 2, 1> Zest;
                        Eigen::Matrix<PF::Float, 2, 1> Zi;
                        Zest( 0, 0 ) = X;
                        Zest( 1, 0 ) = Y;

                        for (int i = 0; (state != NULL) && (i < mN); i++)
                        {
                            double w = state[i].w;
                            sumW += w;
                            Zi( 0, 0 ) = state[i].pos.x;
                            Zi( 1, 0 ) = state[i].pos.y;
                            CovXY += w * ( Zest - Zi ) * ( Zest - Zi ).transpose();
                        }

                        CovXY /= sumW;
                        //std::cout << CovXY << std::endl;

                        double local_cov[2][2] = { { CovXY( 0, 0 ), CovXY( 0, 1 ) }, { CovXY( 1, 0 ), CovXY( 1, 1 ) } };

                        if ((mN > 0) && (state != NULL))
                        {
                            double S = 0; // 2;
                            double c = sqrt( CovXY( 0, 0 ) ) * sqrt( CovXY( 1, 1 ) );

                            c = ( std::abs( c ) <= std::numeric_limits<double>::epsilon() ) ? 0 : CovXY( 0, 1 ) / c;
                            local_cov[0][0] = CovXY( 0, 0 ) + S;
                            local_cov[1][1] = CovXY( 1, 1 ) + S;
                            local_cov[0][1] = c * sqrt( CovXY( 0, 0 ) ) * sqrt( CovXY( 1, 1 ) );
                            local_cov[1][0] = local_cov[0][1];
                        }
                        else
                        {
                            memset( local_cov, 0, sizeof( local_cov ) );
                            local_cov[0][0] = local_cov[1][1] = Sig * Sig;
                        }

                        if (Sig < 0)
                        { // this is for debugg purpouses
                            position.covariance_lat_lon[0][0] = position.covariance_lat_lon[1][1] = 0;
                            position.covariance_lat_lon[0][0] = -Sig*Sig;
                            position.covariance_lat_lon[1][1] = -Sig*Sig;
                        }
                        else
                        {
                            pConverter->Local2Geo_cov_matrix(local_cov, position.covariance_lat_lon);
                        }

                        position.azimuth = H * 180 / M_PI;
                        pCbk->update( position );
                    }
                }
                Fppe::IPositionUpdate *pCbk;
                int mN;
                Fppe::Particle *pParticles;
                const GeoLocConverter2D *pConverter;
                TpnConverter      *pTpnConverter;
        };

        // venue detection calback adaptor
        class VenueDetectionCallbackAdapter : public IVenueDetectionUpdateFF
        {
        public:
            VenueDetectionCallbackAdapter(Fppe::IVenueDetectionUpdate *pVenueDetectionCbk)
            {
                venueCbk = pVenueDetectionCbk;
            }

            ~VenueDetectionCallbackAdapter()
            {
            }

        private:
            void update(bool pos_inside_venue, double t)
            {
                if (venueCbk != NULL)
                {
                    venueCbk->update(pos_inside_venue, t);
                }
            }
            Fppe::IVenueDetectionUpdate *venueCbk;
        };

        // venue detection calback adaptor
        class ExtendedProximityCallbackAdapter : public IExtendedProximityUpdateFF
        {
        public:
            ExtendedProximityCallbackAdapter(const GeoLocConverter2D &position_converter, const TpnConverter &tpn_converter, Fppe::IExtendedProximityUpdate *pExtendedProximityCbk)
            {
                pCbk = pExtendedProximityCbk;
                pTpnConverter = &tpn_converter;
                pPositionConverter = &position_converter;
            }


            ~ExtendedProximityCallbackAdapter()
            {
            }

        private:
            void update(double t, const BLE_position &beacon_pos)
            {
                if (pCbk != nullptr)
                {
                    Fppe::ProximityBeaconData proximity_beacon_data={};
                    
                    proximity_beacon_data.is_valid = pPositionConverter->Local2Geo(beacon_pos.loc.x, beacon_pos.loc.y, &proximity_beacon_data.lattitude, &proximity_beacon_data.longitude);
                    proximity_beacon_data.floor = pTpnConverter->GetCurrentRealFloor((int16_t)round(beacon_pos.loc.z));
                    
                    proximity_beacon_data.elevation = beacon_pos.blp_height;

                    proximity_beacon_data.major = beacon_pos.major;
                    proximity_beacon_data.minor = beacon_pos.minor;
                    //proximity_beacon_data.uuid = ;
                    
                    proximity_beacon_data.txPower = beacon_pos.txPower;
                    proximity_beacon_data.rxPower = beacon_pos.rxPower;
                    proximity_beacon_data.distance = beacon_pos.distance;

                    proximity_beacon_data.beacon_type = beacon_pos.beaconType;
                    
                    proximity_beacon_data.is_valid = beacon_pos.loc.valid;

                    proximity_beacon_data.rms = beacon_pos.loc.rms_xy;

                    pCbk->update(t, proximity_beacon_data);
                }
            }
            
            const GeoLocConverter2D *pPositionConverter;
            const TpnConverter    *pTpnConverter;
            Fppe::IExtendedProximityUpdate *pCbk;
        };

        FusionFilter *mFF;
        IPositionUpdateFF *mCbkMfp;
        IPositionUpdateFF *mCbkMix;
        IPositionUpdateFF *mCbkWiFi;
        IPositionUpdateFF *mCbkBle;
        IPositionUpdateFF *mCbkBleProximity;
        IVenueDetectionUpdateFF *mCbkVenueDetectWiFi;
        IExtendedProximityUpdateFF *mCbkExtendedProximity;

        int wifi_scan_counter;
        int64_t wifi_last_scan_time;

        int ble_scan_counter;
        int ble_proximity_scan_counter;

        //TpnDataConverter converter;
        TpnConverter TPN_converter;
        GeoLocConverter2D mCoordConverter;

        double venue_floor_height;

        bool previous_pos_valid;
        FfPosition previous_ff_position;
        std::vector<LogDescriptor> logs;
        
        MisNoiseEstimator mis_noise_est; // missalignment noise estimator

        FPHeader mfpInfo; // information about MFP db
        FPHeader wfpInfo; // information about WFP db
        FPHeader bfpInfo; // information about BFP db
        FPHeader proximityInfo; // information about proximity db

        BaseVenueType ffVenue; // FF-venue parameters

        AltitudeFilter *altitude_filter;
        bool     isUseBarometer;
};

#endif
