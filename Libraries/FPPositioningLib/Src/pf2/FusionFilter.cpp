/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           The main class of integration different modules.
* Implements platform independent data processing to obtain navigation
* \ingroup         PF
* \file            FusionFilter.cpp
* \author          M.Frolov D.Churikov
* \date            28.11.2014
*/

#define _CRT_SECURE_NO_WARNINGS
#include <cassert>
#include <string>
#include "FusionFilter.hpp"
#include "ff_config.h"
#include "../wifi/wifi_loc.hpp"
#include "../ble_proximity/ble_proximity_loc.hpp"
#include "CRC.h"
#include "BleHash.h"
#include <iostream>
#include <algorithm>
#include <math.h>       /* acos */
#include <eigen/Core>
#include <eigen/Dense>
#include <new>

static const int64_t time_interval = 500;


FusionFilter::FusionFilter() : i_pf_mfp( *this, prediction_mfp ), i_pf_mix( *this, prediction_mix )
{
    initializeVariables();     // important: initialize variables first because  they are used in object initialization
    initializeInitializers();
	initializeInjectors();
    initializeUpdaters();
    initializeFilterMFP();
    initializeFilterMixed();
    pf_log   = &stub_pf_log;
    wifi_log = &stub_wifi_log;
    ble_log  = &stub_ble_log;
    ble_proximity_log = &stub_ble_proximity_log;
    //waypoints_log = &stub_waypoints_log;

    logs.push_back( std::pair<std::string, std::ostream **>( "pf_log", &pf_log ) );
    logs.push_back( std::pair<std::string, std::ostream **>( "wifi_log", &wifi_log ) );
    logs.push_back( std::pair<std::string, std::ostream **>( "ble_log", &ble_log ) );
    logs.push_back( std::pair<std::string, std::ostream **>( "ble_proximity_log", &ble_proximity_log));
    //logs.push_back( std::pair<std::string, std::ostream **>( "waypoints_log", &waypoints_log ) );

    setAdjustWfpParams( true );
    setAdjustMfpParams( true );

    proximity_meas_filter.clear();
    proximity_meas_filter_for_detection.clear();

    platform_type = pftp_PEDESTRIAN;

    peak_detector_max_delay_ms_for_positioning_in_moving = 2500;
    descending_factor_for_positioning_in_moving = 0.04;
    peak_detector_max_delay_ms_for_positioning_in_stop = 10000;
    descending_factor_for_positioning_in_stop = 0.005;

    peak_detector_max_delay_ms_for_detection_in_moving = 100;
    descending_factor_for_detection_in_moving = 0.04;
    peak_detector_max_delay_ms_for_detection_in_stop = 10000;
    descending_factor_for_detection_in_stop = 0.005;

    ble_proximity_detection_enable = true;
    auto_positioning_poximity_params_update_for_os_type = true;
    auto_detection_poximity_params_update_for_os_type = true;
    setUseBarometer(true);

	wifi_pos = {};
	ble_pos = {};
	proximity_pos = {};
}

void FusionFilter::getLogDescriptors( std::vector<LogDescriptor> *log_descriptors )
{
    if ( log_descriptors != 0 )
    {
        log_descriptors->resize( 0 );

        for ( auto it = logs.cbegin(); it != logs.cend(); ++it )
        {
            log_descriptors->push_back( *it );
        }
    }
}

bool FusionFilter::setLogStream( const unsigned int &idx, std::ostream &os )
{
    bool result = false;

    if ( logs.size() > idx )
    {
        *logs[idx].second = &os;
        result = true;
    }

    return result;
}

FusionFilter::~FusionFilter()
{
    closeLogFiles();

    delete pMFP;
    delete pMixed;

    delete init_wifi;
    delete init_ble;
    delete init_proximity;
    delete init_wifi_mm;
    delete init_ble_mm;
    delete init_proximity_mm;
    delete init_pf;
    delete init_fine;
    delete init_mfp_pf;
    delete init_mfp;
    delete init_rbf;
    delete init_mfp_area;
    delete init_normal_mfp;
    delete init_normal_mix;
	delete init_mixed_area;
	delete init_mixed_area_mm;

    delete init_collaboration;
    delete init_collaboration_mm;
	
	delete injector_ble;
	delete injector_ble_mm;
	delete injector_wifi;
	delete injector_wifi_mm;
	delete injector_proximity;
	delete injector_proximity_mm;
    delete injector_collaboration;
    delete injector_collaboration_mm;
	delete injector_normal_mix;

    delete wifi;
    delete ble;
    delete mfp;
    delete mm;

    delete ble_proximity;
    delete ble_proximity_start;
    delete ble_proximity_detection;

}

void FusionFilter::restart_for_reacquisition()
{
    updatersState = eES_Unknown;
    wfpt_count_down = mgpt_count_down = env_count_down = 0;
    szpf_count_down = 20;

    stepCounter = 0;

    wifi_pos.valid = false;
    last_wifi_timestamp = 0;
    start_count_down = -1;
    wifi_injection_count = 0;

    ble_pos.valid = false;
    //ble_proximity_timestamp = 0;
    ble_proximity_timestamp_start = 0;
    ble_proximity_bias_timestamp = 0;

	proximity_pos.valid = false;

    if (pMixed)
    {
        pMixed->restart();
    }
	    
    prediction = Prediction<PF::Float>();
    sum_N = 0;
    mixed_pos = PF::Particle();
    mixed_pos_std = PF::Particle();
    sigma_mfp = 0;
    sigma_mix = 0;

    *pf_log << "Mixed filter reacquisition restart at " << filter_timestamp << std::endl;
}

void FusionFilter::restart(const std::string &file_folder, const std::string &log_time)
{
    restart(file_folder, log_time, true, true);
}

void FusionFilter::restart( const std::string &file_folder, const std::string &log_time, bool reset_mixed, bool reset_MFP )
{
    closeLogFiles();

    if( log_time.length() != 0 )
    {
        openLogFiles( file_folder, log_time );
    }


    updatersState = eES_Unknown;
    wfpt_count_down = mgpt_count_down = env_count_down = 0;
    szpf_count_down = 20;

    //inTracking = false;
    stepCounter = 0;

    if (init_mfp_pf)
    {
        init_mfp_pf->restart();
    }

    if (reset_mixed)
    {
        wifi_pos.valid = false;
		wifi_pos.timestamp = 0;
        last_wifi_timestamp = 0;
        start_count_down = -1;
        wifi_injection_count = 0;

        ble_pos.valid = false;
		ble_pos.timestamp = 0;
        //ble_proximity_timestamp = 0;
        ble_proximity_timestamp_start = 0;
        ble_proximity_bias_timestamp = 0;
        collaboration_timestamp = 0;

		proximity_pos.valid = false;

        if (pMixed)
        {
            pMixed->restart();
        }
    }

    if (reset_MFP)
    {
        if (pMFP)
        {
            pMFP->restart();
        }              
    }
    
    prediction = Prediction<PF::Float>();
    sum_N = 0;
    mixed_pos = PF::Particle();
    mixed_pos_std = PF::Particle();
    sigma_mfp = 0;
    sigma_mix = 0;

    position_history.clear();

    //wifi_helper.clear();
    //ble_helper.clear();
    //ble_proximity_helper.clear();
    //ble_proximity_helper_start.clear();
    //pos_queue = std::queue<PositionWithUncertainties>();
    //proximity_pos_queue= std::queue<PositionWithUncertainties>();

    *pf_log << "Restart FF at " << filter_timestamp << std::endl;

    filter_timestamp = 0;
}

void FusionFilter::process_increments( const double mis_std, const double d_pos[3], const double pos_cov[3][3], const double quat[4], const double quat_cov[4][4], const bool quat_valid, const bool is_motion, const int64_t timestamp, const bool is_transit)
{
    ///TODO
    if (sum_N == 0)    prediction = Prediction<PF::Float>(); // reset prediction
    sum_N++;

	controlBarometer();

    bool process_pf = integrate( mis_std, d_pos, pos_cov, quat, quat_cov, quat_valid, is_motion, timestamp, is_transit);

    if (process_pf )
    {
        ///main task
        filter_timestamp = timestamp;
        pMixed->setTimeStamp(timestamp);
        pMFP->setTimeStamp(timestamp);

        // log FF staus and parameters
        PrintParams();

        /*
        // stop mag start filter after any filter start
        PF::IFilter* pMagStart = init_mfp_pf->getPF();
        if (pMagStart)
        { 
            bool both_filters_state = pMFP->initialized() || pMixed->initialized();
            pMagStart->setEnabled(!both_filters_state);
        }
        */
        
        init_normal_mfp->clear_status(); // clear status of initializers
        init_normal_mix->clear_status(); // clear status of initializers
		injector_normal_mix->clear_status();

        controlInjection();
        controlMagFilter();
        controlMixedFilter();

        //tpn_3d.set_increment( getPrediction() );
        tpn_3d_start.set_increment( getPrediction() );

        // set Wifi position for update
        int64_t max_wifi_delay = (pMFP->initialized() || pMixed->initialized()) ? 5000 : 20000;
        WiFi_Location wifi_pos1 = estimateWiFiPos(filter_timestamp, max_wifi_delay);
        WiFi_Location  wifi_pos_mix = rejectWiFiSpike( wifi_pos1 );
        wifi_update_mix.wifiPos( wifi_pos_mix );


        // set BLE position for update
        WiFi_Location ble_pos1 = estimateBLEPos( filter_timestamp );
        ble_update_mix.wifiPos( ble_pos1 );
        
        PF::Float sigBLE(0), sigLevelBLE(0);
        ble_update_mix.getSig(sigBLE, sigLevelBLE);
        
        const PF::Float k_min_sig = 10.;
        sigBLE = std::max(ble_pos1.rms_xy, k_min_sig);

        ble_update_mix.setSig(sigBLE, sigLevelBLE);

        // set BLE proximity for update
        std::vector<WiFi_Location> LocationList;
        int64_t ble_proximity_timestamp;
        estimateBLEProximityPos(filter_timestamp, ble_proximity_timestamp, LocationList);
        if ((LocationList.size() > 0) && (ble_proximity_timestamp > 0) && ((filter_timestamp - ble_proximity_timestamp) <= 3000))
        {
            ble_proximity_update_mix.bleProximityPositions(LocationList);
        }
        
        // proximity for injection
        if (pMixed->initialized() && (LocationList.size() > 0) && (ble_proximity_timestamp > 0) && ((filter_timestamp - ble_proximity_timestamp) <= 3000))
        {
            WiFi_Location pos_proxi = LocationList[0];

			for (auto it = LocationList.begin(); it != LocationList.end(); ++it)
			{
				if (it->p == -2)
					pos_proxi = *it;
			}

			// instead of LocationList[0] search whole list for the one with metric p = -2
			// save to filter last prox position using sigma from that position (for injectors keep the code below as is)
			PositionWithUncertainties pos;
            pos.timestamp = ble_proximity_timestamp;
            
            pos.x = pos_proxi.x;
            pos.y = pos_proxi.y;
            pos.cov_xy[0][0] = pos.cov_xy[1][1] = 4.;
            pos.cov_xy[0][1] = pos.cov_xy[1][0] = 0;

            pos.z = pos_proxi.z;
            pos.std_z = 0.1;

            pos.heading = 0;
            pos.std_h = -1;

            injector_normal_mix->setPos(pos); // normal mixed for injection

			PositionWithUncertainties prox_pos_for_uniform_injection = {};
			prox_pos_for_uniform_injection.x = pos.x;
			prox_pos_for_uniform_injection.y = pos.y;
			prox_pos_for_uniform_injection.z = pos.z;
			prox_pos_for_uniform_injection.cov_xy[0][0] = 8.0;
			prox_pos_for_uniform_injection.cov_xy[1][1] = 8.0;
			prox_pos_for_uniform_injection.cov_xy[0][1] = 0.0;
			prox_pos_for_uniform_injection.cov_xy[1][0] = 0.0;
			prox_pos_for_uniform_injection.std_z = 0.1;   // this was missing previously, TODO: check if it affects performance
			prox_pos_for_uniform_injection.std_h = 3.14;
			prox_pos_for_uniform_injection.timestamp = pos.timestamp;
			prox_pos_for_uniform_injection.valid = true;

			injector_proximity->setPos(prox_pos_for_uniform_injection); // TODO: decide if we want to use uniform injection for proximity or not
			injector_proximity_mm->setPos(prox_pos_for_uniform_injection);

			proximity_pos = prox_pos_for_uniform_injection;
			proximity_pos.cov_xy[0][0] = pos_proxi.rms_xy * pos_proxi.rms_xy;
			proximity_pos.cov_xy[1][1] = pos_proxi.rms_xy * pos_proxi.rms_xy;
        }

        // external position / framework update
        external_pos = getExternalPos(filter_timestamp);

        // dyn models increments
        if (predict_lks_mfp != 0)
        {
            predict_lks_mfp->set_increment();
        }
        if (predict_lks_mix != 0)
        {
            predict_lks_mix->set_increment();
        }

        //rbf initializer
        init_rbf->updateTime( filter_timestamp );

        //PF mag start
        init_mfp_pf->process();

        //TODO!!
#if ENABLE_MULTIPOINT_WIFI_INITIALIZER
        std::vector<WiFi_Location> loc_list;
        int64_t solution_timestamp;
        if (getWiFiLocationList(timestamp, solution_timestamp, loc_list, 20000) && (loc_list.size() > 0))
        {
            init_wifi->clearPos();
			injector_wifi->clearPos();
            for (auto location : loc_list)
            {
                PositionWithUncertainties wifi_pos_init = {};
                wifi_pos_init.x = location.x;
                wifi_pos_init.y = location.y;
                wifi_pos_init.z = location.z;
                double pos_cov = 25;
                double d_tau = (timestamp - solution_timestamp)/1000.;
                wifi_pos_init.cov_xy[0][0] = std::max(0., pos_cov + d_tau*d_tau / 4.);
                wifi_pos_init.cov_xy[0][1] = 0;
                wifi_pos_init.cov_xy[1][0] = 0;
                wifi_pos_init.cov_xy[1][1] = std::max(0., pos_cov + d_tau*d_tau / 4.);
                wifi_pos_init.std_h = 2 * M_PI;
                wifi_pos_init.timestamp = solution_timestamp;
                wifi_pos_init.valid = true;
                init_wifi->addPos(wifi_pos_init, location.metric);
				injector_wifi->addPos(wifi_pos_init, location.metric);
            }
#if 0 // debub output of WiFi multiplo solution
            int idx(0);
            *pf_log << solution_timestamp << " init_wifi_set_location:";
            for (auto loc : loc_list)
            {
                *pf_log << " ( " << loc.x << " ," << loc.y << " ," << loc.z << " ," << loc.metric << " )";
            }
            *pf_log << std::endl;;
#endif
        }
#else // regular single point initializer
        PositionWithUncertainties wifi_pos_init = {};
        if (((timestamp - this->wifi_pos.timestamp)) > max_wifi_delay)
        {
            init_wifi->setPos(wifi_pos_init); // reset initializer position
            init_wifi_mm->setPos(wifi_pos_init);
        }
        else if (wifi_pos1.valid)
        {
            double wifi_pos_covariance = 36.;
            if (wifi)
            {
                wifi_pos_covariance = (wifi->GetDbApCount() >= 50) ? 36. : 144.;
            }

            double d_tau = (timestamp - this->wifi_pos.timestamp) / 1000.;

            wifi_pos_init.x = wifi_pos1.x;
            wifi_pos_init.y = wifi_pos1.y;
            wifi_pos_init.z = wifi_pos1.z;
            wifi_pos_init.cov_xy[0][0] = wifi_pos_covariance + d_tau*d_tau / 4.;
            wifi_pos_init.cov_xy[0][1] = 0;
            wifi_pos_init.cov_xy[1][0] = 0;
            wifi_pos_init.cov_xy[1][1] = wifi_pos_covariance + d_tau*d_tau / 4.;
            wifi_pos_init.std_h = 2. * M_PI;
            wifi_pos_init.timestamp = this->wifi_pos.timestamp;
            wifi_pos_init.valid = wifi_pos1.valid;
            init_wifi->setPos(wifi_pos_init);
            init_wifi_mm->setPos(wifi_pos_init);

			wifi_pos_init.cov_xy[0][0] = 25.0; // different cov_xx and cov_yy for injection
			wifi_pos_init.cov_xy[1][1] = 25.0;

			injector_wifi->setPos(wifi_pos_init);
			injector_wifi_mm->setPos(wifi_pos_init);

            if (!pMixed->initialized())
            {
                wifi_injection_count = 1000; // temporary: reset after-start injection for wifi start
            }
            else
            {
                wifi_injection_count++;
            }
            
        }
#endif
        // ble, proximity and framework - add max delay (for using old positions for start)
        if ( ble_pos1.valid )
        {
            PositionWithUncertainties ble_pos_init = {};
            ble_pos_init.x = ble_pos1.x;
            ble_pos_init.y = ble_pos1.y;
            ble_pos_init.z = ble_pos1.z;
            ble_pos_init.cov_xy[0][0] = 36;
            ble_pos_init.cov_xy[0][1] = 0;
            ble_pos_init.cov_xy[1][0] = 0;
            ble_pos_init.cov_xy[1][1] = 36;
            ble_pos_init.std_h = 2. * M_PI;
            ble_pos_init.timestamp = filter_timestamp;
            ble_pos_init.valid = ble_pos1.valid;
            init_ble->setPos(ble_pos_init);
            init_ble_mm->setPos(ble_pos_init);
			
			injector_ble->setPos(ble_pos_init);
			injector_ble_mm->setPos(ble_pos_init);
        }

        // set BLE proximity for start
        WiFi_Location ble_proximity_pos_start = estimateBLEProximityPos_for_start(filter_timestamp);
        if (ble_proximity_pos_start.valid)
        {
            PositionWithUncertainties proximity_pos_init = {};
            proximity_pos_init.x = ble_proximity_pos_start.x;
            proximity_pos_init.y = ble_proximity_pos_start.y;
            proximity_pos_init.z = ble_proximity_pos_start.z;
            proximity_pos_init.cov_xy[0][0] = 36;
            proximity_pos_init.cov_xy[0][1] = 0;
            proximity_pos_init.cov_xy[1][0] = 0;
            proximity_pos_init.cov_xy[1][1] = 36;
            proximity_pos_init.std_h = 2. * M_PI;
            proximity_pos_init.timestamp = filter_timestamp;
            proximity_pos_init.valid = ble_proximity_pos_start.valid;
            init_proximity->setPos(proximity_pos_init);
            init_proximity_mm->setPos(proximity_pos_init);
        }

        // set collaboration for injection and initialization
        std::vector<Fppe::CollaborationData> CollaborationList;
        estimateCollaborationPos(filter_timestamp, collaboration_timestamp, CollaborationList);
        if (CollaborationList.size() > 0)
        {
            Fppe::CollaborationData pos = CollaborationList[0];

            PositionWithUncertainties collaboration_pos = {};
            collaboration_pos.x = pos.lattitude;
            collaboration_pos.y = pos.longitude;
            collaboration_pos.z = pos.floor_number;
            collaboration_pos.cov_xy[0][0] = 36;
            collaboration_pos.cov_xy[0][1] = 0;
            collaboration_pos.cov_xy[1][0] = 0;
            collaboration_pos.cov_xy[1][1] = 36;
            collaboration_pos.std_h = 2. * M_PI;
            collaboration_pos.timestamp = collaboration_timestamp;
            collaboration_pos.valid = true;

            init_collaboration->setPos(collaboration_pos);
            init_collaboration_mm->setPos(collaboration_pos);

            injector_collaboration->setPos(collaboration_pos); // TODO: decide if we want to use uniform injection for proximity or not
            injector_collaboration_mm->setPos(collaboration_pos);
        }

        // add framework position (call setStartPositionWithUncertainties?) getExternalPosition -> setStartPositionWithUncertainties

        //PF MFP
        pMFP->predict();
        pMFP->update();
        //PF Mixed
        pMixed->predict();
        pMixed->update();

        ///-------------------------------
        //log output
        *pf_log << filter_timestamp << " " << *init_mfp_pf;
        *pf_log << filter_timestamp << " " << *pMFP;
        *pf_log << filter_timestamp << " " << *pMixed;

        PF::Particle pos_mfp, pos_mfp_std;
        bool inTrackingMfp = pMFP->estimate( pos_mfp, pos_mfp_std );

        // ************DEBUG
#if ESTIMATE_DEBUG
        std::cout << "pos_mfp1-floor: " << filter_timestamp << "; ";
        std::cout << pos_mfp.pos.level;
        std::cout << std::endl;
#endif
        // ************DEBUG-end

        std::list<std::tuple<PF::Particle, PF::Particle, double>> pos_std_weight_mfp;
        pMFP->estimate(pos_std_weight_mfp);
        pos_mfp = (pos_std_weight_mfp.size() > 0) ? std::get<0>(pos_std_weight_mfp.front()) : pos_mfp = {};
        rbf_bias.setKf( true );

        // ************DEBUG
#if ESTIMATE_DEBUG
        std::cout << "pos_mfp2-floor: " << filter_timestamp << "; ";
        for (auto tuple_item : pos_std_weight_mfp)
        {
            std::cout << std::get<0>(tuple_item).pos.level
                << ",( " << std::get<0>(tuple_item).kl
                << ", " << std::get<2>(tuple_item)
                << ", " << int(std::get<0>(tuple_item).ph)
                << " ); ";
        }
        std::cout << std::endl;
#endif
        // ************DEBUG-end


        if ( pMFP->initialized() )
        {
            auto rm = getPrediction().qi2l.toRotationMatrix();

            rbf_bias.setKf( true );
            //TODO move this to operator<<
            *pf_log << filter_timestamp << " " << "pf_mfp: " << pos_mfp.pos.x << " " << pos_mfp.pos.y << " " << pos_mfp.pos.level << " " << pos_mfp.h << " " << pos_mfp.kl << " " <<
                    pos_mfp_std.pos.x << " " << pos_mfp_std.pos.y << " " << pos_mfp_std.pos.level << " " << pos_mfp_std.h << " "
                    << pos_mfp.rbf.b( 0 ) << " "
                    << pos_mfp.rbf.b( 1 ) << " "
                    << pos_mfp.rbf.b( 2 ) << " "
                    << sqrt( pos_mfp.rbf.P( 0, 0 ) ) << " "
                    << sqrt( pos_mfp.rbf.P( 1, 1 ) ) << " "
                    << sqrt( pos_mfp.rbf.P( 2, 2 ) ) << " "
                    << mag_avg.m( 0 ) - pos_mfp.rbf.z( 0 ) << " "
                    << mag_avg.m( 1 ) - pos_mfp.rbf.z( 1 ) << " "
                    << mag_avg.m( 2 ) - pos_mfp.rbf.z( 2 ) << " "
                    << mag_avg.m( 0 ) - pos_mfp.rbf.b( 0 ) << " "
                    << mag_avg.m( 1 ) - pos_mfp.rbf.b( 1 ) << " "
                    << mag_avg.m( 2 ) - pos_mfp.rbf.b( 2 ) << " "
                    << rm( 0, 0 ) << " "
                    << rm( 0, 1 ) << " "
                    << rm( 0, 2 ) << " "
                    << rm( 1, 0 ) << " "
                    << rm( 1, 1 ) << " "
                    << rm( 1, 2 ) << " "
                    << rm( 2, 0 ) << " "
                    << rm( 2, 1 ) << " "
                    << rm( 2, 2 ) << " "
                    << std::endl;
            *pf_log << pos_mfp.rbf.C << std::endl;
            *pf_log << pos_mfp.rbf.S << std::endl;

            sigma_mfp = ( pos_mfp_std.pos.x + pos_mfp_std.pos.y ) / 2;

            if ( pMfpUpd != NULL )
            {
                PF::Particle pos_mfp_corrected = pos_mfp;
                bool result = false;
                if (position_mm_correction_enable)
                {
                    result = correctPositionWithMapMatching(pos_mfp_corrected, pos_mfp_corrected_prev);
                    pos_mfp_corrected_prev = pos_mfp; //pos_mfp_corrected;
                }
                if (result)
                {
                    *pf_log << filter_timestamp << " " << "pf_mfp_corrected: " << pos_mfp_corrected.pos.x << " " << pos_mfp_corrected.pos.y << " "
                        << pos_mfp_corrected.pos.level << std::endl;
                }
                pMfpUpd->update(pos_mfp_corrected.pos.x, pos_mfp_corrected.pos.y, pos_mfp_corrected.pos.level, 
                    pos_mfp_corrected.h, sigma_mfp, ( double )filter_timestamp, pMFP->getState(),
                    pMFP->getParcticlesCount() );
            }
        }
        
        // new estimate technique realisation

        PF::Particle pos_mix, pos_mix_std;
        inTrackingMixed = pMixed->estimate( pos_mix, pos_mix_std );

        // save position in FF

        mixed_pos = pos_mix;
        mixed_pos_std = pos_mix_std;
                
        double internal_sigma_mix = std::max(pos_mix_std.pos.x, pos_mix_std.pos.y);
        
        // Estimate floor
        int floor = pMixed->estimateFloor(); // the function estimates floor with hysteresis filter
        // calculate particle weight for floor
        double floor_weight = pMixed->weightsSum(pMixed->getState(), floor); // the function calculates total particle weight on specified floor
        if (floor_weight > 0.30)
        {
            pMixed->estimatePositionOnFloor(floor, pos_mix, pos_mix_std); // the function estimates position for specified floor
        }
        else
        {
            // do nothig 
            // pos_mix and pos_mix_std  from regulatr estimate
        }
		pos_mix.pos.level = (double)floor;
        
        // ************DEBUG
#if ESTIMATE_DEBUG
        
        std::cout << "pos_mix2-floor: " << filter_timestamp << "; ";
        for (auto tuple_item : pos_std_weight)
        {
            std::cout << std::get<0>(tuple_item).pos.level
                << ",( " << std::get<0>(tuple_item).kl
                << ", " << std::get<2>(tuple_item)
                << ", " << int(std::get<0>(tuple_item).ph)
                << " ); ";
        }
        std::cout << int(getPrediction().is_motion) << ", " << int(getPrediction().is_transit);
        std::cout << std::endl;
#endif
        // ************DEBUG-end

        double output_sigma_mix = std::max(pos_mix_std.pos.x, pos_mix_std.pos.y);

        if ( pMixed->initialized() )
        {
            //updateReacquisitionState(filter_timestamp, std::get<1>(pos_std_weight.front()).h, output_sigma_mix, pos_mix.pos.level);
            updateReacquisitionState(filter_timestamp, pos_mix_std.h, internal_sigma_mix, pos_mix.pos.level);

            if (hard_reset_enabled && hard_reset_required)
            {
                output_sigma_mix += 20; // increasing output sigma as a signal that position is considered to be incorrect and restart is planned
            }

            //TODO move this to operator<<
            *pf_log << filter_timestamp << " " << "pf_mix: " << pos_mix.pos.x << " " << pos_mix.pos.y << " " << pos_mix.pos.level << " " << pos_mix.h << " " << pos_mix.kl << " " <<
                    pos_mix_std.pos.x << " " << pos_mix_std.pos.y << " " << pos_mix_std.pos.level << " " << pos_mix_std.h << " "
                    << pos_mix.rbf.b( 0 ) << " "
                    << pos_mix.rbf.b( 1 ) << " "
                    << pos_mix.rbf.b( 2 ) << " "
                    << sqrt( pos_mix.rbf.P( 0, 0 ) ) << " "
                    << sqrt( pos_mix.rbf.P( 1, 1 ) ) << " "
                    << sqrt( pos_mix.rbf.P( 2, 2 ) ) << " "
                    << mag_avg.m( 0 ) - pos_mix.rbf.z( 0 ) << " "
                    << mag_avg.m( 1 ) - pos_mix.rbf.z( 1 ) << " "
                    << mag_avg.m( 2 ) - pos_mix.rbf.z( 2 ) << " "
                    << std::endl;

            if ( inTrackingMixed )
            {
                PF::Float s = sqrt( pos_mix.rbf.P.maxCoeff() ) + pos_mix.rbf.b.maxCoeff();
                PF::Float sss = pos_mix.rbf.b( 2 );
                init_rbf->updateBias( pos_mix.rbf.b );
                init_rbf->updateSig( s );

                // ble proximity bias estimation
                if ((ble_proximity != 0) && biasBLEProxEstEnabled)
                {
                    if ((true == getPrediction().is_motion) && ( true != pMixed->inPulling))
                    {
                        WiFi_Location location;
                        location.x = pos_mix.pos.x;
                        location.y = pos_mix.pos.y;
                        location.rms_xy = output_sigma_mix;
                        Location_and_time  pos;
                        pos.location = location;
                        pos.t = filter_timestamp;
                        position_history.push_back(pos);
                        const int max_pos_vector_size = 20;
                        auto sz = position_history.size();
                        if (sz > max_pos_vector_size) // clear position vector
                        {
                            position_history.erase(position_history.begin(), position_history.begin() + sz- max_pos_vector_size);
                        }
                    }

                    Fppe::BleScanResult ble_meas;
                    while  (ble_proximity_bias_helper.readMeas(filter_timestamp ,ble_proximity_bias_timestamp, ble_meas))
                    {
                        if (ble_proximity->estimateBias(ble_meas, position_history, ble_proximity_log))
                        {
                            double biasRSSI, biasRssiSigma;
                            ble_proximity->getBias(biasRSSI, biasRssiSigma);
                            *pf_log << timestamp << " ble_proximity_bias_estimate: " << biasRSSI << ", " << biasRssiSigma << std::endl;
                            if (ble_proximity_detection != 0)
                            {
                                ble_proximity_detection->setBias(biasRSSI);
                                ble_proximity_detection->setBiasSigma(biasRssiSigma);
                            }
                            if (ble_proximity_start != 0)
                            {
                                ble_proximity_start->setBias(biasRSSI);
                                ble_proximity_start->setBiasSigma(biasRssiSigma);
                            }
                        }
                    }
                }
            }

            sigma_mix = ( pos_mix_std.pos.x + pos_mix_std.pos.y ) / 2; // internal sigma mix (affets filter resizing)

            if ( pMixedUpd != NULL )
            {
                PF::Particle pos_mix_corrected = pos_mix;
                bool result = false;
                if (position_mm_correction_enable)
                {
                    result = correctPositionWithMapMatching(pos_mix_corrected, pos_mix_corrected_prev);
                    pos_mix_corrected_prev = pos_mix; //pos_mix_corrected;
                }
                if (result)
                {
                    *pf_log << filter_timestamp << " " << "pf_mix_corrected: " << pos_mix_corrected.pos.x << " " << pos_mix_corrected.pos.y << " "
                        << pos_mix_corrected.pos.level << std::endl;
                }

				if (!inTrackingMixed)
					output_sigma_mix *= -1.0;

                pMixedUpd->update(pos_mix_corrected.pos.x, pos_mix_corrected.pos.y, 
                    pos_mix_corrected.pos.level, pos_mix_corrected.h, 
                    output_sigma_mix, ( double )filter_timestamp, pMixed->getState(),
                    pMixed->getParcticlesCount() );
            }
        }

        ///-------------------------------
        //prediction = Prediction<PF::Float>(); // !!! This code has been transfered in the beggining of this function to make prediction structure avaliable for using until next tpn package arrived
        sum_N = 0; // reset integration
    }
    else if ( ( timestamp - filter_timestamp ) < 0 )
    {
        ///unexpected conditions
    }
}

void FusionFilter::process_wifi( const WiFi_Measurement  &measurement, int64_t timestamp , int n )
{
    if (!hard_reset_enabled)
    {
        if (measurement.size() > 0)
        {
            hard_reset_enabled = true;
        }
    }

    for ( unsigned int i = 0; i < measurement.size(); i++ )
    {
        //        fprintf( wifi_file, "number = %d timestamp = %lld bssid = %llu  rssi = %d\n", n, timestamp, measurement[i].bssid, measurement[i].rssi );
        //*wifi_log << "number = " << n << " timestamp = " << timestamp << " bssid = " << measurement[i].bssid << " rssi = " << measurement[i].rssi << std::endl;
        *wifi_log << n; // number
        *wifi_log << ", " << timestamp; // timestamp of scan
        *wifi_log << ", " << measurement[i].timestamp; // timestamp of measurement
        *wifi_log << ", " << measurement[i].bssid;   // bssid
        *wifi_log << ", " << int(measurement[i].rssi); //rssi
        *wifi_log << ", " << measurement[i].frequency; // frequency
        *wifi_log << std::endl;

        wifi_log->flush();
    }

    //wifi_helper.pushMeas( timestamp, n, measurement ); // we don't save measurements anymore, solution is saved instead

    if (wifi)
    {
        // calculate wifi_pos with GetLocation and push wifi_pos to queue
        std::vector<WiFi_Location > LocationList;
        int knn_level = 6;
        WiFi_Location wifi_pos = wifi->GetLocation(measurement, knn_level, LocationList);
        if (wifi_pos.valid)
        {
            wifi_helper.pushWiFiSolution(timestamp, wifi_pos);
            *pf_log << timestamp << " WiFi_location: " << wifi_pos.x << ", " << wifi_pos.y << ", " << wifi_pos.z << ", " << wifi_pos.metric << ", " << wifi_pos.metric << std::endl;
        }
        wifi_helper.pushWiFiSolution(timestamp, LocationList);
        

#if 0 // debug output of WiFi multiple solution
        *pf_log << timestamp << " WiFi_multiple_location:";
        for (auto loc : LocationList)
        {
            *pf_log << " ( " << loc.x << " ," << loc.y << " ," << loc.z << " ," << loc.metric << " )";
        }
        *pf_log << std::endl;;
#endif

#if 0 // debug output of WiFi multiple solution for matlab visualization
        for (auto loc : LocationList)
        {
            *pf_log << "wifi_fp_metrics: " << timestamp;
            *pf_log << ", " << loc.x << " ," << loc.y << " ," << loc.z << " ," << loc.metric;
            *pf_log << std::endl;;
        }
        
#endif

        if (wifi_pos.valid)
        {
            if (biasWiFiEstEnabled && this->getPrediction().is_motion)
            {
                int cnt = 0;

#if ENABLE_MULTIPOINT_WIFI_BIAS_ESTIMATION
                //multipoint bias estimation with wifi location list
                double metric_norm(0);
                for (auto ll_item = LocationList.begin(); ll_item != LocationList.end(); ll_item++)
                {
                    metric_norm += ll_item->metric;
                }
                for (auto ll_item = LocationList.begin(); ll_item != LocationList.end(); ll_item++)
                {
                    WiFi_Location loc = *ll_item;
                    loc.metric /= metric_norm;
                    wifi->estimateBias(measurement, loc, cnt, wifi_log);
                }
#else 
                WiFi_Location loc = wifi_pos;
                loc.metric = 1;
                wifi->estimateBias(measurement, loc, cnt, wifi_log);
#endif
            }

            if (pWiFiUpd)
            {
                double biasRSSI;
                pWiFiUpd->update(wifi_pos.x, wifi_pos.y, wifi_pos.z, 0, wifi_pos.p, (double)timestamp, NULL, 0); // at least one of last two parameters must be '0' for wifi update
                //if (wifi->getBias(biasRSSI))  *pf_log << timestamp << " WiFi_RSSI_bias: " << biasRSSI << std::endl;;  // moved to FF standard output
            }
        }
        // venue detection
        // add to queue: wifi_pos.metric
        // check last N measurements of queue, then check result and call venueWiFiUpd with result value
        bool estimation_successful (false), pos_in_mapped_area (false);
        wifi->EstimatePresenceInMappedArea(estimation_successful, pos_in_mapped_area);
        *pf_log << "venue_detection: " << (void*)venueWiFiUpd << " " << measurement[measurement.size() - 1].timestamp << " " << pos_in_mapped_area << " " << estimation_successful << std::endl;

        if ( (venueWiFiUpd != NULL) && (true == estimation_successful) )
        {
            venueWiFiUpd->update(pos_in_mapped_area, measurement[measurement.size() - 1].timestamp);
        }
    }
}

void FusionFilter::process_beacons( const WiFi_Measurement  &measurement, int64_t scan_timestamp, int n )
{
    for ( unsigned int i = 0; i < measurement.size(); i++ )
    {
        //        fprintf( ble_file, "number = %d timestamp = %lld bssid = %llu  rssi = %d\n", n, timestamp, measurement[i].bssid, measurement[i].rssi );
        //*ble_log << "number = " << n << " timestamp = " << timestamp << " bssid = " << measurement[i].bssid << " rssi = " << measurement[i].rssi << std::endl;
        *ble_log << n; // number
        *ble_log << ", " << scan_timestamp; // timestamp of scan
        *ble_log << ", " << measurement[i].timestamp; // timestamp of measurement
        *ble_log << ", " << measurement[i].bssid;   // bssid
        *ble_log << ", " << int(measurement[i].rssi); //rssi
        *ble_log << ", " << int(measurement[i].tx); // tx power
        *ble_log << ", " << measurement[i].frequency; // frequency
        *ble_log << std::endl;
        ble_log->flush();
    }
    
    //ble_helper.pushMeas(scan_timestamp, n, measurement);

    if (ble && pBleUpd)
    {
        WiFi_Location ble_pos = ble->GetLocation(measurement, 5);
        ble_helper.pushWiFiSolution(scan_timestamp, ble_pos);

        if ( ble_pos.valid )
        {
            if (biasBLEEstEnabled && this->getPrediction().is_motion)
            {
                int cnt = 0;
                double biasRSSI;
                ble->estimateBias( measurement, ble_pos, cnt, ble_log);
                //if(ble->getBias(biasRSSI)) *pf_log << scan_timestamp << " BLE_RSSI_bias: " << biasRSSI << std::endl;; // moved to FF standard output
            }
            int validity_flag = static_cast <int> (ble_pos.valid);
            pBleUpd->update(ble_pos.x, ble_pos.y, ble_pos.z, 0, ble_pos.p, (double)scan_timestamp, NULL, validity_flag);
        }

    }
}

void FusionFilter::process_beacons_proximity( const Fppe::BleScanResult &input_scan_ble, int64_t scan_timestamp, int n )
{
    if ((ble_proximity != 0) && ble_proximity->size() > 0)
    {
        // sort masurement by RSSI - it is requred to select max RSSI if we have more then one meas from one BLE in the scan
        Fppe::BleScanResult scan_ble = input_scan_ble;
        std::sort(scan_ble.scanBle.begin(), scan_ble.scanBle.end(),
            [](const Fppe::BleMeasurement &mea1, const Fppe::BleMeasurement &mea2)
        {
            return (mea1.rssi > mea2.rssi);
        });

        Fppe::BleScanResult proximity_scan;
        proximity_scan.timestamp = scan_timestamp;
        proximity_scan.scanBle.clear();

        Fppe::BleScanResult proximity_scan_d;
        proximity_scan_d.timestamp = scan_timestamp;
        proximity_scan_d.scanBle.clear();

        // select proximity beacons measurement
        std::map < BSSID, BLE_position>  blp_map = ble_proximity->GetBLEProximityMap();
        for (auto ble_meas : scan_ble.scanBle)
        {
            BLE_position location;
            BSSID  ble_hash = getBleHash(ble_meas.major, ble_meas.minor, ble_meas.uuid);
            try
            {
                location = blp_map.at(ble_hash);
            }
            catch (std::out_of_range)
            {
                continue;
            }

            // reject repeated measurements
            auto meas = std::find_if(proximity_scan.scanBle.begin(), proximity_scan.scanBle.end(),
                [&ble_hash](const Fppe::BleMeasurement& x)
            { return getBleHash(x.major, x.minor, x.uuid) == ble_hash; });
            if (meas != proximity_scan.scanBle.end())
            {
                continue;
            }
            //ble_meas.rssi += 10;

            // check that transmitted TxPower for Android matches TxPower from DB
            bool rejected_by_tx_mismatch = (this->os_type == OperationSystemType::OS_ANDROID) && (location.txPower != ble_meas.txPower);

#if 1 // enable regular output
            // proximity measurement logging
            * ble_proximity_log << n; // number
            *ble_proximity_log << ", " << scan_timestamp; // timestamp of scan
            *ble_proximity_log << ", " << ble_meas.timestamp; // timestamp of measurement
            *ble_proximity_log << ", " << ble_meas.major;   // major
            *ble_proximity_log << ", " << ble_meas.minor;   // major
            *ble_proximity_log << ", " << int(ble_meas.rssi); //rssi
            *ble_proximity_log << ", " << ble_meas.frequency; // frequency
            *ble_proximity_log << ", " << location.loc.x; // beacon location
            *ble_proximity_log << ", " << location.loc.y; // beacon location
            *ble_proximity_log << ", " << location.blp_height; // beacon height
            *ble_proximity_log << ", " << int(location.txPower); // Tx power from BLP
            *ble_proximity_log << ", " << int(ble_meas.txPower); // Tx power from beacon
            *ble_proximity_log << ", " << int(location.txPower_correction); // Tx power correction
            *ble_proximity_log << ", " << (rejected_by_tx_mismatch) ? 1: 0; // rejection flag
            
            *ble_proximity_log << std::endl;
            //ble_proximity_log->flush();
#endif

            // adding correction to txPower 
            ble_meas.txPower = location.txPower + location.txPower_correction;

            if (false == rejected_by_tx_mismatch)
            {
                proximity_scan.scanBle.push_back(ble_meas);
                proximity_scan_d.scanBle.push_back(ble_meas);
            }
        }

        *pf_log << "process_beacons_proximity: " << n << ", " << scan_timestamp << ", ";
        *pf_log << scan_ble.scanBle.size() << ", " << proximity_scan.scanBle.size();
        *pf_log << std::endl;

        /* proximity for positioning */
        if (proximity_scan.scanBle.size() > 0)
        {
            process_beacons_proximity_for_positioning(proximity_scan, scan_timestamp, n);
        }
        
        /* proximity for detection */
        if ( (ble_proximity_detection_enable) && (proximity_scan_d.scanBle.size() > 0) )
        {
            process_beacons_proximity_for_detection(proximity_scan_d, scan_timestamp, n);
        }
    }
}

int FusionFilter::get_proximity_beacons_number(const int floor, const BleBeaconTypeTag beacon_type) const
{
    int result(-1);
    if ((ble_proximity != 0) && ble_proximity->size() > 0)
    {
        result = 0;
        std::map < BSSID, BLE_position>  blp_map = ble_proximity->GetBLEProximityMap();
        for (auto ble_beacon : blp_map)
        {
            if ((static_cast<int>(ble_beacon.second.loc.z) == floor) &&
                (ble_beacon.second.beaconType == beacon_type))
            {
                result++;
            }
        }
    }
    return result;
}

void FusionFilter::process_beacons_proximity_for_positioning(const Fppe::BleScanResult &scan_ble, int64_t scan_timestamp, int n)
{
    if ((ble_proximity != 0) && ble_proximity->size() > 0)
    {
        // set specific BLP parameters depends on OS type
        if (auto_positioning_poximity_params_update_for_os_type && 
            this->os_type == OperationSystemType::OS_ANDROID)
        {
            ble_proximity->setBLEProximityLogic(1, 1);
            ble_proximity->setSingleCutoffThreshold(-10);
        }
        std::map < BSSID, BLE_position>  blp_map = ble_proximity->GetBLEProximityMap();
        
        /* proximity for positioning */
        bool enable_pd_filter = true;
        Fppe::BleScanResult proximity_scan_for_positioning;
		if (enable_pd_filter /*&& (this->os_type == OperationSystemType::OS_ANDROID)*/)
        {
            proximity_scan_for_positioning.timestamp = scan_timestamp;
            proximity_scan_for_positioning.scanBle.clear();

            // initialize proximity meas filter
            if (proximity_meas_filter.size() <= 0)
            {
                for (auto ble_beacon : blp_map)
                {
                    const int max_delay_ms = 2500;
                    const double descending_factor = 0.02;
                    BSSID bssid = ble_beacon.first;
                    PeakDetector pd(bssid, descending_factor, max_delay_ms);
                    proximity_meas_filter.insert(std::make_pair(bssid, pd));
                }
            }

            // perform ble meas filtration
            for (auto filter_item = proximity_meas_filter.begin(); filter_item != proximity_meas_filter.end(); filter_item++)
            {
                // adjust filter parameters
                (this->getPrediction().is_motion) ? filter_item->second.set_max_data_gap(peak_detector_max_delay_ms_for_positioning_in_moving)
                    : filter_item->second.set_max_data_gap(peak_detector_max_delay_ms_for_positioning_in_stop);
                (this->getPrediction().is_motion) ? filter_item->second.set_decline_coef(descending_factor_for_positioning_in_moving)
                    : filter_item->second.set_decline_coef(descending_factor_for_positioning_in_stop);

                bool is_tracked = filter_item->second.predict(scan_timestamp);

                // find this beacon in scan
                BSSID bssid = filter_item->first;
                auto meas = std::find_if(scan_ble.scanBle.begin(), scan_ble.scanBle.end(),
                    [&bssid](const Fppe::BleMeasurement& x)
                { return getBleHash(x.major, x.minor, x.uuid) == bssid; });

                // find beacon in scan
                Fppe::BleMeasurement meas_to_push = {};
                if (meas != scan_ble.scanBle.end())
                {
                    is_tracked = filter_item->second.update(scan_timestamp, meas->rssi); // update filter
                    meas_to_push = *meas;
                }
                else if (is_tracked) // create virtual meas
                {
                    BLE_position location;
                    BSSID  ble_hash = filter_item->first;
                    try
                    {
                        location = blp_map.at(ble_hash);
                    }
                    catch (std::out_of_range)
                    {
                        continue;
                    }

                    meas_to_push.major = getBleMajor(filter_item->first); //! WARNING
                    meas_to_push.minor = getBleMinor(filter_item->first); //! WARNING 
                    meas_to_push.mac = filter_item->first;
                    meas_to_push.txPower = location.txPower + location.txPower_correction;
                    meas_to_push.hasMAC = (this->os_type == OperationSystemType::OS_ANDROID);
                    meas_to_push.timestamp = scan_timestamp;
                }

                if (is_tracked)
                {
                    meas_to_push.rssi = round(filter_item->second.get_data());
                    proximity_scan_for_positioning.scanBle.push_back(meas_to_push);
                }
            }
        }
        else
        {
            proximity_scan_for_positioning = scan_ble;
        }

#if 0 // enable pd-filtered measurement output
        for (auto ble_meas : proximity_scan_for_processing.scanBle)
        {
            BLE_position location;
            BSSID  ble_hash = getBleHash(ble_meas.major, ble_meas.minor, ble_meas.uuid);
            try
            {
                location = blp_map.at(ble_hash);
            }
            catch (std::out_of_range)
            {
                continue;
            }

            // proximity measurement logging
            *ble_proximity_log << n; // number
            *ble_proximity_log << ", " << scan_timestamp; // timestamp of scan
            *ble_proximity_log << ", " << ble_meas.timestamp; // timestamp of measurement
            *ble_proximity_log << ", " << ble_meas.major;   // major
            *ble_proximity_log << ", " << ble_meas.minor;   // major
            *ble_proximity_log << ", " << int(ble_meas.rssi); //rssi
            *ble_proximity_log << ", " << ble_meas.frequency; // frequency
            *ble_proximity_log << ", " << location.loc.x; // beacon location
            *ble_proximity_log << ", " << location.loc.y; // beacon location
            *ble_proximity_log << ", " << location.blp_height; // beacon location
            *ble_proximity_log << ", " << location.txPower; // Tx power in DB
            *ble_proximity_log << ", " << ble_meas.txPower; // Tx power transmitted
            *ble_proximity_log << ", " << int(location.txPower_correction); // Tx power correction
            *ble_proximity_log << std::endl;
            //ble_proximity_log->flush();
        }
#endif

        double bias, bias_sigma;
        ble_proximity->getBias(bias, bias_sigma);

        // push data for processing
        if (proximity_scan_for_positioning.scanBle.size() > 0) // 
        {
            ble_proximity_start_helper.pushMeas(scan_timestamp, n, proximity_scan_for_positioning);
            ble_proximity_bias_helper.pushMeas(scan_timestamp, n, proximity_scan_for_positioning);
        }

        if (proximity_scan_for_positioning.scanBle.size() > 0) // 
        {
            std::vector<WiFi_Location > LocationList;

            BLE_position ble_proximity_pos = ble_proximity->GetLocation(proximity_scan_for_positioning, scan_timestamp);
            WiFi_Location &ble_proximity_pos_upd = ble_proximity_pos.loc;

            if (ble_proximity_pos_upd.valid) // proximity callback and pf_log output
            {
                *pf_log << "blp position: " << scan_timestamp 
                    << ", "<< ble_proximity_pos_upd.x << ", " << ble_proximity_pos_upd.y << ", "<< ble_proximity_pos_upd.z
                    << ", " << ble_proximity_pos_upd.rms_xy << ", " << ble_proximity_pos_upd.metric
                    << ", " << ble_proximity_pos_upd.p << ", " << ble_proximity_pos_upd.valid << std::endl;
                
                pBleProximityUpd->update(ble_proximity_pos_upd.x, ble_proximity_pos_upd.y,
                    ble_proximity_pos_upd.z, ble_proximity_pos_upd.h, ble_proximity_pos_upd.rms_xy,
                    (double)scan_timestamp, NULL, static_cast <int> (ble_proximity_pos_upd.valid));
            }
            
			// use multiple update depends on OS type
			if ((true) || (this->os_type == OperationSystemType::OS_IOS)) // 'true' means using multiple update for both Android and iOS
			{
				ble_proximity->GetLocation(proximity_scan_for_positioning, scan_timestamp, LocationList);
			}
			else // use single position from majoritarian logic
			{
				LocationList.push_back(ble_proximity_pos_upd);
			}

            // add logic position into location list to use this one for pulling
            if (ble_proximity_pos_upd.valid)
            {
                ble_proximity_pos_upd.p = -2; // this is a marker of majoritar logic position
                LocationList.push_back(ble_proximity_pos_upd);
            }

            if (LocationList.size() > 0) // push location list
                ble_proximity_helper.pushBLEProxSolution(scan_timestamp, LocationList);
            for (auto it : LocationList) // output loc list in pf_log
                *pf_log << "blp location list: " << it.x << ", " << it.y << ", " << it.z
                    << ", " << it.rms_xy << ", " << it.metric << ", " << it.p << ", " << it.valid << std::endl;
        }
    }
}

void FusionFilter::process_beacons_proximity_for_detection(const Fppe::BleScanResult &scan_ble, int64_t scan_timestamp, int n)
{
    if ((ble_proximity_detection != 0) && ble_proximity_detection->size() > 0)
    {
        if (auto_detection_poximity_params_update_for_os_type && 
            this->os_type == OperationSystemType::OS_ANDROID )
        {
            ble_proximity_detection->setBLEProximityLogic(1, 1);
            ble_proximity_detection->setSingleCutoffThreshold(-10);
        }
        else if (auto_detection_poximity_params_update_for_os_type &&
            this->os_type == OperationSystemType::OS_IOS)
        {
            ble_proximity_detection->setBLEProximityLogic(1, 1);
            ble_proximity_detection->setSingleCutoffThreshold(-10);
        }
        std::map < BSSID, BLE_position>  blp_map = ble_proximity->GetBLEProximityMap();

        /* proximity for detection */
        bool enable_pd_filter = true;
        Fppe::BleScanResult proximity_scan_for_detection;
        if (enable_pd_filter && (this->os_type == OperationSystemType::OS_ANDROID))
        {
            proximity_scan_for_detection.timestamp = scan_timestamp;
            proximity_scan_for_detection.scanBle.clear();

            // initialize proximity meas filter
            if (proximity_meas_filter_for_detection.size() <= 0)
            {
                for (auto ble_beacon : blp_map)
                {
                    const int max_delay_ms = 2500;
                    const double descending_factor = 0.02;
                    BSSID bssid = ble_beacon.first;
                    PeakDetector pd_d(bssid, descending_factor, max_delay_ms);
                    proximity_meas_filter_for_detection.insert(std::make_pair(bssid, pd_d));
                }
            }
            // perform ble meas filtration
            for (auto filter_item = proximity_meas_filter_for_detection.begin(); filter_item != proximity_meas_filter_for_detection.end(); filter_item++)
            {
                // adjust filter parameters
                (this->getPrediction().is_motion) ? filter_item->second.set_max_data_gap(peak_detector_max_delay_ms_for_detection_in_moving)
                    : filter_item->second.set_max_data_gap(peak_detector_max_delay_ms_for_detection_in_stop);
                (this->getPrediction().is_motion) ? filter_item->second.set_decline_coef(descending_factor_for_detection_in_moving)
                    : filter_item->second.set_decline_coef(descending_factor_for_detection_in_stop);

                bool is_tracked = filter_item->second.predict(scan_timestamp);

                // find this beacon in scan
                BSSID bssid = filter_item->first;
                auto meas = std::find_if(scan_ble.scanBle.begin(), scan_ble.scanBle.end(),
                    [&bssid](const Fppe::BleMeasurement& x)
                { return getBleHash(x.major, x.minor, x.uuid) == bssid; });

                // find beacon in scan
                Fppe::BleMeasurement meas_to_push = {};
                if (meas != scan_ble.scanBle.end())
                {
                    is_tracked = filter_item->second.update(scan_timestamp, meas->rssi); // update filter
                    meas_to_push = *meas;
                }
                else if (is_tracked) // create virtual meas
                {
                    BLE_position location;
                    BSSID  ble_hash = filter_item->first;
                    try
                    {
                        location = blp_map.at(ble_hash);
                    }
                    catch (std::out_of_range)
                    {
                        continue;
                    }

                    meas_to_push.major = getBleMajor(filter_item->first); //! WARNING
                    meas_to_push.minor = getBleMinor(filter_item->first); //! WARNING 
                    meas_to_push.mac = filter_item->first;
                    meas_to_push.txPower = location.txPower + location.txPower_correction;
                    meas_to_push.hasMAC = (this->os_type == OperationSystemType::OS_ANDROID);
                    meas_to_push.timestamp = scan_timestamp;
                }

                if (is_tracked)
                {
                    meas_to_push.rssi = round(filter_item->second.get_data());
                    proximity_scan_for_detection.scanBle.push_back(meas_to_push);
                }
            }
        }
        else
        {
            proximity_scan_for_detection = scan_ble;
        }

#if 0 // enable pd-filtered measurement output
        for (auto ble_meas : proximity_scan_for_detection.scanBle)
        {
            BLE_position location;
            BSSID  ble_hash = getBleHash(ble_meas.major, ble_meas.minor, ble_meas.uuid);
            try
            {
                location = blp_map.at(ble_hash);
            }
            catch (std::out_of_range)
            {
                continue;
            }

            // proximity measurement logging
            *ble_proximity_log << n; // number
            *ble_proximity_log << ", " << scan_timestamp; // timestamp of scan
            *ble_proximity_log << ", " << ble_meas.timestamp; // timestamp of measurement
            *ble_proximity_log << ", " << ble_meas.major;   // major
            *ble_proximity_log << ", " << ble_meas.minor;   // major
            *ble_proximity_log << ", " << int(ble_meas.rssi); //rssi
            *ble_proximity_log << ", " << ble_meas.frequency; // frequency
            *ble_proximity_log << ", " << location.loc.x; // beacon location
            *ble_proximity_log << ", " << location.loc.y; // beacon location
            *ble_proximity_log << ", " << location.blp_height; // beacon location
            *ble_proximity_log << ", " << location.txPower; // Tx power in DB
            *ble_proximity_log << ", " << ble_meas.txPower; // Tx power transmitted
            *ble_proximity_log << ", " << int(location.txPower_correction); // Tx power correction
            *ble_proximity_log << std::endl;
            //ble_proximity_log->flush();
        }
#endif
        double bias, bias_sigma;
        ble_proximity->getBias(bias, bias_sigma);

        if (proximity_scan_for_detection.scanBle.size() > 0) //
        {
            std::vector<WiFi_Location > LocationList;
            BLE_position ble_proximity_pos = ble_proximity_detection->GetLocation(proximity_scan_for_detection, scan_timestamp);
            WiFi_Location &ble_proximity_pos_upd = ble_proximity_pos.loc;

            if (ble_proximity_pos_upd.valid) // proximity callback and pf_log output
            {
                *pf_log << "blp detection location (major logic): " << scan_timestamp << ", " << ble_proximity_pos_upd.x << ", " << ble_proximity_pos_upd.y << ", " << ble_proximity_pos_upd.z << ", " << ble_proximity_pos_upd.h << ", " << ble_proximity_pos_upd.valid << ", " << ble_proximity_pos_upd.p << ", " << std::endl;
                //pBleProximityUpd->update(ble_proximity_pos_upd.x, ble_proximity_pos_upd.y, ble_proximity_pos_upd.z, ble_proximity_pos_upd.h, ble_proximity_pos_upd.p, (double)scan_timestamp, NULL, static_cast <int> (ble_proximity_pos_upd.valid));
                if (pExtendedProximityUpd != nullptr)
                    pExtendedProximityUpd->update(double(scan_timestamp), ble_proximity_pos);
            }
        }
    }
}

/**
* pushes Mag measurement into the processing pipeline
* \param[in] measurement Mag 3-components vector
* \param[in] timestamp [ms]
*/
void FusionFilter::process_mag( const std::vector<double>  &measurement, int64_t timestamp )
{
    MagMeas m = MagMeas( timestamp, measurement );
    magFifo.push( m );

    //limit fifo size to prevent memory leakage
    if ( magFifo.size() > 1000 ) magFifo.front();
}

void FusionFilter::process_external_pos( const PositionWithUncertainties &pos )
{
    const std::size_t k_max_framework_queue_size = 20;
    
    pos_queue.push( pos );

    if (pos_queue.size() > k_max_framework_queue_size)
    {
        pos_queue.pop();
    }
}

void FusionFilter::process_collaboration(std::vector <Fppe::CollaborationData> &collaboration_position)
{
    const std::size_t k_max_collaboration_queue_size = 20;

    collaboration_queue.push(collaboration_position);

    if (collaboration_queue.size() > k_max_collaboration_queue_size)
    {
        collaboration_queue.pop();
    }
}

bool FusionFilter::initializeWiFi( const char* const dDb, const size_t szDB, const double minProb )
{
    std::string function_name("initializeWiFi");

    *pf_log << function_name << ": " << (void *)dDb;
    *pf_log << " " << szDB;
    *pf_log << " " << minProb;
    pf_log->flush();
    
    uint32_t crc = crc32((uint8_t*)dDb, szDB);
    *pf_log << " " << crc << std::endl;

    bool status = (dDb != 0) && (szDB != 0);

    if (true == status)
    {
        WiFi *pInstance = new (std::nothrow) WiFi_Locator(dDb, szDB, minProb, 5, -100, 0);

        status = (nullptr != pInstance) ? pInstance->status() : false;

        if (true == status)
        {
            delete wifi;
            wifi = pInstance;

            wifi->setBias(0.0, 0);
            wifi->setBiasGain(biasWiFiGain);
            wifi->setBiasRssiThreshold(-85);
        }
        else
        {
            delete pInstance;
        }
    }

    *pf_log << function_name << ": " << ( status ? "success" : "fail") << std::endl;

    return status;
}

bool FusionFilter::initializeBLE( const char* const dDb, const size_t szDB, const double minProb )
{
    std::string function_name("initializeBLE");

    *pf_log << function_name << ": " << (void *)dDb;
    *pf_log << " " << szDB;
    *pf_log << " " << minProb;
    pf_log->flush();

    uint32_t crc = crc32((uint8_t*)dDb, szDB);
    *pf_log << " " << crc << std::endl;

    bool status = (dDb != 0) && (szDB != 0);
    if (true == status)
    {
        const int min_ble_amount = 5; // minimal bluetooth beacons number for valid solution
        WiFi *pInstance = new (std::nothrow) WiFi_Locator(dDb, szDB, minProb, min_ble_amount, -95, 0);

        status = (nullptr != pInstance) ? pInstance->status() : false;

        if (true == status)
        {
            delete ble;
            ble = pInstance;
            ble->setBias(biasBLERSSI, biasBLEEstEnabled);
            ble->setBiasGain(biasBLEGain);
            // ble->setBiasRssiThreshold(-85); disabled utntil it will have been tested with KPI
        }
        else
        {
            delete pInstance;
        }
        *pf_log << function_name << ": " << (status ? "success" : "fail") << std::endl;
    }
        return status;
}

bool FusionFilter::initializeBLEProximity(IBLEProximity ** ppProxInstance, std::string ProxInstanceName, const char* const dDb, const size_t szDB,
    const GeoLocConverter2D &converter, const int N, const int M, const int SingleCutoffThreshold, const int MultipleCutoffThreshold)
{
    IBLEProximity *pProxInstance = *ppProxInstance;
    
    std::string function_name("initialize " + ProxInstanceName);

    *pf_log << function_name << ": " << (void *)dDb;
    *pf_log << " " << szDB;
    *pf_log << " " << N;
    *pf_log << " " << M;
    *pf_log << " "<< SingleCutoffThreshold;
    *pf_log << " " << MultipleCutoffThreshold;
    pf_log->flush();
    uint32_t crc = crc32((uint8_t*)dDb, szDB);
    *pf_log << " " << crc << std::endl;

    IBLEProximity *pInstance = nullptr;
    bool status = (dDb != 0) && (szDB != 0) && converter.IsInitialized();
    if (true == status) 
    {
        pInstance = new (std::nothrow) BLE_Proximity_Locator(dDb, szDB, converter, N, M, SingleCutoffThreshold);
    }

    status = (nullptr != pInstance) ? pInstance->status() : false;
    if (true == status)
    {
        delete pProxInstance;
        pProxInstance = pInstance;
        pProxInstance->setMultipleCutoffThreshold(MultipleCutoffThreshold); // set special settings for MultipleCutoffThreshold
        pProxInstance->setBias(biasBLEProxRSSI);
        *ppProxInstance = pProxInstance;
    }
    else
    {
        delete pInstance;
    }

    *pf_log << function_name << ": " << (status ? "success" : "fail");
    *pf_log << ", " << pProxInstance->size() << " beacons" << std::endl;

    return status;
}

bool FusionFilter::initializeBLEProximity_for_positioning(const char* const pProxMap, const size_t bleFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int rejectThreshold)
{
    return initializeBLEProximity(&ble_proximity, std::string("Positioning proximity"), pProxMap, bleFileSizeInBytes, converter, N, M, rejectThreshold, -10);
}

bool FusionFilter::initializeBLEProximity_for_detection(const char* const pProxMap, const size_t bleFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int rejectThreshold)
{
    return initializeBLEProximity(&ble_proximity_detection, std::string("Detection proximity"), pProxMap, bleFileSizeInBytes, converter, N, M, rejectThreshold, -10);
}

bool FusionFilter::initializeBLEProximity_for_start(const char* const pProxMap, const size_t bleFileSizeInBytes, const GeoLocConverter2D &converter, const int N, const int M, const int rejectThreshold)
{
    return initializeBLEProximity(&ble_proximity_start, std::string("Start proximity"), pProxMap, bleFileSizeInBytes, converter, N, M, rejectThreshold, rejectThreshold);
}

bool FusionFilter::initializeMFP( const char* const dDb, const size_t szDB, double max_X, double max_Y, double cellSize, int minFloor, int maxFloor )
{
    std::string function_name("initializeMFP");

    *pf_log << function_name << ": " << (void *)dDb;
    *pf_log << " " << szDB;
    *pf_log << " " << max_X;
    *pf_log << " " << max_Y;
    *pf_log << " " << cellSize;
    *pf_log << " " << minFloor;
    *pf_log << " " << maxFloor;
    pf_log->flush();
    uint32_t crc = crc32((uint8_t*)dDb, szDB);
    *pf_log << " " << crc << std::endl;

    bool status = (dDb != 0) && (szDB != 0);
    
    if (true == status)
    {
        tMFP *pInstance = new (std::nothrow) tMFP(0., max_X * 100, 0., max_Y * 100, cellSize * 100, minFloor, maxFloor);

        status = (nullptr != pInstance) ?
            (pInstance->status() && (pInstance->GetMapSize() == szDB)) :
            false;

        if (true == status)
        {
            delete mfp;
            mfp = pInstance;

            memcpy(mfp->GetMfpMap(), dDb, szDB);

            mfp->LimitMagneticMap();

            //TODO: make correct outdoor mag vector initialization
            tMFP_CellDescrApprox distrOutdoor;
            memset(&distrOutdoor, 0, sizeof(distrOutdoor));
            double h = -M_PI / 8;
            distrOutdoor.x.mu1 = (float)(-15 * cos(h) - 2 * sin(h));
            distrOutdoor.y.mu1 = (float)(2 * cos(h) + (-15) * sin(h));
            distrOutdoor.z.mu1 = -45;
            distrOutdoor.x.s1 = distrOutdoor.y.s1 = distrOutdoor.z.s1 = 10;
            mfp->setOutOfMapDistrib(distrOutdoor, false);

        }
        else
        {
            delete pInstance;
        }
    }

    *pf_log << function_name << ": " << (status ? "success" : "fail") << std::endl;

    return status;
}


bool FusionFilter::initializeMapMatching(const uint8_t* const pMap, const size_t szMap, double max_X, double max_Y, int16_t floor_shift, bool floor_zero_enable, const GeoLocConverter2D &converter)
{
    std::string function_name("initializeMapMatching");
    *pf_log << "initializeMapMatching: " << (void*)pMap;
    *pf_log << " " << szMap;
    *pf_log << " " << max_X;
    *pf_log << " " << max_Y;
    pf_log->flush();
    uint32_t crc = crc32((uint8_t*)pMap, szMap);
    *pf_log << " " << crc << std::endl;

    bool status = (pMap != 0) && (szMap != 0);

    if (true == status)
    {
        MapMatching *pInstance = new (std::nothrow) MapMatching(pMap, szMap, max_X, max_Y, floor_shift, floor_zero_enable, converter);

        status = (nullptr != pInstance) ? pInstance->get_validity() : false;
        if (true == status)
        {
            delete mm;
            mm = pInstance;
        }
        else
        {
            delete pInstance;
        }
    }

    *pf_log << function_name << ": " << (status ? "success" : "fail") << std::endl;

    return status;
}

void FusionFilter::setUpdateCompass( bool enable )
{
    mfp3D_update.setUpdate( enable );
}

void FusionFilter::setUpdateWiFi( bool enable )
{
    wifi_update_mix.setUpdate( enable );
}

void FusionFilter::setUpdateBLE( bool enable )
{
    ble_update_mix.setUpdate( enable );
}

void FusionFilter::setUpdateBLEProximity( bool enable )
{
    ble_proximity_update_mix.setUpdate(enable);
}

void FusionFilter::setUpdateMFP( bool enable )
{
    mfp3D_update_mix.setUpdate( enable );
}

void FusionFilter::setUpdateMM(bool enable)
{
    mm_update.setUpdate(enable);
}

void FusionFilter::setUpdatePortals(bool enable)
{
    mfp_portals.setUpdate(enable);
}

void FusionFilter::setUpdateExternalPosition( bool enable )
{
    pos_update.setUpdate( enable );
}

void FusionFilter::setUpdateCollaboration(bool enable)
{
    ble_collaboration_update.setUpdate(enable);
}

void FusionFilter::setCorrectFusionFilterPosition(bool enable)
{
    position_mm_correction_enable = enable;
}

void FusionFilter::setUseBarometer(bool enable)
{
    *pf_log << "SetUseBarometer: " << int(enable) << std::endl;
    isUseBarometer = enable;
    predict_lks_multypoint_pf1.set_enable_floor_prediction(enable);
    predict_lks_multypoint_pf2.set_enable_floor_prediction(enable);
}

void FusionFilter::setAdjustWfpParams( bool enable )
{
    bWfpAdjustment = enable;
}

void FusionFilter::setAdjustMfpParams( bool enable )
{
    bMfpAdjustment = enable;
}

void FusionFilter::setOsType(OperationSystemType os_type)
{
    this->os_type = os_type;
}


bool FusionFilter::GetMagneticBiasAndCov( int64_t* time, double * pMagBias, double * pMagBiasCov )
{
    bool result = false;
    *time = filter_timestamp;
    
    pMagBiasCov[0] = 1E6;
    pMagBiasCov[1] = 1E6;
    pMagBiasCov[2] = 1E6;

    //PF::Particle state;

    //if (pMixed->initialized() && pMixed->estimate(state))
    if (pMixed->initialized() && this->inTrackingMixed)
    {
        pMagBias[0] = mixed_pos.rbf.b[0];
        pMagBias[1] = mixed_pos.rbf.b[1];
        pMagBias[2] = mixed_pos.rbf.b[2];
        
        pMagBiasCov[0] = mixed_pos.rbf.P(0, 0);
        pMagBiasCov[1] = mixed_pos.rbf.P(1, 1);
        pMagBiasCov[2] = mixed_pos.rbf.P(2, 2);

        result = true;
    }
    return result;
}

void FusionFilter::SetMagneticBias( int64_t time, double * pMagBias )
{
    init_rbf->updateTime( time );
    init_rbf->updateBias( Eigen::Matrix<PF::Float, 3, 1>( pMagBias[0], pMagBias[1], pMagBias[2] ) );
    *pf_log << "SetMagneticBias: " << time << " "
            << pMagBias[0] << " "
            << pMagBias[1] << " "
            << pMagBias[2] << " "
            << std::endl;

}

void FusionFilter::SetMagneticBiasCov( double * pMagBiasCov )
{
    double s = 0;

    for( int i = 0 ; i < 3; ++i )
    {
        s  = std::max( s, pMagBiasCov[i] );
    }

    *pf_log << "SetMagneticCov: " << pMagBiasCov[0] << " "
            << pMagBiasCov[1] << " "
            << pMagBiasCov[2] << " "
            << std::endl;

    s = sqrt( s );
    init_rbf->updateSig( s );
}

// Gets platform type --------------------------------------------------------
PlatformType FusionFilter::getPlatformType()
{
    return this->platform_type;
}

// Sets platform type----------------------------------------------------------
void FusionFilter::setPlatformType(PlatformType platform_type)
{
    *pf_log << "setPlatformType: " << (int)platform_type << std::endl;
    this->platform_type = platform_type;

    setPredictionModelForMagFilter(this->venue_type, this->platform_type);
    setPredictionModelForMixedFilter(this->venue_type, this->platform_type);
}

// Sets venue type----------------------------------------------------------
void FusionFilter::setVenueType(VenueType venue_type)
{
    *pf_log << "setVenueType: " << (int)venue_type << std::endl;
    this->venue_type = venue_type;

    setPredictionModelForMagFilter(this->venue_type, this->platform_type);
    setPredictionModelForMixedFilter(this->venue_type, this->platform_type);
}

// Sets BLP pulling type ------------------------------------------------------
void FusionFilter::setBlpPulling(ePullingType type, double pulling_distance, double pulling_sigma)
{
    *pf_log << "setBlpPulling: " << (int)type << ",  pulling_distance: " << pulling_distance
        << ",  pulling_sigma: " << pulling_sigma << std::endl;
    ble_proximity_update_mix.setBlpPulling(type, pulling_distance, pulling_sigma);
}

void FusionFilter::setBlpDetectionEnable(bool enable)
{
    *pf_log << "setBlpDetectionEnable: " << (int)enable  << std::endl;
    ble_proximity_detection_enable = enable;
}

void FusionFilter::setBlpPositioningPdFilterParams(
    int peak_detector_max_delay_ms_in_moving,
    double descending_factor_in_moving,
    int peak_detector_max_delay_ms_in_stop,
    double descending_factor_in_stop
)
{
    *pf_log << "setBlpPositioningPdFilterParams: "
        << ",  peak_detector_max_delay_ms_in_moving: " 
        << peak_detector_max_delay_ms_in_moving
        << ",  descending_factor_in_moving: " << descending_factor_in_moving
        << ",  peak_detector_max_delay_ms_in_stop: "
        << peak_detector_max_delay_ms_in_stop
        << ",  descending_factor_in_stop: " << descending_factor_in_stop
        << std::endl;
    peak_detector_max_delay_ms_for_positioning_in_moving = peak_detector_max_delay_ms_in_moving;
    descending_factor_for_positioning_in_moving = descending_factor_in_moving;
    peak_detector_max_delay_ms_for_positioning_in_stop = peak_detector_max_delay_ms_in_stop;
    descending_factor_for_positioning_in_stop = descending_factor_in_stop;
}

void FusionFilter::setBlpDetectionPdFilterParams(
    int peak_detector_max_delay_ms_in_moving,
    double descending_factor_in_moving,
    int peak_detector_max_delay_ms_in_stop,
    double descending_factor_in_stop
)
{
    *pf_log << "setBlpDetectionPdFilterParams: "
        << ",  peak_detector_max_delay_ms_in_moving: "
        << peak_detector_max_delay_ms_in_moving
        << ",  descending_factor_in_moving: " << descending_factor_in_moving
        << ",  peak_detector_max_delay_ms_in_stop: "
        << peak_detector_max_delay_ms_in_stop
        << ",  descending_factor_in_stop: " << descending_factor_in_stop
        << std::endl;
    peak_detector_max_delay_ms_for_detection_in_moving = peak_detector_max_delay_ms_in_moving;
    descending_factor_for_detection_in_moving = descending_factor_in_moving;
    peak_detector_max_delay_ms_for_detection_in_stop = peak_detector_max_delay_ms_in_stop;
    descending_factor_for_detection_in_stop = descending_factor_in_stop;
}

void FusionFilter::setBlpPositioningLogicParams(int filter_length, int repeat_number,
    int cutoff)
{
    if ((ble_proximity != 0) && ble_proximity->size() > 0)
    {
        if (filter_length > 0)
        {
            *pf_log << "setBlpPositioningLogicParams: " << ",  filter_length: " << filter_length
                << ",  repeat_number: " << repeat_number << ",  cutoff: " << cutoff << std::endl;
            ble_proximity->setBLEProximityLogic(filter_length, repeat_number);
            ble_proximity->setSingleCutoffThreshold(cutoff);
        }
        else
        {
            *pf_log << "setBlpMultiBeaconUpdateParams: " << "cutoff: " << cutoff << std::endl;
            ble_proximity->setMultipleCutoffThreshold(cutoff);
        }
    }
    auto_positioning_poximity_params_update_for_os_type = false;
}

void FusionFilter::setBlpDetectionLogicParams(int filter_length, int repeat_number,
    int cutoff)
{
    if ((ble_proximity_detection != 0) && ble_proximity->size() > 0)
    {
        *pf_log << "setBlpDetectionLogicParams: "
            << ",  filter_length: " << filter_length
            << ",  repeat_number: " << repeat_number
            << ",  cutoff: " << cutoff << std::endl;
        ble_proximity_detection->setBLEProximityLogic(filter_length, repeat_number);
        ble_proximity_detection->setSingleCutoffThreshold(cutoff);
    }
    auto_detection_poximity_params_update_for_os_type = false;
}

void FusionFilter::resizePF()
{

}

WiFi_Location FusionFilter::estimateBLEPos(const int64_t timestamp)
{
    bool is_ble_solution = true;
    WiFi_Location ble_pos_upd;
    WiFi_Location tmp_ble_pos;
    WiFi_Measurement ble_meas;
    int64_t ble_timestamp = this->ble_pos.timestamp;
    int64_t solution_timestamp = 0;

    while ((timestamp - ble_timestamp) > time_interval && (is_ble_solution == true))
    {
        is_ble_solution = ble_helper.readWiFiSolution(solution_timestamp, tmp_ble_pos);
        if (is_ble_solution) 
        {
            ble_timestamp = solution_timestamp;
            ble_pos_upd = tmp_ble_pos;
        }
        //if( is_ble_solution && ble )
        //{
        //    //*ble_log << "estimateBLEPos  " << "ble_pos_upd.x = " << ble_pos_upd.x << "  ble_pos_upd.y = " << ble_pos_upd.y << "  ble_pos_upd.valid = " << ble_pos_upd.valid << std::endl;
        //}
    }
    
    // save current solution
#if 1
	this->ble_pos = ble_pos_upd;
	if (ble_pos_upd.valid)
		this->ble_pos.timestamp = ble_timestamp;
#else
    // repeat BLE solution using in fusion filter
    if (ble_pos_upd.valid)
    {
        this->ble_pos = ble_pos_upd;
		this->ble_pos.timestamp = ble_timestamp;
    }
    else
    {
        ble_pos_upd = this->ble_pos;
        ble_timestamp = this->ble_pos.timestamp;
    }
#endif

    // BLE solution delay check
    int64_t const k_ble_solution_delay_time = 5000;
    if ((timestamp - ble_timestamp) < -500 ||
        (timestamp - ble_timestamp) >  k_ble_solution_delay_time)
    {
        ble_pos_upd.valid = false;
    }
    return ble_pos_upd;
}

void FusionFilter::estimateBLEProximityPos(const int64_t step_timestamp, int64_t &ble_proximity_timestamp, std::vector<WiFi_Location > &LocationList)
{
    bool is_ble = false;
    ble_proximity_timestamp = 0;
    LocationList.clear();
    do
    {
        is_ble = ble_proximity_helper.readBLEProxSolution(ble_proximity_timestamp, LocationList);

    } 
    while ((is_ble == true) && (std::abs(step_timestamp - ble_proximity_timestamp) > 500));
}

WiFi_Location FusionFilter::estimateBLEProximityPos_for_start(const int64_t step_timestamp)
{
    bool is_ble = true;
    WiFi_Location ble_proximity_pos_upd_start;
    Fppe::BleScanResult ble_meas;
    int64_t ble_proximity_timestamp_start = this->ble_proximity_timestamp_start;

    while ((step_timestamp - ble_proximity_timestamp_start) > 500 && is_ble == true)
    {
        int measN;
        is_ble = ble_proximity_start_helper.readMeas(ble_proximity_timestamp_start, measN, ble_meas);

        if (is_ble && (ble_proximity_start != 0))
        {
            BLE_position ble_proximity_pos = ble_proximity_start->GetLocation(ble_meas, step_timestamp);
            ble_proximity_pos_upd_start = ble_proximity_pos.loc;
            if (ble_proximity_pos_upd_start.valid)
            {
                *pf_log << "ble proximity location for start  " << ble_proximity_pos_upd_start.x << "    " << ble_proximity_pos_upd_start.y << "    " << ble_proximity_pos_upd_start.z << "    " << ble_proximity_pos_upd_start.h << "    " << ble_proximity_pos_upd_start.valid << "    " << ble_proximity_pos_upd_start.p << "    " << std::endl;
            }
            if ((ble_proximity_pos_upd_start.x > 0.0) && (ble_proximity_pos_upd_start.y > 0.0) )
            {
                int validity_flag = static_cast <int> (ble_proximity_pos_upd_start.valid);
                //pBleProximityUpd->update(ble_proximity_pos_upd_start.x, ble_proximity_pos_upd_start.y, ble_proximity_pos_upd_start.z, ble_proximity_pos_upd_start.h, -1., (double)step_timestamp, NULL, validity_flag); // -1. - to indicate solution from start pipeline
            }
        }
    }
    this->ble_proximity_timestamp_start = ble_proximity_timestamp_start;
    return ble_proximity_pos_upd_start;
}

void FusionFilter::estimateCollaborationPos(const int64_t filter_timestamp, int64_t &collaboration_timestamp, std::vector<Fppe::CollaborationData> &LocationList)
{
    const double v_max = 1; // m/s
    const double d_max = 10; // m
    const double alfa = 1. / 12;
    double collaboration_speed_max = 1.0; // m/s
    std::vector<Fppe::CollaborationData > LocationList_tmp = getCollaborationPos(filter_timestamp);
    LocationList.clear();
    if (LocationList_tmp.size()> 0)
    {
        int index = -1;
        double min_pos_uncertainty = std::numeric_limits<double>::max();
        for (int i = 0; i < LocationList_tmp.size(); i++)
        {
            double x00 = LocationList_tmp[i].cov_ne[0][0];
            double x01 = LocationList_tmp[i].cov_ne[0][1];
            double x10 = LocationList_tmp[i].cov_ne[1][0];
            double x11 = LocationList_tmp[i].cov_ne[1][1];
            double d_amendment = alfa * d_max * d_max;
            double tau = (filter_timestamp - LocationList_tmp[i].timestamp)/1000.;
            double t_amendment = v_max * v_max * tau * tau;

            x00 += d_amendment;
            x00 += t_amendment;
            x11 += d_amendment;
            x11 += t_amendment;

            LocationList_tmp[i].cov_ne[0][0] = x00;
            LocationList_tmp[i].cov_ne[0][1] = x01;
            LocationList_tmp[i].cov_ne[1][0] = x10;
            LocationList_tmp[i].cov_ne[1][1] = x11;

            Eigen::MatrixXd cov_xy = Eigen::MatrixXd::Zero(2, 2);
            // filling the 2x2 matrix with values
            cov_xy(0, 0) = x00;
            cov_xy(0, 1) = x01;
            cov_xy(1, 0) = x10;
            cov_xy(1, 1) = x11;

            Eigen::EigenSolver<Eigen::MatrixXd> es;

            es.compute(cov_xy, false);

            double eigen_value_1_real = es.eigenvalues().transpose()(0, 0).real();
            double eigen_value_2_real = es.eigenvalues().transpose()(0, 1).real();
            double eigen_value_1_imag = es.eigenvalues().transpose()(0, 0).imag();
            double eigen_value_2_imag = es.eigenvalues().transpose()(0, 1).imag();

            // here we check that imaginary part of complex number is 0.0 for both eigen values
            if ((std::abs(eigen_value_1_imag) > std::numeric_limits<double>::epsilon())
                || (std::abs(eigen_value_2_imag) > std::numeric_limits<double>::epsilon()))
                continue;

            double pos_uncertainty = std::sqrt(std::max(eigen_value_1_real, eigen_value_2_real));
            if (pos_uncertainty < min_pos_uncertainty)
            {
                min_pos_uncertainty = pos_uncertainty;
                index = i;
            }

        }
        if (index >= 0)
        {
            collaboration_timestamp = LocationList_tmp[index].timestamp;
            LocationList.push_back(LocationList_tmp[index]);
        }
    }
}

bool FusionFilter::getWiFiLocationList(const int64_t timestamp, int64_t &solution_timestamp, std::vector<WiFi_Location> &loc_list, int64_t max_delay)
{
    bool is_valid(false), is_ok(false);
    solution_timestamp = timestamp;

    while ((is_ok = wifi_helper.readWiFiSolution(solution_timestamp, loc_list)) && ((timestamp - solution_timestamp) > 500))
    {
        is_valid = is_valid || is_ok;
    } 

    is_valid = (is_valid || is_ok)  && ((timestamp - solution_timestamp) < max_delay);

    return is_valid;
}

WiFi_Location FusionFilter::estimateWiFiPos(const int64_t timestamp, const int64_t wifi_solution_delay_limit)
{
    WiFi_Measurement wifi_meas;
    int64_t wifi_timestamp = this->wifi_pos.timestamp;
	this->wifi_timestamp = this->wifi_pos.timestamp;
	
	bool wifi_new_solution = false;
    
    WiFi_Location wifi_pos_upd;
    int64_t solution_timestamp = 0;
    bool is_wifi_solution = true;

    while ( ( timestamp - wifi_timestamp ) > 500 && is_wifi_solution == true )
    {
        is_wifi_solution = wifi_helper.readWiFiSolution(solution_timestamp, wifi_pos_upd);
        
        wifi_timestamp = solution_timestamp;
                
        if (is_wifi_solution && wifi)
        {
            //wifi_new_solution = true; // only used in "patch for rejection of WiFi solution repeating using"

#if 0 // patch for rejection of WiFi solution after gap
            /*********** START OF PATCH***************/
            const  long k_solution_delay = 1;

            if (last_wifi_timestamp <= 0)
            {
                last_wifi_timestamp = wifi_timestamp;
            }

            if ((last_wifi_timestamp > 0) && (std::abs(wifi_timestamp - last_wifi_timestamp) > 9000) && (wifi_pos_upd.valid == true))
            {
                //std::cout << "set:" << last_wifi_timestamp << "," << wifi_timestamp << "," << start_count_down << " to " << k_solution_delay << std::endl;
                start_count_down = k_solution_delay;
            }
            if (wifi_pos_upd.valid == true)
            {
                last_wifi_timestamp = wifi_timestamp;
            }
            if ((this->wifi_pos.timestamp > 0) && (std::abs(wifi_timestamp - this->wifi_pos.timestamp) > 9000) && (wifi_pos_upd.valid == true))
            {
                if (start_count_down > 0)
                {
                    wifi_pos_upd.valid = false;
                }
                //std::cout << "reset:" << this->wifi_pos.timestamp << "," << wifi_timestamp << "," << start_count_down << ", " << (int)wifi_pos_upd.valid << std::endl;
                start_count_down--;
            }
            /*********** END OF PATCH***************/
#endif
        }
    }
    
#if 0 // patch for rejection of WiFi solution repeating using
    /*********** START OF PATCH***************/
    if ((wifi_new_solution) && (this->wifi_pos.timestamp != wifi_timestamp))
    {
        if ((std::abs(this->wifi_pos.x - wifi_pos_upd.x) + std::abs(this->wifi_pos.y - wifi_pos_upd.y)) < 0.01)
        {
            wifi_pos_upd.valid = false;
        }
    }
    /*********** END OF PATCH***************/
#endif

    // save current solution
    if (wifi_pos_upd.valid)
    {
        this->wifi_pos = wifi_pos_upd;
		this->wifi_pos.timestamp = wifi_timestamp;
    }
    else
    {
        wifi_pos_upd = this->wifi_pos;
		wifi_timestamp = this->wifi_pos.timestamp;
    }

    //int64_t k_wifi_solution_repeat_time = 5000;
    // select solution for current solution
    if ( ( timestamp - wifi_timestamp ) < -500 ||
            ( timestamp - wifi_timestamp ) >  wifi_solution_delay_limit )
    {
        wifi_pos_upd.valid = false;
    }
    
    return wifi_pos_upd;
}

WiFi_Location  FusionFilter::rejectWiFiSpike( const WiFi_Location &wifi_pos )
{
    WiFi_Location wifi_pos_upd = wifi_pos;

    //if( wifi_pos_upd.valid && pMixed->initialized() && inTracking )
    if (wifi_pos_upd.valid && pMixed->initialized() && false /*pMixed->is_in_tracking()*/)
    {
        double wifi_delta = sqrt(   ( mixed_pos.pos.x - wifi_pos_upd.x ) * ( mixed_pos.pos.x - wifi_pos_upd.x ) +
                                    ( mixed_pos.pos.y - wifi_pos_upd.y ) * ( mixed_pos.pos.y - wifi_pos_upd.y ) );

        if( ( wifi_delta > WIFI_SPIKE_THRESHOLD ) && ( wifi_spike_count < WIFI_MAX_SPIKE_COUNT ) )
        {
            wifi_pos_upd.valid = false;
            wifi_spike_count++;
        }
        else
        {
            wifi_spike_count = 0;
        }
    }

    return wifi_pos_upd;
}

PositionWithUncertainties FusionFilter::getExternalPos( const int64_t step_timestamp )
{
    PositionWithUncertainties pos = {};

    while ( (pos_queue.size() > 0) && ((step_timestamp - pos_queue.front().timestamp) > integration_time))
    {
        pos_queue.pop();
    }

    if ( (pos_queue.size() > 0) && 
         (std::abs(step_timestamp - pos_queue.front().timestamp) <= integration_time))
    {
        pos = pos_queue.front();
        pos_queue.pop();
    }

    return pos;
}

std::vector <Fppe::CollaborationData> FusionFilter::getCollaborationPos(const int64_t step_timestamp)
{
    std::vector <Fppe::CollaborationData> pos = {};
    if (collaboration_queue.size() > 0)
    {
        pos = collaboration_queue.back();
        while (collaboration_queue.size() > 0)
            collaboration_queue.pop();
    }
    return pos;
}


/*PositionWithUncertainties FusionFilter::getProximityPos( const int64_t step_timestamp )
{
    PositionWithUncertainties pos = {};

    while ( (proximity_pos_queue.size() > 0) && ((step_timestamp - proximity_pos_queue.front().timestamp) > integration_time))
    {
        proximity_pos_queue.pop();
    }

    if ( (proximity_pos_queue.size() > 0) && 
         (std::abs(step_timestamp - proximity_pos_queue.front().timestamp) <= integration_time))
    {
        pos = proximity_pos_queue.front();
        proximity_pos_queue.pop();
    }

    return pos;
}*/

bool FusionFilter::getWiFiBias( double &bias )
{
    if (wifi)
    {
        return  wifi->getBias(bias);
    }
    else
    {
        bias = 0;
        return false;
    }
}

void FusionFilter::setWiFiBias( double  bias, int64_t delta_t )
{
    if (wifi)
    {
        biasWiFiEstEnabled = true;
        if (std::abs(bias) < 20)
            wifi->setBias(bias, delta_t);
    }
}

bool FusionFilter::getBFPBias( double &bias )
{
    if (ble)
    {
        return  ble->getBias(bias);
    }
    else
    {
        bias = 0;
        return false;
    }
}

void FusionFilter::setBFPBias( double  bias, int64_t delta_t )
{
    if (ble)
    {
        biasBLEEstEnabled = true;
        if (std::abs(bias) < 20)
            ble->setBias(bias, delta_t);
    }
}

bool FusionFilter::getBLEProxBias(double &bias)
{
    if (ble_proximity)
    {
        double biasRssiSigma;
        return  ble_proximity->getBias(bias, biasRssiSigma);
    }
    else
    {
        bias = 0;
        return false;
    }
}

bool FusionFilter::getBLEBiasWithUncertainty(double &bias, double &bias_uncertainty)
{
	if (ble_proximity)
	{
		return ble_proximity->getBias(bias, bias_uncertainty);
	}

	else
	{
		if (ble)
		{
			bool result = ble->getBias(bias);
			bias_uncertainty = 5.0;
			return result;
		}

		else
		{
			bias = 0;
			bias_uncertainty = 5.0;
			return false;
		}
	}
}

void FusionFilter::setBLEProxBias(double  bias, int64_t delta_t )
{
    if (0 != ble_proximity)
    {
        ble_proximity->setBias(bias);
    }
    if (0 != ble_proximity_detection)
    {
        ble_proximity_detection->setBias(bias);
    }
    if (0 != ble_proximity_start)
    {
        ble_proximity_start->setBias(bias);
    }

}

void FusionFilter::setBLEBiasWithUncertainty(double bias, double bias_uncertainty, int64_t bias_age)
{
    // setting bias and uncertainty
    // uncertainty is modified based on bias_age
    double k = 1. / (60 * 60 * 24 * 1000) * 0.5; // 0.5 dBm every  in 24h
    double s = bias_uncertainty + k * bias_age;
    double actual_bias_uncertainty = std::min(5.0, s);
    if (0 != ble_proximity)
    {
        ble_proximity->setBias(bias);
        ble_proximity->setBiasSigma(actual_bias_uncertainty);
    }
    if (0 != ble_proximity_detection)
    {
        ble_proximity_detection->setBias(bias);
        ble_proximity_detection->setBiasSigma(actual_bias_uncertainty);
    }
    if (0 != ble_proximity_start)
    {
        ble_proximity_start->setBias(bias);
        ble_proximity_start->setBiasSigma(actual_bias_uncertainty);
    }

}

bool FusionFilter::getOutputBLEBias(double &bias)
{
    bool return_status = false;
    if (ble)
    {
        return_status = getBFPBias(bias);
    }

    if (!return_status)
    {
        return_status = getBLEProxBias(bias);
    }

    return return_status;
}

void FusionFilter::setStartPositionWithUncertainties( double _x, double _y, double _z, double _heading, const double cov_xy[2][2], double _std_z, double _std_h, int64_t timestamp )
{
	PositionWithUncertainties pos = {};
    pos.timestamp = timestamp;
    pos.x = _x;
    pos.y = _y;
    pos.z = _z;
    pos.heading = _heading;

    pos.cov_xy[0][0] = cov_xy[0][0];
	pos.cov_xy[0][1] = cov_xy[0][1];
	pos.cov_xy[1][0] = cov_xy[0][1];
	pos.cov_xy[1][1] = cov_xy[1][1];
    pos.std_z = _std_z;
    pos.std_h = _std_h;
    pos.valid = true;
	
    double average_sigma = ( sqrt( cov_xy[0][0] ) + sqrt( cov_xy[1][1] ) ) / 2;

    *pf_log << "SetStartPosition:" << pos.timestamp
        << " ," << pos.x
        << " ," << pos.y
        << " ," << pos.z
        << " ," << pos.heading
        << " ," << "[" << cov_xy[0][0] << " ," << cov_xy[0][1] << " ," << cov_xy[1][0] << " ," << cov_xy[1][1] << "]"
        << " (" << average_sigma << ")"
        << " ," << pos.std_z
        << " ," << pos.std_h
        << std::endl;

    if ( average_sigma > 5.5 ) // previouse settins was 4
    {
        //init_mfp_area->setPos( pos ); // init_mfp_area initializer for mag_start_pf
		init_mixed_area->setPos( pos ); 
		init_mixed_area_mm->setPos( pos );
    }
    else
    {
        init_normal_mix->setPos( pos ); // direct mixed filter initialization
        init_normal_mfp->setPos( pos ); // direct mfp filter initialization
    }
}



//----------------------------------------------------
void FusionFilter::openLogFiles( const std::string &file_folder, const std::string &log_time )
{

}

void FusionFilter::closeLogFiles()
{

}


///<TODO
void FusionFilter::initializeVariables()
{
    wifi = 0;
    ble = 0;
    mfp = 0;
    mm = 0;

    biasWiFiRSSI = 0;
    biasWiFiGain = 0.01; //WiFi bias estimation step
    biasWiFiEstEnabled = true;
    //biasWiFiEstSleeped = false;
    //wifi_new_meas = false;

    biasBLERSSI = 0;
    biasBLEGain = 0.01; //BLE bias estimation step
    biasBLEEstEnabled = true;
    //biasWiFiEstSleeped = false;

    biasBLEProxRSSI = 0;
    biasBLEProxEstEnabled = true;

    is_motion = false;
    updatersState = eES_Unknown;
    wfpt_count_down = mgpt_count_down = env_count_down = 0;
    szpf_count_down = 20;

    pWiFiUpd = NULL;
    pBleUpd = NULL;
    pMfpUpd = NULL;
    pMixedUpd = NULL;
    venueWiFiUpd = NULL;
    pExtendedProximityUpd = nullptr;

    //wifi_file = 0;
    //ble_file = 0;
    //waypoints_file = 0;

    wifi_spike_count = 0;

    wifi_pos.valid = false;
	wifi_pos.timestamp = 0;
    last_wifi_timestamp = 0;
    start_count_down = -1;
    wifi_injection_count = 0;

    ble_pos.valid = false;
	ble_pos.timestamp = 0;

	ble_proximity_timestamp_start = 0;
	ble_proximity_bias_timestamp = 0;
    collaboration_timestamp = 0;

    filter_timestamp = 0;
    integration_time = 500;

    baroEnabled = false;
    bWfpAdjustment = false;
    bMfpAdjustment = false;
    pfMixedEnabled = true;

    magFifo = MagFIFO();
    prediction = Prediction<PF::Float>();

    sum_N = 0;

    pMFP = 0;
    pMixed = 0;

    init_wifi = 0;
    init_ble = 0;
    init_proximity = 0;
    init_wifi_mm = 0;
    init_ble_mm = 0;
    init_proximity_mm = 0;
    init_pf = 0;
    init_fine = 0;
    init_mfp_pf = 0;
    init_mfp_area = 0;
    init_normal_mfp = 0;
    init_normal_mix = 0;
    init_mfp = 0;
    init_rbf = 0;
	init_mixed_area = 0;
	init_mixed_area_mm = 0;

    init_collaboration = 0;
    init_collaboration_mm = 0;

    mixed_pos = PF::Particle();
    sigma_mfp = 0;
    sigma_mix = 0;

    pf_mix_min_plimit = PF1_PARTICLE_COUNT;
    pf_mix_max_plimit = static_cast<int>(2.5 * PF1_PARTICLE_COUNT);
    pf_mfp_min_plimit = PF1_PARTICLE_COUNT;
    pf_mfp_max_plimit = static_cast<int>(5 * PF1_PARTICLE_COUNT);

    pos_queue = std::queue<PositionWithUncertainties>();
    //proximity_pos_queue = std::queue<PositionWithUncertainties>();

    hard_reset_enabled = false;
    hard_reset_required = false;
    heading_is_stable = false;
	reacquisition_injection_percentage = 0.0;
    reacquisition_injection_percentage_for_change_floor = 0.45;
    last_hard_reset_time = 0;
	last_soft_injection_time = 0;
    reacquisition_block_in_stops = 0.0;
    level_queue = std::deque<double>();

    position_mm_correction_enable = true;

    os_type = OperationSystemType::OS_ANDROID;
    platform_type = PlatformType::pftp_PEDESTRIAN;
    venue_type = VenueType::kAisleVenue;
}

// Updaters initialization
void FusionFilter::initializeUpdaters()
{
    //PF MFP
    mfp3D_update.initialize( this );
    mfp3D_update.setName( "mfp3D_updater_mfp" );
    mfp3D_update.setUpdate( true );

    //PF Mix
    mfp_portals.initialize(this);
    mfp_portals.setName("mfp_portals");
    mfp_portals.setUpdate(true);
    pf_mix_updaters.push_back(&mfp_portals);
    //----------------------------
    wifi_update_mix.initialize(&wifi, this);
    wifi_update_mix.setName( "wifi_updater_mix" );
    wifi_update_mix.setSig( PF1_SIGMA_WIFI, 0.5 );
    wifi_update_mix.setUpdate( false );
    pf_mix_updaters.push_back( &wifi_update_mix );
    //----------------------------
    ble_update_mix.initialize(&ble, this);
    ble_update_mix.setName( "ble_updater_mix" );
    ble_update_mix.setSig( PF1_SIGMA_BLE, 0.5 );
    ble_update_mix.setUpdate( false );
    pf_mix_updaters.push_back( &ble_update_mix );
    //----------------------------
    ble_proximity_update_mix.initialize(&ble_proximity, this);
    ble_proximity_update_mix.setName("ble_proximity_update_mix");
    ble_proximity_update_mix.setSig(PF1_SIGMA_BLE_PROXIMITY, 0.5);
    ble_proximity_update_mix.setUpdate(false);
    pf_mix_updaters.push_back(&ble_update_mix);
    //----------------------------
    mfp3D_update_mix.initialize(this);
    mfp3D_update_mix.setName( "mfp3D_updater_mix" );
    mfp3D_update_mix.setUpdate( false );
    mfp3D_update_mix.set_mfp_map_matching( false );
    pf_mix_updaters.push_back( &mfp3D_update_mix );
    //----------------------------
    pos_update.initialize( this );
    pos_update.setName( "pos_updater" );
    pos_update.setUpdate( false );
    pf_mix_updaters.push_back( &pos_update );
    //----------------------------
    mm_update.initialize(this);
    mm_update.setName("mm_updater");
    mm_update.setUpdate(false);
    pf_mix_updaters.push_back(&mm_update);

    //----------------------------
    ble_collaboration_update.initialize(this);
    ble_collaboration_update.setName("ble_collaboration_updater");
    ble_collaboration_update.setUpdate(false);
    pf_mix_updaters.push_back(&ble_collaboration_update);

    //PF Mag start
    mfp3D_update_init.initialize( this );
    mfp3D_update_init.setName( "mfp3D_update_init" );
    mfp3D_update_init.setUpdate( true );
    //----------------------------
    rbf_bias_init.initialize( this );
    rbf_bias_init.setName( "rbf_bias_init" );
    rbf_bias_init.setD( 12 );
    rbf_bias_init.setUpdate( true );

    //Common
    rbf_bias.initialize( this );
    rbf_bias.setName( "rbf_bias_mfp_mix" );
    rbf_bias.setD( 12 );
    rbf_bias.setUpdate( true );
}

///<TODO
void FusionFilter::initializeInitializers()
{
    PF::FilterInitializer *pNext; 

    // mag start filter
    init_rbf = new PF::RBFInitializer();
    //-----------------------------------
    init_mfp = new PF::InitializerMFP_RBF(init_rbf, &mfp);
    init_mfp_area = new PF::MagInitializerInArea_RBF(init_rbf, &mfp, init_mfp);
    //init_mfp_frw = new PF::MagInitializerInArea_RBF(init_rbf, &mfp, init_mfp_area);
    //-----------------------------------
    init_mfp_pf = new PF::InitializerMfpPF("InitializerMfpPF");
    init_mfp_pf->setInitializer(init_mfp_area); // init_mfp is now used as next by init_mfp_area
    //init_mfp_pf->setInitializer(init_mfp_frw); // init_mfp is now used as next by init_mfp_area
    init_mfp_pf->setPredictionModel(&tpn_3d_start);
    tpn_3d_start.set_sigma_H((PF::Float)(2 * M_PI / 180));
    init_mfp_pf->setUpdater(&mfp3D_update_init, &rbf_bias_init);
    
    //wifi initializers
#if ENABLE_MULTIPOINT_WIFI_INITIALIZER
    pNext = init_wifi = new PF::InitializerInSeveralAreas_RBF(init_rbf, &mfp, init_mfp_pf);
#else
    pNext = init_wifi = new PF::InitializerInArea_RBF(init_rbf, &mfp, init_mfp_pf, "MagInAreaRbf_Wifi");
#endif
    pNext = init_wifi_mm = new PF::InitializerInAreaMapMatching_RBF(init_rbf, &mm, &mfp, pNext, "MagInArea_MM_Rbf_Wifi");

    // ble initializers
    pNext = init_ble = new PF::InitializerInArea_RBF(init_rbf, &mfp, pNext, "MagInAreaRbf_BLE");
    pNext = init_ble_mm = new PF::InitializerInAreaMapMatching_RBF(init_rbf, &mm, &mfp, pNext, "MagInArea_MM_Rbf_BLE");

    // proximity initializers
    pNext = init_proximity = new PF::InitializerInArea_RBF(init_rbf, &mfp, pNext, "MagInAreaRbf_Proximity");
    pNext = init_proximity_mm = new PF::InitializerInAreaMapMatching_RBF(init_rbf, &mm, &mfp, pNext, "MagInArea_MM_Rbf_Proximity");
	pNext = init_mixed_area = new PF::InitializerInArea_RBF(init_rbf, &mfp, pNext, "MagInAreaRbf_Start_Position");
	pNext = init_mixed_area_mm = new PF::InitializerInAreaMapMatching_RBF(init_rbf, &mm, &mfp, pNext, "MagInArea_MM_Rbf_Start_Position");

    pNext = init_collaboration = new PF::InitializerInArea_RBF(init_rbf, &mfp, pNext, "MagInAreaRbf_BLECollaboration");
    pNext = init_collaboration_mm = new PF::InitializerInAreaMapMatching_RBF(init_rbf, &mm, &mfp, pNext, "MagInArea_MM_Rbf_BLECollaboration");

    //mfp end point initializer
    init_normal_mfp = new PF::InitializerNormal_RBF(init_rbf, pNext, "Mfp.InitNormalRbf");
    
    //mixed end point initializer
    init_normal_mix = new PF::InitializerNormal_RBF(init_rbf, pNext, "Mixed.InitNormalRbf");
}

void FusionFilter::initializeInjectors()
{
	injector_rbf = new PF::RBFInitializer();

	PF::FilterInitializer *pNext;
#if ENABLE_MULTIPOINT_WIFI_INITIALIZER
	pNext = injector_wifi = new PF::InitializerInSeveralAreas_RBF(injector_rbf, &mfp, "UniformInjectorInSeveralAreas_Wifi");
	injector_wifi->setRBFinheritance(true);
#else
    pNext = injector_wifi = new PF::InitializerInArea_RBF(injector_rbf, &mfp, "UniformInjectorInArea_Wifi");
	injector_wifi->setRBFinheritance(true);
#endif

    pNext = injector_wifi_mm = new PF::InitializerInAreaMapMatching_RBF(injector_rbf, &mm, &mfp, pNext, "UniformInjectorInArea_MM_Wifi");

    pNext = injector_ble = new PF::InitializerInArea_RBF(injector_rbf, &mfp, pNext, "UniformInjectorInArea_BLE");

    pNext = injector_ble_mm = new PF::InitializerInAreaMapMatching_RBF(injector_rbf, &mm, &mfp, pNext, "UniformInjectorInArea_MM_BLE");

    pNext = injector_proximity = new PF::InitializerInArea_RBF(injector_rbf, &mfp, pNext, "UniformInjectorInArea_Proximity");

    pNext = injector_proximity_mm = new PF::InitializerInAreaMapMatching_RBF(injector_rbf, &mm, &mfp, pNext, "UniformInjectorInArea_MM_Proximity");

    pNext = injector_collaboration = new PF::InitializerInArea_RBF(injector_rbf, &mfp, pNext, "UniformInjectorInArea_BLE_Collaboration");

    pNext = injector_collaboration_mm = new PF::InitializerInAreaMapMatching_RBF(injector_rbf, &mm, &mfp, pNext, "UniformInjectorInArea_MM_BLE_Collaboration");

    injector_normal_mix = new PF::InitializerNormal_RBF(injector_rbf, pNext, "NormalInjectorInArea_Proximity");
	
	injector_ble->setRBFinheritance(true);
	injector_proximity->setRBFinheritance(true);
	injector_normal_mix->setRBFinheritance(true);

	injector_ble_mm->setRBFinheritance(true);
	injector_proximity_mm->setRBFinheritance(true);
	injector_wifi_mm->setRBFinheritance(true);

    injector_collaboration->setRBFinheritance(true);
    injector_collaboration_mm->setRBFinheritance(true);

	injector_wifi->set_max_pos_uncertainty(10.0);
	injector_wifi_mm->set_max_pos_uncertainty(10.0);

	injector_ble->set_max_pos_uncertainty(10.0);
	injector_ble_mm->set_max_pos_uncertainty(10.0);

	injector_proximity->set_max_pos_uncertainty(10.0);
	injector_proximity_mm->set_max_pos_uncertainty(10.0);

	injector_normal_mix->set_max_pos_uncertainty(10.0);
	
	injector_collaboration->set_max_pos_uncertainty(10.0);
	injector_collaboration_mm->set_max_pos_uncertainty(10.0);
}

//Initialize MFP filter
///<TODO Check
void FusionFilter::initializeFilterMFP()
{
    pMFP = new PF::ParticleFilter( pf_mfp_min_plimit );
    dynamic_cast<PF::ParticleFilter*>( pMFP )->setName( "ParticleFilter_MFP" );
    pMFP->setInitializer( init_normal_mfp ); //TODO  add covariance params to init fine
    
    setPredictionModelForMagFilter(venue_type , platform_type);

    pMFP->addUpdateModel( &mfp3D_update );
    pMFP->addRBFModel( &rbf_bias );
    pMFP->setInjection( false );
    pMFP->setEnabled( true );
}

//Initialize Mixed filter
///<TODO Check
void FusionFilter::initializeFilterMixed()
{
    pMixed = new PF::ParticleFilter(pf_mix_min_plimit);
    dynamic_cast<PF::ParticleFilter*>( pMixed )->setName( "ParticleFilter_Mixed" );
    pMixed->setInitializer( init_normal_mix );
    //pMixed->setInitializer(init_mfp); // TODO: check if this needs to be changed back 
	pMixed->setInjector(injector_normal_mix);
	
    setPredictionModelForMixedFilter(venue_type, platform_type);
    
    pMixed->addUpdateModel( &mfp3D_update_mix );
    pMixed->addUpdateModel( &wifi_update_mix );
    pMixed->addUpdateModel( &ble_update_mix );
    pMixed->addUpdateModel(&ble_proximity_update_mix);
    pMixed->addUpdateModel(&pos_update);
    pMixed->addUpdateModel(&mfp_portals);
    pMixed->addUpdateModel(&mm_update);
    pMixed->addUpdateModel(&ble_collaboration_update);
    pMixed->addRBFModel( &rbf_bias );
    pMixed->setInjection( false );
    pMixed->setEnabled( true );
}


const Prediction<PF::Float>& FusionFilter::getPrediction() const
{
    return prediction;
}

const MagMeasurements<>& FusionFilter::getMagMeas() const
{
    return mag_avg;
}

tMFP * const & FusionFilter::getMFP() const
{
    return mfp;
}

MapMatching * const & FusionFilter::getMapMatching() const
{
	return mm;
}

const PositionWithUncertainties& FusionFilter::getExternalPosition() const
{
    return external_pos;
}

bool FusionFilter::integrate( const double mis_std, const double d_pos[3], const double pos_cov[3][3], const double quat[4], const double quat_cov[4][4], const bool quat_valid, const bool is_motion, const int64_t timestamp, const bool is_transit)
{
    bool next_iter = false;
    prediction.t = timestamp;
    prediction.x += d_pos[0];
    prediction.y += d_pos[1];
    prediction.floor += d_pos[2];
		
    for ( int i = 0; i < 3; ++i )
    {
        for ( int j = 0; j < 3; ++j )
        {
            prediction.pos_cov( i, j ) += pos_cov[i][j];
        }
    }

    // TO DO: review averaging procedure,ake in to account quat_valid flag
    Eigen::Quaternion<PF::Float> qi2l( quat[0], quat[1], quat[2], quat[3] );
    prediction.C_avg += qi2l.toRotationMatrix();

    if ((timestamp - filter_timestamp) >= (10. * integration_time))
    {// reset integration
        prediction = Prediction<PF::Float>();
        sum_N = 0;
        filter_timestamp = timestamp;
        while (!magFifo.empty())
        {
            magFifo.pop();
        }
    }
    else if ( ( timestamp - filter_timestamp ) >= integration_time )
    {
        //double sigma = ( pMFP->getEnabled() ) ? sigma_mfp : sigma_mix;
        ////double sigma = ( pMixed->getEnabled() ) ? sigma_mix : sigma_mfp;
        //bool pf_started = ( pMFP->getEnabled() ) ? pMFP->initialized() : pMixed->initialized();
        ////bool pf_started = (pMixed->getEnabled()) ? pMixed->initialized() : pMFP->initialized();

        //if ( pf_started && is_step == false && sigma > 6 )
        //{
        //    prediction.pos_cov.block<2, 2>( 0, 0 ).setZero();
        //    prediction.pos_cov( 0, 0 ) = 1E-10;
        //    prediction.pos_cov( 1, 1 ) = 1E-10;
        //}

        // TODO: add motion flag accumulation
        if (is_motion == false )
        {
            prediction.x = 0;
            prediction.y = 0;
        }

        prediction.C_avg /= sum_N;

        ///sync magnetic
        MagMeas m = MagMeas( 0, std::vector<double>( 3 ) );

        // to do: suppotr forward and backward time
        mag_avg.m.setZero();
        int mn = 0;

        while ( magFifo.size() > 0 && m.first < timestamp )
        {
            m = magFifo.front();
            magFifo.pop();
            mag_avg.m( 0 ) += m.second[0];
            mag_avg.m( 1 ) += m.second[1];
            mag_avg.m( 2 ) += m.second[2];
            ++mn;
        }

        if ( mn > 0 )
        {
            mag_avg.m /= mn;
        }

        //m.second[0] = mag( 0 );
        //m.second[1] = mag( 1 );
        //m.second[2] = mag( 2 );

        if ( ( m.first != 0 ) && ( std::abs( m.first - timestamp ) <= 25 ) ) // 25ms - permissible time lag for 20 Hz TPN sample rate
        {
            mag_avg.valid = true;
            mag.valid = true;
        }

        next_iter = true;
    }

    prediction.is_motion = is_motion; // To Do: calc integral flag
    prediction.is_transit = is_transit; // To Do: calc integral flag
    // prediction.qi2l = qi2l;
    prediction.quat_valid = quat_valid; // To Do: calc integral flag
    prediction.mis_std = mis_std;

    prediction.qi2l = Eigen::Quaternion<PF::Float>( prediction.C_avg );

    prediction_mfp = prediction;
    prediction_mix = prediction;

    return next_iter;
}

void FusionFilter::controlMagFilter()
{
    int particle_number = std::max(pf_mfp_min_plimit, pMFP->getParcticlesCount());
    if ( sigma_mfp > 3 )
    {
        particle_number = (int)((sigma_mfp / 3) * (sigma_mfp / 3) * pf_mfp_min_plimit);
    }
    else if ( sigma_mfp < 2 )
    {
        particle_number = pf_mfp_min_plimit;
    }

    particle_number = std::min(particle_number, pf_mfp_max_plimit);
    particle_number = std::max(particle_number, pf_mfp_min_plimit);

    bool needResize = (particle_number != pMFP->getParcticlesCount());
    //needResize = needResize && (pMFP->is_in_tracking() || (!pMFP->is_in_tracking() && !pMFP->initialized()));
    bool needIncrease = (particle_number > pMFP->getParcticlesCount());
    needResize = needResize && (pMFP->is_in_tracking() || (!pMFP->is_in_tracking() && needIncrease));

    if (needResize)
    {
        pMFP->resize(particle_number);

		if (needIncrease)
		{
			static_cast<PF::ParticleFilter*>(pMFP)->do_resample();
		}
    }

    if ( pMFP->initialized() && prediction_mfp.is_motion == false && sigma_mfp > 6 )
    {
        prediction_mfp.pos_cov.block<2, 2>( 0, 0 ).setZero();
        prediction_mfp.pos_cov( 0, 0 ) = 1E-10;
        prediction_mfp.pos_cov( 1, 1 ) = 1E-10;
    }
}

void FusionFilter::controlMixedFilter()
{
    bool enable = false;
    int cnt = 0;

	// adaptation of mixed filter lkh-motion model in depending on mixed pos dispersion
	if (venue_type == VenueType::kFactoryVenue)
	{
		if (pMixed->initialized() && (sigma_mix > 0.) && (sigma_mix < 2.))
		{
			setPredictionModelForMixedFilter(VenueType::kAisleVenue, platform_type);
		}
		else
		{
			setPredictionModelForMixedFilter(venue_type, platform_type);
		}
	}

    // MM updater controll for start and tracking
    if ((pMixed->is_in_tracking() == true) || (sigma_mix < 3.))
    {
        pMixed->addUpdateModel(&mm_update);
    }
    else
    {
        pMixed->removeUpdateModel(&mm_update);
    }

    // enabled updater count calculation
    for ( auto it = pf_mix_updaters.begin(); it != pf_mix_updaters.end(); ++it )
    {
        bool b = ( *it )->enabled();
        enable |= b;

        if ( b )
        {
            cnt++;
        }
    }

    // enable filter in dependance on enabled updater number
    enable &= pfMixedEnabled;
    pMixed->setEnabled( enable );

    // control of max particle number
    int max_particle_number(pf_mix_max_plimit); // default value

    if ( cnt == 1 && mfp3D_update_mix.enabled() ) // special mix filter mode in dependance of FF settings
    {
        max_particle_number = pf_mfp_max_plimit;
    }
    
    const int64_t k_max_wifi_solution_gap = 5000; // ms
    bool is_wifi_gap = (std::abs(filter_timestamp - this->wifi_pos.timestamp) > k_max_wifi_solution_gap);
    bool is_ble_gap = (std::abs(filter_timestamp - this->ble_pos.timestamp) > k_max_wifi_solution_gap);
    if (is_wifi_gap && is_ble_gap)
    {
        max_particle_number = pf_mfp_max_plimit;
    }

    // control of current particle number
    int particle_number = std::max(pf_mix_min_plimit, pMixed->getParcticlesCount());
    if ( sigma_mix > 3 )
    {
        particle_number = (int)((sigma_mix / 3) * (sigma_mix / 3) * pf_mix_min_plimit);
    }
    else if ( sigma_mix < 2 )
    {
        particle_number = pf_mix_min_plimit;
    }

    particle_number = std::min(particle_number, max_particle_number);
    particle_number = std::max(particle_number, pf_mix_min_plimit);

    // resize PF
    bool needResize = (particle_number != pMixed->getParcticlesCount());
    //needResize = needResize && (pMixed->is_in_tracking() || (!pMixed->is_in_tracking() && !pMixed->initialized()));
    bool needIncrease = (particle_number > pMixed->getParcticlesCount());
    needResize = needResize && (pMixed->is_in_tracking() || (!pMixed->is_in_tracking() && needIncrease));
    if (needResize)
    {
        pMixed->resize(particle_number);
		if (needIncrease)
		{
			static_cast<PF::ParticleFilter*>(pMixed)->do_resample();
		}
    }

    // position covariance 
    if ( pMixed->initialized() && prediction_mix.is_motion == false && sigma_mix > 6 )
    {
        prediction_mix.pos_cov.block<2, 2>( 0, 0 ).setZero();
        prediction_mix.pos_cov( 0, 0 ) = 1E-10;
        prediction_mix.pos_cov( 1, 1 ) = 1E-10;
    }
}

void FusionFilter::controlInjection()
{
	// managing of pf-injection sources
    if (pMixed->initialized())
    {
        // disable all initialisers 
        init_wifi_mm->enable(false);
        init_ble_mm->enable(false);
        init_proximity_mm->enable(false);
        init_wifi->enable(false);
        init_proximity->enable(false);
        init_ble->enable(false);
        init_normal_mix->enable(false);
        init_mixed_area->enable(false);
        init_mixed_area_mm->enable(false);

        init_collaboration->enable(false);
        init_collaboration_mm->enable(false);

        //enable all injectors
        injector_ble->enable(true);
        injector_ble_mm->enable(true);
        injector_wifi->enable(true);
        injector_wifi_mm->enable(true);
        injector_proximity->enable(true);
        injector_proximity_mm->enable(true);
        injector_normal_mix->enable(false);
        injector_collaboration->enable(false);
        injector_collaboration_mm->enable(false);
        //init_mfp_area
    }
    else
    {
        // enable all initialisers
        init_wifi_mm->enable(true);
        init_ble_mm->enable(true);
        init_proximity_mm->enable(true);
        init_wifi->enable(true);
        init_proximity->enable(true);
        init_ble->enable(true);
        init_normal_mix->enable(true);
		init_mixed_area->enable(true);
		init_mixed_area_mm->enable(true);
        init_collaboration->enable(false);
        init_collaboration_mm->enable(false);
        //init_mfp_area

        // disable all injectors
        injector_ble->enable(false);
        injector_ble_mm->enable(false);
        injector_wifi->enable(false);
        injector_wifi_mm->enable(false);
        injector_proximity->enable(false);
        injector_proximity_mm->enable(false);
        injector_normal_mix->enable(false);
        injector_collaboration->enable(false);
        injector_collaboration_mm->enable(false);

		// turn off reacquisition, or it will happen immediately after initialization
		hard_reset_enabled = false;
		hard_reset_required = false;
		heading_is_stable = false;

        // turn off reacquisition, or it will happen immediately after initialization
        hard_reset_enabled = false;
        hard_reset_required = false;
        heading_is_stable = false;
    }
    // set injection pf-injection 
	double p_inj = 0;

	bool have_new_solution_for_injection = false;
	bool have_new_wifi_solution = false;
	bool have_new_ble_solution = false;
	bool have_new_proximity_solution = false;
    bool have_new_collaboration_solution = false;
	
	have_new_wifi_solution = ((this->wifi_pos.timestamp > 0) && (std::abs(filter_timestamp - this->wifi_pos.timestamp) < 15000));
	have_new_ble_solution = ((this->ble_pos.timestamp > 0) && (std::abs(filter_timestamp - this->ble_pos.timestamp) < 15000));
	have_new_proximity_solution = ((this->proximity_pos.timestamp > 0) && (std::abs(filter_timestamp - this->proximity_pos.timestamp) < 5000));
    have_new_collaboration_solution = ((collaboration_timestamp > 0) && (std::abs(filter_timestamp - collaboration_timestamp) < 5000));
	
	if ((wifi_injection_count < 3 * 5 * 2) && (have_new_wifi_solution)) // max count =3 scanc * 5 sec * 2 scan per sec
	{
		// p_inj = 0.1 gets triggered in dataset with no wi-fi unless we add the condition for a new wifi_timestamp 
		injector_wifi->enable(true);
		injector_wifi_mm->enable(true);
		p_inj = 0.1;
	}

	// pulling adaptation
	if ((pMixed->inPulling) && have_new_proximity_solution)
	{
		//init_proximity->enable(true);
		//init_proximity_mm->enable(true);

		injector_normal_mix->enable(true);
		p_inj = 0.1;
	}
	else
	{
		injector_normal_mix->enable(false);
	}

    if (0)//(have_new_collaboration_solution)
    {
        injector_collaboration->enable(true);
        injector_collaboration_mm->enable(true);
        p_inj = 0.1;
    }

	// additional injection when mix position sigma is above threshold
	// reacquisition from wi-fi or from proximity is used
	have_new_solution_for_injection = (have_new_wifi_solution || have_new_ble_solution || 
                                       have_new_proximity_solution || have_new_collaboration_solution);

	if (have_new_solution_for_injection)
	{
		if (reacquisition_injection_percentage > 0.0000001)
		{
			p_inj = reacquisition_injection_percentage;
			reacquisition_injection_percentage = 0.0;
		}
	}

	if ( hard_reset_enabled && hard_reset_required && have_new_solution_for_injection)
	{
		//restart_for_reacquisition();		
		//using alternative restart via injection
		p_inj = 0.45;
		pMixed->setInjection(p_inj > 0, p_inj);

		hard_reset_required = false;
		heading_is_stable = false;
	}

    //pMixed->setInjection(p_inj > 0, p_inj);
	pMixed->setInjection(true, p_inj); // set to true because there is another code (particle degradation), which checks this
}

void FusionFilter::controlBarometer()
{
	// checking if wifi/ble/prox position appeared recently
	// if no - freeze height updates, if yes - unfreeze height updates

	int64_t most_recent_updater_pos_time = 0;
	int64_t last_wifi_time = 0;
	int64_t last_ble_time = 0;
	int64_t last_prox_time = 0;

	if (wifi_update_mix.enabled())
		last_wifi_time = wifi_pos.timestamp;

	if (ble_update_mix.enabled())
		last_ble_time = ble_pos.timestamp;

	if (ble_proximity_update_mix.enabled())
		last_prox_time = proximity_pos.timestamp;

	most_recent_updater_pos_time = std::max(last_wifi_time, last_ble_time);
	most_recent_updater_pos_time = std::max(most_recent_updater_pos_time, last_prox_time);

	double dt = std::abs(filter_timestamp - most_recent_updater_pos_time) / 1000.0;

	if (predict_lks_mix != 0)
	{
		double max_dt = PF_MAX_ALLOWED_BAROMETER_TIME;
		if (dt > max_dt)
			predict_lks_mix->set_floor_change_blocked(true);
		else
			predict_lks_mix->set_floor_change_blocked(false);
	}
}

/**
* Control Mag PF
* param[in] enable disables/enables Mag filter
*/
void FusionFilter::setMFPFilterEnabled( bool enable )
{
    pMFP->setEnabled( enable );
}

/**
* Control Mixed PF
* param[in] enable disables/enables Mixed filter
*/
void FusionFilter::setMixedFilterEnabled( bool enable )
{
    pfMixedEnabled = enable;
}

/**
* return Mag filter state
*/
bool FusionFilter::getMagFilterEnabled()
{
    return pMFP->getEnabled();
}

/**
* return Mixed filter state
*/
bool FusionFilter::getMixedFilterEnabled()
{
    return pfMixedEnabled;
}

void FusionFilter::setLkhUpdateRatio(double upd_ratio)
{
    predict_lks_mfp->set_lkh_update_ratio(upd_ratio);
    predict_lks_mix->set_lkh_update_ratio(upd_ratio);
}

//-----------------------------------------------------------------------------
bool FusionFilter::setPredictionModelForMagFilter(VenueType venue_type, PlatformType platform_type)
{
    bool result(true);

    if (platform_type == PlatformType::pftp_FORKLIFT)
    {
        predict_lks_mfp = &predict_vdr_pf1;
        rbf_bias.setKfWhenNoMotion(false); // disable RBF/UKF when no motion
    }
    else //if (platform_type == PlatformType::pftp_PEDESTRIAN)
    {
        predict_lks_mfp = &predict_lks_multypoint_pf1;
        rbf_bias.setKfWhenNoMotion(true); // enable RBF/UKF when no motion
    }

    predict_lks_mfp->setIff(&i_pf_mfp);
    pMFP->setPredictionModel(predict_lks_mfp);
    pMFP->setVenueType(venue_type);

    return result;
}

//-----------------------------------------------------------------------------
bool FusionFilter::setPredictionModelForMixedFilter(VenueType venue_type, PlatformType platform_type)
{
    bool result(true);

    if (platform_type == PlatformType::pftp_FORKLIFT)
    {
        predict_lks_mix = &predict_vdr_pf2;
        rbf_bias.setKfWhenNoMotion(false); // disable RBF/UKF when no motion
    }
    else //if (platform_type == PlatformType::pftp_PEDESTRIAN)
    {
        predict_lks_mix = &predict_lks_multypoint_pf2;
        rbf_bias.setKfWhenNoMotion(true); // enable RBF/UKF when no motion
    }

    predict_lks_mix->setIff(&i_pf_mfp);
    pMixed->setPredictionModel(predict_lks_mix);
    pMixed->setVenueType(venue_type);

    return result;
}

void FusionFilter::setParticleCountForMixedFilter(int32_t min_particle_count, int32_t max_particle_count)
{
    pf_mix_min_plimit = min_particle_count;
    pf_mix_max_plimit = max_particle_count;
	controlInjection();
    controlMixedFilter();	
}

void FusionFilter::setParticleCountForMfpFilter(int32_t min_particle_count, int32_t max_particle_count)
{
    pf_mfp_min_plimit = min_particle_count;
    pf_mfp_max_plimit = max_particle_count;
    controlMagFilter();
}

void FusionFilter::setRandomSeeds()
{
    pMixed->setRandomSeeds();
    pMFP->setRandomSeeds();
    init_mfp_pf->getPF()->setRandomSeeds();
}

double FusionFilter::getMixedFilterSigma()
{
    return sigma_mix;
}

bool FusionFilter::isMixedFilterInitialized()
{
    return pMixed->initialized();
}

void FusionFilter::updateReacquisitionState(int64_t time, double std_heading, double sigma_mix, double level)
{
    const double max_stable_heading_threshold = 0.2; // radians
    const double min_unstable_heading_threshold = 0.3; // radians
    const int64_t min_time_between_hard_resets = 30000; // ms
	const int64_t min_time_between_soft_injection = 5000; // ms
    const double min_sigma_mix_threshold = 3.0; // meters
    const uint16_t max_level_queue_size = 60;
    bool level_is_stable = false;
    const int flt_window_size = 10;
	
    level_queue.push_back(level);
    if (level_queue.size() > max_level_queue_size)
    {
        level_queue.pop_front();
    }

    double min_level = 10000;
    double max_level = -10000;

    std::deque<double> filtered_level_queue;

    if (level_queue.size() == max_level_queue_size)
    {
        // checking that height didn't change significantly in the last 25 seconds
        // if height was going up or down significantly, then restart is disabled
        // spikes are ignored 
        for (int i = flt_window_size - 1; i < level_queue.size() - 1; ++i)
        {
            double t = 0;
            for (int j = i - (flt_window_size - 1); j < i + 1; ++j)
            {
                t += level_queue[j];
            }
            t /= flt_window_size;
            filtered_level_queue.push_back(t);
        }
        
        for (int i = 0; i < filtered_level_queue.size(); ++i)
        {
            if (filtered_level_queue[i] > max_level)
                max_level = filtered_level_queue[i];
            if (filtered_level_queue[i] < min_level)
                min_level = filtered_level_queue[i];
        }

        if (max_level - min_level < 0.5)
            level_is_stable = true;
    }
    
    if (!heading_is_stable)
    {
        if (std_heading < max_stable_heading_threshold)
        {
            heading_is_stable = true;
        }
    }

    // patch to disable reacquisition in stop: reset time of last reacquisition
    if ( !this->getPrediction().is_motion)
    {
        reacquisition_block_in_stops = 0.8 * reacquisition_block_in_stops + 0.2;
    }
    else
    {
        reacquisition_block_in_stops = 0.0;
    }
    
    if (reacquisition_block_in_stops > 0.6)
    {
        last_hard_reset_time = time;
    }

    if (level_is_stable && heading_is_stable)
    {
        if ((std_heading > min_unstable_heading_threshold) && (sigma_mix > min_sigma_mix_threshold) && 
            (time - last_hard_reset_time > min_time_between_hard_resets))
        {
            hard_reset_required = true;
            heading_is_stable = false;
            last_hard_reset_time = time;
        }

        // adding injection if only position is noisy, but noise of heading is low
        if ((sigma_mix > min_sigma_mix_threshold) && (time - last_soft_injection_time > min_time_between_soft_injection))
        {
            reacquisition_injection_percentage = 0.15;
			last_soft_injection_time = time;
        }
    }
#if 1
	// checking if floor is the same for proximity position and current filter solution
	if (proximity_pos.valid && (! isUseBarometer))
	{
		if ((time - proximity_pos.timestamp) < 5000)
		{
			// new proximity position exists and its floor number is different from floor number estimated in fusion filter
			// adding injection in this case
			if (std::abs(level - proximity_pos.z) >= 0.5)
			{
				reacquisition_injection_percentage = std::max(reacquisition_injection_percentage, reacquisition_injection_percentage_for_change_floor);
			    last_soft_injection_time = time;
			}
		}
	}
#endif
#if 1
    // checking if floor is the same for wifi position and current filter solution
    if (wifi_pos.valid && (! isUseBarometer))
    {
        if ((time - wifi_timestamp) < 5000)
        {
            // new proximity position exists and its floor number is different from floor number estimated in fusion filter
            // adding injection in this case
            if (std::abs(level - wifi_pos.z) >= 0.5)
            //if ( round(level) != round(wifi_pos.z) )
            {
                reacquisition_injection_percentage = std::max(reacquisition_injection_percentage, reacquisition_injection_percentage_for_change_floor);
			    last_soft_injection_time = time;
            }
        }
    }
#endif
#if 1
    // checking if floor is the same for wifi position and current filter solution
    if (ble_pos.valid && (! isUseBarometer))
    {
        if ((time - ble_pos.timestamp) < 5000)
        {
            // new proximity position exists and its floor number is different from floor number estimated in fusion filter
            // adding injection in this case
            if (std::abs(level - ble_pos.z) >= 0.5)
            //if ( round(level) != round(ble_pos.z) )
            {
                reacquisition_injection_percentage = std::max(reacquisition_injection_percentage, reacquisition_injection_percentage_for_change_floor);
			    last_soft_injection_time = time;
            }
        }
    }
#endif


    *pf_log << "updateReacquisitionState: " << time << " " << last_hard_reset_time << " "
        << level_is_stable << " " << heading_is_stable 
        << " " << std_heading << " " << sigma_mix << " " << hard_reset_required << std::endl;
}

bool FusionFilter::correctPositionWithMapMatching(PF::Particle &position, const PF::Particle &prev_position)
{
    bool result = false;
    if (prev_position.pos.x == 0 && prev_position.pos.y == 0)
        return result;
    if (mm == NULL)
        return result;
	if (!mm->get_validity())
		return result;

    //static int correct_count = 0;
    static const uint8_t min_transparency_level = 255;
    
    double x = position.pos.x;
    double y = position.pos.y;
    double x_prev = prev_position.pos.x;
    double y_prev = prev_position.pos.y;
    double level = position.pos.level;

	int8_t  availability = mm->CheckPositionStatus(x, y, level, min_transparency_level);
	if (availability == 0)
	{
		double R = sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev));
		std::vector<std::pair<double, double>> list;
		result = false;
		double minX, maxX, minY, maxY, maxR;
		mm->GetMapDim(minX, maxX, minY, maxY);
		maxR = std::max(maxX - minX, maxY - minY);
		while (!result && R < maxR)
		{
			result = mm->GetTraversableCells(x_prev, y_prev, level, R, min_transparency_level, list);
			R = result ? R : R + 0.25;
		}
		if (result) // there are available cells
		{
			double min_angle = 3.14;
			double x_closest = x;
			double y_closest = y;
			for (auto it = list.begin(); it != list.end(); ++it)
			{
				double x1 = it->first;
				double y1 = it->second;
				double t = (x - x_prev)*(x1 - x_prev) + (y - y_prev)*(y1 - y_prev);
				double r = sqrt((x1 - x_prev)*(x1 - x_prev) + (y1 - y_prev)*(y1 - y_prev));
				double angle = acos(t / (R*r));
				if (angle < min_angle)
				{
					min_angle = angle;
					x_closest = x1;
					y_closest = y1;
				}
			}
			position.pos.x = x_closest;
			position.pos.y = y_closest;
		}
	}
    
    return result;
}


void FusionFilter::PrintParams()
{
    double biasRSSI, biasRssiSigma;
    bool bias_enabled;
    
    *pf_log << filter_timestamp << " " << "FF";

    if (wifi)
    {
        bias_enabled = wifi->getBias(biasRSSI);
        *pf_log << " " << "<wifiBias>: " << bias_enabled << " " << biasRSSI;
    }

    if (ble)
    {
        bias_enabled = ble->getBias(biasRSSI);
        *pf_log << " " << "<bleBias>: " << bias_enabled << " " << biasRSSI;
    }

    if (ble_proximity)
    {
        bias_enabled = ble_proximity->getBias(biasRSSI, biasRssiSigma);
        *pf_log << " <proximityBias>: " << bias_enabled << " " << biasRSSI << " " << biasRssiSigma;
    }

    *pf_log << std::endl;
}