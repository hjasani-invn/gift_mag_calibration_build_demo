/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           WiFi positioning
* \file            wifi_loc.cpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/


#include "wifi_loc.hpp"
#include "WiFiBLEBiasEstimator.hpp"
#include <cmath>
#include "median.h"
#include <new>

#ifdef __ANDROID__
#   include <android/log.h>
#	ifdef LOGI
#		undef LOGI
#	endif
#   define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "Wi-Fi loc", __VA_ARGS__))
#else
#   define LOGI(...)
#endif

const double WiFi_Locator::k_default_maxKnnScatter = 25;

Position_status_estimator_WiFi::Position_status_estimator_WiFi()
{
	last_metric_time = 0;
}

Position_status_estimator_WiFi::~Position_status_estimator_WiFi()
{

}

void Position_status_estimator_WiFi::add_new_metric_value(metric m)
{
	if (m.timestamp > last_metric_time)
	{
		metric_fifo.push_back(m);
		
		last_metric_time = m.timestamp;

		if (metric_fifo.size() > N)
		{
			metric_fifo.pop_front();
		}
	}
}

bool Position_status_estimator_WiFi::estimate_position_status(bool &is_inside)
{
	bool result = false;

	if (metric_fifo.size() < N)
	{
		result = false;
		return result; // minimum number of metric results are needed
	}

	else
	{
		int dT = abs(metric_fifo[N - 1].timestamp - metric_fifo[0].timestamp);
		if (dT > max_delta_T)
		{
			result = false;
			return result;	// time gap between results is large, estimation can't be made
		}
	}

	// we should give result if at least K out of N measurements have the same result
	unsigned int num_inside = 0;
	unsigned int num_outsude = 0;

	for (unsigned int i = 0; i < N; i++)
	{
		if (metric_fifo[i].m1 < metric_threshold)
		{
			num_outsude++;
		}
		else
		{
			num_inside++;
		}
	}
	
	if ((num_inside < K) && (num_outsude < K))
	{
		result = false; // evaluation is impossible, didn't get same K results out of N past metrics
	}
	else if (num_inside >= K)
	{
 		result = true;
 		is_inside = true;
	}
	else if (num_outsude >= K)
	{
		result = true;
		is_inside = false;
	}
	
	return result;
}

WiFi_Locator::WiFi_Locator(const char* const pWiFiMap, const size_t wifiFileSizeInBytes, double minProb, int minCnt, double thresh, double margin)
{
    initialized = false;
    db = 0;
    db_hist = 0;
      pos_estimator = 0;

    db_hist = new (std::nothrow)WiFi_DB_HIST();
    pos_estimator = new (std::nothrow) Position_status_estimator_WiFi();
    biasWiFiBLEEstimator = new (std::nothrow) WiFiBLEBiasEstimator();

    if ((nullptr != db_hist) && (nullptr != pos_estimator) && (nullptr != biasWiFiBLEEstimator))
    {
        initialized = db_hist->readFormat3_body(pWiFiMap, wifiFileSizeInBytes);
    }

    call_id  = 0;
    isLogging = false;
    minApCnt = minCnt;
    minProbMetric = minProb;
    strongRssiThreshold = thresh;
    strongRssiMargin = margin;
    maxKnnScatter = this->k_default_maxKnnScatter;

    biasRSSI = 0;
    biasGain = 0.0;
    biasRssiThreshold = -200.0; // default value, -200 means it does not impact on bias estimation
    biasEstEnabled = false;
    biasCounter = 0;
}

WiFi_Location WiFi_Locator::GetLocation(const WiFi_Measurement  &meas, int K)
{
    std::vector<WiFi_Location> DummyList;
    return GetLocation(meas, K, DummyList);
}

WiFi_Location WiFi_Locator::GetLocation(const WiFi_Measurement  &meas, int K, std::vector<WiFi_Location> &LocationList)
{
    WiFi_Location result;
    call_id++;

  WiFi_Measurement  measurement = meas;
  if (biasEstEnabled)
  {
      for (WiFi_Measurement::iterator it = measurement.begin(); it != measurement.end() && biasEstEnabled; ++it)
      {
          (*it).rssi += (RSSI)round(biasRSSI); //original
      }
  }
  if (db != 0)
    {
        std::vector<WiFi_Locator::Tresults> results(db->size());
        std::vector<WiFi_Locator::Tresults>::iterator r_it = results.begin();


		for (WiFi_DB::const_iterator it = db->begin(); it != db->end(); ++it)
		{
			(*r_it).metric = wifi_compare_metric((*it).fingerprint, measurement);
			(*r_it).idx = it;
			++r_it;
		}

		std::sort(results.begin(), results.end(), WiFi_Locator::f_compare<Tresults>);
		int v_size = results.size();
        result = KNearest(results, std::min(K, v_size));
	}
	else if (db_hist != 0)
	{
		std::vector<WiFi_Locator::Tresults_hist> results_hist(db_hist->size());
		std::vector<WiFi_Locator::Tresults_hist>::iterator r_it_hist = results_hist.begin();

        for (WiFi_DB_HIST::const_iterator it = db_hist->begin(); it != db_hist->end(); ++it)
        {
            if ((*it).fingerprint.size() > 0)
            {
                (*r_it_hist).metric = wifi_compare_metric_hist((*it).fingerprint, measurement);
            }
            else
            {
                (*r_it_hist).metric = wifi_compare_metric_gm((*it).fingerprintGM, measurement);
            }

            (*r_it_hist).idx = it;
            ++r_it_hist;
        }
        double sum = 0;
        for (auto r_it = results_hist.begin(); (r_it != results_hist.end()); r_it++)
        {
            sum += r_it->metric;
        }
        for (auto r_it = results_hist.begin(); (r_it != results_hist.end()); r_it++)
        {
            r_it->metric /= sum;
        }

        std::sort(results_hist.begin(), results_hist.end(), WiFi_Locator::f_compare<Tresults_hist>);
        int v_size = results_hist.size();

        // save multipoint solution
        int cell_count = K;
        for (auto r_it = results_hist.begin(); (r_it != results_hist.end()) && (r_it != results_hist.begin() + cell_count); r_it++)
        //for (auto r_it = results_hist.begin(); (r_it != results_hist.end()); r_it++)
        {
            WiFi_Location location;
            location = r_it->idx->location;
            location.metric = r_it->metric;
            LocationList.push_back(location);
        }

        //K = 6;      SetMaxKnnScatter(25);
        result = KNearest(results_hist, std::min(K, v_size));

    }

    int meas_size = measurement.size();

    if (meas_size > 0)
    {
        metric m;
        m.timestamp = measurement[meas_size - 1].timestamp;
        m.m1 = result.metric;
        pos_estimator->add_new_metric_value(m);
    }

    return result;
}

tProb WiFi_Locator::EstimateLikehood ( const WiFi_Measurement  &measurement,  const WiFi_Location &pos )
{
    measurement;
    pos;
    return 0;
}

void WiFi_Locator::EstimatePresenceInMappedArea(bool &estimation_successful, bool &pos_in_mapped_area)
{
    estimation_successful = pos_estimator->estimate_position_status(pos_in_mapped_area);
}

bool  WiFi_Locator::getPositionData(std::map<BSSID, RSSI> &data, const WiFi_Location &pos, bool median, std::ostream *log)
{
    bool result = false;
    median;
    data.clear();

    if ( pos.valid )
    {
        if ( db != 0 )
        {
        }
        else
        {
            std::vector< NPoint<WiFi_DB_HIST::const_iterator> > points;

            for ( WiFi_DB_HIST::const_iterator it = db_hist->begin(); it != db_hist->end(); ++it )
            {
                NPoint<WiFi_DB_HIST::const_iterator> pt;
                pt.metric = pos.norm ( ( *it ).location );
                pt.it = it;
                points.push_back ( pt );
            }

            std::sort ( points.begin(), points.end(), NPoint<WiFi_DB_HIST::const_iterator>::compare );

            if ((points.size() > 0) ? points.front().metric < 3 : false)
            {
                const WiFi_DB_HIST::APList &ap_list = (*points.front().it).fingerprint;
                const WiFi_DB_HIST::APListGM &ap_listGM = (*points.front().it).fingerprintGM;
                if (ap_list.size() > 0)
                {
                    for (WiFi_DB_HIST::APList::const_iterator it = ap_list.begin(); it != ap_list.end(); ++it)
                    {
                        double mean_rssi(0), sum(0);
                        int rssi = 0;
                        const WiFi_DB_HIST::APHist &hist = (*it).second;

                        for (WiFi_DB_HIST::APHist::const_iterator h_it = hist.begin(); h_it != hist.end(); ++h_it)
                        {
                            mean_rssi += rssi * (*h_it);
                            sum += (*h_it);
                            --rssi;
                        }

                        mean_rssi /= sum;

                        if ( mean_rssi < -45 && mean_rssi > -85 )
                        {
                            data[(*it).first] = (RSSI)floor(mean_rssi + 0.5);
                        }
                    }
                }
                else
                {
                    for (WiFi_DB_HIST::APListGM::const_iterator it = ap_listGM.begin(); it != ap_listGM.end(); ++it)
                    {
                        double mean_rssi(0), sum(0);
                        int rssi = 0;
                        const WiFi_DB_HIST::GMixture &histGM = (*it).second;

                        //mean_rssi = histGM.mu1*histGM.w1 + histGM.mu2*histGM.w2;
                        mean_rssi = 0;
                        const double k_rssi_min = -85;

                        if (std::min(histGM.mu1, histGM.mu2) >= k_rssi_min)
                        {
                            mean_rssi = histGM.mu1*histGM.w1 + histGM.mu2*histGM.w2;
                        }
                        else if (histGM.mu1 >= histGM.mu2)
                        {
                            if (histGM.w1 > 0.5)
                                mean_rssi = histGM.mu1;
                        }
                        else // histGM.mu1 <  histGM.mu2
                        {
                            if (histGM.w2 > 0.5)
                                mean_rssi = histGM.mu2;
                        }

                        if (mean_rssi < -45 && mean_rssi > k_rssi_min)
                        {
                            data[(*it).first] = (RSSI)floor(mean_rssi + 0.5);
                        }
                    }
                }
                result = true;
            }
        }
    }
    return result;
}

size_t WiFi_Locator::GetDbApCount()
{
    return db_hist->getAPpCount();
}

void WiFi_Locator::GetMapFitness ( const WiFi_Measurement  &measurement,  const WiFi_Location &pos, int &hit, int &miss, int &ext )
{
    hit = miss = ext = 0;

    if ( db != 0 )
    {
        WiFi_DB::const_iterator pos_it = db->begin();
        Coordinates dist = ( *pos_it ).location.norm ( pos );

        for ( WiFi_DB::const_iterator it = db->begin(); it != db->end(); ++it )
        {
            Coordinates tmpDist = ( *it ).location.norm ( pos );

            if ( tmpDist < dist )
            {
                pos_it = it;
                dist = tmpDist;
            }
        }

        if ( dist < 10. )
        {
            int N = ( *pos_it ).fingerprint.size();

            for ( WiFi_Measurement::const_iterator ap_it = measurement.begin(); ap_it != measurement.end(); ++ap_it )
            {
                if ( ( *pos_it ).fingerprint.find ( ( *ap_it ).bssid ) != ( *pos_it ).fingerprint.end() )
                {
                    ++hit;

                }
                else
                {
                    ++ext;
                }
            }

            miss = N - hit;
        }

    }
    else if ( db_hist != 0 )
    {
        WiFi_DB_HIST::const_iterator pos_it = db_hist->begin();
        Coordinates dist = ( *pos_it ).location.norm ( pos );

        for ( WiFi_DB_HIST::const_iterator it = db_hist->begin(); it != db_hist->end(); ++it )
        {
            Coordinates tmpDist = ( *it ).location.norm ( pos );

            if ( tmpDist < dist )
            {
                pos_it = it;
                dist = tmpDist;
            }
        }

        if ( dist < 10. )
        {
            int N = ( *pos_it ).fingerprint.size();

            for ( WiFi_Measurement::const_iterator ap_it = measurement.begin(); ap_it != measurement.end(); ++ap_it )
            {
                if ( ( *pos_it ).fingerprint.find ( ( *ap_it ).bssid ) != ( *pos_it ).fingerprint.end() )
                {
                    ++hit;

                }
                else
                {
                    ++ext;
                }
            }

            miss = N - hit;
        }
    }
}

tProb WiFi_Locator::wifi_compare_metric ( const WiFi_DB::APList &fingerprint, const WiFi_Measurement &measurement ) const
{
    tProb result = 1;
    tProb result_log = 0;
    int N = 0;

    for ( WiFi_Measurement::const_iterator it = measurement.begin(); it != measurement.end(); ++it )
    {
        const BSSID bssid = ( *it ).bssid;
        const RSSI  rssi  = ( *it ).rssi;
        WiFi_DB::APList::const_iterator ap_it = fingerprint.find ( bssid );

        if ( ap_it != fingerprint.end() )
        {
            N++;
            WiFi_DB::APHist ap_hist =  WiFi_DB::APData2Hist ( ( *ap_it ).second );
            double med = Median ( ap_hist );
            std::vector<double> tmpV;

            for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
            {
                tmpV.push_back ( fabs ( ( *h_it ) - med ) );
            }

            double sig = Median ( tmpV ) / 0.6745;
            double u = 0;

            if ( sig <= 0 )
            {
                std::sort ( ap_hist.begin(), ap_hist.end() );
                WiFi_DB::APHist::iterator first = ap_hist.begin();
                WiFi_DB::APHist::iterator last  = ap_hist.begin() + ap_hist.size() - 1;
                sig = ( *last ) - ( *first );
            }

            if ( sig > 0 )
            {
                u = sig * pow ( ( 4. / ( 3 * tmpV.size() ) ) , ( 1. / 5 ) );
            }
            else
            {
                u = 1.0;
            }


            for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
            {
                ( *h_it ) = ( rssi - ( *h_it ) ) / u;
            }


            double sum = 0;

            for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
            {
                double w = 1. / ap_hist.size();
                sum += w * normpdf ( *h_it, 2. );
            }

            sum /= u;
            result *= sum;
            result_log += log ( sum );
        }
    }


    if ( N > minApCnt )
    {
        result = pow ( result, 1. / N );
        result_log = pow ( M_E, result_log / N );
    }
    else
    {
        result = 0;
        result_log = 0;
    }

    return result_log;
    //    return result;
}


bool WiFi_Locator::estimateBias(const WiFi_Measurement &measurement, const WiFi_Location &location, int &cnt, std::ostream *log)
{
    return biasWiFiBLEEstimator->estimateBias(this, measurement, location, cnt, biasCounter, biasRSSI, log);
}

void WiFi_Locator::setBias(double  bias, int64_t delta_t)
{
    biasRSSI = bias;
    biasEstEnabled = true;

    if (biasEstEnabled == false)
    {
        biasRSSI = 0.; //reset bias
    }
}

bool WiFi_Locator::getBias(double &bias)
{
    bias = biasRSSI;
    return biasEstEnabled;
}

void  WiFi_Locator::setBiasGain(double  gain)
{
    biasGain = gain;
}

void  WiFi_Locator::getBiasGain(double &gain)
{
    gain = biasGain;
}

void  WiFi_Locator::setBiasRssiThreshold(double  RssiThreshold)
{
    biasRssiThreshold = RssiThreshold;
}

double  WiFi_Locator::getBiasRssiThreshold(void)
{
    return biasRssiThreshold;
}

/*
tProb WiFi_Locator::wifi_compare_metric2 ( const WiFi_DB::APList &fingerprint, const WiFi_Measurement &measurement ) const
{
    tProb result = 1;

    for ( WiFi_DB::APList::const_iterator ap_it = fingerprint.begin(); ap_it != fingerprint.end(); ++ap_it )
    {
        const BSSID bssid = ( *ap_it ).first;
        RSSI rssi = -100;

        for ( WiFi_Measurement::const_iterator m_it = measurement.begin(); m_it != measurement.end(); ++m_it )
        {
            if ( ( *m_it ).bssid == bssid )
            {
                rssi = ( *m_it ).rssi;
                break;
            }
        }

        if ( ap_it != fingerprint.end() )
        {
            WiFi_DB::APHist ap_hist =  WiFi_DB::APData2Hist ( ( *ap_it ).second );
            double med = Median ( ap_hist );
            std::vector<double> tmpV;

            for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
            {
                tmpV.push_back ( fabs ( ( *h_it ) - med ) );
            }

            double sig = Median ( tmpV ) / 0.6745;
            double u = 0;

            if ( sig <= 0 )
            {
                std::sort ( ap_hist.begin(), ap_hist.end() );
                WiFi_DB::APHist::iterator first = ap_hist.begin();
                WiFi_DB::APHist::iterator last  = ap_hist.begin() + ap_hist.size() - 1;
                sig = ( *last ) - ( *first );
            }

            if ( sig > 0 )
            {
                u = sig * pow ( ( 4. / ( 3 * tmpV.size() ) ) , ( 1. / 5 ) );
            }
            else
            {
                u = 1.0;
            }


            for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
            {
                ( *h_it ) = ( rssi - ( *h_it ) ) / u;
            }


            double sum = 0;

            for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
            {
                double w = 1. / ap_hist.size();
                sum += w * normpdf ( *h_it, 2. );
            }

            sum /= u;
            result *= sum;
        }
    }

    return result;
}
*/


//WiFi_Location WiFi_Locator::KNearest( const std::vector<Tresults> &results, int K )
//{
//    WiFi_Location location;
//    std::vector<Tresults> resultsKN;
//    int N = 0;
//
//    //    Clusters KK = clusterize(results);
//
//    for( std::vector<Tresults>::const_iterator it = results.begin(); it != results.end() && it < results.begin() + K; ++it )
//    {
//        // it - location index in WiFi db
//        const WiFi_Location kL = ( *( *it ).idx ).location;
//        const tProb w = ( *it ).metric;
//
//        location += kL *  w;
//        if( isLogging )
//        {
//            wifi_log << ++N << ": metric: " << ( *it ).metric << " x " << ( *( *it ).idx ).location.x << " y " << ( *( *it ).idx ).location.y << std::endl;
//        }
//        resultsKN.push_back( *it );
//
//    }
//    Clusters KK = clusterize( resultsKN );
//    Clusters::const_iterator it_K = KK.begin();
//    tProb cl_pow = ClusterPow( *KK.begin() );
//
//    for( Clusters::iterator it = KK.begin(); it != KK.end(); ++it )
//    {
//        if( ClusterPow( *KK.begin() ) > cl_pow )
//        {
//            cl_pow = ClusterPow( *KK.begin() );
//            it_K = it;
//        }
//    }
//
//    location = WiFi_Location();
//    for( ClustValue::const_iterator it = ( *it_K ).begin(); it != ( *it_K ).end(); ++it )
//    {
//        const WiFi_Location kL = ( *( *( *it ) ).idx ).location;
//        const tProb w = ( *( *it ) ).metric;
//
//        location += kL *  w;
//    }
//
//
//    tProb P_norm = 0;
//    for( std::vector<Tresults>::const_iterator it = results.begin(); it != results.end(); ++it )
//    {
//        P_norm += ( *it ).metric;
//    }
//
//    if( isLogging )
//    {
//        wifi_log << "Call id:" << call_id << " loc.p:" << location.p <<  " P_norm:" << P_norm << std::endl;
//    }
//    if( location.p > 0 && P_norm > 0 )
//    {
//        tProb p = location.p;
//        location /=  p ;
//        location.p =  p / P_norm;
//        location.valid = true;
//    }
//    if( location.p == 0 || P_norm <= 0.3 )
//    {
//        location.valid = false;
//    }
//    if( isLogging )
//    {
//        wifi_log << "Call id:" << call_id << " loc.p:" << location.p <<  " P_norm:" << P_norm << " valid: " << location.valid << std::endl;
//        wifi_log << "Call id:" << call_id << " x " << location.x << " y " << location.y  << std::endl;
//    }
//
//    return location;
//}
//





double WiFi_Locator::Median ( WiFi_DB::APHist hist ) const
{

    double res = median ( hist.begin(), hist.end() );
    return res;
}


WiFi_Locator::~WiFi_Locator()
{
    wifi_log.close();
    delete db;
    delete db_hist;
	delete pos_estimator;
}






WiFi_Locator::ClustValue WiFi_Locator::ClustInit ( const std::vector<Tresults> &results )
{
    ClustValue res;

    for ( std::vector<Tresults>::const_iterator it = results.begin(); it != results.end(); ++it )
    {
        res.push_back ( it );
    }

    return res;
}

WiFi_Locator::ClustValue WiFi_Locator::Cluster ( const ClustValue &U, WiFi_Location x0, Coordinates r )
{
    ClustValue res;

    for ( ClustValue::const_iterator it = U.begin(); it != U.end(); ++it )
    {
        if ( x0.norm ( ( * ( * ( *it ) ).idx ).location ) < r )
        {
            res.push_back ( *it );
        }
    }

    return res;
}

WiFi_Location WiFi_Locator::ClusterCenter ( const ClustValue &K )
{
    WiFi_Location res;
    int N = 0;

    for ( ClustValue::const_iterator it = K.begin(); it != K.end(); ++it )
    {
        res += ( * ( * ( *it ) ).idx ).location;
        N++;
    }

    res /= N;
    return res;
}


void WiFi_Locator::ClustRemove ( ClustValue &U, const ClustValue &K0 )
{
    for ( ClustValue::const_iterator k_it = K0.begin(); k_it != K0.end(); ++k_it )
    {
        ClustValue::iterator u_it;

        for ( u_it = U.begin(); u_it != U.end(); ++u_it )
        {
            if ( ( *u_it ) == ( *k_it ) )
            {
                break;
            }
        }

        if ( u_it != U.end() )
        {
            U.remove ( *u_it );
        }
    }
}


tProb WiFi_Locator::ClusterPow ( const ClustValue &v )
{
    tProb p = 0;

    for ( ClustValue::const_iterator it = v.begin(); it != v.end(); ++it )
    {
        p += ( * ( *it ) ).metric;
    }

    return p;
}

tProb WiFi_Locator::hist2prob ( const WiFi_DB::APData &fp, int rssi, tProb kernel_sig )
{
    tProb p_rssi = 0;
    WiFi_DB::APHist ap_hist =  WiFi_DB::APData2Hist ( fp );
    double med = median ( ap_hist.begin(), ap_hist.end() );
    std::vector<double> tmpV;

    for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
    {
        tmpV.push_back ( fabs ( ( *h_it ) - med ) );
    }

    double sig = median ( tmpV.begin(), tmpV.end() ) / 0.6745;
    double u = 0;

    if ( sig <= 0 )
    {
        std::sort ( ap_hist.begin(), ap_hist.end() );
        WiFi_DB::APHist::iterator first = ap_hist.begin();
        WiFi_DB::APHist::iterator last  = ap_hist.begin() + ap_hist.size() - 1;
        sig = ( *last ) - ( *first );
    }

    if ( sig > 0 )
    {
        u = sig * pow ( ( 4. / ( 3 * tmpV.size() ) ) , ( 1. / 5 ) );
    }
    else
    {
        u = 1.0;
    }


    for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
    {
        ( *h_it ) = ( rssi - ( *h_it ) ) / u;
    }


    double sum = 0;

    for ( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
    {
        double w = 1. / ap_hist.size();
        sum += w * normpdf ( *h_it, kernel_sig );
    }

    sum /= u;
    p_rssi = sum;
    return p_rssi;

}

tProb WiFi_Locator::wifi_compare_metric_hist ( const WiFi_DB_HIST::APList &fingerprint, const WiFi_Measurement &measurement ) const
{
    tProb result = 1;
    tProb result_log = 0;
    int N = 0;

    for ( WiFi_Measurement::const_iterator it = measurement.begin(); it != measurement.end(); ++it )
    {
        const BSSID bssid = ( *it ).bssid;
        const int  i_rssi  = - ( int ) ( *it ).rssi;
        WiFi_DB_HIST::APList::const_iterator ap_it = fingerprint.find ( bssid );

        if ( ap_it != fingerprint.end() && ( i_rssi >= 0 ) && ( i_rssi <= 100 ) )
        {
            ++N;
            tProb p = ( *ap_it ).second[i_rssi];
            result *= p;
            result_log += log ( p );
        }
    }


    if ( N > minApCnt )
    {
        result = pow ( result, 1. / N );
        result_log = pow ( M_E, result_log / N );
    }
    else
    {
        result = 0;
        result_log = 0;
    }

    return result_log;
    //    return result;
}

// ARG IN: const WiFi_DB_HIST::APListGM &fingerprint - one cell
tProb WiFi_Locator::wifi_compare_metric_gm ( const WiFi_DB_HIST::APListGM &fingerprint, const WiFi_Measurement &original_measurement ) 
{
    tProb result = 1;
    tProb result_log = 0;
    int N = 0;
    int K = 1;
    
    WiFi_Measurement measurement = original_measurement;
#if ENTIRE_HIPOTESIS_MODEL_FOR_CELL_LKH_CALCULATION
    // adding absent AP 
    for (auto  it = fingerprint.begin(); it != fingerprint.end(); ++it)
    {
        const BSSID bssid = it->first;
        auto meas_ap = std::find_if(original_measurement.begin(), original_measurement.end(), [bssid](const WiFi_ApInfo& x)
        {
            return bssid == x.bssid;
        }
        );
        
        if (meas_ap == original_measurement.end())
        {//add virtual ap meassurement if no data
            WiFi_ApInfo new_item = {};
            new_item.bssid = bssid;
            new_item.rssi = -100;
            measurement.push_back(new_item);
        }
    }
#endif

    // loop for measurements (one meas is AP and its RSSI)
    for ( WiFi_Measurement::const_iterator it = measurement.begin(); it != measurement.end(); ++it )
    {
        const BSSID bssid = ( *it ).bssid;
        const RSSI  rssi  =  ( *it ).rssi;
        WiFi_DB_HIST::APListGM::const_iterator ap_it = fingerprint.find ( bssid );

        if ( ap_it != fingerprint.end() && ( rssi <= 0 ) && ( rssi >= -100 ) )
        {
            const WiFi_DB_HIST::GMixture &gm = ( *ap_it ).second;

            // if ( !( (gm.mu1 >-100 &&  gm.w1>0.2)||(gm.mu2 >-100 &&  gm.w2>0.2))) continue;
            ++N; // number of matching APs between meas and FP

            //tProb p = gm.w1*normpdf_fast(gm.mu1-rssi, gm.sig1) + gm.w2*normpdf_fast(gm.mu2-rssi, gm.sig2);

            // probability that this RSSI from this AP can be obtained in current cell
            tProb p = gm.w1 * normpdf( gm.mu1 - rssi, gm.sig1 ) + gm.w2 * normpdf( gm.mu2 - rssi, gm.sig2 );

            //if (p > normpdf(3*5., gm.sig1))
            {
                result *= p;
                result_log += log ( p );
            }

            //if (p > normpdf_fast(5*2., gm.sig1)) ++K;
            if ( gm.w1 > gm.w2 )
            {
                if ( ( gm.mu1 > strongRssiThreshold ) && ( p > normpdf( 5., gm.sig1 ) ) ) K += ( int )strongRssiMargin;
            }
            else
            {
                if ( ( gm.mu2 > strongRssiThreshold ) && ( p > normpdf( 5., gm.sig2 ) ) ) K += ( int )strongRssiMargin;
            }

        }
    }

    //for (WiFi_DB_HIST::APListGM::const_iterator it_f = fingerprint.begin(); it_f != fingerprint.end(); ++it_f )
    //{
    //    const BSSID bssid = ( *it_f ).first;
    //    RSSI  rssi  =  -100;

    //    for ( WiFi_Measurement::const_iterator ap_it =  measurement.begin(); ap_it !=  measurement.end(); ++ap_it)
    //    {
    //        if (bssid == (*ap_it).bssid)
    //        {
    //            rssi = (*ap_it).rssi;
    //            break;
    //        }
    //    }

    //    tProb p_vis = 0.99;
    //    const WiFi_DB_HIST::GMixture &gm = ( *it_f ).second;
    //    if (gm.mu1 < -80) p_vis = 0.95;
    //    if (gm.mu1 < -85) p_vis = 0.9;
    //    if (gm.mu1 < -90) p_vis = 0.7;
    //    if (rssi != -100)
    //    {
    //        ++N;
    //        tProb p = gm.w1*normpdf_fast(gm.mu1-rssi, gm.sig1) + gm.w2*normpdf_fast(gm.mu2-rssi, gm.sig2);
    //        result *= p*p_vis;
    //        result_log += log ( p * p_vis );
    //    }
    //    else
    //    {
    //        tProb p = (1.-p_vis);
    //        result *= p;
    //        result_log += log ( p );
    //    }
    //}


    // result now is p1*p2*...*pN

    // checking that num of found AP is > than minimum, else probability = 0
    if ( N > minApCnt )
    {
#if 0  // 1 - n-degry root, 0 no root
        result = pow ( result, 1. / N );
        result_log = pow ( M_E, result_log / N );
#endif
        //result_log = pow ( M_E, result_log );

        //result_log += sqrt((tProb)N/measurement.size())/100;
        result_log *= K;//sqrt((double)K);
    }
    else
    {
        result = 0;
        result_log = 0;
    }

    //return result_log;
    return result;
    // result_log - the higher it is, the higher the probability of being in the current cell (input parameter)
}






//template <class T> WiFi_Location KNearest( std::vector<T> &results, int K )
//{
//    WiFi_Location location;
//    std::vector<T> resultsKN;
//    int N = 0;

//    for( std::vector<T>::const_iterator it = results.begin(); it != results.end() && it < results.begin() + K; ++it )
//    {
//        // it - location index in WiFi db
//        const WiFi_Location kL = ( *( *it ).idx ).location;
//        const tProb w = ( *it ).metric;

//        location += kL *  w;
//        resultsKN.push_back( *it );
//    }


//    tProb P_norm = 0;
//    for( std::vector<T>::const_iterator it = results.begin(); it != results.end(); ++it )
//    {
//        P_norm += ( *it ).metric;
//    }

//    if( location.p > 0 && P_norm > 0 )
//    {
//        tProb p = location.p;
//        location /=  p ;
//        location.p =  p / P_norm;
//        location.valid = true;
//    }
//    if( location.p == 0 || P_norm <= 0.3 )
//    {
//        location.valid = false;
//    }
//    return location;
//}
