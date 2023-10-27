/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           WiFi positioning
* \file            wifi_loc.hpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#ifndef WIFI_LOC_H
#define WIFI_LOC_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <list>
#include <deque>
#include <stdint.h>

#include "wifi_if.hpp"
#include "wifi_db.hpp"
#include "wifi_db_hist.hpp"
#include "WiFiBLEBiasEstimator.hpp"


#define ENTIRE_HIPOTESIS_MODEL_FOR_CELL_LKH_CALCULATION 0

/**
* \ingroup         WiFi
*/
struct metric
{
	int64_t timestamp;
	double m1;
};

class Position_status_estimator_WiFi
{
public:
	Position_status_estimator_WiFi();
	~Position_status_estimator_WiFi();

	void add_new_metric_value(metric m);
	bool estimate_position_status(bool &is_inside);

private:
	std::deque<metric> metric_fifo;
	const unsigned int K = 3; // K metric results have to be the same to provide output result
	const unsigned int N = 3;
	int64_t last_metric_time;
	const double metric_threshold = 0.05; // TODO: set correct metric threshold! 
	const unsigned int max_delta_T = 30000; // maximum time between first and last element in the queue
};

class WiFi_Locator : public WiFi
{
    public:
        /**
        * Fingerprinting object constructor, implements WiFi interface
        * \param[in] db_name fingerprint data file
        * \param[in] minProb validation threshold
        * \param[in] minCnt AP count threshold
        * \param[in] thresh RSSI low level for empiric weighting
        * \param[in] margin empiric weighting for strong RSSI levels
        */
        WiFi_Locator(const char* const pWiFiMap, const size_t wifiFileSizeInBytes, double minProb, int minCnt, double thresh, double margin);

        WiFi_Location GetLocation(const WiFi_Measurement  &measurement, int K);
        WiFi_Location GetLocation(const WiFi_Measurement  &measurement, int K, std::vector<WiFi_Location> &LocationList);
        tProb EstimateLikehood( const WiFi_Measurement  &measurement,  const WiFi_Location &pos );
        void EstimatePresenceInMappedArea(bool &estimation_successful, bool &pos_in_mapped_area);
        bool  getPositionData(std::map<BSSID, RSSI> &data, const WiFi_Location &pos, bool median, std::ostream *log);
        void  GetMapFitness( const WiFi_Measurement  &measurement,  const WiFi_Location &pos, int &hit, int &miss, int &ext );
        size_t GetDbApCount();
        bool  estimateBias(const WiFi_Measurement &measurement, const WiFi_Location &location, int &cnt, std::ostream *log);
        void  setBias(double  bias, int64_t delta_t);
        bool  getBias(double  &bias);
        void  setBiasGain(double  gain);
        void  getBiasGain(double  &gain);
        void  setBiasRssiThreshold(double  RssiThreshold);
        double  getBiasRssiThreshold(void);
        
        bool  status()
        {
            return initialized;
        };

        /** destructior */
        ~WiFi_Locator();

        /**
        * estimates rssi probability based on Parsen-window method, using gaussian kernel
        * \param[in] fp rssi histogram
        * \param[in] rssi evaluation point
        * \param[in] kernel_sig gaussian kernel parameter
        * \return probability of the rssi level
        */
        static tProb hist2prob( const WiFi_DB::APData &fp, int rssi, tProb kernel_sig );

        void SetMaxKnnScatter(double value) { maxKnnScatter = value; };

    private:
        WiFi_DB *db;
        WiFi_DB_HIST *db_hist;
        Position_status_estimator_WiFi *pos_estimator;
        bool initialized;
        typedef struct
        {
            tProb metric;
            WiFi_DB::const_iterator idx;
        } Tresults;
        typedef struct
        {
            tProb metric;
            WiFi_DB_HIST::const_iterator idx;
        } Tresults_hist;


        int minApCnt;
        double minProbMetric;
        double strongRssiThreshold;
        double strongRssiMargin;
        double maxKnnScatter;

        double biasRSSI;
        double biasGain;
        double biasRssiThreshold;
        bool   biasEstEnabled;
        unsigned long   biasCounter;

        static const double k_default_maxKnnScatter;

        template <class T> static bool f_compare( const T &a, const T &b )
        {
            return ( a.metric > b.metric );
        }

        tProb wifi_compare_metric( const WiFi_DB::APList &fingerprint, const WiFi_Measurement &measurement ) const;
        tProb wifi_compare_metric_hist( const WiFi_DB_HIST::APList &fingerprint, const WiFi_Measurement &measurement ) const;
        tProb wifi_compare_metric_gm( const WiFi_DB_HIST::APListGM &fingerprint, const WiFi_Measurement &measurement );
        
        uint32_t getApCount();

        //WiFi_Location KNearest( const std::vector<Tresults> &results, int K );
        template <class T> WiFi_Location KNearest( T &results, int K )
        {
            WiFi_Location location;
            T resultsKN;
            std::map<int, int> mfloors; // floor_number : number_of_solutions

            // results were already calculated for all cells
            for( typename T::const_iterator it = results.begin(); it != results.end() && it < results.begin() + K; ++it )
            {
                // it - location index in WiFi db
                const WiFi_Location kL = ( *( *it ).idx ).location;
                const tProb w = ( *it ).metric;

                location += kL *  w;
                resultsKN.push_back( *it );


                std::map<int , int>::iterator f_it = mfloors.find( ( int )floor( kL.z + 0.5 ) );

                // if didn't find before a solution on this floor -> add this floor number to mfloors
                if( f_it == mfloors.end() )
                {
                    mfloors.insert( std::pair<int, int>( ( int )floor( kL.z + 0.5 ), 1 ) );
                }
                else
                {
                    ( *f_it ).second += 1; // increase counter by 1, if we already had a solution on this floor
                }
            }

            tProb P_norm = 0;

            for (typename T::const_iterator it = results.begin(); it != results.end() && it < results.begin() + K; ++it)
            {
                P_norm += ( *it ).metric;
            }

            //tProb p_meas = location.p;
            if( location.p > 0 && P_norm > 0 )
            {
                tProb p = location.p;
                location /=  p ;
                location.p =  p / P_norm;
                location.valid = true;


                double rms = 0;

                for ( typename T::const_iterator it = resultsKN.begin(); it != resultsKN.end(); ++it )
                {
                    double err = location.norm( ( *it ).idx->location );
                    rms += pow( err, 2 );
                }

                rms = sqrt( rms / resultsKN.size() );
                location.rms_xy = rms; 
                location.metric = P_norm; // added metric for venue detection
            }

            //AHTUNG TODO correct floor detection
            std::pair<int, int> f( 0, 0 );

            for( std::map<int , int>::iterator f_it = mfloors.begin(); f_it != mfloors.end(); ++f_it )
            {
                if( ( *f_it ).second > f.second )
                {
                    f = ( *f_it );
                }
            }

            location.z = ( Coordinates )f.first;
            // found the floor number with maximum number of measurements, set current floor number to that floor number

            //location.z = (*(*results.begin()).idx).location.z;
            if( location.p == 0 || P_norm <= minProbMetric )
            {
                location.valid = false;
            }

            // solution rejection with scatter threshold
            if ((location.p > 0) && (P_norm > 0) && location.valid && results.size() >= K)
            {
                typename T::const_iterator it0 = results.begin();
                const WiFi_Location kL = (*(*it0).idx).location;
                double x_min = kL.x;
                double x_max = kL.x;
                double y_min = kL.y;
                double y_max = kL.y;
                
                for (typename T::const_iterator it = results.begin(); it != results.end() && it < results.begin() + K; ++it)
                {
                    const WiFi_Location kL = (*(*it).idx).location;
                    x_max = std::max(x_max, kL.x);
                    x_min = std::min(x_min, kL.x);
                    y_max = std::max(y_max, kL.y);
                    y_min = std::min(y_min, kL.y);
                }
               
                if ((std::abs(x_max - x_min) > maxKnnScatter) || (std::abs(y_max - y_min) > maxKnnScatter))
                {
                    location.valid = false;
                }
            }

            return location;  // location goes to PF
        }

        double Median( WiFi_DB::APHist hist ) const;

        std::fstream wifi_log;
        int call_id;
        bool isLogging;

        template<class T> static T normpdf( T x, T sig )
        {
            return 1 / ( sqrt( 2 * M_PI * sig * sig ) ) * exp( -( x * x ) / ( 2 * sig * sig ) );
        }

        template<class T> static T normpdf_fast( T x, T sig )
        {
            union
            {
                double d;
                int64_t x;
            } u = { x };

            double val = -( x * x ) / ( 2 * sig * sig );
#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4244 )
#endif
            u.x = ( 1512775 * val + 1072632447 );
#ifdef _MSC_VER
#pragma warning( pop )
#endif
            u.x <<= 32;
            val = u.d;

            return 1 / ( sqrt( 2 * M_PI * sig * sig ) ) * val;
        }


        typedef std::list<std::vector<Tresults>::const_iterator> ClustValue;
        typedef std::vector<ClustValue> Clusters;

        Clusters clusterize( const std::vector<Tresults> &results );
        ClustValue ClustInit( const std::vector<Tresults> &results );
        ClustValue Cluster( const ClustValue &U, WiFi_Location x0, Coordinates r );
        WiFi_Location ClusterCenter( const ClustValue &K );
        void ClustRemove( ClustValue &U, const ClustValue &K0 );
        tProb ClusterPow( const ClustValue &v );

        template<class T_it> struct NPoint
        {
            NPoint<T_it>(): metric( 0 ), it() {};
            Coordinates metric;
            T_it it;
            static bool compare( const NPoint &a, const NPoint &b )
            {
                return ( a.metric < b.metric );
            }
        };

        IBiasEstimator *biasWiFiBLEEstimator;

};
#endif
