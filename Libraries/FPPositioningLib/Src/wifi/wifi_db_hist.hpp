/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Fingerprint storage class. Implements histogram and GM models
* \file            wifi_db_hist.hpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#ifndef WIFI_DB_HIST_H
#define WIFI_DB_HIST_H

#include <string>
#include <map>
#include "wifi_data.hpp"
#include "wifi_db.hpp"

#include <iostream>


class WiFi_DB_HIST
{
    public:
        /**
        * Two gaussian mixture model (GM)
        */
        typedef struct
        {
            double mu1;  /**< gaussian #1, mean */
            double sig1; /**< gaussian #1, sigma */
            double w1;   /**< gaussian #1, weight */
            double mu2;  /**< gaussian #2, mean */
            double sig2; /**< gaussian #2, sigma */
            double w2;   /**< gaussian #2, weight */
        } GMixture;

        typedef std::vector<double>   APHist; /**< single AP histogram model */
        typedef std::map<BSSID, APHist> APList; /**< location histogram model */
        typedef std::map<BSSID, GMixture> APListGM; /**< location GM model */

        /** fingerprint record */
        typedef struct
        {
            APList     fingerprint;    //< AP histograms
            APListGM   fingerprintGM;  //< AP GMs
            WiFi_Location location;    //< coordinates (x,y,z)
        } DBRecord;

        /** fingerprint object */
        typedef std::vector<DBRecord> LocalDB;
        /** figerprint record iterator */
        typedef LocalDB::const_iterator const_iterator;

        /** default constructor */
        WiFi_DB_HIST() {};

        /**
        * creates object and loads data from file
        * \param[in] db_name fingerprint db filename
        */
        WiFi_DB_HIST( std::string db_name );
        /**
        * creates object and loads data from old-format db
        * \param[in] tmp_db fingerprint object
        */
        WiFi_DB_HIST( const WiFi_DB *db );

        const_iterator begin() const; /**< iterator begin */
        const_iterator end() const;   /**< iterator end */

        /**
        * \return fingerprint size
        */
        size_t size() const;

        /**
        * load DB format v.2 (histogram)
        * \return success status
        */
        bool readFormat2( const char* const pWiFiMap, const size_t wifiFileSizeInBytes );
        /**
        * load DB format v.3 (GM)
        * \return success status
        */
        bool readFormat3( const char* const pWiFiMap, const size_t wifiFileSizeInBytes );
        /**
        * load DB body of format v.3 (GM)
        * \return success status
        */
        bool readFormat3_body(const char* const pWiFiMap, const size_t wifiFileSizeInBytes);
        
        /**
        * get AP count presented in DB
        * \return AP count
        */
        size_t getAPpCount();

    private:
        LocalDB db;
        void readFormatDB( const WiFi_DB *tmp_db );
        void onNewAP( const char * str );
        bool onNewAP3( const char * str );
        void onNewMeas( const  char * str );
        void onNewPos( const char * str );
        APHist ap_hist;
        BSSID bssid;
        APList ap_list;
        APListGM ap_list_gm;
        DBRecord db_rec;
};
#endif
