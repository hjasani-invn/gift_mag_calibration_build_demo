/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Fingerprint storage class. Stores raw rssi measurements.
* Esimates additional metrics for AP's selection
* \file            wifi_db.hpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#ifndef BLE_DB_H
#define BLE_DB_H

#include <string>
#include <map>
#include "ble_data.hpp"
#include "Fpbl.hpp"



#include <iostream>


/**
* \ingroup         WiFi
*/
class Ble_DB
{
    public:
        typedef std::map<RSSI, int> APData;     /**< counts measurements with a specific level*/
        typedef std::vector<double>   APHist;   /**< rssi histogram for single AP*/
        typedef std::map<BSSID, APData> APList; /**< list of AP data and its rssi histograms */

        typedef std::map<BSSID, int> APMeas;    /**< single measurement vector, pairs <bssid, rssi> */
        typedef std::vector<APMeas> MeasVector; /**< contains list of measurements for a specific location */

        /**< describes single fingerprint record */
        typedef struct
        {
            APList        fingerprint;  /**< organized fingerprint data */
            Ble_Location location;     /**< coordinates */
            MeasVector    measVector;   /**< raw mesuremets data */
        } DBRecord;

        typedef std::vector<DBRecord> LocalDB; /**< fingerprint storage class */
        typedef LocalDB::const_iterator const_iterator; /**< fingerprint iterator */

		Ble_DB(const Fpbl::BleGrid &bleGrid);

        /**
        * creates object and load data from specific file
        * \param[in] db_name fingerprint filename
        */
        Ble_DB( std::string db_name );
        /**< default constructor, creates empty fingerprint */
        Ble_DB()
        {
            ;
        }

        const_iterator begin() const; /**< fingerprint iterator begin() */
        const_iterator end() const;   /**< fingerprint iterator end() */

        /**
        * \return fingerprint size
        */
        size_t size() const;
        /**
        * converts measurements to histogram
        * \param[in] Ap rssi observations
        * \return histogram
        */
        static Ble_DB::APHist APData2Hist( const Ble_DB::APData &Ap );

		static tProb hist2prob(const Ble_DB::APData &fp, int rssi, tProb kernel_sig);

        /**
        * parse raw measurements logs
        * \param[in] fname input log filename
        * \param[in] x_min,x_max,y_min,y_max describes rectangle area which will be loaded [m]
        * \param[in] z_ default z coordinate value [m]
        */
        void readFormatLogs( const std::string &fname, double x_min, double x_max, double y_min, double y_max, double z_ = 0. );
        /**
        * \return fingerprint entropy
        */
        double entropy_map();
        /**
        * \param[in] bssid AP's unique id
        * \return  full AP's entropy
        */
        double entropy_bssid( BSSID bssid );
        /**
        * \param[in] bssid AP's unique id
        * \param[in] rssi signal level condition
        * \return  conditional AP's entropy
        */
        double entropy_rssi( BSSID bssid, RSSI rssi );
        std::vector<BSSID> getApList(); /**< \return list of AP's identificators */


    private:
        LocalDB db;
        void readFormat1( const std::string &fname );
        void normalizeRecord( DBRecord &rec );
        void normalizeDB();

        std::vector<std::string> tokenize( const std::string &s, const char &delim ) const;
};
#endif
