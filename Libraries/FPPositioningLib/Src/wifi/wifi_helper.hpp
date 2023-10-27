/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Reading WiFi logs, buffering and filtering wifi data
* \file            wifi_helper.hpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#ifndef _WIFI_HELPER_H
#define _WIFI_HELPER_H

#include <fstream>
#include <queue>
#include <stdint.h>
#include "wifi_data.hpp"

/**
* \ingroup         WiFi
*/
class WiFi_Helper
{
    public:
        WiFi_Helper() {};   /**< default constructor */
        explicit WiFi_Helper(const std::string &wifi_log); /**< Creates object and select data file */
        ~WiFi_Helper() {}; /**< destructor */

        /**
        * puts measurement vector into the fifo
        * \param[in] timestamp milliseconds
        * \param[in] n measurement number
        * \param[in] ap single AP data
        */
        void pushMeas( int64_t timestamp, int n, const WiFi_Measurement &meas );
        /**
        * gets measurement vector from the fifo
        * \param[out] timestamp milliseconds
        * \param[out] n measurement number
        * \param[out] ap single AP data
        * \return success status
        */
        bool readmeas( int64_t &timestamp, int &n, WiFi_Measurement &meas );
        /**
        * puts WiFi solution into solution fifo
        * \param[in] timestamp milliseconds
        * \param[in] wifi_pos - WiFi position
        * \param[in] loc_list - WiFi position list (multipoint location)
        */
        void pushWiFiSolution(int64_t timestamp, const WiFi_Location &wifi_pos);
        void pushWiFiSolution(int64_t timestamp, const std::vector<WiFi_Location> &loc_list);
        /**
        * gets WiFi solution from solution fifo
        * \param[out] timestamp milliseconds
        * \param[out] wifi_pos - WiFi position
        * \param[out] loc_list - WiFi position list (multipoint location)
        * \return success status
        */
        bool readWiFiSolution(int64_t &timestamp, WiFi_Location &wifi_pos);
        bool readWiFiSolution(int64_t &timestamp, std::vector<WiFi_Location> &loc_list);
        /**
        * gets next measurement timestamp in the fifo
        * \param[out] timestamp milliseconds
        * \return success status
        */
        bool getNextTimestamp( int64_t &timestamp );

        /**
        * open selected log-file for reading
        * \param[in] wifi_log full file name
        * \return success status
        */
        bool open(const std::string &wifi_log );

        /**
        * finds and open first file with selected prefix
        * \param[in] dir_log directory path for search
        * \param[in] prefix search prefix
        * \return success status
        */
        //bool openDIR( std::string dir_log, const std::string &prefix );
        /**
        * reads data from the selected log-file and push into the fifo
        */
        //void readall();
        //< clears the fifo buffer
        void clear()
        {
            while( !fifo.empty() )
                fifo.pop();
            while (!wifi_pos_queue.empty())
                wifi_pos_queue.pop();

            //fifo.swap(std::queue< M >());
        }

        //std::string logname() const
        //{
        //    return fname;
        //}


    private:
        struct M
        {
            WiFi_Measurement data;
            int64_t t;
            int n;
        };
        struct Location_and_time
        {
            WiFi_Location location;
            int64_t t;
        };

        struct Multi_location_and_time
        {
            std::vector<WiFi_Location> location_list;
            int64_t t;
        };

        std::queue<M> fifo;
        std::queue<Location_and_time> wifi_pos_queue;
        std::queue<Multi_location_and_time> wifi_multi_pos_queue;
        std::vector<std::string> parse( const std::string &s, const char &delim )const;
        void newAp( const int64_t &t, const std::string &mac, const std::string &ssid, const int &rssi1,  const int &rssi2 );
        bool readNext( int64_t &timestamp, int &n, WiFi_Measurement &meas );
        std::ifstream is;
        std::string fname;
        WiFi_Measurement tmp_meas;
        int current_n;
        int64_t current_timestamp;
};

#endif //_WIFI_HELPER_H

