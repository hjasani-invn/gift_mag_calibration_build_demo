/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Saving/restoring BLA data
* \defgroup        ble_proximity
* \file            ble_proximity_helper.hpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/

#ifndef _BLE_PROXIMITY_HELPER_H
#define _BLE_PROXIMITY_HELPER_H

#include <fstream>
#include <queue>
#include <stdint.h>

#include "Fppe.hpp"
#include "ble_proximity_if.hpp"
#include "wifi_data.hpp"

class BLE_Proximity_Helper
{
    public:
        BLE_Proximity_Helper() {};   /**< default constructor */
        //explicit BLE_Proximity_Helper(const std::string &ble_proximity_log); /**< Creates object and select data file */
        ~BLE_Proximity_Helper() {}; /**< destructor */

        /**
        * puts measurement vector into the fifo
        * \param[in] timestamp milliseconds
        * \param[in] n measurement number
        * \param[in] meas BLE scan data
        */
        void pushMeas(int64_t timestamp, int n, const Fppe::BleScanResult &meas);

        /**
        * gets measurement vector from the fifo and remove it
        * \param[out] timestamp milliseconds
        * \param[out] n measurement number
        * \param[out] meas BLE scan data
        * \return success status
        */
        bool readMeas(int64_t &timestamp, int &n, Fppe::BleScanResult &meas);

        /**
        * gets measurement vector from the fifo and remove it
        * \param[in] current_timestamp in milliseconds, older than current_timestamp data is read
        * \param[out] meas_timestamp timestap milliseconds
        * \param[out] meas BLE scan data
        * \return success status
        */
        bool readMeas(int64_t current_timestamp, int64_t &meas_timestamp, Fppe::BleScanResult &meas);

        /**
        * puts Prox solution into solution fifo
        * \param[in] timestamp milliseconds
        * \param[in] loc_list - WiFi position list (multipoint location)
        */
        void pushBLEProxSolution(int64_t timestamp, const std::vector<WiFi_Location> &loc_list);
        /**
        * gets Prox solution from solution fifo
        * \param[out] timestamp milliseconds
        * \param[out] loc_list - WiFi position list (multipoint location)
        * \return success status
        */
        bool readBLEProxSolution(int64_t &timestamp, std::vector<WiFi_Location> &loc_list);
        bool max_min_time(int64_t &min_timestamp, int64_t &max_timestamp, Fppe::BleScanResult &meas);

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
        //bool open(const std::string &ble_proximity_log );

        //< clears the fifo buffer
        void clear()
        {
            while( !fifo.empty() )
                fifo.pop();

            //fifo.swap(std::queue< M >());
        }

    private:
        double getDistance(double measuredPower, double rssi);

        std::queue<Fppe::BleScanResult> fifo;
        std::queue < std::pair <int64_t, const std::vector<WiFi_Location>>> ble_prox_pos_list_queue;

        std::vector<std::string> parse( const std::string &s, const char &delim )const;
        Fppe::BleScanResult tmp_meas;
        int current_n;
        int64_t current_timestamp;
};

#endif //_BLE_PROXIMITY_HELPER_H

