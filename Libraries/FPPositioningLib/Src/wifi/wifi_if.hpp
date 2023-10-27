/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           WiFi positioning interface
* \file            wifi_if.hpp
* \addtogroup      WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#ifndef WIFI_IF_H
#define WIFI_IF_H


#include <vector>
#include <string>
#include <map>
#include "wifi_data.hpp"

/**
* \ingroup         WiFi
*/
class WiFi
{
    public:
        /**
        * estimates the location
        * \param[in] measurement measurement vector
        * \param[in] K the number of points used for interpolation
        * \return estimated location
        */
        virtual  WiFi_Location GetLocation(const WiFi_Measurement  &measurement, int K) = 0;

        /**
        * estimates the location
        * \param[in] measurement measurement vector
        * \param[in] K the number of points used for interpolation
        * \return estimated location
        */
        virtual  WiFi_Location GetLocation(const WiFi_Measurement  &measurement, int K, std::vector<WiFi_Location> &LocationList) = 0;
        

        /**
        * estimates the likehood of the measurement for the location
        * \param[in] measurement measurement vector
        * \param[in] pos location
        * \return likehood
        */
        virtual  tProb EstimateLikehood( const WiFi_Measurement  &measurement,  const WiFi_Location &pos ) = 0;

        /**
        * estimates presence of last calculated WiFi positions in fingerprinted are
        * \param[out] estimation_successful estimate status
        * \param[out] pos_in_mapped_area presence status
        */
        virtual void EstimatePresenceInMappedArea(bool &estimation_successful, bool &pos_in_mapped_area) = 0;

        /**
        * gets number of acces points in fingerprint db
        * \return number of acces points in fingerprint db
        */
        virtual size_t GetDbApCount() = 0;

        /**
        * counts the number of visible, invisible and external AP for the location
        * \param[in] measurement measurement vector
        * \param[in] pos location
        * \param[out] hit count of visible AP (both present in measurement vector and database)
        * \param[out] miss count of AP present in database but not visible now
        * \param[out] ext count of external visible AP
        */
        virtual  void GetMapFitness( const WiFi_Measurement  &measurement,  const WiFi_Location &pos, int &hit, int &miss, int &ext ) = 0;

        /**
        * estimates the bias for the location
        * \param[out] estimated bias
        * \param[out] number of used measurements
        */
        virtual  bool  estimateBias(const WiFi_Measurement &measurement, const WiFi_Location &location, int &cnt, std::ostream *log) = 0;

        /**
        * sets new bias value and enables bias estimation
        * \param[in] new bias to be set
        * \param[in] biaas estimation enable
        */
        virtual  void  setBias(double  bias, int64_t delta_t) = 0;

        /**
        * gets current bias value and bias estimation enable flag
        * \param[out] current bias value
        * \return biaas estimation enable
        */
        virtual  bool  getBias(double &bias) = 0;

        /**
        * sets new bias gain coeficient
        * \param[in] the new bias gain coeficient to be set
        */
        virtual  void  setBiasGain(double  gain) = 0;

        /**
        * gets current bias gain coeficient
        * \param[out] current gain coeficient value
        */
        virtual  void  getBiasGain(double &gain) = 0;

        /**
        * sets new rssi threshold for meassurements which is used for bias estimation
        * \param[in] new rssi threshold for bias estimation
        */
        virtual  void  setBiasRssiThreshold(double  RssiThreshold) = 0;

        /**
        * gets current rssi threshold for meassurements which is used for bias estimation
        * \return current rssi threshold for bias estimation
        */
        virtual  double  getBiasRssiThreshold(void) = 0;

        /**
        * returns AP for the location
        * \param[out] data list of pairs BSSID and RSSI
        * \param[in] pos location
        * \param[in] median flag, if true return median, otherwise return mean rssi value
        * \return success status
        */
        virtual  bool  getPositionData(std::map<BSSID, RSSI> &data, const WiFi_Location &pos, bool median, std::ostream *log) = 0;

        /**
        * \return initialization status (true if ok)
        */
        virtual  bool status() = 0;

        virtual ~WiFi() {};
};

//WiFi::~WiFi()
//{
//}
#endif
