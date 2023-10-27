/**
* \file WiFiBuilder.hpp
* \brief WiFi Fingerprint Builder
* \author Mikhail Frolov (mfrolov@invensense.com)
* \version 0.1
* \copyright InvenSense, all rights reserved
* \date February 18, 2016
*/
#ifndef WIFI_BUILDER_HPP
#define WIFI_BUILDER_HPP
#include "Fpbl.hpp"
#include <map>
#include <algorithm>
#include <cmath>

/**
 * Library namespace
 */
namespace Fpbl{



/**
* WiFiBuilder generate WiFi fingerprint for specified area
*/
class IWiFiBuilder;
class WiFiBuilder
{
public:
	typedef struct
	{
		double mu1;  /**< gaussian #1, mean */
		double sig1; /**< gaussian #1, sigma */
		double w1;   /**< gaussian #1, weight */
		double mu2;  /**< gaussian #2, mean */
		double sig2; /**< gaussian #2, sigma */
		double w2;   /**< gaussian #2, weight */
		uint16_t scan_count; /**< mumber of scans for this AP in this cell*/
	} GMixture;

	//typedef std::vector<double>   APHist; /**< single AP histogram model */
	//typedef std::map<BSSID, APHist> APList; /**< location histogram model */
	typedef std::map<BSSID, GMixture> APListGM; /**< location GM model */

	/** fingerprint record */
	typedef struct
	{
		//APList     fingerprint;    //< AP histograms
		APListGM   fingerprintGM;  //< AP GMs
		Fpbl::CoordinatesInGrid location;    //< coordinates (x,y,z)
	} DBRecord;

	/** fingerprint object */
	typedef std::vector<DBRecord> LocalDB;

	WiFiBuilder(); /**< default constructor */
  WiFiBuilder(Fpbl::FPGrid wifi_fp); /**< constructor with finger print settings */
  ~WiFiBuilder(); /**< destructor */

    /**
    * process wifi-grid and builds fingerprint for specified area
    * \param[in] wifiGrid wifi-grid from GridBuilder
    * \param[out] wifiFp generated  fingerprint
    * \return success status
    */
	Fpbl::ReturnStatus buildFingerprint(const WiFiGrid &wifiGrid, LocalDB *wifiFp);
private:
	WiFiBuilder(const WiFiBuilder &); //disable copy constructor due pointer to implementation
	const WiFiBuilder &operator=(const WiFiBuilder &); // disable copy
	//friend class IWiFiBuilder;
	IWiFiBuilder * mWiFiBuilder;

};
}

#endif //WIFI_BUILDER_HPP
