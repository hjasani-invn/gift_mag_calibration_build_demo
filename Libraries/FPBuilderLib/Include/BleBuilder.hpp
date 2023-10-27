#ifndef BLE_BUILDER_HPP
#define BLE_BUILDER_HPP
#include "Fpbl.hpp"
#include <map>
#include <algorithm>
#include <cmath>

namespace Fpbl{


class IBleBuilder;
class BleBuilder
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
		uint16_t scan_count; /**< mumber of scans for this beacon in this cell*/
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

	BleBuilder();  /**< default constructor */
	BleBuilder(FPGrid ble_fp);  /**< constructor with finger print settings */
	~BleBuilder(); /**< destructor */

  /**
  * process blei-grid and builds fingerprint for specified area
  * \param[in] bleGrid ble-grid from GridBuilder
  * \param[out] bleFp generated  fingerprint
  * \return success status
  */
	Fpbl::ReturnStatus buildFingerprint(const BleGrid &bleGrid, LocalDB *bleFp);
private:
	BleBuilder(const BleBuilder &); //disable copy constructor due pointer to implementation
	const BleBuilder &operator=(const BleBuilder &); // disable copy
	//friend class IBleBuilder;
	IBleBuilder * mBleBuilder;

};
}

#endif //BLE_BUILDER_HPP
