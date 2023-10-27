#ifndef IFPBL_HPP
#define IFPBL_HPP

#include "Fpbl.hpp"

//using namespace Fpbl;

//namespace impl{

class Fpbl::IGridBuilder {
public:
	/** default constructor */
	IGridBuilder() { ; }
	virtual ~IGridBuilder() { ; }
	/** \return version info*/
	virtual VersionNumber getVersionNumber() const = 0;
	//        ReturnStatus loadModule(const std::string &module);
	//        ReturnStatus unloadModule(const std::string &module);
	//        std::string getLoadedModules() const;
	//        ReturnStatus setModuleParameter(const std::string &module, const std::string &parameter, const double &value);

	/** TODO position configuration
	* interpolation settings
	*/
	/** Puts single path coordinates into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] devicePosition coordinates with uncertainties
	* \return status code*/
	//virtual Fpbl::ReturnStatus processDevicePosition(const uint64_t &timestamp, const Fpbl::Position &devicePosition) = 0;
	/** Puts single path device orientation into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] deviceAttitude attitude with uncertainties
	* \return status code*/
	virtual Fpbl::ReturnStatus processDeviceAttitude(const uint64_t &timestamp, const Fpbl::Attitude &deviceAttitude) = 0;


    /** Puts single path device orientation into processing
    * \param[in] irlOutput  IRL position attitude and magnetic data
    * \return status code*/
    virtual Fpbl::ReturnStatus processIrlOutput(const TpnOutput &irlOutput, DataSetType dataset_type) = 0;

	/** TODO wifi and Ble configuration
	* scan latency
	* RSSI std
	* auto grid assignment?
	*/


	/** Puts single path wifi measurements into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] wifi  WiFi scan result
	* \return status code*/
	virtual Fpbl::ReturnStatus processWiFiMeasurement(const uint64_t &timestamp, const WiFiScanResult &wifi) = 0;

	/** Puts single path ble measurements into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] ble  BLE scan result
	* \return status code*/
	virtual Fpbl::ReturnStatus processBleMeasurement(const uint64_t &timestamp, const BleScanResult &ble) = 0;

	/** Puts single path ble measurements into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] mag  magnetic measurement vector
	* \return status code*/
	virtual Fpbl::ReturnStatus processMFPMeasurement(const uint64_t &timestamp, const MagneticVector &mag) = 0;

	/** Setup venue params
	* \param[in] venue venue parameters
	* \return status code */
  virtual Fpbl::ReturnStatus setVenue(const BaseVenueType &venue) = 0;

	/** Sets temporary folder with write access
	* \param[in] folderPath full path
	* \return status code */
	virtual Fpbl::ReturnStatus setTemporaryFolder(const std::string &folderPath) = 0;

	/** return filled wifi grid for selected venue and grid configuration
	* \param[in] venueId unique venue id
	* \param[in] grid grid configuration
	* \param[out] wifiGrid grid with assigned wifi data
	* \return status code */
	virtual Fpbl::ReturnStatus updateGridWiFi(const venue_id &venueId, const Fpbl::Grid &grid, Fpbl::WiFiGrid & wifiGrid) = 0;

	/** return filled ble grid for selected venue and grid configuration
	* \param[in] venueId unique venue id
	* \param[in] grid grid configuration
	* \param[out] bleGrid grid with assigned ble data
	* \return status code */
	virtual Fpbl::ReturnStatus updateGridBLE(const venue_id &venueId, const Fpbl::Grid &grid, Fpbl::BleGrid & bleGrid) = 0;

	/** return filled mfp grid for selected venue and grid configuration
	* \param[in] venueId unique venue id
	* \param[in] grid grid configuration
	* \param[out] magGrid grid with assigned mfp data
	* \return status code */
	virtual Fpbl::ReturnStatus updateGridMFP(const venue_id &venueId, const Fpbl::Grid &grid, Fpbl::MagneticGrid & magGrid) = 0;

    /** return filled portals grid for selected venue and grid configuration
    * \param[in] venueId unique venue id
    * \param[in] grid grid configuration
    * \param[out] portalsGrid grid with assigned mfp data
    * \return status code */
    virtual Fpbl::ReturnStatus updateGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid) = 0;

  virtual Fpbl::ReturnStatus createEmptyGridMFP(const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid) = 0;
  virtual Fpbl::ReturnStatus createEmptyGridWiFi(const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid) = 0;
  virtual Fpbl::ReturnStatus createEmptyGridBLE(const venue_id &venueId, const Grid &grid, BleGrid & bleGrid)= 0;
  virtual Fpbl::ReturnStatus createEmptyGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid) = 0;
  virtual Fpbl::ReturnStatus createEmptyGridCS(const venue_id &venueId, const Grid &grid, CSGrid & csGrid) = 0;

  virtual void set_max_uncertainty_wifi(double wifi_max_unc) = 0;
  virtual void set_max_uncertainty_ble(double ble_max_unc) = 0;
  virtual void set_max_uncertainty_mfp(double mfp_max_unc) = 0;

};
//}



#endif //IFPBL
