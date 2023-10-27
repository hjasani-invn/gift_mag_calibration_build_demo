#include "IFpbl.hpp"
#include <iostream>

class MocFpbl : public Fpbl::IGridBuilder
{
public:
	/** default constructor */
	MocFpbl(){
		///TODO
		std::cout << "MocFpbl()" << std::endl;
	}
	~MocFpbl(){
		///TODO
		std::cout << "~MocFpbl()" << std::endl;
	}
	/** \return version info*/
	Fpbl::VersionNumber getVersionNumber() const {
		Fpbl::VersionNumber vn;
		memset(&vn, 0, sizeof(vn));
		vn.major = 0;
		vn.minor = 1;
		vn.build = 1;
		vn.releaseId = Fpbl::VersionNumberReleaseId::VERSION_ALPHA;
		return vn;
	}
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
	Fpbl::ReturnStatus processDevicePosition(const uint64_t &timestamp, const Fpbl::Position &devicePosition){
		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}
	/** Puts single path device orientation into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] deviceAttitude attitude with uncertainties
	* \return status code*/
	Fpbl::ReturnStatus processDeviceAttitude(const uint64_t &timestamp, const Fpbl::Attitude &deviceAttitude){
		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}

	/** TODO wifi and Ble configuration
	* scan latency
	* RSSI std
	* auto grid assignment?
	*/


	/** Puts single path wifi measurements into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] wifi  WiFi scan result
	* \return status code*/
	Fpbl::ReturnStatus processWiFiMeasurement(const uint64_t &timestamp, const Fpbl::WiFiScanResult &wifi){
		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}

	/** Puts single path ble measurements into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] ble  BLE scan result
	* \return status code*/
	Fpbl::ReturnStatus processBleMeasurement(const uint64_t &timestamp, const Fpbl::BleScanResult &ble){
		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}

	/** Puts single path ble measurements into processing
	* \param[in] timestamp unix time in [us]
	* \param[in] mag  magnetic measurement vector
	* \return status code*/
	Fpbl::ReturnStatus processMFPMeasurement(const uint64_t &timestamp, const Fpbl::MagneticVector &mag){
		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}

	/** Setup venue params
	* \param[in] venue venue parameters
	* \return status code */
	Fpbl::ReturnStatus setVenue(const Fpbl::Venue &venue){
		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}

	/** Sets temporary folder with write access
	* \param[in] folderPath full path
	* \return status code */
	Fpbl::ReturnStatus setTemporaryFolder(const std::string &folderPath){
		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}

	/** return filled wifi grid for selected venue and grid configuration
	* \param[in] venueId unique venue id
	* \param[in] grid grid configuration
	* \param[out] wifiGrid grid with assigned wifi data
	* \return status code */
	Fpbl::ReturnStatus updateGridWiFi(const Fpbl::venue_id &venueId, const Fpbl::Grid &grid, Fpbl::WiFiGrid * wifiGrid){
		for (int i_pos = 0; i_pos < 10; ++i_pos)
		{
			double c_size = grid.size;
			Fpbl::CoordinatesInGrid pos = { c_size/2 + i_pos*c_size, c_size/2, 0 };
			Fpbl::WifiCell cell;
			cell.coordinates = pos;

			for (int i_scan = 0; i_scan < 100; ++i_scan)
			{
				Fpbl::WiFiScanResult scan;
				scan.timestamp = i_scan * 1000000;
				for (int i_ap = 0; i_ap < 5; ++i_ap)
				{
					double rssi = -60 + (i_ap/5.+1)*(i_ap % 2 == 0 ? -1 : 1)*i_pos * 2 + 5 * randnf<double>();
					//double rssi = -60 + 2 * i_ap + 5 * randnf<double>();
					Fpbl::WiFiMeasurement ap_meas = { scan.timestamp + i_ap, i_ap, static_cast<int8_t>(floor(rssi + 0.5)) };
					scan.scanWiFi.push_back(ap_meas);

				}
				cell.wifiData.push_back(scan);
			}
			wifiGrid->push_back(cell);
		}

		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}

	/** return filled ble grid for selected venue and grid configuration
	* \param[in] venueId unique venue id
	* \param[in] grid grid configuration
	* \param[out] bleGrid grid with assigned ble data
	* \return status code */
	Fpbl::ReturnStatus updateGridBLE(const Fpbl::venue_id &venueId, const Fpbl::Grid &grid, Fpbl::BleGrid * bleGrid){
		return Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
	}

	/** return filled mfp grid for selected venue and grid configuration
	* \param[in] venueId unique venue id
	* \param[in] grid grid configuration
	* \param[out] magGrid grid with assigned mfp data
	* \return status code */
	Fpbl::ReturnStatus updateGridMFP(const Fpbl::venue_id &venueId, const Fpbl::Grid &grid, Fpbl::MagneticGrid * magGrid){
		return Fpbl::ReturnStatus::STATUS_SUCCESS;
	}

	private:
		template<class T> T  randnf()
		{
			T r = 0;
			static const int N = 12;
			for (int i = 0; i < N; i++)
			{
				r += rand();
			}

			T res = (r * (static_cast<T>(1) / (RAND_MAX)) - N / 2);
			return res;
		}
};