// FPBuilderLib.cpp : Defines the entry point for the console application.
//

#include <iostream>  // for std::cout and std::cerr

#include "IFpbl.hpp"
#include "GridBuilderPrivate.hpp"

#include "FpblVersion.h"

namespace Fpbl
{


    VersionNumberBase getVersionNumber()
    {
        return VersionNumber(uint8_t(FPBL_VERSION_MAJOR), uint8_t(FPBL_VERSION_MINOR), uint8_t(FPBL_VERSION_BUILD), VersionNumberReleaseId(FPBL_VERSION_RELEASE_ID));
    }

    GridBuilder::GridBuilder() //: mGridBuilder(new IGridBuilder::IGridBuilder() )
    {
        mGridBuilder =  new GridBuilderPrivate();
    }

    /** destructor */
    GridBuilder::~GridBuilder()
    {
        delete mGridBuilder;
    }
    VersionNumber GridBuilder::getVersionNumber() const
    {
        return mGridBuilder->getVersionNumber();
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
#if 0
    ReturnStatus GridBuilder::processDevicePosition( const uint64_t &timestamp, const Position &devicePosition )
    {
        return mGridBuilder->processDevicePosition( timestamp, devicePosition );
    }
#endif
    /** Puts single path device orientation into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] deviceAttitude attitude with uncertainties
    * \return status code*/
    ReturnStatus GridBuilder::processDeviceAttitude( const uint64_t &timestamp, const Attitude &deviceAttitude )
    {
        return mGridBuilder->processDeviceAttitude( timestamp, deviceAttitude );
    }

    /** Puts single path device orientation into processing
    * \param[in] irlOutput  IRL position attitude and magnetic data
    * \return status code*/
    ReturnStatus GridBuilder::processIrlOutput(const TpnOutput &irlOutput, DataSetType dataset_type)
    {
        return mGridBuilder->processIrlOutput(irlOutput, dataset_type);
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
    ReturnStatus GridBuilder::processWiFiMeasurement( const uint64_t &timestamp, const WiFiScanResult &wifiScan )
    {
        return mGridBuilder->processWiFiMeasurement( timestamp, wifiScan );
    }

    /** Puts single path ble measurements into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] ble  BLE scan result
    * \return status code*/
    ReturnStatus GridBuilder::processBleMeasurement( const uint64_t &timestamp, const BleScanResult &bleScan )
    {
        /*
        static long ble_count;
        ble_count++;
        std::cout << std::endl << "ble_count = " << ble_count << "      " << timestamp << std::endl;
        for (auto bleScanRes = bleScan.scanBle.begin(); bleScanRes != bleScan.scanBle.end(); ++bleScanRes)
            std::cout << "      " << bleScanRes->timestamp << "      " << bleScanRes->mac << std::endl;
        */
        return mGridBuilder->processBleMeasurement( timestamp, bleScan );
    }

    /** Puts single path ble measurements into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] mag  magnetic measurement vector
    * \return status code*/
    ReturnStatus GridBuilder::processMFPMeasurement( const uint64_t &timestamp, const MagneticVector &mag )
    {
        return mGridBuilder->processMFPMeasurement( timestamp, mag );
    }

    /** Setup venue params
    * \param[in] venue venue parameters
    * \return status code */
    ReturnStatus GridBuilder::setVenue(const BaseVenueType &venue)
    {
        return mGridBuilder->setVenue( venue );
    }

    /** Sets temporary folder with write access
    * \param[in] folderPath full path
    * \return status code */
    ReturnStatus GridBuilder::setTemporaryFolder( const std::string &folderPath )
    {
        return mGridBuilder->setTemporaryFolder( folderPath );
    }

    /** return filled wifi grid for selected venue and grid configuration
    * \param[in] venueId unique venue id
    * \param[in] grid grid configuration
    * \param[out] wifiGrid grid with assigned wifi data
    * \return status code */
    ReturnStatus GridBuilder::updateGridWiFi( const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid )
    {
        return mGridBuilder->updateGridWiFi( venueId, grid, wifiGrid );
    }

    /** return filled ble grid for selected venue and grid configuration
    * \param[in] venueId unique venue id
    * \param[in] grid grid configuration
    * \param[out] bleGrid grid with assigned ble data
    * \return status code */
    ReturnStatus GridBuilder::updateGridBLE( const venue_id &venueId, const Grid &grid, BleGrid & bleGrid )
    {
        return mGridBuilder->updateGridBLE( venueId, grid, bleGrid );
    }

    /** return filled mfp grid for selected venue and grid configuration
    * \param[in] venueId unique venue id
    * \param[in] grid grid configuration
    * \param[out] magGrid grid with assigned mfp data
    * \return status code */
    ReturnStatus GridBuilder::updateGridMFP( const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid )
    {
        return mGridBuilder->updateGridMFP( venueId, grid, magGrid );
    }
    ReturnStatus GridBuilder::updateGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid)
    {
        return mGridBuilder->updateGridPortals(venueId, grid, portalsGrid);
    }

    ReturnStatus GridBuilder::createEmptyGridMFP(const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid)
    {
        return mGridBuilder->createEmptyGridMFP(venueId, grid, magGrid);
    }
    ReturnStatus GridBuilder::createEmptyGridWiFi(const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid)
    {
        return mGridBuilder->createEmptyGridWiFi(venueId, grid, wifiGrid);
    }
    ReturnStatus GridBuilder::createEmptyGridBLE(const venue_id &venueId, const Grid &grid, BleGrid & bleGrid)
    {
        return mGridBuilder->createEmptyGridBLE(venueId, grid, bleGrid);
    }
    ReturnStatus GridBuilder::createEmptyGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid)
    {
        return mGridBuilder->createEmptyGridPortals(venueId, grid, portalsGrid);
    }

	ReturnStatus GridBuilder::createEmptyGridCS(const venue_id &venueId, const Grid &grid, CSGrid & csGrid)
	{
		return mGridBuilder->createEmptyGridCS(venueId, grid, csGrid);
	}

    void GridBuilder::set_max_uncertainty_wifi(double wifi_max_unc)
    {
        mGridBuilder->set_max_uncertainty_wifi(wifi_max_unc);
    }

    void GridBuilder::set_max_uncertainty_ble(double ble_max_unc)
    {
        mGridBuilder->set_max_uncertainty_ble(ble_max_unc);
    }

    void GridBuilder::set_max_uncertainty_mfp(double mfp_max_unc)
    {
        mGridBuilder->set_max_uncertainty_mfp(mfp_max_unc);
    }


    GridBuilder::GridBuilder( const GridBuilder & ) //disable copy constructor due pointer to implementation
    {
    }

    const GridBuilder &GridBuilder::operator=( const GridBuilder &GridBuilder ) // disable copy
    {
        return GridBuilder;
    }
};

