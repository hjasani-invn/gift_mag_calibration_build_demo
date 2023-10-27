/**
 * \file fpbl.hpp
 * \brief Fingerprint Builder Library API
 * \author Mikhail Frolov (mfrolov@invensense.com)
 * \version 0.1
 * \copyright InvenSense, all rights reserved
 * \date February 1, 2016
 */

#ifndef FPBL_API_HPP
#define FPBL_API_HPP

#include <vector>
#include <string>
#include <stdint.h>
#include "TpnData.hpp"
#include "MagData.hpp"
#include "LocalData.hpp"
#include "WiFiBleData.hpp"
#include "IStdEstimator.hpp"
#include "LibraryInfo.hpp"
#include "Venue.h"

/**
 * Library namespace
 */
namespace Fpbl
{

    /**
     * operation status codes */
    enum class ReturnStatus
    {
        STATUS_SUCCESS = 0,            /**< operation success */
        STATUS_UNKNOWN_ERROR = -1      /**< undefined error occurs due operation */
    };

    /** input coordinates description (in WGS84)*/
    struct Position
    {
        double x;       /**< x [m] */
        double y;       /**< y [m] */
        double altitude;        /**< altitude (sea level?) [m] */
        int16_t floorNumber;    /**< discrete floor number [0..32767], must be positive or zero */
        double covarianceXY[2][2]; /**< Lattitude/Longitude covariance matrix in column order [rad^2]\n
                                        *  cov(lat,lat), cov(lon,lat)\n
                                        *  cov(lat,lon), cov(lon,lon) */
        double altitudeStd;     /**< altitude standard deviation [m]     */
        double floorStd;        /**< altitude standard deviation [floor] */
        bool isValid;           /**< position validity flag */
    };

    /** Attitude inforamtion (relative to ENU?) */
    struct Attitude
    {
        double quaternion[4];   /**< orientation quaternion [q0, q1, q2, q3] should be normalized */
        double covarianceQuaternion[4][4]; /**< quaternion covariance matrix, column order\n
                                            * cov(q0, q0), cov(q1, q0), cov(q2, q0), cov(q3, q0)\n
                                            * cov(q0, q1), cov(q1, q1), cov(q2, q1), cov(q3, q1)\n
                                            * cov(q0, q2), cov(q1, q2), cov(q2, q2), cov(q3, q2)\n
                                            * cov(q0, q3), cov(q1, q2), cov(q2, q3), cov(q3, q3) */
        bool isValid;           /**< orientation validity flag */
    };



// replaced to venue.h
//    /** defines cell configuration in grid*/
//    enum class CellType
//    {
//        CELL_SQUARE = 0,     /**< use square cells */
//        CELL_HEXAGONAL = 1   /**< use hexagonal cells */
//    };

    /** grid coordinates */
    struct CoordinatesInGrid
    {
        double x; ///< x-coordinate [m]
        double y; ///< y-coordinate [m]
        int16_t floor; ///< discrete floor number [floor], must be positive or zero
    };

    /** grid coordinates extended */
    struct GridDataExtended : CoordinatesInGrid
    {
        // x, y, floor are inherited
        bool is_valid;
        int8_t mode_of_transit;
        double altitude;
        bool is_crowdsourced;
        double mag_x;
        double mag_y;
        double mag_z;
        GridDataExtended()
        {
            x = 0;
            y = 0;
            floor = 0;
            is_valid = true;
            mode_of_transit = 1;
            altitude = 0;
            is_crowdsourced = false;
            mag_x = 0;
            mag_y = 0;
            mag_z = 0;
        }
        explicit GridDataExtended(const CoordinatesInGrid &grid)
        {
            *(CoordinatesInGrid*)this = grid;
            is_valid = true;
            mode_of_transit = 1;
            altitude = 0;
            is_crowdsourced = false;
            mag_x = 0;
            mag_y = 0;
            mag_z = 0;
        }
    };

    /** defines grid configuration */
    struct Grid
    {
        CellType type; /**< cell type */
        double size;   /**< cell size [m]*/
        CoordinatesInGrid min; /**< left bottom corner of grid */
        CoordinatesInGrid max; /**< right top corner of grid */
        bool  enable; // enable/disable grid generation
        double   max_position_uncertainty;        // max position uncertainty allowed to put data in grid
    };

    /** defines finger print configuration */
    struct FPGrid
    {
        Grid         grid;
        int          min_data_count;        // minimal permited data number per cell
        int          max_bssid_count;       // max AP number in FP
        double       min_ig;                // min information gain value to include AP in FP
        eStdEstimatorType  stdEstimatorType;      // type of magnetic std estimator
    };

    /** Route mode enumeration*/
    typedef enum RouteModeTag {
        Unknown = 0,
        NormalMode = 1,
        ElevatorMode = 2,
        EscalatorMode = 3,
        StairsMode = 4,
        ConveyorBeltMode = 5,
        SingleCellMode = 6
    } RouteMode_t;

    /** Dataset type enumeration*/
    enum DataSetType
    {
        kUnKnown = 0,
        kIvlData = 1,
        kSurveyData = 2,
        kMapperData = 3,
        kRetailData = 4,
        kRoboticData = 5
    };

    /** defines mag data with uncertainties and dataset type*/
    struct MagneticMeasurementAndDataSetType : public MagneticMeasurement
    {
        DataSetType dataset_type;
        MagneticMeasurementAndDataSetType() : MagneticMeasurement(), dataset_type(kSurveyData) {}
        explicit MagneticMeasurementAndDataSetType(const MagneticMeasurement& meas)
        {
            *(MagneticMeasurement*)this = meas;
            dataset_type = kSurveyData;
        }
    };

    /** assigned magnetic data*/
    struct MagneticCell
    {
        CoordinatesInGrid coordinates; ///< cell coordinates
        std::vector<MagneticMeasurementAndDataSetType> magData; ///< magnetic data
    };

    /** assigned wifi data*/
    struct WifiCell
    {
        CoordinatesInGrid coordinates; ///< cell coordinates
        std::vector<WiFiScanResult> wifiData; ///< wifi data
    };

    /** assigned BLE data*/
    struct BleCell
    {
        CoordinatesInGrid coordinates; ///< cell coordinates
        std::vector<BleScanResult> bleData; ///< ble data
    };

    /** assigned to grid magnetic data*/
    typedef std::vector<MagneticCell> MagneticGrid;
    /** assigned to grid wifi data*/
    typedef std::vector<WifiCell> WiFiGrid;
    /** assigned to grid ble data*/
    typedef std::vector<BleCell> BleGrid;

    struct PortalData
    {
        GridDataExtended positionInPortal;
        std::vector<FfPosition> resultsInPortal;
    };

	struct CSData
	{
		GridDataExtended position;
	};

    /** assigned to portals grid */
    typedef std::vector<PortalData> PortalsGrid;

	/** assigned to CS grid */
	typedef std::vector<CSData> CSGrid;

    class IGridBuilder;
    /** this class generates "static" grid of measurements*/
    class GridBuilder
    {
        public:
            /** default constructor */
            GridBuilder();
            /** destructor */
            ~GridBuilder();

            /** \return version info*/
            VersionNumber getVersionNumber() const;
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
            //ReturnStatus processDevicePosition( const uint64_t &timestamp, const Position &devicePosition );
            /** Puts single path device orientation into processing
             * \param[in] timestamp unix time in [us]
             * \param[in] deviceAttitude attitude with uncertainties
             * \return status code*/
            ReturnStatus processDeviceAttitude( const uint64_t &timestamp, const Attitude &deviceAttitude );

            /** Puts single path device orientation into processing
            * \param[in] irlOutput  irl position attitude and magnetic data
            * \return status code*/
            ReturnStatus processIrlOutput( const TpnOutput &irlOutput, DataSetType dataset_type);

            /** TODO wifi and Ble configuration
             * scan latency
             * RSSI std
             * auto grid assignment?
            */

            /** Puts single path wifi measurements into processing
            * \param[in] wifiScan  WiFi scan result
            * \return status code*/
            ReturnStatus processWiFiMeasurement( const WiFiScanResult &wifiScan );

            /** Puts single path ble measurements into processing
            * \param[in] bleScan  BLE scan result
            * \return status code*/
            ReturnStatus processBleMeasurement( const BleScanResult &bleScan );

            /** Puts single path wifi measurements into processing
             * \param[in] timestamp unix time in [us]
             * \param[in] wifi  WiFi scan result
             * \return status code*/
            ReturnStatus processWiFiMeasurement( const uint64_t &timestamp, const WiFiScanResult &wifiScan );

            /** Puts single path ble measurements into processing
             * \param[in] timestamp unix time in [us]
             * \param[in] ble  BLE scan result
             * \return status code*/
            ReturnStatus processBleMeasurement( const uint64_t &timestamp, const BleScanResult &bleScan );

            /** Puts single path ble measurements into processing
             * \param[in] timestamp unix time in [us]
             * \param[in] mag  magnetic measurement vector
             * \return status code*/
            ReturnStatus processMFPMeasurement( const uint64_t &timestamp, const MagneticVector &mag );

            /** Setup venue params
             * \param[in] venue venue parameters
             * \return status code */
            ReturnStatus setVenue(const BaseVenueType &venue);

            /** Sets temporary folder with write access
             * \param[in] folderPath full path
             * \return status code */
            ReturnStatus setTemporaryFolder( const std::string &folderPath );

            /** return filled wifi grid for selected venue and grid configuration
             * \param[in] venueId unique venue id
             * \param[in] grid grid configuration
             * \param[in,out] wifiGrid grid with assigned wifi data
             * \return status code */
            ReturnStatus updateGridWiFi( const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid );

            /** return filled ble grid for selected venue and grid configuration
             * \param[in] venueId unique venue id
             * \param[in] grid grid configuration
             * \param[in,out] bleGrid grid with assigned ble data
             * \return status code */
            ReturnStatus updateGridBLE( const venue_id &venueId, const Grid &grid, BleGrid & bleGrid );

            /** return filled mfp grid for selected venue and grid configuration
             * \param[in] venueId unique venue id
             * \param[in] grid grid configuration
             * \param[in,out] magGrid grid with assigned mfp data
             * \return status code */
            ReturnStatus updateGridMFP( const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid );

            /** return filled portals grid for selected venue and grid configuration
            * \param[in] venueId unique venue id
            * \param[in] grid grid configuration
            * \param[out] portalsGrid grid with assigned mfp data
            * \return status code */
            ReturnStatus updateGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid);

            /** create empty mfp grid for selected venue and grid configuration
            * \param[in] venueId unique venue id
            * \param[in] grid grid configuration
            * \param[out] empty magGrid grid 
            * \return status code */
            ReturnStatus createEmptyGridMFP(const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid);

            /** create empty WiFi grid for selected venue and grid configuration
            * \param[in] venueId unique venue id
            * \param[in] grid grid configuration
            * \param[out] empty WiFi wifiGrid
            * \return status code */
            ReturnStatus createEmptyGridWiFi(const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid);
            /** create empty Ble grid for selected venue and grid configuration

            * \param[in] venueId unique venue id
            * \param[in] grid grid configuration
            * \param[out] empty Ble wifiGrid
            * \return status code */
            ReturnStatus createEmptyGridBLE(const venue_id &venueId, const Grid &grid, BleGrid & bleGrid);

            /** create empty mfp grid for selected venue and grid configuration
            * \param[in] venueId unique venue id
            * \param[in] grid grid configuration
            * \param[out] empty portal grid
            * \return status code */
            ReturnStatus createEmptyGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid);

			/** create empty mfp grid for selected venue and grid configuration
			* \param[in] venueId unique venue id
			* \param[in] grid grid configuration
			* \param[out] empty CS grid
			* \return status code */
			ReturnStatus createEmptyGridCS(const venue_id &venueId, const Grid &grid, CSGrid & csGrid);

            /** set maximum allowed position uncertainty for Wi-Fi grid
            * \param[in] wifi_max_unc
            */
            void set_max_uncertainty_wifi(double wifi_max_unc);

            /** set maximum allowed position uncertainty for BLE grid
            * \param[in] ble_max_unc
            */
            void set_max_uncertainty_ble(double ble_max_unc);

            /** set maximum allowed position uncertainty for MFP grid
            * \param[in] mfp_max_unc
            */
            void set_max_uncertainty_mfp(double mfp_max_unc);

            //protected:
            //friend class impl::IGridBuilder;
        private:
            GridBuilder( const GridBuilder & ); //disable copy constructor due pointer to implementation
            const GridBuilder &operator=( const GridBuilder & ); // disable copy
            friend class IGridBuilder;
            IGridBuilder *mGridBuilder;
            //friend class IQ;
            //IQ *mQ;
    };


    VersionNumberBase getVersionNumber();
}

#endif /* FPBL_API_HPP */

