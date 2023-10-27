/*
 * Copyright (c) InvenSense Inc
 */

/*
 * File:   fpbl.hpp
 * Author: Mikhail Frolov (mfrolov@invensense.com)
 * Version: 0.1
 *
 * Created on February 1, 2016, 1:38 PM
 */

#ifndef GRIDBUILDER_P_HPP
#define GRIDBUILDER_P_HPP

#include <vector>
#include <list>
#include <stdint.h>
#include <fstream>

#include "IFpbl.hpp"
#include "TpnData.hpp"
#include "LocalData.hpp"
#include "Venue.h"


namespace Fpbl
{

    /** this class generates "static" grid of measurements*/
    class GridBuilderPrivate : public Fpbl::IGridBuilder
    {
        public:
            GridBuilderPrivate();
            /** \return version info*/
            VersionNumber getVersionNumber() const;

            /** Setup venue params
            * \param[in] venue venue parameters
            * \return status code */
            ReturnStatus setVenue(const BaseVenueType &venue);

			BaseVenueType getVenue();


            /** Sets temporary folder with write access
            * \param[in] folderPath full path
            * \return status code */
            ReturnStatus setTemporaryFolder( const std::string &folderPath );

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


            /** Puts single path coordinates into processing
            * \param[in] timestamp unix time in [us]
            * \param[in] devicePosition coordinates with uncertainties
            * \return status code*/
            ReturnStatus processDeviceFfPosition( const FfPosition &devicePosition );

            /** Puts single path device orientation into processing
             * \param[in] timestamp unix time in [us]
             * \param[in] deviceAttitude attitude with uncertainties
             * \return status code*/
            ReturnStatus processDeviceAttitude( const uint64_t &timestamp, const Attitude &deviceAttitude );

            /** Puts single path device orientation into processing
            * \param[in] deviceAttitude attitude with uncertainties
            * \return status code*/
            ReturnStatus processDeviceMffAttitude( const MffAttitude &deviceAttitude );

            /** Puts single path device orientation into processing
            * \param[in] irlOutput  IRL position attitude and magnetic data
            * \return status code*/
            ReturnStatus processIrlOutput( const TpnOutput &irlOutput, DataSetType dataset_type);

            /** TODO wifi and Ble configuration
             * scan latency
             * RSSI std
             * auto grid assignment?
            */


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

            /** Puts single path ble measurements into processing
            * \param[in] timestamp unix time in [us]
            * \param[in] mag  magnetic measurement vector with uncertainties
            * \return status code*/
            ReturnStatus processMFPMeasurementWithUncertainties( const MagneticMeasurementAndDataSetType &mag );

            /** return filled wifi grid for selected venue and grid configuration
             * \param[in] venueId unique venue id
             * \param[in] grid grid configuration
             * \param[out] wifiGrid grid with assigned wifi data
             * \return status code */
            ReturnStatus updateGridWiFi( const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid );

            /** return filled ble grid for selected venue and grid configuration
             * \param[in] venueId unique venue id
             * \param[in] grid grid configuration
             * \param[out] bleGrid grid with assigned ble data
             * \return status code */
            ReturnStatus updateGridBLE( const venue_id &venueId, const Grid &grid, BleGrid & bleGrid );

            /** return filled mfp grid for selected venue and grid configuration
             * \param[in] venueId unique venue id
             * \param[in] grid grid configuration
             * \param[out] magGrid grid with assigned mfp data
             * \return status code */
            ReturnStatus updateGridMFP( const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid );

            /** return filled portals grid for selected venue and grid configuration
             * \param[in] venueId unique venue id
             * \param[in] grid grid configuration
             * \param[out] portalsGrid grid with assigned mfp data
             * \return status code */
            ReturnStatus updateGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid);

            ReturnStatus createEmptyGridMFP(const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid);
            ReturnStatus createEmptyGridWiFi(const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid);
            ReturnStatus createEmptyGridBLE(const venue_id &venueId, const Grid &grid, BleGrid & bleGrid);
            ReturnStatus createEmptyGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid);
			ReturnStatus createEmptyGridCS(const venue_id &venueId, const Grid &grid, CSGrid & csGrid);

            //  getting methods for testeng only
            std::string getTemporaryFolder();
            uint64_t getDevicePositionTimestamp( const uint32_t number );
            Position getDevicePosition( const uint32_t number );
            uint64_t getDeviceAttitudeTimestamp( const uint32_t number );
            MffAttitude getDeviceAttitude( const uint32_t number );

            uint64_t getWiFiMeasurementTimestamp( const uint32_t number );
            WiFiScanResult getWiFiMeasurement( const uint32_t number );

            uint64_t getBleMeasurementTimestamp( const uint32_t number );
            BleScanResult getBleMeasurement( const uint32_t number );

            uint64_t getMFPMeasurementTimestamp( const uint32_t number );
            MagneticVector getMFPMeasurement( const uint32_t number );

            void set_max_uncertainty_wifi(double wifi_max_unc);
            void set_max_uncertainty_ble(double ble_max_unc);
            void set_max_uncertainty_mfp(double mfp_max_unc);

        private:

            struct PositionExt
            {
                Position   position;
                uint64_t   timestamp;
            };

            //struct AttitudeExt
            //{
            //    Attitude   attitude;
            //    uint64_t   timestamp;
            //};

            struct Quaternion
            {
                double w;
                double x;
                double y;
                double z;
            };

            GridBuilderPrivate( const GridBuilderPrivate & ); //disable copy constructor due pointer to implementation
            const GridBuilderPrivate &operator=( const GridBuilderPrivate & ); // disable copy

            VersionNumber mVersionNumber;
            BaseVenueType         mVenue;
            std::string   mTemporaryFolder;

            double max_uncertainty_wifi; 
            double max_uncertainty_ble; 
            double max_uncertainty_mfp; 
            
            //std::vector<uint64_t> mDevicePositionTimestamps;
            //std::vector<Position> mDevicePositions;
            std::vector<PositionExt> mDevicePositions;
            //std::list<PositionExt> mDevicePositions;

            std::vector<FfPosition> mPortalPositions;

            //std::vector<uint64_t> mDeviceAttitudeTimestamps;
            std::vector<MffAttitude> mDeviceAttitudes;

            std::vector<uint64_t> mWiFiScanResultTimestamps;
            std::vector<WiFiScanResult> mWiFiScanResults;

            std::vector<uint64_t> mBleScanResultTimestamps;
            std::vector<BleScanResult> mBleScanResults;

            //std::vector<uint64_t> mMagneticVectorTimestamps;
            std::vector<MagneticMeasurementAndDataSetType> mMagneticVectors;

            MagneticGrid  mMagneticGrid;
            WiFiGrid      mWiFiGrid;
            BleGrid       mBleGrid;
            PortalsGrid   mPortalGrid;

            /** assigned posotion data*/
            struct PositionsCell
            {
                CoordinatesInGrid coordinates; ///< cell coordinates
                std::vector<std::pair<uint64_t, double>> timestamp_with_uncertainty; ///< wifi data
            };

            /** assigned to posotion ble data*/
            typedef std::vector<PositionsCell> PositionsGrid;
            PositionsGrid mPositionsGrid;

            ReturnStatus buildPositionTimestampsGrid( const venue_id &venueId, const Grid &grid, PositionsGrid & positionsGrid );

            ReturnStatus buildPositionGrid(const venue_id &venueId, const Grid &grid, PositionsGrid & positionsGrid);

            ReturnStatus addPositionTimestampsToCell(const venue_id &venueId, const Grid &grid, PositionsCell &cell, PositionsGrid & positionsGrid);

            ReturnStatus buildWiFiDataGrid( const venue_id &venueId, const Grid &grid, PositionsGrid * positionsGrid, WiFiGrid & wifiGrid );
            //void sortWiFiGrid(Fpbl::WiFiGrid &wifi_grid);

            ReturnStatus removeFrontWiFiScanResults();

            ReturnStatus removeLastWiFiScanResults();

            ReturnStatus buildBleDataGrid( const venue_id &venueId, const Grid &grid, PositionsGrid * positionsGrid, BleGrid & bleGrid );
            //void sortBleGrid(Fpbl::BleGrid &ble_grid);

            ReturnStatus buildMfpDataGrid( const venue_id &venueId, const Grid &grid, PositionsGrid * positionsGrid, MagneticGrid & magGrid );
            //void sortMagneticGrid(Fpbl::MagneticGrid &magnetic_grid);

            ReturnStatus buildPortalsDataGrid(const venue_id &venueId, const Grid &grid, PositionsGrid * positionsGrid, PortalsGrid & portalsGrid);
			
            //ReturnStatus findNearestMeasurementInGrid( const venue_id &venueId, const Grid &grid, const PositionsGrid &positionsGrid,
            //        const uint64_t &measurementTimestamp, const double &uncertainty, int64_t *positionInGrid );

         
            ReturnStatus findNearestPosition(const venue_id &venueId,
                uint64_t measurementTimestamp, double uncertainty, PositionExt &devicePosition, std::vector<PositionExt>::iterator   &pos_it_cur);

            ReturnStatus findNearestAttitude(const venue_id &venueId,
                uint64_t measurementTimestamp, MffAttitude &deviceAttitude, std::vector<MffAttitude>::iterator   &att_it_cur);

            ReturnStatus rotateMagneticVector(MagneticVector &magVector, const MffAttitude &deviceAttitude);
            MagneticMeasurementAndDataSetType calculateUncertainties( MagneticMeasurementAndDataSetType &magVector, const MffAttitude &deviceAttitude );

            void quaternionsMultiplication( Quaternion first, Quaternion second, Quaternion &result );

            ReturnStatus clearDevicePositionsAndTimestamps();

            int32_t getNeededGridSize(const venue_id &venueId, const Grid &grid);
    };

}

#endif /* GRIDBUILDER_P_HPP */

