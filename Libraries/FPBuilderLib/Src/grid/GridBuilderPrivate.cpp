// FPBuilderLib.cpp : Defines the entry point for the console application.
//

#include "Fpbl.hpp"
#include "GridBuilderPrivate.hpp"
#include "iostream"
#include "tpn_converter.hpp"
#include "Venue.h"
#include "BleHash.h"

#include <eigen/Geometry>


namespace Fpbl
{

    const int64_t maxAttitudeMeasTimeTolerance = 2; // [ms]
    const int64_t maxBleMeasTimeTolerance = 1000; // [ms]
    const int64_t maxWifiMeasTimeTolerance = 1000;// [ms]
    const int64_t maxMagMeasTimeTolerance = 50;// [ms]


    GridBuilderPrivate::GridBuilderPrivate()
    {
        mVersionNumber = VersionNumber( 0, 0, 0, VersionNumberReleaseId::VERSION_ALPHA );

        //mDevicePositionTimestamps.clear();
        mDevicePositions.clear();

        //mDeviceAttitudeTimestamps.clear();
        mDeviceAttitudes.clear();

        mWiFiScanResultTimestamps.clear();
        mWiFiScanResults.clear();

        mBleScanResultTimestamps.clear();
        mBleScanResults.clear();

        //mMagneticVectorTimestamps.clear();
        mMagneticVectors.clear();

        max_uncertainty_wifi = 1e6;
        max_uncertainty_ble = 1e6;
        max_uncertainty_mfp = 1e6;
    }

    /** \return version info*/
    VersionNumber GridBuilderPrivate::getVersionNumber() const
    {
        return mVersionNumber;
    }

    /** Setup venue params
    * \param[in] venue venue parameters
    * \return status code */
    ReturnStatus GridBuilderPrivate::setVenue(const BaseVenueType &venue)
    {
        mVenue = venue;
        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    /** \return venue info*/
    BaseVenueType GridBuilderPrivate::getVenue()
    {
        return mVenue;
    }


    /** Sets temporary folder with write access
    * \param[in] folderPath full path
    * \return status code */
    ReturnStatus GridBuilderPrivate::setTemporaryFolder( const std::string &folderPath )
    {
        mTemporaryFolder = folderPath;

        //const std::string  magLogFileName = mTemporaryFolder + "\\tmp_log.txt";
        //mg_log.open(magLogFileName.c_str(), std::ios::out);
        //std::fstream  mg_log(folderPath.c_str(), std::ios::out);
        //mg_log.open(folderPath.c_str(), std::ios::out);
        //mg_log << mTemporaryFolder;
        //mg_log.close();


        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    /** Puts single path coordinates into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] devicePosition coordinates with uncertainties
    * \return status code*/
    ReturnStatus GridBuilderPrivate::processDeviceFfPosition( const FfPosition &devicePosition )
    {
        if ( devicePosition.is_valid )
        {
            PositionExt devicePositionExt;
            devicePositionExt.timestamp = devicePosition.timestamp;
            devicePositionExt.position.x = devicePosition.x;
            devicePositionExt.position.y = devicePosition.y;
            devicePositionExt.position.altitude = devicePosition.altitude;
            devicePositionExt.position.floorNumber = devicePosition.floor_number;
            memcpy( devicePositionExt.position.covarianceXY, devicePosition.covariance_xy, sizeof( devicePosition.covariance_xy ) );
            devicePositionExt.position.altitudeStd = devicePosition.altitude_std;
            devicePositionExt.position.floorStd = devicePosition.floor_std;
            devicePositionExt.position.isValid = devicePosition.is_valid;
            mDevicePositions.push_back( devicePositionExt );                                   

            if ((devicePosition.mode_of_transit == kElevator) || (devicePosition.mode_of_transit == kStairs) ||
                (devicePosition.mode_of_transit == kEscalatorWalking) || (devicePosition.mode_of_transit == kEscalatorStanding))
                mPortalPositions.push_back(devicePosition);
        }

        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }


    /** Puts single path device orientation into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] deviceAttitude attitude with uncertainties
    * \return status code*/
    ReturnStatus GridBuilderPrivate::processDeviceAttitude( const uint64_t &timestamp, const Attitude &deviceAttitude )
    {
        if ( deviceAttitude.isValid )
        {
            //mDeviceAttitudeTimestamps.push_back(timestamp);
            //mDeviceAttitudes.push_back(deviceAttitude);

            MffAttitude deviceMffAttitude;
            deviceMffAttitude.heading_std = 0;
            deviceMffAttitude.is_valid = true;
            deviceMffAttitude.pitch_std = 0;
            deviceMffAttitude.roll_std = 0;

            deviceMffAttitude.timestamp = timestamp;
            memcpy( deviceMffAttitude.quaternion, deviceAttitude.quaternion, sizeof( deviceAttitude.quaternion ) );
            mDeviceAttitudes.push_back( deviceMffAttitude );
        }

        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    ReturnStatus GridBuilderPrivate::processDeviceMffAttitude( const MffAttitude &deviceAttitude )
    {
        if ( deviceAttitude.is_valid )
        {
            mDeviceAttitudes.push_back( deviceAttitude );
        }

        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    /** Puts single path device orientation into processing
    * \param[in] tpnOutput  TPN position attitude and magnetic data
    * \return status code*/
    ReturnStatus GridBuilderPrivate::processIrlOutput( const TpnOutput &irlOutput, DataSetType dataset_type)
    {
        TpnConverter converter ( mVenue );

        if ( irlOutput.position.is_valid == false )
        {
            return Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        FfPosition ff_position = converter.ConvertPositionData( irlOutput.timestamp, irlOutput.position );
#if 0
std::cout << std::endl;
std::cout << ff_position.timestamp << ", ";
std::cout << ff_position.x << ", ";
std::cout << ff_position.y << ", ";
std::cout << (int)ff_position.mode_of_transit << ", ";
std::cout << ff_position.covariance_xy[0][0] << ", ";
std::cout << ff_position.covariance_xy[1][1] << ", ";
std::cout << irlOutput.position.is_valid << irlOutput.attitude.is_valid << irlOutput.mag_meas.is_valid;
std::cout << std::endl;
std::cout.flush();
#endif
        MffAttitude mff_attitude = converter.ConvertAttitudeData( irlOutput.timestamp, irlOutput.attitude );
        MagneticMeasurement mag = converter.ConvertMagneticData( irlOutput.timestamp, irlOutput.mag_meas );
        MagneticMeasurementAndDataSetType mag_with_dataset_type = MagneticMeasurementAndDataSetType(mag);
        mag_with_dataset_type.dataset_type = dataset_type;

        processDeviceFfPosition( ff_position );

        if (irlOutput.mag_meas.is_valid && irlOutput.attitude.is_valid)
        {
            processDeviceMffAttitude(mff_attitude);
            processMFPMeasurementWithUncertainties( mag_with_dataset_type );
        }

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
    ReturnStatus GridBuilderPrivate::processWiFiMeasurement( const uint64_t &timestamp, const WiFiScanResult &wifiScan )
    {
        mWiFiScanResultTimestamps.push_back( timestamp );
        mWiFiScanResults.push_back( wifiScan );

        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    /** Puts single path ble measurements into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] ble  BLE scan result
    * \return status code*/
    ReturnStatus GridBuilderPrivate::processBleMeasurement( const uint64_t &timestamp, const BleScanResult &inBleScan )
    {
        BleScanResult bleScan;

        // set BLE hash instead MAC address
        bleScan.scanBle.clear();
        bleScan.timestamp = inBleScan.timestamp;
        bleScan.dataset_id = inBleScan.dataset_id;
        for (auto item: inBleScan.scanBle)
        {
            if ((item.major != 0) && (item.minor != 0)) // "zero" beacons rejection
            {
                BleMeasurement meas = item;
                meas.mac = getBleHash(item.major, item.minor, item.uuid);
                bleScan.scanBle.push_back(meas);
            }
        }
        
        if (bleScan.scanBle.size() > 0)
        {
            mBleScanResultTimestamps.push_back(timestamp);
            mBleScanResults.push_back(bleScan);
        }

        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    /** Puts single path ble measurements into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] mag  magnetic measurement vector
    * \return status code*/
    ReturnStatus GridBuilderPrivate::processMFPMeasurement( const uint64_t &timestamp, const MagneticVector &mag )
    {
        //mMagneticVectorTimestamps.push_back( timestamp );
        MagneticMeasurement mag_data = {};
        mag_data.mX = mag.mX;
        mag_data.mY = mag.mY;
        mag_data.mZ = mag.mZ;
        mag_data.timestamp = timestamp;

        mag_data.sigma_mX = mag_data.sigma_mY = mag_data.sigma_mZ = 1.;
        memset( mag_data.covarianceMatrix, 0, sizeof( mag_data.covarianceMatrix ) );
        mag_data.covarianceMatrix[0][0] = mag_data.covarianceMatrix[1][1] = mag_data.covarianceMatrix[2][2] = 1.;

        MagneticMeasurement mag_data1;
        *(MagneticVector*)&mag_data1 = mag_data;
        MagneticMeasurementAndDataSetType mag_data2 = MagneticMeasurementAndDataSetType(mag_data1);

        mMagneticVectors.push_back( mag_data2 );

        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    /** Puts single path ble measurements into processing
    * \param[in] timestamp unix time in [us]
    * \param[in] mag  magnetic measurement vector
    * \return status code*/
    ReturnStatus GridBuilderPrivate::processMFPMeasurementWithUncertainties( const MagneticMeasurementAndDataSetType &mag )
    {
        mMagneticVectors.push_back( mag );
        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    /** return filled wifi grid for selected venue and grid configuration
    * \param[in] venueId unique venue id
    * \param[in] grid grid configuration
    * \param[out] wifiGrid grid with assigned wifi data
    * \return status code */
    ReturnStatus GridBuilderPrivate::updateGridWiFi( const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid )
    {
        PositionsGrid positionsGrid;

        //ReturnStatus status = buildPositionTimestampsGrid( venueId, grid, positionsGrid );
        ReturnStatus status = buildPositionGrid( venueId, grid, positionsGrid );


        size_t wifiGridSize = wifiGrid.size();

        if ( wifiGridSize == 0 )
        {
            wifiGrid.resize( positionsGrid.size() );

            for ( size_t i = 0; i < wifiGrid.size(); ++i )
            {
                wifiGrid [i].coordinates = positionsGrid[i].coordinates;
            }
        }
        else
        {
            // wifiGrid have some data yet
        }

        removeFrontWiFiScanResults();
        removeLastWiFiScanResults();

        status = buildWiFiDataGrid( venueId, grid, &positionsGrid, wifiGrid );
        return status;
    }


    //TODO refactoring
    ReturnStatus GridBuilderPrivate::removeFrontWiFiScanResults()
    {
        if (mDevicePositions.size() > 0)
        {
            PositionExt frontDevicePosition = mDevicePositions.front();
            uint64_t frontDevicePositionTimestamp = frontDevicePosition.timestamp;
            uint64_t frontWiFiScanResultTimestamp;

            if (mWiFiScanResultTimestamps.size() > 0)
            {
                frontWiFiScanResultTimestamp = mWiFiScanResultTimestamps.front();
            }
            else
            {
                return Fpbl::ReturnStatus::STATUS_SUCCESS;
            }

            while (frontWiFiScanResultTimestamp < frontDevicePositionTimestamp)
            {
                mWiFiScanResultTimestamps.erase(mWiFiScanResultTimestamps.begin());
                mWiFiScanResults.erase(mWiFiScanResults.begin());

                if (mWiFiScanResultTimestamps.size() > 0)
                {
                    frontWiFiScanResultTimestamp = mWiFiScanResultTimestamps.front();
                }
                else
                {
                    return Fpbl::ReturnStatus::STATUS_SUCCESS;
                }
            }
        }
        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    ReturnStatus GridBuilderPrivate::removeLastWiFiScanResults()
    {
        if (mDevicePositions.size() > 0)
        {
            PositionExt lastDevicePosition = mDevicePositions.back();
            uint64_t lastDevicePositionTimestamp = lastDevicePosition.timestamp;
            uint64_t lastWiFiScanResultTimestamp;

            if (mWiFiScanResultTimestamps.size() > 0)
            {
                lastWiFiScanResultTimestamp = mWiFiScanResultTimestamps.back();

            }
            else
            {
                return Fpbl::ReturnStatus::STATUS_SUCCESS;
            }

            while (lastWiFiScanResultTimestamp > lastDevicePositionTimestamp)
            {
                mWiFiScanResultTimestamps.pop_back();
                mWiFiScanResults.pop_back();

                if (mWiFiScanResultTimestamps.size() > 0)
                {
                    lastWiFiScanResultTimestamp = mWiFiScanResultTimestamps.back();
                }
                else
                {
                    return Fpbl::ReturnStatus::STATUS_SUCCESS;
                }
            }
        }
        return Fpbl::ReturnStatus::STATUS_SUCCESS;
    }

    /** return filled ble grid for selected venue and grid configuration
    * \param[in] venueId unique venue id
    * \param[in] grid grid configuration
    * \param[out] bleGrid grid with assigned ble data
    * \return status code */
    ReturnStatus GridBuilderPrivate::updateGridBLE( const venue_id &venueId, const Grid &grid, BleGrid & bleGrid )
    {
        PositionsGrid positionsGrid;

        //ReturnStatus status = buildPositionTimestampsGrid( venueId, grid, positionsGrid );
        ReturnStatus status = buildPositionGrid( venueId, grid, positionsGrid );

        size_t bleGridSize = bleGrid.size();

        if ( bleGridSize == 0 )
        {
            bleGrid.resize( positionsGrid.size() );

            for ( size_t i = 0; i < bleGrid.size(); ++i )
            {
                bleGrid [i].coordinates = positionsGrid[i].coordinates;
            }
        }
        else
        {
            // bleGrid have some data yet
        }

        status = buildBleDataGrid( venueId, grid, &positionsGrid, bleGrid );
        return status;
    }

    /** return filled mfp grid for selected venue and grid configuration
    * \param[in] venueId unique venue id
    * \param[in] grid grid configuration
    * \param[out] magGrid grid with assigned mfp data
    * \return status code */
    ReturnStatus GridBuilderPrivate::updateGridMFP( const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid )
    {
        PositionsGrid positionsGrid;

        //ReturnStatus status = buildPositionTimestampsGrid( venueId, grid, positionsGrid );
        ReturnStatus status = buildPositionGrid( venueId, grid, positionsGrid );

        size_t magGridSize = magGrid.size();

        if ( magGridSize == 0 )
        {
            magGrid.resize( positionsGrid.size() );

            for ( size_t i = 0; i < magGrid.size(); ++i )
            {
                magGrid[i].coordinates = GridDataExtended(positionsGrid[i].coordinates);
            }
        }
        else
        {
            // magGrid have some data yet
        }

        status = buildMfpDataGrid( venueId, grid, &positionsGrid, magGrid );

        return status;
    }

    /** return filled portals grid for selected venue and grid configuration
     * \param[in] venueId unique venue id
     * \param[in] grid grid configuration
     * \param[out] portalsGrid grid with assigned mfp data
     * \return status code */
    ReturnStatus  GridBuilderPrivate::updateGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid)
    {
        PositionsGrid positionsGrid = {};
        ReturnStatus status = buildPositionGrid(venueId, grid, positionsGrid);
        /*
        for (size_t i = 0; i < portalsGrid.size(); ++i)
        {
            portalsGrid[i].positionInPortal = {};
        }
        */
        if (portalsGrid.size() == 0)
        {
            portalsGrid.resize(positionsGrid.size());

            //GridDataExtended positionInPortal = {};

            for (size_t i = 0; i < portalsGrid.size(); ++i)
            {
                portalsGrid[i].positionInPortal = {};// positionInPortal;
            }
        }
        else
        {
            // magGrid have some data yet
        }

        status = buildPortalsDataGrid(venueId, grid, &positionsGrid, portalsGrid);

        return status;
    }


ReturnStatus GridBuilderPrivate::buildPositionGrid( const venue_id &venueId, const Grid &grid, PositionsGrid & positionsGrid )
{
    // calculation of grid  coordinates
    double cell_size = grid.size; // grid.size parameter is actually the size of cell

    uint32_t cellXQuantity = (uint32_t)ceil(mVenue.size_x / cell_size);
    uint32_t cellYQuantity = (uint32_t)ceil(mVenue.size_y / cell_size);
    uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;

    positionsGrid.resize((std::vector<int>::size_type)(mVenue.floors_count * gridCellsQuantity));

    for ( int16_t currentFloor = 0; currentFloor < mVenue.floors_count; currentFloor++ )
    {
        for ( uint32_t x_pos = 0; x_pos < cellXQuantity; ++x_pos )
        {
            for ( uint32_t y_pos = 0; y_pos < cellYQuantity; ++y_pos )
            {
                Fpbl::CoordinatesInGrid coordinates = { cell_size / 2 + x_pos * cell_size, cell_size / 2 + y_pos * cell_size, currentFloor };
                positionsGrid[(std::vector<int>::size_type)(y_pos + x_pos * cellYQuantity + currentFloor * gridCellsQuantity)].coordinates = coordinates;
            }
        }
    }

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

// private method
ReturnStatus GridBuilderPrivate::buildWiFiDataGrid(const venue_id &venueId, const Grid &grid, PositionsGrid * positionsGrid, WiFiGrid & wifiGrid)
{
    size_t wifiMeasurementTimestampsSize = mWiFiScanResultTimestamps.size();
    int64_t positionInGrid = -1;
    uint64_t wifiMeasurementTimestamp;
    WiFiScanResult wifiScanResult;

    PositionExt devicePosition;

    double cell_size = grid.size; // grid.size parameter is actually the size of cell
    uint32_t cellXQuantity = (uint32_t)ceil(mVenue.size_x / cell_size);
    uint32_t cellYQuantity = (uint32_t)ceil(mVenue.size_y / cell_size);
    uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;

    if (mDevicePositions.size() == 0)
        return Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;

    std::vector<PositionExt>::iterator   pos_it_cur = mDevicePositions.begin();

    for ( size_t i = 0; i < wifiMeasurementTimestampsSize; i++ )
    {
        wifiMeasurementTimestamp = mWiFiScanResultTimestamps[i];
        wifiScanResult = mWiFiScanResults[i];

        findNearestPosition( venueId, wifiMeasurementTimestamp, 1e6, devicePosition, pos_it_cur );

        double pos_unc = sqrt(devicePosition.position.covarianceXY[0][0] + devicePosition.position.covarianceXY[1][1]);
        uint64_t timestampDifference = abs((int64_t)(devicePosition.timestamp - wifiMeasurementTimestamp));

        if ((pos_unc <= max_uncertainty_wifi) && (timestampDifference <= maxWifiMeasTimeTolerance))
        {
            int32_t x_pos = (int32_t)(devicePosition.position.x / mVenue.size_x * (mVenue.size_x / cell_size));
            int32_t y_pos = (int32_t)(devicePosition.position.y / mVenue.size_y * (mVenue.size_y / cell_size));
            int16_t floorNumber = devicePosition.position.floorNumber;

            positionInGrid = y_pos + x_pos * cellYQuantity + floorNumber * gridCellsQuantity;

            if (positionInGrid < 0 || positionInGrid > wifiGrid.size())
                continue;

            wifiGrid[(std::vector<int>::size_type)positionInGrid].wifiData.push_back(wifiScanResult);
        }
    }

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

// private method
ReturnStatus GridBuilderPrivate::buildBleDataGrid( const venue_id &venueId, const Grid &grid, PositionsGrid * positionsGrid, BleGrid & bleGrid )
{
    size_t bleMeasurementTimestampsSize = mBleScanResultTimestamps.size();
    int64_t positionInGrid = -1;
    BleScanResult bleScanResult;

    PositionExt devicePosition;

    double cell_size = grid.size; // grid.size parameter is actually the size of cell
    uint32_t cellXQuantity = (uint32_t)ceil(mVenue.size_x / cell_size);
    uint32_t cellYQuantity = (uint32_t)ceil(mVenue.size_y / cell_size);
    uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;

    if (mDevicePositions.size() == 0 )
        return Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;

    std::vector<PositionExt>::iterator   pos_it_cur = mDevicePositions.begin();

    for ( size_t i = 0; i < bleMeasurementTimestampsSize; i++ )
    {
        uint64_t bleMeasurementTimestamp = mBleScanResultTimestamps[i];
        bleScanResult = mBleScanResults[i];

        findNearestPosition( venueId, bleMeasurementTimestamp, 1e6, devicePosition, pos_it_cur );

        double pos_unc = sqrt(devicePosition.position.covarianceXY[0][0] + devicePosition.position.covarianceXY[1][1]);
        uint64_t timestampDifference = abs((int64_t)(devicePosition.timestamp - bleMeasurementTimestamp));

        if ((pos_unc <= max_uncertainty_ble) && (timestampDifference <= maxBleMeasTimeTolerance))
        {
            int32_t x_pos = (int32_t)(devicePosition.position.x / mVenue.size_x * (mVenue.size_x / cell_size));
            int32_t y_pos = (int32_t)(devicePosition.position.y / mVenue.size_y * (mVenue.size_y / cell_size));
            int16_t floorNumber = devicePosition.position.floorNumber;

            positionInGrid = y_pos + x_pos * cellYQuantity + floorNumber * gridCellsQuantity;

            if (positionInGrid < 0 || positionInGrid > bleGrid.size())
                continue;

            bleGrid[(std::vector<int>::size_type)positionInGrid].bleData.push_back(bleScanResult);
        }
    }

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

ReturnStatus GridBuilderPrivate::buildMfpDataGrid( const venue_id &venueId, const Grid &grid, PositionsGrid * positionsGrid, MagneticGrid & magGrid )
{
    size_t magVectorTimestampsSize = mMagneticVectors.size();
    int64_t positionInGrid = -1;
    MffAttitude deviceAttitude;
    PositionExt devicePosition;

    double cell_size = grid.size; // grid.size parameter is actually the size of cell
    uint32_t cellXQuantity = (uint32_t)ceil(mVenue.size_x / cell_size);
    uint32_t cellYQuantity = (uint32_t)ceil(mVenue.size_y / cell_size);
    uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;

    if (mDevicePositions.size() == 0 || mDeviceAttitudes.size() == 0)
        return Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;

    std::vector<MffAttitude>::iterator   att_it_cur = mDeviceAttitudes.begin();
    std::vector<PositionExt>::iterator   pos_it_cur = mDevicePositions.begin();

    for ( size_t i = 0; i < magVectorTimestampsSize; i++ )
    {
        uint64_t magMeasurementTimestamp = mMagneticVectors[i].timestamp;
        MagneticMeasurementAndDataSetType magMeasurement = mMagneticVectors[i];

        findNearestPosition( venueId, magMeasurementTimestamp, 1e6, devicePosition, pos_it_cur );

        double pos_unc = sqrt(devicePosition.position.covarianceXY[0][0] + devicePosition.position.covarianceXY[1][1]);
        uint64_t timestampDifference = abs((int64_t)(devicePosition.timestamp - magMeasurementTimestamp));

        if ((pos_unc <= max_uncertainty_mfp) && (timestampDifference <= maxMagMeasTimeTolerance))
        {
            int32_t x_pos = (int32_t)(devicePosition.position.x / mVenue.size_x * (mVenue.size_x / cell_size));
            int32_t y_pos = (int32_t)(devicePosition.position.y / mVenue.size_y * (mVenue.size_y / cell_size));
            int16_t floorNumber = devicePosition.position.floorNumber;

            positionInGrid = y_pos + x_pos * cellYQuantity + floorNumber * gridCellsQuantity;

            if (positionInGrid < 0 || positionInGrid > magGrid.size())
                continue;

            findNearestAttitude(venueId, magMeasurementTimestamp, deviceAttitude, att_it_cur);
            timestampDifference = abs((int64_t)(deviceAttitude.timestamp - magMeasurementTimestamp));

            if (timestampDifference <= maxAttitudeMeasTimeTolerance)
            {
                auto mag_data = calculateUncertainties(magMeasurement, deviceAttitude);

                magGrid[(std::vector<int>::size_type)positionInGrid].magData.push_back(mag_data);
            }
        }
    }

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

ReturnStatus GridBuilderPrivate::buildPortalsDataGrid(const venue_id &venueId, const Grid &grid, PositionsGrid * positionsGrid, PortalsGrid & portalsGrid)
{
    size_t portalPositionsSize = mPortalPositions.size();
    if (portalPositionsSize == 0 )
        return Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;

    int64_t positionInGrid = -1;

    double cell_size = grid.size; // grid.size parameter is actually the size of cell
    uint32_t cellXQuantity = (uint32_t)ceil(mVenue.size_x / cell_size);
    uint32_t cellYQuantity = (uint32_t)ceil(mVenue.size_y / cell_size);
    uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;

    for (size_t i = 0; i < portalPositionsSize; i++)
    {
        FfPosition portalPosition = mPortalPositions[i];

        int32_t x_pos = (int32_t)(portalPosition.x / mVenue.size_x * (mVenue.size_x / cell_size));
        int32_t y_pos = (int32_t)(portalPosition.y / mVenue.size_y * (mVenue.size_y / cell_size));
        int16_t floorNumber = portalPosition.floor_number;

        positionInGrid = y_pos + x_pos * cellYQuantity + floorNumber * gridCellsQuantity;
        //if (portalPosition.altitude_std > 0)
        {
            //std::cout << "x = " << portalPosition.x << " , y = " << portalPosition.y << " , floorNumber = " << floorNumber << " , mode_of_transit = " << (int)portalPosition.mode_of_transit << " , altitude = " << portalPosition.altitude << " , positionInGrid = " << positionInGrid << std::endl;
            //std::cout << "x_pos = " << x_pos << " , y_pos = " << y_pos << " , floorNumber = " << floorNumber << " , altitude = " << portalPosition.altitude << " , positionInGrid = " << positionInGrid << std::endl;
        }
        if (positionInGrid < 0 || positionInGrid > portalsGrid.size())
            continue;

        PositionsCell positionsCell = positionsGrid->at(positionInGrid);
        portalPosition.x = positionsCell.coordinates.x;
        portalPosition.y = positionsCell.coordinates.y;
        portalPosition.floor_number = positionsCell.coordinates.floor;
        portalPosition.is_valid = true;

        portalsGrid[(std::vector<int>::size_type)positionInGrid].resultsInPortal.push_back(portalPosition);
        portalsGrid[(std::vector<int>::size_type)positionInGrid].positionInPortal.is_valid = true;
        portalsGrid[(std::vector<int>::size_type)positionInGrid].positionInPortal.mode_of_transit = portalPosition.mode_of_transit;
    }

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}


ReturnStatus GridBuilderPrivate::findNearestPosition( const venue_id &venueId,
        uint64_t measurementTimestamp, double uncertainty, PositionExt &devicePosition, std::vector<PositionExt>::iterator   &pos_it_cur )
{
    //uint64_t   minTimestampDifference = UINT64_MAX;
    int64_t   currentTimestampDifference = UINT64_MAX;
    int64_t   nextTimestampDifference = UINT64_MAX;
    PositionExt   nearestDevicePosition = devicePosition;

    for ( auto pos_it = pos_it_cur; pos_it != mDevicePositions.end(); ++pos_it )
    {
        //double unc = sqrt( pos_it->position.covarianceXY[0][0] + pos_it->position.covarianceXY[1][1] );
        //double unc = std::max( sqrt(pos_it->position.covarianceXY[0][0]), sqrt(pos_it->position.covarianceXY[1][1]) );

        // TODO: add different condition here (sqrt from (sigX^2 + sigY^2))
        // also - change MAX_UNCERTAINTY_WIFI in self-healing mode to 5.0 meters

        //double unc = sqrt(pos_it->position.covarianceXY[0][0] * pos_it->position.covarianceXY[0][0] + pos_it->position.covarianceXY[1][1] * pos_it->position.covarianceXY[1][1]);
        double unc = sqrt(pos_it->position.covarianceXY[0][0] + pos_it->position.covarianceXY[1][1]);
        
        if (unc > uncertainty)
            continue;

        auto pos_it_next = pos_it;
        pos_it_next++;

        if ( pos_it_next == mDevicePositions.end() )
        {
            nearestDevicePosition = *pos_it;
            break;
        }

        uint64_t positionTimestamp = pos_it->timestamp;
        uint64_t positionTimestampNext = pos_it_next->timestamp;

        currentTimestampDifference =
            ( int64_t )( positionTimestamp - measurementTimestamp );
        nextTimestampDifference =
            ( int64_t )( positionTimestampNext - measurementTimestamp );

        if ( currentTimestampDifference > 0 )
        {
            nearestDevicePosition = *pos_it;
            break;
        }

        if ( currentTimestampDifference <= 0 && nextTimestampDifference >= 0 ) // between positions
        {
            if ( abs( currentTimestampDifference ) < abs( nextTimestampDifference ) )
            {
                nearestDevicePosition = *pos_it;
                break;
            }
            else
            {
                nearestDevicePosition = *pos_it_next;

                pos_it_cur = pos_it_next;
                break;
            }

        }
    }

    devicePosition = nearestDevicePosition;
    //std::cout << devicePosition.timestamp << std::endl;

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}


ReturnStatus GridBuilderPrivate::findNearestAttitude( const venue_id &venueId,
        uint64_t measurementTimestamp, MffAttitude &deviceAttitude, std::vector<MffAttitude>::iterator   &att_it_cur )
{
    //uint64_t   minTimestampDifference = UINT64_MAX;
    int64_t   currentTimestampDifference = UINT64_MAX;
    int64_t   nextTimestampDifference = UINT64_MAX;
    MffAttitude   nearestdeviceAttitude = deviceAttitude;

    for ( auto att_it = att_it_cur;
            att_it != mDeviceAttitudes.end(); ++att_it )

    {
        auto att_it_next = att_it;
        att_it_next++;

        if ( att_it_next == mDeviceAttitudes.end() )
        {
            nearestdeviceAttitude = *att_it;
            break;
        }

        uint64_t attitudeTimestamp = att_it->timestamp;
        uint64_t attitudeTimestampNext = att_it_next->timestamp;

        currentTimestampDifference =
            ( int64_t )( attitudeTimestamp - measurementTimestamp );
        nextTimestampDifference =
            ( int64_t )( attitudeTimestampNext - measurementTimestamp );

        if ( currentTimestampDifference > 0 )
        {
            nearestdeviceAttitude = *att_it;
            break;
        }

        if ( currentTimestampDifference <= 0 && nextTimestampDifference >= 0 ) // between attidudes
        {
            if ( abs( currentTimestampDifference ) < abs( nextTimestampDifference ) )
            {
                nearestdeviceAttitude = *att_it;
                break;
            }
            else
            {
                nearestdeviceAttitude = *att_it_next;

                att_it_cur = att_it_next;
                break;
            }

        }
    }

    deviceAttitude = nearestdeviceAttitude;

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}



ReturnStatus GridBuilderPrivate::rotateMagneticVector( MagneticVector &magVector, const MffAttitude &deviceAttitude )
{
    Quaternion first, second, result, resulttmp;

    first.w = deviceAttitude.quaternion[0];
    first.x = deviceAttitude.quaternion[1];
    first.y = deviceAttitude.quaternion[2];
    first.z = deviceAttitude.quaternion[3];

    second.w = 0;
    second.x = magVector.mX;
    second.y = magVector.mY;
    second.z = magVector.mZ;

    //  quaternion by magVector  multiplication
    quaternionsMultiplication( first,  second,  resulttmp );

    // quaternion conjugation
    first.x = -first.x;
    first.y = -first.y;
    first.z = -first.z;

    //  quaternion resulttmp by quaternion conjugation multiplication
    quaternionsMultiplication( resulttmp, first, result );

    magVector.mX = result.x;
    magVector.mY = result.y;
    magVector.mZ = result.z;

    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

MagneticMeasurementAndDataSetType GridBuilderPrivate::calculateUncertainties(MagneticMeasurementAndDataSetType &magMeasurement, const MffAttitude &deviceAttitude )
{
    MagneticMeasurementAndDataSetType result = magMeasurement;

    //P1
    Eigen::Matrix3d P1 = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d P2 = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d P3 = Eigen::Matrix3d::Zero();

    double s2_phi   = deviceAttitude.roll_std * deviceAttitude.roll_std;
    double s2_tetta = deviceAttitude.pitch_std * deviceAttitude.pitch_std;
    double s2_psi   = deviceAttitude.heading_std * deviceAttitude.heading_std;

    double m_x = magMeasurement.mX;
    double m_y = magMeasurement.mY;
    double m_z = magMeasurement.mZ;
    double m2_x = m_x * m_x;
    double m2_y = m_y * m_y;
    double m2_z = m_z * m_z;

    P1( 0, 0 ) =  s2_psi * m2_y + s2_tetta * m2_z;   P1( 0, 1 ) = -s2_psi * m_x * m_y;              P1( 0, 2 ) = -s2_tetta * m_x * m_z;
    P1( 1, 0 ) = -s2_psi * m_x * m_y;                P1( 1, 1 ) =  s2_psi * m2_x + s2_phi * m2_z;   P1( 1, 2 ) = -s2_phi * m_y * m_z;
    P1( 2, 0 ) = -s2_tetta * m_x * m_z;              P1( 2, 1 ) = -s2_phi * m_y * m_z;              P1( 2, 2 ) =  s2_phi * m2_y + s2_tetta * m2_x;

    //P2
    //P2( 0, 0 ) = magMeasurement.sigma_mX * magMeasurement.sigma_mX * ( 1 + s2_tetta + s2_psi );
    //P2( 1, 1 ) = magMeasurement.sigma_mY * magMeasurement.sigma_mY * ( 1 + s2_phi   + s2_psi );
    //P2( 2, 2 ) = magMeasurement.sigma_mZ * magMeasurement.sigma_mZ * ( 1 + s2_tetta + s2_phi );

    P2( 0, 0 ) = magMeasurement.sigma_mX * magMeasurement.sigma_mX;
    P2( 1, 1 ) = magMeasurement.sigma_mY * magMeasurement.sigma_mY;
    P2( 2, 2 ) = magMeasurement.sigma_mZ * magMeasurement.sigma_mZ;

    //P3
    for ( auto i = 0; i < 3; ++i )
    {
        for ( auto j = 0; j < 3; ++j )
        {
            P3( i, j ) = magMeasurement.covarianceMatrix[i][j];
        }
    }

    auto q = deviceAttitude.quaternion;

    //std::cout << P1 << std::endl;
    //std::cout << P2 << std::endl;
    //std::cout << P3 << std::endl;

    Eigen::Matrix3d R = Eigen::Quaternion<double>( q[0], q[1], q[2], q[3] ).matrix();

    //std::cout << R << std::endl;

    Eigen::Matrix3d COV = R * ( P1 + P2 + P3 ) * R.transpose();

    //std::cout << "COV " << COV << std::endl;
    //result
    for ( auto i = 0; i < 3; ++i )
    {
        for ( auto j = 0; j < 3; ++j )
        {
            result.covarianceMatrix[i][j] = COV( i, j );
        }
    }

    rotateMagneticVector( result, deviceAttitude );

    return result;
}


void GridBuilderPrivate::quaternionsMultiplication( Quaternion first, Quaternion second, Quaternion &result )
{
    double w1 = first.w;
    double x1 = first.x;
    double y1 = first.y;
    double z1 = first.z;

    double w2 = second.w;
    double x2 = second.x;
    double y2 = second.y;
    double z2 = second.z;

    double w3 = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    double x3 = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    double y3 = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    double z3 = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

    result.w = w3;
    result.x = x3;
    result.y = y3;
    result.z = z3;
}

/////////////////////////////
ReturnStatus GridBuilderPrivate::clearDevicePositionsAndTimestamps()
{
    //mDevicePositionTimestamps.clear();
    mDevicePositions.clear();
    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

ReturnStatus GridBuilderPrivate::createEmptyGridMFP(const venue_id &venueId, const Grid &grid, MagneticGrid & magGrid)
{
    PositionsGrid positionsGrid;
    ReturnStatus status = buildPositionGrid(venueId, grid, positionsGrid);

    magGrid.clear();

    magGrid.resize(positionsGrid.size());

    for (size_t i = 0; i < magGrid.size(); ++i)
    {
        magGrid[i].coordinates = GridDataExtended(positionsGrid[i].coordinates);
    }
    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

ReturnStatus GridBuilderPrivate::createEmptyGridWiFi(const venue_id &venueId, const Grid &grid, WiFiGrid & wifiGrid)
{
    PositionsGrid positionsGrid;
    ReturnStatus status = buildPositionGrid(venueId, grid, positionsGrid);

    wifiGrid.clear();

    wifiGrid.resize(positionsGrid.size());

    for (size_t i = 0; i < wifiGrid.size(); ++i)
    {
        wifiGrid[i].coordinates = positionsGrid[i].coordinates;
    }
    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

ReturnStatus GridBuilderPrivate::createEmptyGridBLE(const venue_id &venueId, const Grid &grid, BleGrid & bleGrid)
{
    PositionsGrid positionsGrid;
    ReturnStatus status = buildPositionGrid(venueId, grid, positionsGrid);

    bleGrid.clear();

    bleGrid.resize(positionsGrid.size());

    for (size_t i = 0; i < bleGrid.size(); ++i)
    {
        bleGrid[i].coordinates = positionsGrid[i].coordinates;
    }
    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

ReturnStatus GridBuilderPrivate::createEmptyGridPortals(const venue_id &venueId, const Grid &grid, PortalsGrid & portalsGrid)
{
    PositionsGrid positionsGrid;
    ReturnStatus status = buildPositionGrid(venueId, grid, positionsGrid);

    portalsGrid.clear();

    portalsGrid.resize(positionsGrid.size());

    for (size_t i = 0; i < portalsGrid.size(); ++i)
    {
        portalsGrid[i].positionInPortal.x = positionsGrid[i].coordinates.x;
        portalsGrid[i].positionInPortal.y = positionsGrid[i].coordinates.y;
        portalsGrid[i].positionInPortal.floor = positionsGrid[i].coordinates.floor;
        portalsGrid[i].positionInPortal.altitude = 0;
        portalsGrid[i].positionInPortal.is_crowdsourced = false;
        portalsGrid[i].positionInPortal.mag_x = 0.0;
        portalsGrid[i].positionInPortal.mag_y = 0.0;
        portalsGrid[i].positionInPortal.mag_z = 0.0;

        //portalsGrid[i].numberDataInPortalsPosition = 0;
    }
    return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

ReturnStatus GridBuilderPrivate::createEmptyGridCS(const venue_id &venueId, const Grid &grid, CSGrid & csGrid)
{
	PositionsGrid positionsGrid;
	ReturnStatus status = buildPositionGrid(venueId, grid, positionsGrid);

	csGrid.clear();

	csGrid.resize(positionsGrid.size());

	for (size_t i = 0; i < csGrid.size(); ++i)
	{
		csGrid[i].position.x = positionsGrid[i].coordinates.x;
		csGrid[i].position.y = positionsGrid[i].coordinates.y;
		csGrid[i].position.floor = positionsGrid[i].coordinates.floor;
		csGrid[i].position.altitude = 0;
		csGrid[i].position.is_crowdsourced = false;
		csGrid[i].position.mag_x = 0.0;
		csGrid[i].position.mag_y = 0.0;
		csGrid[i].position.mag_z = 0.0;

	}
	return Fpbl::ReturnStatus::STATUS_SUCCESS;
}

#if 0
void FP_builder::sortWiFiGrid(Fpbl::WiFiGrid &wifiGrid)
{
    uint32_t i, j, k;

    for (i = 0; i < wifiGrid.size(); i++)
    {
        if (wifiGrid[i].wifiData.size() > 0)
        {
            std::stable_sort(wifiGrid[i].wifiData.begin(), wifiGrid[i].wifiData.end(),
                [](const WiFiScanResult &a, const WiFiScanResult &b)
            {
                return (a.timestamp < b.timestamp);
            });

            std::vector<WiFiMeasurement> data_in_cell;
            size_t  data_in_cell_count = 0;
            for (j = 0; j < wifiGrid[i].wifiData.size(); j++)
            {
                data_in_cell_count += wifiGrid[i].wifiData[j].scanWiFi.size();
                for (k = 0; k < wifiGrid[i].wifiData[j].scanWiFi.size(); k++)
                {
                    data_in_cell.push_back(wifiGrid[i].wifiData[j].scanWiFi[k]);
                }
            }
            std::stable_sort(data_in_cell.begin(), data_in_cell.end(),
                [](const WiFiMeasurement &a, const WiFiMeasurement &b)
            {
                return (a.timestamp < b.timestamp);
            });
            std::stable_sort(data_in_cell.begin(), data_in_cell.end(),
                [](const WiFiMeasurement &a, const WiFiMeasurement &b)
            {
                if (a.timestamp == b.timestamp)
                {
                    return (a.mac < b.mac);
                }
                else
                {
                    return false;
                }
            });
            std::stable_sort(data_in_cell.begin(), data_in_cell.end(),
                [](const WiFiMeasurement &a, const WiFiMeasurement &b)
            {
                if ((a.timestamp == b.timestamp) && (a.mac == b.mac))
                {
                    return (a.rssi < b.rssi);
                }
                else
                {
                    return false;
                }
            });
            int n = 0;
            int64_t t = 0;
            {
                wifiGrid[i].wifiData.clear();
                while (n < data_in_cell.size())
                {
                    WiFiScanResult wifiScanResult;
                    t = data_in_cell[n].timestamp;
                    while (t == data_in_cell[n].timestamp)
                    {
                        wifiScanResult.scanWiFi.push_back(data_in_cell[n]);
                        n++;
                        if (n >= data_in_cell.size())
                            break;
                    }
                    if (wifiScanResult.scanWiFi.size() > 0)
                    {
                        wifiScanResult.timestamp = wifiScanResult.scanWiFi[0].timestamp;
                        wifiGrid[i].wifiData.push_back(wifiScanResult);
                    }
                }
            }
        }
    }
}
#endif

#if 0
void FP_builder::sortBleGrid(Fpbl::BleGrid &bleGrid)
{
    uint32_t i, j, k;

    for (i = 0; i < bleGrid.size(); i++)
    {
        if (bleGrid[i].bleData.size() > 0)
        {
            std::vector<BleMeasurement> data_in_cell;
            size_t  data_in_cell_count = 0;
            for (j = 0; j < bleGrid[i].bleData.size(); j++)
            {
                data_in_cell_count += bleGrid[i].bleData[j].scanBle.size();
                for (k = 0; k < bleGrid[i].bleData[j].scanBle.size(); k++)
                {
                    data_in_cell.push_back(bleGrid[i].bleData[j].scanBle[k]);
                }
            }
            std::stable_sort(data_in_cell.begin(), data_in_cell.end(),
                [](const BleMeasurement &a, const BleMeasurement &b)
            {
                return (a.timestamp < b.timestamp);
            });
            std::stable_sort(data_in_cell.begin(), data_in_cell.end(),
                [](const BleMeasurement &a, const BleMeasurement &b)
            {
                if (a.timestamp == b.timestamp)
                {
                    return (a.mac < b.mac);
                }
                else
                {
                    return false;
                }
            });
            std::stable_sort(data_in_cell.begin(), data_in_cell.end(),
                [](const BleMeasurement &a, const BleMeasurement &b)
            {
                if ((a.timestamp == b.timestamp) && (a.mac == b.mac))
                {
                    return (a.rssi < b.rssi);
                }
                else
                {
                    return false;
                }
            });

            int n = 0;
            int64_t t = 0;
            {
                bleGrid[i].bleData.clear();
                while (n < data_in_cell.size())
                {
                    BleScanResult bleScanResult;
                    t = data_in_cell[n].timestamp;
                    while (t == data_in_cell[n].timestamp)
                    {
                        bleScanResult.scanBle.push_back(data_in_cell[n]);
                        n++;
                        if (n >= data_in_cell.size())
                            break;
                    }
                    if (bleScanResult.scanBle.size() > 0)
                    {
                        bleScanResult.timestamp = bleScanResult.scanBle[0].timestamp;
                        bleGrid[i].bleData.push_back(bleScanResult);
                    }
                }
            }


        }
    }
}
#endif

#if 0
void FP_builder::sortMagneticGrid(Fpbl::MagneticGrid &magneticGrid)
{
    for (size_t i = 0; i < magneticGrid.size(); i++)
    {
        if (magneticGrid[i].magData.size() > 0)
        {
            std::stable_sort(magneticGrid[i].magData.begin(), magneticGrid[i].magData.end(),
                [](const MagneticData &a, const MagneticData &b)
            {
                return (a.timestamp < b.timestamp);
            });

            std::stable_sort(magneticGrid[i].magData.begin(), magneticGrid[i].magData.end(),
                [](const MagneticData &a, const MagneticData &b)
            {
                if (a.timestamp == b.timestamp)
                {
                    return (a.mX < b.mX);
                }
                else
                {
                    return false;
                }
            });

        }
    }
}
#endif

int32_t GridBuilderPrivate::getNeededGridSize(const venue_id &venueId, const Grid &grid)
{
    PositionsGrid positionsGrid;
    ReturnStatus status = buildPositionGrid(venueId, grid, positionsGrid);
    return positionsGrid.size();
}

//     getting methods for testeng only
std::string GridBuilderPrivate::getTemporaryFolder()
{
    return mTemporaryFolder;
}
uint64_t GridBuilderPrivate::getDevicePositionTimestamp( const uint32_t number )
{
    return mDevicePositions[number].timestamp;
}
Position GridBuilderPrivate::getDevicePosition( const uint32_t number )
{
    return mDevicePositions[number].position;
}
uint64_t GridBuilderPrivate::getDeviceAttitudeTimestamp( const uint32_t number )
{
    return mDeviceAttitudes[number].timestamp;
}
MffAttitude GridBuilderPrivate::getDeviceAttitude( const uint32_t number )
{
    return mDeviceAttitudes[number];
}

uint64_t GridBuilderPrivate::getWiFiMeasurementTimestamp( const uint32_t number )
{
    return mWiFiScanResultTimestamps[number];
}
WiFiScanResult GridBuilderPrivate::getWiFiMeasurement( const uint32_t number )
{
    return mWiFiScanResults[number];
}

uint64_t GridBuilderPrivate::getBleMeasurementTimestamp( const uint32_t number )
{
    return mBleScanResultTimestamps[number];
}
BleScanResult GridBuilderPrivate::getBleMeasurement( const uint32_t number )
{
    return mBleScanResults[number];
}

uint64_t GridBuilderPrivate::getMFPMeasurementTimestamp( const uint32_t number )
{
    return mMagneticVectors[number].timestamp;
}
MagneticVector GridBuilderPrivate::getMFPMeasurement( const uint32_t number )
{
    return mMagneticVectors[number];
}

void GridBuilderPrivate::set_max_uncertainty_wifi(double wifi_max_unc)
{
    this->max_uncertainty_wifi = wifi_max_unc;
}

void GridBuilderPrivate::set_max_uncertainty_ble(double ble_max_unc)
{
    this->max_uncertainty_ble = ble_max_unc;
}

void GridBuilderPrivate::set_max_uncertainty_mfp(double mfp_max_unc)
{
    this->max_uncertainty_mfp = mfp_max_unc;
}

GridBuilderPrivate::GridBuilderPrivate( const GridBuilderPrivate & ) //disable copy constructor due pointer to implementation
{
}

const GridBuilderPrivate &GridBuilderPrivate::operator=( const GridBuilderPrivate &iGridBuilder ) // disable copy
{
    return iGridBuilder;
}
};

