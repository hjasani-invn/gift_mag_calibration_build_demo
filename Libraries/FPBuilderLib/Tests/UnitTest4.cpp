#include "stdafx.h"
#include "CppUnitTest.h"

#include <direct.h>

#include "GridBuilderPrivate.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace FPBuilderLibTests
{
	TEST_CLASS(UnitTest4)
	{
	public:
        //Fpbl::GridBuilderPrivate *pStaticBuilder = new Fpbl::GridBuilderPrivate();
		
		TEST_METHOD(TestMethod1) // test for building wifi grid
		{
			// TODO: Your test code here
            Fpbl::GridBuilderPrivate *pStaticBuilder = new Fpbl::GridBuilderPrivate();

            Fpbl::Venue venue;

            // 55.7352576, 37.6419266 Moscow Avrora Bisness Centre

            venue.id = 0;   /**< venue identifier */
            venue.originLattitude = 0;     /**< bottom left corner lattitude [rad] [-pi/2..pi/2] */
            venue.originLongitude = 0;     /**< bottom left corner longitude [rad] [-pi..pi] */
            venue.originAltitude = 0;      /**< zero floor altitude [m] */
            venue.originAzimuth = 0;       /**< venue y-axix rotation to true north [rad] [0..pi] */
            venue.floorsCount = 0;       /**< total floor number in venue */
            venue.sizeX = 100;               /**< x axis venue size [m] */
            venue.sizeY = 20;               /**< y axis venue size [m] */

            pStaticBuilder->setVenue(venue);

            Fpbl::Grid grid = { Fpbl::CellType::CELL_SQUARE, 5.0 };
            Fpbl::WiFiGrid wifiGrid;

            Fpbl::ReturnStatus status = pStaticBuilder->updateGridWiFi(venue.id, grid, &wifiGrid);

            int n = venue.sizeX / grid.size;
            int m = venue.sizeY / grid.size;

            Assert::AreEqual((uint32_t)wifiGrid.size(), (uint32_t)n * m);


            char current_work_dir[1000];
#ifdef _WIN32
            //GetCurrentDirectory(1000, current_work_dir);
            _getcwd(current_work_dir, sizeof(current_work_dir));
#else
            getcwd(current_work_dir, sizeof(current_work_dir));
#endif
            const std::string folderPath(current_work_dir);

            const std::string logFimeName = folderPath + "\\wifiposgrid_log.txt";
            std::fstream  wifigrid_log(logFimeName.c_str(), std::ios::out);

            wifigrid_log << logFimeName << std::endl;
            wifigrid_log << std::right << wifiGrid.size() << std::endl;

            int width = 15;
            wifigrid_log.width(width);


            for (int i = 0; i < wifiGrid.size(); ++i)
            {
                wifigrid_log.width(width);
                wifigrid_log << wifiGrid[i].coordinates.x;
                wifigrid_log.width(width);
                wifigrid_log << wifiGrid[i].coordinates.y;
                wifigrid_log.width(width);
                wifigrid_log << wifiGrid[i].coordinates.floor
                    << std::endl;
            }


		}

        TEST_METHOD(TestMethod2) // test for building wifi grid
        {
            Fpbl::GridBuilderPrivate *pStaticBuilder = new Fpbl::GridBuilderPrivate();

            char current_work_dir[1000];
#ifdef _WIN32
            //GetCurrentDirectory(1000, current_work_dir);
            _getcwd(current_work_dir, sizeof(current_work_dir));
#else
            getcwd(current_work_dir, sizeof(current_work_dir));
#endif
            const std::string folderPath(current_work_dir);

            const std::string logFimeName = folderPath + "\\wifidatagrid_log.txt";
            std::fstream  wifidatagrid_log(logFimeName.c_str(), std::ios::out);

            wifidatagrid_log << logFimeName << std::endl;
            wifidatagrid_log << std::right << std::endl;

            int width = 15;



            Fpbl::Venue venue;

            // 55.7352576, 37.6419266 Moscow Avrora Bisness Centre

            venue.id = 0;   /**< venue identifier */
            venue.originLattitude = 0;     /**< bottom left corner lattitude [rad] [-pi/2..pi/2] */
            venue.originLongitude = 0;     /**< bottom left corner longitude [rad] [-pi..pi] */
            venue.originAltitude = 0;      /**< zero floor altitude [m] */
            venue.originAzimuth = 0;       /**< venue y-axix rotation to true north [rad] [0..pi] */
            venue.floorsCount = 0;       /**< total floor number in venue */
            venue.sizeX = 100;               /**< x axis venue size [m] */
            venue.sizeY = 20;               /**< y axis venue size [m] */

            pStaticBuilder->setVenue(venue);

            Fpbl::Grid grid = { Fpbl::CellType::CELL_SQUARE, 5.0 };
            Fpbl::WiFiGrid wifiGrid;



            // TODO: Your test code here
            Fpbl::Position devicePosition;
            uint64_t timestamp;

            std::vector<uint64_t> posMeasurementTimestamps;
            std::vector<Fpbl::Position> posMeasurements;

            int N = 20 * 2;
            int M = 4 * 2;

            srand(1000);
            //srand(time(NULL));

            timestamp = 1000000; // us

            devicePosition.lattitude = 5.0 / 4;       /**< lattitude [rad] [-pi/2..pi/2]*/
            devicePosition.longitude = 5.0 / 4;       /**< longitude [rad] [-pi..pi] */
            devicePosition.altitude = 0;        /**< altitude (sea level?) [m] */

            devicePosition.floorNumber = 0;    /**< discrete floor number [0..32767], must be positive or zero */
            devicePosition.covarianceLatLon[0][0] = 0;
            devicePosition.covarianceLatLon[0][1] = 0;
            devicePosition.covarianceLatLon[1][0] = 0;
            devicePosition.covarianceLatLon[1][1] = 0;
            /** Lattitude/Longitude covariance matrix in column order [rad^2]\n
            *  cov(lat,lat), cov(lon,lat)\n
            *  cov(lat,lon), cov(lon,lon) */
            devicePosition.altitudeStd = 1;     /**< altitude standard deviation [m]     */
            devicePosition.floorStd = 1;        /**< altitude standard deviation [floor] */
            devicePosition.isValid = true;           /**< position validity flag */

            for (int m = 0; m < M; m++)
            {
                for (int n = 0; n < N; n++)
                {
                    wifidatagrid_log.width(width);
                    wifidatagrid_log << timestamp;
                    wifidatagrid_log.width(width);
                    wifidatagrid_log << devicePosition.lattitude;
                    wifidatagrid_log.width(width);
                    wifidatagrid_log << devicePosition.longitude;
                    wifidatagrid_log << std::endl;

                    posMeasurementTimestamps.push_back(timestamp);
                    posMeasurements.push_back(devicePosition);

                    pStaticBuilder->processDevicePosition(timestamp, devicePosition);

                    timestamp += 1000000; // us
                    if (m % 2 == 0)
                        devicePosition.lattitude += 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
                    else
                        devicePosition.lattitude -= 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/

                }

                if (m % 2 == 0)
                    devicePosition.lattitude -= 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
                else
                    devicePosition.lattitude += 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
                devicePosition.longitude += 5.0 / 2;       /**< longitude [rad] [-pi..pi] */

            }


            // Wifi

            wifidatagrid_log << "Wifi====================================";
            wifidatagrid_log << std::endl;


            Fpbl::WiFiMeasurement wifi;

            std::vector<uint64_t> wiFiMeasurementTimestamps;

            N = 20 * 2;
            M = 4;

            timestamp = 1500000;
            wifi.timestamp = timestamp; /**< unix time [us] */
            wifi.mac = 0;       /**< MAC addres in decimal form */
            wifi.rssi = 0;        /**< RSSI value [dbm] */
            wifi.frequency = 2500; /**< central channel frequency [MHz] */


            for (int m = 0; m < M; m++)
            {
                for (int n = 0; n < N; n++)
                {
                    wifidatagrid_log.width(width);
                    wifidatagrid_log << timestamp;
                    wifidatagrid_log.width(width);
                    wifidatagrid_log << wifi.timestamp;
                    wifidatagrid_log.width(width);
                    wifidatagrid_log << wifi.mac;
                    wifidatagrid_log.width(width);
                    wifidatagrid_log << (uint16_t)wifi.rssi;
                    wifidatagrid_log << std::endl;

                    wiFiMeasurementTimestamps.push_back(timestamp);
					Fpbl::WiFiScanResult wifiScan;
					wifiScan.timestamp = timestamp;
					wifiScan.scanWiFi.push_back(wifi);

                    pStaticBuilder->processWiFiMeasurement(timestamp, wifiScan);

                    wifi.mac += 5;       /**< MAC addres in decimal form */

                    timestamp += 2000000; // us
                    wifi.timestamp = timestamp;

                }
                wifi.rssi += 5.0;        /**< RSSI value [dbm] */


            }


            Fpbl::WiFiGrid wifiGrid1;

            Fpbl::ReturnStatus status = pStaticBuilder->updateGridWiFi(venue.id, grid, &wifiGrid1);

            int n = venue.sizeX / grid.size;
            int m = venue.sizeY / grid.size;

            //Assert::AreEqual((uint32_t)wifiGrid1.size(), (uint32_t)n * m);

            wifidatagrid_log.width(width);
            wifidatagrid_log << n;
            wifidatagrid_log.width(width);
            wifidatagrid_log << m;
            wifidatagrid_log.width(width);
            wifidatagrid_log << (uint16_t)wifiGrid1.size();
            wifidatagrid_log << std::endl;

            wifidatagrid_log << "WifiGrid====================================";
            wifidatagrid_log << std::endl;


            Fpbl::WifiCell wifiCell;
            for (int i = 0; i < wifiGrid1.size(); i++)
            {
                wifiCell = wifiGrid1[i];

                wifidatagrid_log << "Position    ";
                wifidatagrid_log.width(width);
                wifidatagrid_log << wifiCell.coordinates.x;
                wifidatagrid_log.width(width);
                wifidatagrid_log << wifiCell.coordinates.y;
                wifidatagrid_log.width(width);
                wifidatagrid_log << wifiCell.coordinates.floor;
                //wifidatagrid_log.width(width);
                //wifidatagrid_log << wifiCell.wifiData.size();
                wifidatagrid_log << std::endl;

                wifidatagrid_log << "WifiCell    "
                    << wifiCell.wifiData.size();
                wifidatagrid_log << std::endl;

				for (auto scan_it = wifiCell.wifiData.begin(); scan_it != wifiCell.wifiData.end(); ++scan_it)
                {
					for (auto meas_it = scan_it->scanWiFi.begin(); meas_it != scan_it->scanWiFi.end(); ++meas_it)
					{
						wifidatagrid_log.width(width);
						wifidatagrid_log << meas_it->timestamp;
						wifidatagrid_log.width(width);
						wifidatagrid_log << meas_it->mac;
						wifidatagrid_log.width(width);
						wifidatagrid_log << (int16_t)meas_it->rssi;
						wifidatagrid_log.width(width);
						wifidatagrid_log << meas_it->frequency;
						wifidatagrid_log << std::endl;
					}
                }
                wifidatagrid_log << "*****************";
                wifidatagrid_log << std::endl;

            }

        }

        TEST_METHOD(TestMethod3) // test for building wifi grid
        {
            Fpbl::GridBuilderPrivate *pStaticBuilder = new Fpbl::GridBuilderPrivate();

            char current_work_dir[1000];
#ifdef _WIN32
            //GetCurrentDirectory(1000, current_work_dir);
            _getcwd(current_work_dir, sizeof(current_work_dir));
#else
            getcwd(current_work_dir, sizeof(current_work_dir));
#endif
            const std::string folderPath(current_work_dir);

            const std::string logFimeName = folderPath + "\\bledatagrid_log.txt";
            std::fstream  bledatagrid_log(logFimeName.c_str(), std::ios::out);

            bledatagrid_log << logFimeName << std::endl;
            bledatagrid_log << std::right << std::endl;

            int width = 15;



            Fpbl::Venue venue;

            // 55.7352576, 37.6419266 Moscow Avrora Bisness Centre

            venue.id = 0;   /**< venue identifier */
            venue.originLattitude = 0;     /**< bottom left corner lattitude [rad] [-pi/2..pi/2] */
            venue.originLongitude = 0;     /**< bottom left corner longitude [rad] [-pi..pi] */
            venue.originAltitude = 0;      /**< zero floor altitude [m] */
            venue.originAzimuth = 0;       /**< venue y-axix rotation to true north [rad] [0..pi] */
            venue.floorsCount = 0;       /**< total floor number in venue */
            venue.sizeX = 100;               /**< x axis venue size [m] */
            venue.sizeY = 20;               /**< y axis venue size [m] */

            pStaticBuilder->setVenue(venue);

            Fpbl::Grid grid = { Fpbl::CellType::CELL_SQUARE, 5.0 };
            Fpbl::BleGrid bleGrid;



            // TODO: Your test code here
            Fpbl::Position devicePosition;
            uint64_t timestamp;

            std::vector<uint64_t> posMeasurementTimestamps;
            std::vector<Fpbl::Position> posMeasurements;

            int N = 20 * 2;
            int M = 4 * 2;

            srand(1000);
            //srand(time(NULL));

            timestamp = 1000000; // us

            devicePosition.lattitude = 5.0 / 4;       /**< lattitude [rad] [-pi/2..pi/2]*/
            devicePosition.longitude = 5.0 / 4;       /**< longitude [rad] [-pi..pi] */
            devicePosition.altitude = 0;        /**< altitude (sea level?) [m] */

            devicePosition.floorNumber = 0;    /**< discrete floor number [0..32767], must be positive or zero */
            devicePosition.covarianceLatLon[0][0] = 0;
            devicePosition.covarianceLatLon[0][1] = 0;
            devicePosition.covarianceLatLon[1][0] = 0;
            devicePosition.covarianceLatLon[1][1] = 0;
            /** Lattitude/Longitude covariance matrix in column order [rad^2]\n
            *  cov(lat,lat), cov(lon,lat)\n
            *  cov(lat,lon), cov(lon,lon) */
            devicePosition.altitudeStd = 1;     /**< altitude standard deviation [m]     */
            devicePosition.floorStd = 1;        /**< altitude standard deviation [floor] */
            devicePosition.isValid = true;           /**< position validity flag */

            for (int m = 0; m < M; m++)
            {
                for (int n = 0; n < N; n++)
                {
                    bledatagrid_log.width(width);
                    bledatagrid_log << timestamp;
                    bledatagrid_log.width(width);
                    bledatagrid_log << devicePosition.lattitude;
                    bledatagrid_log.width(width);
                    bledatagrid_log << devicePosition.longitude;
                    bledatagrid_log << std::endl;

                    posMeasurementTimestamps.push_back(timestamp);
                    posMeasurements.push_back(devicePosition);

                    pStaticBuilder->processDevicePosition(timestamp, devicePosition);

                    timestamp += 1000000; // us
                    if (m % 2 == 0)
                        devicePosition.lattitude += 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
                    else
                        devicePosition.lattitude -= 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/

                }

                if (m % 2 == 0)
                    devicePosition.lattitude -= 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
                else
                    devicePosition.lattitude += 5.0 / 2;       /**< lattitude [rad] [-pi/2..pi/2]*/
                devicePosition.longitude += 5.0 / 2;       /**< longitude [rad] [-pi..pi] */

            }


            // ble

            bledatagrid_log << "ble====================================";
            bledatagrid_log << std::endl;


            Fpbl::BleMeasurement ble;

            std::vector<uint64_t> bleMeasurementTimestamps;

            N = 20 * 2;
            M = 4;

            timestamp = 1500000;
            ble.timestamp = timestamp; /**< unix time [us] */
            ble.mac = 0;       /**< MAC addres in decimal form */
            ble.rssi = 0;        /**< RSSI value [dbm] */
            ble.frequency = 2500; /**< central channel frequency [MHz] */

			ble.major = 1;
			ble.minor = 1;
			ble.txPower = 20;
			ble.hasMAC;
			for (int i = 0; i < 16; i++)
				ble.uuid[i] = i;

            for (int m = 0; m < M; m++)
            {
                for (int n = 0; n < N; n++)
                {
                    bledatagrid_log.width(width);
                    bledatagrid_log << timestamp;
                    bledatagrid_log.width(width);
                    bledatagrid_log << ble.timestamp;
                    bledatagrid_log.width(width);
                    bledatagrid_log << ble.mac;
                    bledatagrid_log.width(width);
                    bledatagrid_log << (uint16_t)ble.rssi;
					bledatagrid_log.width(width);
					bledatagrid_log << ble.major;
					bledatagrid_log.width(width);
					bledatagrid_log << ble.minor;
					bledatagrid_log.width(width);
					bledatagrid_log << (uint16_t)ble.txPower;
					bledatagrid_log << std::endl;

                    bleMeasurementTimestamps.push_back(timestamp);
					Fpbl::BleScanResult bleScan;
					bleScan.timestamp = timestamp;
					bleScan.scanBle.push_back(ble);

                    pStaticBuilder->processBleMeasurement(timestamp, bleScan);

                    ble.mac += 5;       /**< MAC addres in decimal form */

					ble.major += 1;
					ble.minor += 1;
					ble.txPower += 1;

                    timestamp += 2000000; // us
                    ble.timestamp = timestamp;

                }
                ble.rssi += 5.0;        /**< RSSI value [dbm] */

				            ble.major += 1;
            ble.minor += 1;
            ble.txPower += 1;

            }


            Fpbl::BleGrid bleGrid1;

            Fpbl::ReturnStatus status = pStaticBuilder->updateGridBLE(venue.id, grid, &bleGrid1);

            int n = venue.sizeX / grid.size;
            int m = venue.sizeY / grid.size;

            //Assert::AreEqual((uint32_t)bleGrid1.size(), (uint32_t)n * m);

            bledatagrid_log.width(width);
            bledatagrid_log << n;
            bledatagrid_log.width(width);
            bledatagrid_log << m;
            bledatagrid_log.width(width);
            bledatagrid_log << (uint16_t)bleGrid1.size();
            bledatagrid_log << std::endl;

            bledatagrid_log << "bleGrid====================================";
            bledatagrid_log << std::endl;


            Fpbl::BleCell bleCell;
            for (int i = 0; i < bleGrid1.size(); i++)
            {
                bleCell = bleGrid1[i];

                bledatagrid_log << "Position    ";
                bledatagrid_log.width(width);
                bledatagrid_log << bleCell.coordinates.x;
                bledatagrid_log.width(width);
                bledatagrid_log << bleCell.coordinates.y;
                bledatagrid_log.width(width);
                bledatagrid_log << bleCell.coordinates.floor;
                //bledatagrid_log.width(width);
                //bledatagrid_log << bleCell.bleData.size();
                bledatagrid_log << std::endl;

                bledatagrid_log << "bleCell    "
                    << bleCell.bleData.size();
                bledatagrid_log << std::endl;

				for (auto scan_it = bleCell.bleData.begin(); scan_it != bleCell.bleData.end(); ++scan_it)
                {
					for (auto meas_it = scan_it->scanBle.begin(); meas_it != scan_it->scanBle.end(); ++meas_it)
					{
						bledatagrid_log.width(width);
						bledatagrid_log << meas_it->timestamp;
						bledatagrid_log.width(width);
						bledatagrid_log << meas_it->mac;
						bledatagrid_log.width(width);
						bledatagrid_log << (int16_t)meas_it->rssi;
						bledatagrid_log.width(width);
						bledatagrid_log << meas_it->frequency;
						bledatagrid_log.width(width);
						bledatagrid_log << meas_it->major;
						bledatagrid_log.width(width);
						bledatagrid_log << meas_it->minor;
						bledatagrid_log.width(width);
						bledatagrid_log << (uint16_t)meas_it->txPower;
						bledatagrid_log << std::endl;
					}
                }
                bledatagrid_log << "*****************";
                bledatagrid_log << std::endl;

            }

        }
	};

}
