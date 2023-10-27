#include "stdafx.h"
#include "CppUnitTest.h"
#include <ctime>
#include <fstream>
#include <iomanip>

#include <direct.h>

#include "GridBuilderPrivate.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace FPBuilderLibTests1
{
	TEST_CLASS(UnitTest3)
	{
	public:
        Fpbl::GridBuilderPrivate *pStaticBuilder = new Fpbl::GridBuilderPrivate();

        TEST_METHOD(TestMethod1) // device position saving/restoring data test
        {
            // TODO: Your test code here
            Fpbl::Position devicePosition;
            uint64_t timestamp;

            std::vector<uint64_t> posMeasurementTimestamps;
            std::vector<Fpbl::Position> posMeasurements;

            const int N = 100;

            srand(1000);
            //srand(time(NULL));

            for (int n = 0; n < N; n++)
            {
                timestamp = rand() * 1000;

                devicePosition.lattitude = (M_PI / 2) * ( (rand() - RAND_MAX / 2) / ((double)RAND_MAX / 2) );       /**< lattitude [rad] [-pi/2..pi/2]*/
                devicePosition.longitude = (M_PI ) * ((rand() - RAND_MAX / 2) / ((double)RAND_MAX / 2));       /**< longitude [rad] [-pi..pi] */
                devicePosition.altitude = rand();        /**< altitude (sea level?) [m] */
                devicePosition.floorNumber = rand()/ 1000;    /**< discrete floor number [0..32767], must be positive or zero */
                devicePosition.covarianceLatLon[0][0] = rand() / (double)1000;
                devicePosition.covarianceLatLon[0][1] = rand() / (double)1000;
                devicePosition.covarianceLatLon[1][0] = rand() / (double)1000;
                devicePosition.covarianceLatLon[1][1] = rand() / (double)1000;
                /** Lattitude/Longitude covariance matrix in column order [rad^2]\n
                *  cov(lat,lat), cov(lon,lat)\n
                *  cov(lat,lon), cov(lon,lon) */
                devicePosition.altitudeStd = rand() / (double)1000;     /**< altitude standard deviation [m]     */
                devicePosition.floorStd = rand() / (double)10000;        /**< altitude standard deviation [floor] */
                devicePosition.isValid = true;           /**< position validity flag */

                posMeasurementTimestamps.push_back(timestamp);
                posMeasurements.push_back(devicePosition);

                pStaticBuilder->processDevicePosition(timestamp, devicePosition);

            }

            Assert::AreEqual((uint32_t)N, (uint32_t)posMeasurementTimestamps.size());

            uint64_t  timestampBack;

            char current_work_dir[1000];
#ifdef _WIN32
            //GetCurrentDirectory(1000, current_work_dir);
            _getcwd(current_work_dir, sizeof(current_work_dir));
#else
            getcwd(current_work_dir, sizeof(current_work_dir));
#endif
            const std::string folderPath(current_work_dir);

            const std::string logFimeName = folderPath + "\\pos_log.txt";
            std::fstream  pos_log(logFimeName.c_str(), std::ios::out);

            pos_log << logFimeName << std::endl;
            pos_log << std::right << std::endl;

            for (int n = 0; n < N; n++)
            {

                timestampBack = pStaticBuilder->getDevicePositionTimestamp(n);

                Assert::AreEqual((uint64_t)posMeasurementTimestamps[n], (uint64_t)timestampBack);

                Fpbl::Position devicePositionBack = pStaticBuilder->getDevicePosition(n);
                
                Assert::AreEqual(posMeasurements[n].lattitude, devicePositionBack.lattitude);
                Assert::AreEqual(posMeasurements[n].longitude, devicePositionBack.longitude);
                Assert::AreEqual(posMeasurements[n].altitude, devicePositionBack.altitude);
                Assert::AreEqual(posMeasurements[n].floorNumber, devicePositionBack.floorNumber);
                
                for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 2; j++)
                        Assert::AreEqual(posMeasurements[n].covarianceLatLon[i][j], devicePositionBack.covarianceLatLon[i][j]);
                
                Assert::AreEqual(posMeasurements[n].altitudeStd, devicePositionBack.altitudeStd);
                Assert::AreEqual(posMeasurements[n].floorStd, devicePositionBack.floorStd);
                Assert::AreEqual(posMeasurements[n].isValid, devicePositionBack.isValid);
                
                //pos_log.fill('#');
                int width = 15;
                
                pos_log.width(width);
                pos_log << posMeasurementTimestamps[n];
                pos_log.width(width);
                pos_log << timestampBack << std::endl;

                pos_log.width(width);
                pos_log << posMeasurements[n].lattitude;
                pos_log.width(width);
                pos_log << devicePositionBack.lattitude;

                pos_log.width(width);
                pos_log << posMeasurements[n].longitude;
                pos_log.width(width);
                pos_log << devicePositionBack.longitude;

                pos_log.width(width);
                pos_log << (uint16_t)posMeasurements[n].altitude;
                pos_log.width(width);
                pos_log << (uint16_t)devicePositionBack.altitude;

                pos_log.width(width);
                pos_log << posMeasurements[n].floorNumber;
                pos_log.width(width);
                pos_log << devicePositionBack.floorNumber;
                
                for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 2; j++)
                    {
                        pos_log.width(width);
                        pos_log << posMeasurements[n].covarianceLatLon[i][j];
                        pos_log.width(width);
                        pos_log << devicePositionBack.covarianceLatLon[i][j];
                    }
                pos_log << std::endl;
                
                pos_log.width(width);
                pos_log << posMeasurements[n].altitudeStd;
                pos_log.width(width);
                pos_log << devicePositionBack.altitudeStd;

                pos_log.width(width);
                pos_log << (uint16_t)posMeasurements[n].floorStd;
                pos_log.width(width);
                pos_log << (uint16_t)devicePositionBack.floorStd;

                pos_log.width(width);
                pos_log << posMeasurements[n].isValid;
                pos_log.width(width);
                pos_log << devicePositionBack.isValid;
                
            }
        }


        TEST_METHOD(TestMethod3) // wifi saving/restoring data test
        {
            // TODO: Your test code here
            Fpbl::WiFiMeasurement wifi;
            uint64_t timestamp;

            std::vector<uint64_t> wiFiMeasurementTimestamps;
            std::vector<Fpbl::WiFiScanResult> wifiScanResults;

            const int N = 100;

            srand(3000);
            //srand(time(NULL));

            for (int n = 0; n < N; n++)
            {
                timestamp = rand() * 1000;

                wifi.timestamp = rand() * 1000; /**< unix time [us] */
                wifi.mac = rand() * 1000;       /**< MAC addres in decimal form */
                wifi.rssi = rand() / 100;        /**< RSSI value [dbm] */
                wifi.frequency = 2500; /**< central channel frequency [MHz] */

                timestamp = wifi.timestamp;

                wiFiMeasurementTimestamps.push_back(timestamp);
				Fpbl::WiFiScanResult scan;
				scan.timestamp = timestamp;
				wifiScanResults.push_back(scan);

				pStaticBuilder->processWiFiMeasurement(timestamp, scan);
            }

            Assert::AreEqual((uint32_t)N, (uint32_t)wiFiMeasurementTimestamps.size());

            uint64_t  timestampBack;

            char current_work_dir[1000];
#ifdef _WIN32
            //GetCurrentDirectory(1000, current_work_dir);
            _getcwd(current_work_dir, sizeof(current_work_dir));
#else
            getcwd(current_work_dir, sizeof(current_work_dir));
#endif
            const std::string folderPath(current_work_dir);

            const std::string logFimeName = folderPath + "\\wifi_log.txt";
            std::fstream  wifi_log(logFimeName.c_str(), std::ios::out);

            wifi_log << logFimeName << std::endl;
            wifi_log << std::right << std::endl;

            for (int n = 0; n < N; n++)
            {

                timestampBack = pStaticBuilder->getWiFiMeasurementTimestamp(n);

                Assert::AreEqual((uint64_t)wiFiMeasurementTimestamps[n], (uint64_t)timestampBack);

                Fpbl::WiFiScanResult wifiBack = pStaticBuilder->getWiFiMeasurement(n);

				Assert::AreEqual((uint64_t)wifiScanResults[n].timestamp, (uint64_t)wifiBack.timestamp);

				Assert::AreEqual(wifiScanResults[n].scanWiFi.size(), wifiBack.scanWiFi.size());

				for (size_t ii = 0; ii < wifiScanResults[n].scanWiFi.size(); ++ii)
				{

					Assert::AreEqual(wifiScanResults[n].scanWiFi[ii].timestamp, wifiBack.scanWiFi[ii].timestamp);
					Assert::AreEqual(wifiScanResults[n].scanWiFi[ii].mac, wifiBack.scanWiFi[ii].mac);
					Assert::AreEqual(wifiScanResults[n].scanWiFi[ii].rssi, wifiBack.scanWiFi[ii].rssi);
					Assert::AreEqual((uint32_t)wifiScanResults[n].scanWiFi[ii].frequency, (uint32_t)wifiBack.scanWiFi[ii].frequency);
				}


                //wifi_log.fill('#');
                int width = 15;
                wifi_log.width(width);
                wifi_log << wiFiMeasurementTimestamps[n];
                wifi_log.width(width);
                wifi_log << timestampBack << std::endl;

                wifi_log.width(width);
                wifi_log << wifiScanResults[n].timestamp;
                wifi_log.width(width);
                wifi_log << wifiBack.timestamp;
                
				for (size_t ii = 0; ii < wifiScanResults[n].scanWiFi.size(); ++ii)
				{
					wifi_log.width(width);
					wifi_log << wifiScanResults[n].scanWiFi[ii].timestamp;
					wifi_log.width(width);
					wifi_log << wifiBack.scanWiFi[ii].timestamp;

					wifi_log.width(width);
					wifi_log << wifiScanResults[n].scanWiFi[ii].mac;
					wifi_log.width(width);
					wifi_log << wifiBack.scanWiFi[ii].mac;

					wifi_log.width(width);
					wifi_log << wifiScanResults[n].scanWiFi[ii].mac;
					wifi_log.width(width);
					wifi_log << wifiBack.scanWiFi[ii].mac;

					wifi_log.width(width);
					wifi_log << (uint16_t)wifiScanResults[n].scanWiFi[ii].rssi;
					wifi_log.width(width);
					wifi_log << (uint16_t)wifiBack.scanWiFi[ii].rssi;

					wifi_log.width(width);
					wifi_log << wifiScanResults[n].scanWiFi[ii].frequency;
					wifi_log.width(width);
					wifi_log << wifiBack.scanWiFi[ii].frequency;

					wifi_log << std::endl;
				}
            }
        }

        TEST_METHOD(TestMethod4) // ble saving/restoring data test
        {
            // TODO: Your test code here
            Fpbl::BleMeasurement ble;
            uint64_t timestamp;

            std::vector<uint64_t> bleMeasurementTimestamps;
            std::vector<Fpbl::BleScanResult> bleScanResults;

            const int N = 100;

            srand(3000);
            //srand(time(NULL));

            for (int n = 0; n < N; n++)
            {
                timestamp = rand() * 1000;

                ble.timestamp = rand() * 1000; /**< unix time [us] */
                ble.mac = rand() * 1000;       /**< MAC addres in decimal form */
                ble.rssi = rand() / 100;        /**< RSSI value [dbm] */
                ble.frequency = 2500; /**< central channel frequency [MHz] */

                ble.major = rand(); /**< iBeacon major number */
                ble.minor = rand(); /**< iBeacon minor number */
                /**< iBeacon uuid 128 bit value */
                for (int i = 0; i < 16;  i++)
                    ble.uuid[i] = rand() / 200;

                ble.txPower = rand() / 100;   /**< iBeacon tx power level [dbm] on 1m distance */
                ble.hasMAC = true;      /**< mac address avaliability flag*/


                timestamp = ble.timestamp;

                bleMeasurementTimestamps.push_back(timestamp);

				Fpbl::BleScanResult bleScan;
				bleScan.timestamp = timestamp;
				bleScan.scanBle.push_back(ble);
				bleScanResults.push_back(bleScan);

				pStaticBuilder->processBleMeasurement(timestamp, bleScan);
            }

            Assert::AreEqual((uint32_t)N, (uint32_t)bleMeasurementTimestamps.size());

            uint64_t  timestampBack;

            char current_work_dir[1000];
#ifdef _WIN32
            //GetCurrentDirectory(1000, current_work_dir);
            _getcwd(current_work_dir, sizeof(current_work_dir));
#else
            getcwd(current_work_dir, sizeof(current_work_dir));
#endif
            const std::string folderPath(current_work_dir);

            const std::string logFimeName = folderPath + "\\ble_log.txt";
            std::fstream  ble_log(logFimeName.c_str(), std::ios::out);

            ble_log << logFimeName << std::endl;
            ble_log << std::right << std::endl;
            
            int width = 15;
            for (int n = 0; n < N; n++)
            {
                
                timestampBack = pStaticBuilder->getBleMeasurementTimestamp(n);

                Assert::AreEqual((uint64_t)bleMeasurementTimestamps[n], (uint64_t)timestampBack);

                Fpbl::BleScanResult bleBack = pStaticBuilder->getBleMeasurement(n);

                Assert::AreEqual((uint64_t)bleScanResults[n].timestamp, (uint64_t)bleBack.timestamp);
				Assert::AreEqual(bleScanResults[n].scanBle.size(), bleBack.scanBle.size());


				for (size_t ii = 0; ii < bleScanResults[n].scanBle.size(); ++ii)
				{
					Assert::AreEqual(bleScanResults[n].scanBle[ii].timestamp, bleBack.scanBle[ii].timestamp);
					Assert::AreEqual(bleScanResults[n].scanBle[ii].mac, bleBack.scanBle[ii].mac);
					Assert::AreEqual(bleScanResults[n].scanBle[ii].rssi, bleBack.scanBle[ii].rssi);
					Assert::AreEqual((uint32_t)bleScanResults[n].scanBle[ii].frequency, (uint32_t)bleBack.scanBle[ii].frequency);
				}
                            
                //ble_log.fill('#');
                width = 15;
                ble_log.width(width);
                ble_log << bleMeasurementTimestamps[n];
                ble_log.width(width);
                ble_log << timestampBack << std::endl;

                ble_log.width(width);
                ble_log << bleScanResults[n].timestamp;
                ble_log.width(width);
                ble_log << bleBack.timestamp;

				for (size_t ii = 0; ii < bleScanResults[n].scanBle.size(); ++ii)
				{
					ble_log.width(width);
					ble_log << bleScanResults[n].scanBle[ii].mac;
					ble_log.width(width);
					ble_log << bleBack.scanBle[ii].mac;

					ble_log.width(width);
					ble_log << (uint16_t)bleScanResults[n].scanBle[ii].rssi;
					ble_log.width(width);
					ble_log << (uint16_t)bleBack.scanBle[ii].rssi;

					ble_log.width(width);
					ble_log << bleScanResults[n].scanBle[ii].frequency;
					ble_log.width(width);
					ble_log << bleBack.scanBle[ii].frequency;

					ble_log.width(width);
					ble_log << bleScanResults[n].scanBle[ii].major;
					ble_log.width(width);
					ble_log << bleBack.scanBle[ii].major;

					ble_log.width(width);
					ble_log << bleScanResults[n].scanBle[ii].minor;
					ble_log.width(width);
					ble_log << bleBack.scanBle[ii].minor;

					ble_log.width(width);
					ble_log << (uint16_t)bleScanResults[n].scanBle[ii].txPower;
					ble_log.width(width);
					ble_log << (uint16_t)bleBack.scanBle[ii].txPower;

					ble_log.width(width);
					ble_log << bleScanResults[n].scanBle[ii].hasMAC;
					ble_log.width(width);
					ble_log << bleBack.scanBle[ii].hasMAC;

					ble_log << std::endl;
				}

                
                width = 5;

                ble_log << "uuid_start  ";
				for (size_t ii = 0; ii < bleScanResults[n].scanBle.size(); ++ii)
				{
					for (int i = 0; i < 16; i++)
					{
						Assert::AreEqual((uint32_t)bleScanResults[n].scanBle[ii].uuid[i], (uint32_t)bleBack.scanBle[ii].uuid[i]);

						ble_log.width(width);
						ble_log << (uint16_t)bleScanResults[n].scanBle[ii].uuid[i];
						ble_log.width(width);
						ble_log << (uint16_t)bleBack.scanBle[ii].uuid[i];
					}
					ble_log << "  uuid_end";
					ble_log << std::endl;
				}
            }
            
        }

        TEST_METHOD(TestMethod5) // magnetic saving/restoring data test
        {
            // TODO: Your test code here
            Fpbl::MagneticVector mag;
            uint64_t timestamp;

            std::vector<uint64_t> magneticVectorTimestamps;
            std::vector<Fpbl::MagneticVector> magneticVectors;

            const int N = 100;

            srand(5000);
            //srand(time(NULL));

            for (int n = 0; n < N; n++)
            {
                timestamp = rand() * 1000;
                mag.mX = rand() / 100;  ///< magnetic field on x-axis [uT] 
                mag.mY = rand() / 100; ///< magnetic field on y-axis [uT]
                mag.mZ = rand() / 100;; ///< magnetic field on z-axis [uT]

                magneticVectorTimestamps.push_back(timestamp);
                magneticVectors.push_back(mag);

                pStaticBuilder->processMFPMeasurement(timestamp, mag);
            }

            Assert::AreEqual((uint32_t)N, (uint32_t)magneticVectorTimestamps.size());

            uint64_t  timestampBack;

            char current_work_dir[1000];
#ifdef _WIN32
            //GetCurrentDirectory(1000, current_work_dir);
            _getcwd(current_work_dir, sizeof(current_work_dir));
#else
            getcwd(current_work_dir, sizeof(current_work_dir));
#endif
            const std::string folderPath(current_work_dir);

            const std::string logFimeName = folderPath + "\\mg_log.txt";
            std::fstream  mg_log(logFimeName.c_str(), std::ios::out);

            mg_log << logFimeName << std::endl;
            mg_log << std::right << std::endl;

            for (int n = 0; n < N; n++)
            {

                timestampBack = pStaticBuilder->getMFPMeasurementTimestamp(n);

                Assert::AreEqual((uint64_t)magneticVectorTimestamps[n], (uint64_t)timestampBack);

                Fpbl::MagneticVector magBack = pStaticBuilder->getMFPMeasurement(n);

                Assert::AreEqual(magneticVectors[n].mX, magBack.mX);
                Assert::AreEqual(magneticVectors[n].mY, magBack.mY);
                Assert::AreEqual(magneticVectors[n].mZ, magBack.mZ);

                //mg_log.fill('#');
                int width = 15;
                mg_log.width(width);
                mg_log << magneticVectorTimestamps[n];
                mg_log.width(width);
                mg_log << timestampBack << std::endl;
                mg_log.width(width);
                mg_log << magneticVectors[n].mX;
                mg_log.width(width);
                mg_log << magBack.mX;
                mg_log.width(width);
                mg_log << magneticVectors[n].mY;
                mg_log.width(width);
                mg_log << magBack.mY;
                mg_log.width(width);
                mg_log << magneticVectors[n].mZ;
                mg_log.width(width);
                mg_log << magBack.mZ << std::endl;

            }
        }

	};
}