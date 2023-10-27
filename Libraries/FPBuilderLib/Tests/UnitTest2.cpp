#include "stdafx.h"
#include "CppUnitTest.h"

#include "GridBuilderPrivate.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace FPBuilderLibTests1
{
	TEST_CLASS(UnitTest2)
	{
	public:
        Fpbl::GridBuilderPrivate *pStaticBuilder = new Fpbl::GridBuilderPrivate();

        TEST_METHOD(TestMethod1) // device position saving/restoring data test
        {
            // TODO: Your test code here
            uint64_t timestamp = 1000;
            Fpbl::Position devicePosition;

            devicePosition.lattitude = 3;       /**< lattitude [rad] [-pi/2..pi/2]*/
            devicePosition.longitude = 1;       /**< longitude [rad] [-pi..pi] */
            devicePosition.altitude = 10;        /**< altitude (sea level?) [m] */
            devicePosition.floorNumber = 0;    /**< discrete floor number [0..32767], must be positive or zero */
            devicePosition.covarianceLatLon[0][0] = 1; 
            devicePosition.covarianceLatLon[0][1] = 2;
            devicePosition.covarianceLatLon[1][0] = 6;
            devicePosition.covarianceLatLon[1][1] = 7;
            /** Lattitude/Longitude covariance matrix in column order [rad^2]\n
                                           *  cov(lat,lat), cov(lon,lat)\n
                                           *  cov(lat,lon), cov(lon,lon) */
            devicePosition.altitudeStd = 1;     /**< altitude standard deviation [m]     */
            devicePosition.floorStd = 1;        /**< altitude standard deviation [floor] */
            devicePosition.isValid = true;           /**< position validity flag */


            pStaticBuilder->processDevicePosition(timestamp, devicePosition);

            uint64_t  timestampBack = pStaticBuilder->getDevicePositionTimestamp(0);

            Assert::AreEqual((uint64_t)timestamp, (uint64_t)timestampBack);

            Fpbl::Position devicePositionBack = pStaticBuilder->getDevicePosition(0);

            Assert::AreEqual(devicePosition.altitude, devicePositionBack.altitude);

            Assert::AreEqual(devicePosition.altitude, devicePositionBack.altitude);
            Assert::AreEqual(devicePosition.longitude, devicePositionBack.longitude);
            Assert::AreEqual(devicePosition.altitude, devicePositionBack.altitude);
            Assert::AreEqual(devicePosition.floorNumber, devicePositionBack.floorNumber);

            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    Assert::AreEqual(devicePosition.covarianceLatLon[i][j], devicePosition.covarianceLatLon[i][j]);

            Assert::AreEqual(devicePosition.altitudeStd, devicePositionBack.altitudeStd);
            Assert::AreEqual(devicePosition.floorStd, devicePositionBack.floorStd);
            Assert::AreEqual(devicePosition.isValid, devicePosition.isValid);

        }

        TEST_METHOD(TestMethod2) // device attitude saving/restoring data test
        {
            // TODO: Your test code here
            uint64_t timestamp = 2000;
            Fpbl::Attitude deviceAttitude;
			
            deviceAttitude.quaternion[0] = 0;       
            deviceAttitude.quaternion[1] = 1;       
            deviceAttitude.quaternion[2] = 2;       
            deviceAttitude.quaternion[3] = 3;       
            deviceAttitude.covarianceQuaternion[0][0] = 0;       
            deviceAttitude.covarianceQuaternion[0][1] = 3;       
            deviceAttitude.covarianceQuaternion[0][2] = 4;       
            deviceAttitude.covarianceQuaternion[0][3] = 20;      
            deviceAttitude.covarianceQuaternion[1][0] = 50;      
            deviceAttitude.covarianceQuaternion[1][1] = 4;       
            deviceAttitude.covarianceQuaternion[1][2] = 6;       
            deviceAttitude.covarianceQuaternion[1][3] = 7;       
            deviceAttitude.covarianceQuaternion[2][0] = 2;       
            deviceAttitude.covarianceQuaternion[2][1] = 7;       
            deviceAttitude.covarianceQuaternion[2][2] = 8;       
            deviceAttitude.covarianceQuaternion[2][3] = 8;       
            deviceAttitude.covarianceQuaternion[3][0] = 3;       
            deviceAttitude.covarianceQuaternion[3][1] = 8;       
            deviceAttitude.covarianceQuaternion[3][2] = 7;       
            deviceAttitude.covarianceQuaternion[3][3] = 8;       
            deviceAttitude.isValid = true;           //*< orientation validity flag *

            pStaticBuilder->processDeviceAttitude(timestamp, deviceAttitude);

            uint64_t  timestampBack = pStaticBuilder->getDeviceAttitudeTimestamp(0);

			Assert::AreEqual((uint64_t)timestamp, (uint64_t)timestampBack);

            Fpbl::Attitude deviceAttitudeBack = pStaticBuilder->getDeviceAttitude(0);

            int i, j;
            for (i = 0; i < 4; i++)
                Assert::AreEqual(deviceAttitude.quaternion[i], deviceAttitudeBack.quaternion[i]);

            for (i = 0; i < 4; i++)
                for (j = 0; j < 4; j++)
                    Assert::AreEqual(deviceAttitude.covarianceQuaternion[i][j], deviceAttitudeBack.covarianceQuaternion[i][j]);

                Assert::AreEqual(deviceAttitude.isValid, deviceAttitudeBack.isValid);
        }

        TEST_METHOD(TestMethod3) // wifi  saving/restoring data test
        {
            // TODO: Your test code here
            Fpbl::WiFiMeasurement wifi;

            wifi.timestamp = 3000; /**< unix time [us] */
            wifi.mac = 2345678;       /**< MAC addres in decimal form */
            wifi.rssi = 71;        /**< RSSI value [dbm] */
            wifi.frequency = 2500; /**< central channel frequency [MHz] */

            uint64_t timestamp = wifi.timestamp;

			Fpbl::WiFiScanResult wifiScan;
			wifiScan.timestamp = timestamp;
			wifiScan.scanWiFi.push_back(wifi);


            pStaticBuilder->processWiFiMeasurement(timestamp, wifiScan);

            uint64_t  timestampBack = pStaticBuilder->getWiFiMeasurementTimestamp(0);

            Assert::AreEqual((uint64_t)timestamp, (uint64_t)timestampBack);

            Fpbl::WiFiScanResult wifiBack = pStaticBuilder->getWiFiMeasurement(0);

            Assert::AreEqual((uint64_t)wifiScan.timestamp, (uint64_t)wifiBack.timestamp);
			Assert::AreEqual(wifiScan.scanWiFi.size(), wifiBack.scanWiFi.size());

			for (size_t ii = 0; ii < wifiScan.scanWiFi.size(); ++ii)
			{
				Assert::AreEqual(wifiScan.scanWiFi[ii].mac, wifiBack.scanWiFi[ii].mac);
				Assert::AreEqual(wifiScan.scanWiFi[ii].rssi, wifiBack.scanWiFi[ii].rssi);
				Assert::AreEqual((uint64_t)wifiScan.scanWiFi[ii].frequency, (uint64_t)wifiBack.scanWiFi[ii].frequency);
			}

        }

        TEST_METHOD(TestMethod4) // ble saving/restoring data test
        {
            // TODO: Your test code here
            Fpbl::BleMeasurement ble;

            ble.timestamp = 3000; /**< unix time [us] */
            ble.mac = 2345678;       /**< MAC addres in decimal form */
            ble.rssi = 71;        /**< RSSI value [dbm] */
            ble.frequency = 2500; /**< central channel frequency [MHz] */

            ble.major = 545; /**< iBeacon major number */
            ble.minor = 187; /**< iBeacon minor number */
             /**< iBeacon uuid 128 bit value */
            for (int i = 0; i < 16;  i++)
                ble.uuid[i] = i;

            ble.txPower = 60;   /**< iBeacon tx power level [dbm] on 1m distance */
            ble.hasMAC = true;      /**< mac address avaliability flag*/


            uint64_t timestamp = ble.timestamp;

			Fpbl::BleScanResult bleScan;
			bleScan.timestamp = timestamp;
			bleScan.scanBle.push_back(ble);


            pStaticBuilder->processBleMeasurement(timestamp, bleScan);

            uint64_t  timestampBack = pStaticBuilder->getBleMeasurementTimestamp(0);

            Assert::AreEqual((uint64_t)timestamp, (uint64_t)timestampBack);

            Fpbl::BleScanResult bleBack = pStaticBuilder->getBleMeasurement(0);

            Assert::AreEqual(bleScan.timestamp, bleBack.timestamp);

			for (size_t ii = 0; ii < bleScan.scanBle.size(); ++ii)
			{
				Assert::AreEqual(bleScan.scanBle[ii].mac, bleBack.scanBle[ii].mac);
				Assert::AreEqual(bleScan.scanBle[ii].rssi, bleBack.scanBle[ii].rssi);
				Assert::AreEqual((uint64_t)bleScan.scanBle[ii].frequency, (uint64_t)bleBack.scanBle[ii].frequency);

				Assert::AreEqual((uint32_t)bleScan.scanBle[ii].major, (uint32_t)bleBack.scanBle[ii].major);
				Assert::AreEqual((uint32_t)bleScan.scanBle[ii].minor, (uint32_t)bleBack.scanBle[ii].minor);

				for (int i = 0; i < 16; i++)
				{
					Assert::AreEqual((uint32_t)bleScan.scanBle[ii].uuid[i], (uint32_t)bleBack.scanBle[ii].uuid[i]);
				}
				Assert::AreEqual((uint32_t)bleScan.scanBle[ii].txPower, (uint32_t)bleBack.scanBle[ii].txPower);
				Assert::AreEqual((uint32_t)bleScan.scanBle[ii].hasMAC, (uint32_t)bleBack.scanBle[ii].hasMAC);
			}
        }
        
        TEST_METHOD(TestMethod5) // magnetic saving/restoring data test
        {
            // TODO: Your test code here
            Fpbl::MagneticVector mag;

            mag.mX = 56; ///< magnetic field on x-axis [uT] 
            mag.mY = 26; ///< magnetic field on y-axis [uT]
            mag.mZ = 17; ///< magnetic field on z-axis [uT]

            uint64_t timestamp = 3456783;

            pStaticBuilder->processMFPMeasurement(timestamp, mag);

            uint64_t  timestampBack = pStaticBuilder->getMFPMeasurementTimestamp(0);

            Assert::AreEqual((uint64_t)timestamp, (uint64_t)timestampBack);

            Fpbl::MagneticVector magBack = pStaticBuilder->getMFPMeasurement(0);

            Assert::AreEqual(mag.mX, magBack.mX);
            Assert::AreEqual(mag.mY, magBack.mY);
            Assert::AreEqual(mag.mZ, magBack.mZ);

        }
        
    };
}