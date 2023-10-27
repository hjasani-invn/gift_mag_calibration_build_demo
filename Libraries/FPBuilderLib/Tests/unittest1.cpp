#include <string>
#include <cmath>

#include <direct.h>

#include "stdafx.h"
#include "CppUnitTest.h"

#include "GridBuilderPrivate.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace FPBuilderLibTests1
{		
	TEST_CLASS(UnitTest1)
	{
	public:
        Fpbl::GridBuilderPrivate *pStaticBuilder = new Fpbl::GridBuilderPrivate();

		TEST_METHOD(TestMethod1)
		{
			// TODO: Your test code here

            Fpbl::VersionNumber versionNumber = pStaticBuilder->getVersionNumber();

            Assert::AreEqual((int)versionNumber.major, (int)0);
            Assert::AreEqual((int)versionNumber.minor, (int)0);
            Assert::AreEqual((int)versionNumber.build, (int)0);
            Assert::AreEqual((int)versionNumber.releaseId, (int)0);
        }

        TEST_METHOD(TestMethod2) // pathFolder test
        {
            // TODO: Your test code here

            char current_work_dir[1000];
#ifdef _WIN32
            //GetCurrentDirectory(1000, current_work_dir);
            _getcwd(current_work_dir, sizeof(current_work_dir));
#else
            getcwd(current_work_dir, sizeof(current_work_dir));
#endif
            const std::string folderPath(current_work_dir);

            pStaticBuilder->setTemporaryFolder(folderPath);

            std::string folderPathBack = pStaticBuilder->getTemporaryFolder();

            Assert::AreEqual(folderPath, folderPathBack);
        }

        TEST_METHOD(TestMethod3) // venue test
        {
            // TODO: Your test code here

            Fpbl::Venue venue;

            // 55.7352576, 37.6419266 Moscow Avrora Bisness Centre

            venue.id = 0;   /**< venue identifier */
            venue.originLattitude = 55.7352576;     /**< bottom left corner lattitude [rad] [-pi/2..pi/2] */
            venue.originLongitude = 37.6419266;     /**< bottom left corner longitude [rad] [-pi..pi] */
            venue.originAltitude = 200;      /**< zero floor altitude [m] */
            venue.originAzimuth  = 0.6 * M_PI;       /**< venue y-axix rotation to true north [rad] [0..pi] */
            venue.floorsCount = 3;       /**< total floor number in venue */
            venue.sizeX = 20;               /**< x axis venue size [m] */
            venue.sizeY = 100;               /**< y axis venue size [m] */

            pStaticBuilder->setVenue(venue);

            Fpbl::Venue venueBack;
            venueBack = venueBack = pStaticBuilder->getVenue();

            Assert::AreEqual(venue.id, venue.id);
            Assert::AreEqual(venue.originLattitude, venueBack.originLattitude);
            Assert::AreEqual(venue.originLongitude, venueBack.originLongitude);
            Assert::AreEqual(venue.originAltitude, venueBack.originAltitude);
            Assert::AreEqual(venue.originAzimuth, venueBack.originAzimuth);
            Assert::AreEqual((int16_t)venue.floorsCount, (int16_t)venueBack.floorsCount);
            Assert::AreEqual(venue.sizeX, venueBack.sizeX);
            Assert::AreEqual(venue.sizeY, venueBack.sizeY);
            
        }

    };
}