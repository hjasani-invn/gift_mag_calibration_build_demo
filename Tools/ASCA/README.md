# ASCA - Automated Survey Completeness Analyzer

python version: 3.6 or late

main file: main.py

function call:ASCA (settings_json, BFP_gridFile, WFP_gridFile, MFP_gridFile, BFP_db4_file, WFP_db4_file, MFP_db4_file, geo_jsons_file, out_folder)

settings_json - JSON file with venue description
BFP_gridFile - BLE grid file
WFP_gridFile - WiFi grid file
MFP_gridFile -  magnetic grid file
BFP_db4_file, -  BFP database (format ble4)
WFP_db4_file - WFP database (format wifi4)
MFP_db4_file - MFP database (format mfp4)
geo_jsons_file - JSON file with list of routes in GEOJSON format for every floor (for example geojsons.json name)
out_folder - an output folder. It is created if not exist.


Release notes
ASCA v1.0.0
+initial version

ASCA v1.0.1
+fix bug with empty files utilization
+code refactoring
+add version number
+add RelNotes and short description file

ASCA v1.0.2
+update ASCA interface
+add ASCA test plan

ASCA v1.0.4
+ empty grid picture output
+ bugfixes and improvements of coverage maps drawing
+ exception handling

ASCA v1.0.5
+ changed floor number in output image for Beacon Placement from internal to real floor numbering

ASCA v1.0.6
+ use colors modification for crowdsoursing cells for total coverage image

ASCA v1.0.7
+ magnetic grid quality metric is added
+ magnetic grid quality metric take into account the WiFi and BLE measurements number
+ available cells without data have gray color
+ add creation the output folder if it is necessary

ASCA v1.0.8
+ correct TotalCoverage image size

ASCA v1.0.9
+ change red color threshold for mag_quality

ASCA v1.0.10
+ correct MFP uncertainty
+ add ASCA MFP + WFP uncertainty

ASCA v1.0.11
+ correct MFP coverage (last cell was ignored)

ASCA v1.0.12
+ added output in JSON format with cell information (currently includes coordinates and total coverage score)

ASCA v1.0.13
+ added output for mag quality and mfp coverage in text format (percentage of red, yellow, gray and green cells)
+ fixed bug with MFP quality and uncertainty calculation due to incorrect data loading from grid file in case of multifloor venue
+ fixed another bug in MFP quality calculations (one extra measurement from the next cell was taken)
+ updated MFP grid loading to use less RAM (prevents memory error on laptop for venues with large MFP grid size)

ASCA v1.0.14
+ added support for single-cell survery routes

ASCA v1.0.15
+ added support for utf8 characters in geojson (Japanese names of routes don't cause crash anymore)

ASCA v1.0.16
+ updated Wi-Fi and BLE grid loading to use less RAM (prevents memory errors for venues with large grid size)

ASCA v1.0.17
+ minor bug fixes:
+  mag quality wasn't using last measurement in each cell from grid
+  old format wi-fi and ble grid reading was not working, now it's fixed

ASCA v1.1.0
+ removed magnetic, Wi-Fi and BLE grid files processing 
  now ASCA gets data from updated mfp3, wifi3 and ble3

ASCA v1.1.1
+ changed logic for total uncertainty image when there is issue with Wi-Fi uncertainty
  now if WFP is not present, or if not enough Wi-Fi data exists to calculate Wi-Fi uncertainty,
  ASCA tool will output magnetic uncertainty as total uncertainty.
+ Minor refactoring was also done, it doesn't affect functionality.

ASCA v1.2.0
+ changed input from mfp3, wifi3 and ble3 to mfp4, wifi4 and ble4
+ added reading of survey type from mfp4 and added "survey_type_F.png" output images
+ adjusted MFP coverage for robot survey 
+ moved ASCA test files to ASCA main folder in order to run tests in Linux without problems

