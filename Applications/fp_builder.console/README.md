# fp_builder.console

Cross-platform fingerprint builder console

========================================================================
    CONSOLE APPLICATION : FP_builder Project Overview
========================================================================


/////////////////////////////////////////////////////////////////////////////

Release notes:

As part of release FPBL v0.11.0
release date: sep-06-2017
- magnetic data calibration with FDMC was realized
- mixing of avaliable magnetic calibration data was realized
- WiFi and BLE data utilization from dat-files was realized
- json settings options was extended by suppotr of new settings
- multyfloor support for FP build was realized
- arbitrary catalog structur support was realized for input data

As part of release FPBL v1.0.4
release date: aug-29-2018
+ add repeatable wifi measurements rejection
+ support of reordering of wifi measurements according to specified wifi scan period
+ set default wifi scan period as 3000ms
+ support of "wifi_scan_period" command in json file, which specifies wifi scan period in milliseconds


!!!The relnotes for FPBL is now presented here

FPBL v1.1.1
-release date: mar-4-2019
FPBL:
- no changes
fp_builder.console:
-fix bug of floor number conversion in parse_ble_proximity_db_file function

FPBL v1.1.2
-release date: mar-18-2019
FPBL:
- no changes
fp_builder.console:
-fix bug of line ending output in binary fingerprints wfp4, bfp4, blp4

FPBL v1.1.3
-release date: mar-20-2019
FPBL:
- no changes
fp_builder.console:
-fix bug of reading floor_hight field from vene.json

FPBL v1.1.4, may/8/2019, hotfix
FPBL:
- Add new corrector Corrector_InterpolationInElevator for floor data interpolation
- Improve Corrector_RejectMagDataInStops for bypassing of data in elevators
fp_builder.console:
- no changes

FPBL v1.2.0, may/17/2019, release
FPBL:
- Mag validators bugfix: invalidate mag data without position data invalidation - allow to use position data for WiFi and BLE grids
fp_builder.console:
- Add new field "beacon floor height" in BLP db, FP builder takes this data from last (8-th) column of csv file if presented
- WiFi and BLE build logs output
- Output of magnetic validation log, wifi and ble build logs in “out” folder specified in json file.

FPBL v1.2.1, may/30/2019, hotfix
FPBL:
-no changes
fp_builder.console:
-fix bug of UIID output into BLP DB
-add autotesting scripts in gift/tools/fpbl_testing

FPBL v1.2.2, July/16/2019
new items for  FPBL release 1.2.2 
Set position validity flag disable when nav_phase flag < 1
Fix bug with magnetic data zeroes in maggid
Update validator for speed rejection
Add median filter for speed to reject spikes in IRL data
Use validity flag for median filter reset
Update corrector constructor interface and validator list in venue.json: exclude threshold for whole track invalidation
Exclude whole track invalidation by total rejection threshold
Add validator (corrector) for track invalidation by total speed rejection threshold 
Add validator (corrector) for track invalidation by single max speed threshold (speed > 5m)
Add three new validators in survey tool default list (for_coursa_survey_tool) in order: V1, V2, V3
Fix bug with #ind values in validation log

FPBL v1.2.3, July/23/2019
new items for  FPBL release 1.2.3
New mag corrector - rejects parts of magnetic data between significant shifts in MFP frame magnetic field values.
Fixed bug in validator "reject mag data in stops".

FPBL v1.2.4, October/16/2019
Support WiFI and BLE ignore lists from venue json
Add '--ignore-list' key for command line to specify overall ignore list json-file instead separated keys '--wifi-ignore-list' and '--blp-ignore-list'
Stop support '--wifi-ignore-list' and '--blp-ignore-list' command line keys

FPBL v1.2.5, October/30/2019
Support WiFI white list (alowed APs)
Support masks in white and black/ignore lists for WiFI
Fix bug in wifi_log output: ignored APs are not accounted in parsed AP count

FPBL v1.3.0, November 13 2019
Support of crowdsourcing functionality, initial feature set

FPBL v1.4.0, December 05 2019
Update iterative build
Add dataset id in wifi and ble grid
Support dataset list
Add default mag validators set for ivl data sets processing
Add RejectPositionInStops corrector

FPBL v1.4.1, December 13 2019, hotfix
Reject data with big positioning error in FPBL processing, separated threshold for ivl/survey/irl/mapper datasets
Add mag measurement tolerance time for grid data as 50 ms
Add BLE and WiFi measurement tolerance time for grid data as 1000 ms
Correction of input_list/input_folder utilization logic, input list has priority if specified
Add support of alternative name for wifi white list: "wifi_white_list". It is supported together with "wifi_ap_white_list"
Improve console logging

FPBL v1.4.2, February 13 2020, hotfix
Set default information gain limit for WFP and BFP as 0.25,
Set default AP count limit for WFP and BFP as 1000
Set MFP sigma as 10uT for crowdsourced cells
Fix bug of input grids paths configuration from json data

FPBL v1.4.3, April 2 of 2020, release
Add IStdEstimator - Interface class for magnetic std calculation for MFP cell
Add RobustStdEstimator implementation of IStdEstimator witch realized robust std estimation algorithm
Add QuantileStdEstimator implementation of IStdEstimator witch realized std estimation algorithm based on quantile estimates
Add CombineStdEstimator implementation of IStdEstimator witch realized combination of robust and quantile std estimation algorithms
Set RobustStdEstimator by default
Correction of input data reading from dat-files: additional check of calibrated data accuracy flag
Correct console logging
Fix bug of dataset id calculation when wifi and ble data obtained from data text logs
Add wifi stuck scan protection algorithm (disabled)

FPBL v1.4.4, Juny 08 of 2020, release
Utilization of dataset Quality Assessment Score
Updated Checker_MeanMagZ validator to ignore mag measurement in stops
Updated Checker_MeanMagHor validator to ignore mag measurement in stops
Fixed bug in Checker_B_value validator
Added Checker_DataSetScoreForPosMag validator to reject datasets with low QAS
Added Corrector_DataSetScoreForPosition corrector to disable datasets with low QAS
Fixed bug in Corrector_SetMagMeasCovMatrixToMaxValues validator
Added Corrector_RejectMagMeasByPosUncertainty 
Added Corrector_DataSetScoreForPosition
Updated general default validator set: 
    added Corrector_DataSetScoreForPosition, Checker_DataSetScoreForPosMag
Updated eIrlData validator set: 
    removed Corrector_RejectPositionsByPosUncertainty
    added Corrector_RejectDataByPosUncertainty, Corrector_RejectMagMeasByPosUncertainty
Updated eCoursaSurveyData validator set: 
    added Corrector_RejectDataByPosUncertainty, Corrector_RejectMagMeasByPosUncertainty
Updated eIvlData validator set: 
    removed Corrector_RejectPositionsByPosUncertainty
    added Corrector_RejectDataByPosUncertainty, Corrector_RejectMagMeasByPosUncertainty
    added Corrector_RejectDataByPosUncertainty

FPBL v1.4.5, July 17 of 2020, release
Added Corrector_RejectSegmentByHighSpeed, Checker_MagDataPercent, Checker_PositionPercent
Correct Checker_MeanMagHor, Checker_MeanMagZ 
Correct Corrector_MinNavFlag, Corrector_RejectTrackByVeryManyInvalidPositions - mode_of_portal_transit is taken into account
Updated eIvlData validator set: 
    added Checker_MagBiasLevel
Correct mag bias utilization logic

FPBL v1.4.6, August 27 of 2020, release
Added fast activation of portals - portal cells are included in magnetic fingerprint with a few number of measurement
Added processing of single-cell route 
Changed portal grid to disable none-crowdsoursed cells output 
Added validators invalid_mag_data_percent and invalid_position_percent
Correct reading of magnetic covariation from tpn data

FPBL v1.4.7, September 23 of 2020, hotfix
Change Checker_MeanMagHor , Checker_MeanMagZ thresholds

FPBL v1.4.8, October 26 of 2020, release
Change to correctors - allowed user speed threshold changed from 0.5-2.0 m/s to 0.0-2.0 m/s and removed mag data rejection during stops.

FPBL v1.5.0, November 9 of 2020, release
Change writing portal information into MFP fingerprint - use route_type parameter from datasets input list.
Added mag recalibration option.
Changed usage of input portal grid to input crowdsourcing grid, since it was only used for crowdsourced survey. 
Output portalsgrid file (used only for results checking) doesn't contain crowdsourcing cells in it anymore.
Added TxPower_correction reading and saving to BLP database (it is set to 0 if it doesn't exist in input CSV file).

FPBL v1.5.1, Junuary 8 of 2021, release
Disable parsing of fields from *_grid, *_fp structures of venue.json

FPBL v1.5.2, Febrary 8 of 2021, release
Correct error in FPBL v1.5.1
New parsing of fields from *_grid, *_fp structures of venue.json

FPBL v1.5.3, March 24 of 2021, release
Added mag coverage and mag quality to mfp3/mfp4 files.
Added Wi-Fi and BLE coverage to wifi3/wifi4 and ble3/ble4 files.
Update loading and saving  portal grid

FPBL v1.5.4, May 20 of 2021, release
Added interpolation of mag measurements if one measurement is not valid, but both neighbor measurements are valid.

FPBL v1.5.5, June 10 of 2021, release
Added blp data statistics output for using with proximity beacons placement error detection tool. The output is activated with --xblp_detection key

FPBL v1.6.0, July 22 of 2021, release
Robotic survey support
Add robotic survey datasets processing. data_type = "robotic_survey_data" in input list json.
Support validation set "for_robo_survey"
Disable altitude output in MFP cells reserved parameters y.reserved[0-1] and z.reserved[0-1]
Add survey data type in maggrid.
Add survey data type into MFP cell in y.reserved[0]. 
Set the data type follows:
    i. All data has robotic_survey_data: set robotic_survey_data
	ii. Otherwise set a type which is presented mostly (except RoboticMode)
Add validator for mag bias sigma settings depends on calibration level. Enable this validator for robotic survey.
Bug fix: default route type according to requirements is "NormalMode" but in code this is set to "Unknown". As result datasets without route specification in input_lisst.json are not processed. Fixed: Set default route type as normal if no route types specified.
Bug fix: route type for "Single-Cell" is set incorrectly as NormalMode. Fixed: Set route type in this case as SingleCellMode.
Add especial validator to realize extended nav_phase utilization logic (using zero nav_phase flag) for begining part of dataset. Enable this validator for robotic survey.
Change proximity cutoff threshold from 5 to 10 dBm in blp4




