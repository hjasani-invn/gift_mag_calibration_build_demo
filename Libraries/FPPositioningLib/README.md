# FPPositioningLib

Cross-platform Finger Print Positioning library – M2

Release 0.16.2
- Enable minimal wifi measurement delay

Release 0.16.3
- Coverety-warnings fix

Release 0.16.4
- Add FPPE interface for LkH update control
- Disable measurement noise adjustment depending on misalignment deviation
- Coverety-warnings fix

Release 0.16.5
- Fix issue with checking of floor_hight correctness
- Set default floor height 5m (instead 1m in earlier versions)
- Set default floor height for single-floor venues (5*5m)
- Add altitude outliers rejection

Release 0.16.6
- Change wifi/ble measurement log format
- Add particle count extension in mix mode up to 10K during wifi gap.
- Add rejection of similar wifi solutions (with 0.1m difference between neighbor solutions); but this option is disabled in this version.
- Fix bug with out of time wifi measurement rejection for first scan.
- Fix bug with rejection first wifi solution after gap
- Disable increasing of estimated solution covariance by 2

Release 0.16.7
- Fix linux compilation errors in C-files that hapends with some compilator versions
- Fix bug with disabling of mix filter start by BLE data

Release 1.0.0
- MVP version

Release 1.1.0
- Likelyhood sampling algorithm improvement
- Support mfp4, bfp4, wfp4 formats of fingerprint DB. Add interface funtions to support these DB formats.
- Add getWfpInfo(), getMfpInfo(), getBfpInfo() funtions to API

Release 1.1.1
+ Cloud spread freezing during stops; spread dispersion 0.5cm
+ LKH resamping during stops: from each particle , heading resampling only, 5% particles
+ pdr_scale factor estimation in PF, freezing after 200 PF steps

Release 1.1.2
+ LKH=5% when missalignment error is detected
+ PDR scale adaptaton is disabled when LKH is enabled

Release 1.1.3 - betta
+ Set sigma BLE updater to 20m

Release 1.1.4
+ C-API for fp4
+ Add FPEngine_setVenueParamsEx, FPEngine_getVenueParamsEx, FPEngine_getWfpInfo, FPEngine_getMfpInfo, FPEngine_getBfpInfo funtions into C-API
+ Fix issue with fp4 header upload on Android

Release 1.1.5
+ MFP positioning adaptation in depending on the venue type (it is supported with mfp4 databases)

Release 1.1.6
Fix bug of azimuth utilization in setStartPosition function.
Add start position covariance matrix transformation from geo to local frame
Add debug output for start position

Release 1.2.0
Add BLE proximity updater in mixed filter
Add interface for BLE proximity in RTFPPL
Improve BFP based start in mied filter
Add start with using proximity data in mied filter

Release 1.2.1
Add support of format-4 of BLE proximity database
Add using default BFP/WFP/BLP venue parameters if they are not defined in database-files
Expand debug output for fingerprint initialization functions

Release 1.2.2
Fix bug in blp4-data loading

Release 1.2.3 - BLE optimization
BLE positioning algorithm settings have been optimized: minAPCount=5; margin=0
Metric pNorm of WiFi and BLE positioning algorithm are calculated by only cells used in KNN instead all cells as before
BLE position rms is utilized for BLE updater: sig = 1rms, min sigma limit = 10m
Rejection of BLE solution delayed more than for 5 sec
Metric min_prob set as 0.005 by default for BLE and WiFi positioning
Metric min_prob set as 0.005 for BLE for all venues - temporary patch

Release 1.3.0
+ Add gyro drift correction during stop for multipoint LKH predictor
+ Add BLE measurement bias estimation, enabled
+ Add WiFi measurement bias estimation, disabled
+ Add interface for venue detection using WiFI measurements
+ Fix issues of BFPVenue and Bfp4Header inheritance

Release 1.3.1 - hot fix
+ Disable position freezing during stop intervals

Release 1.3.2 - hot fix
+ Set min_prob metric from bfp4 database

Release 1.4.0 - calculation-optimized version
+ Remove additive noise from PF propagation models
+ Combine misalignment noise with heading noise in PF propagation models
+ Add single floor mode for propagation model to decrease calculation consumption by decreasing particle count
+ Automatic activation of single floor mode depends on venue type in mfp4 file
+ Remove WiFi/BLE position calculation overhead

Release 1.4.1 - calculation-optimized version
+ Optimization of memory operations in LKH algorithm

Release 1.4.2
+ Fix bug of LKH object initialization

Release 1.4.3 - calculation-optimized version
+ Calculation optimization in LKH algorithm

Release 1.4.4
+ Bag fix in Zigurat-random algorithm

Release 1.4.5
+ Bag fix of proximity solution false repeat

Release 1.4.6 - WiFi adjustment
+ Enable WiFi bias estimation
+ Adjustment of gain factor of wifi bias estimation algorithm
+ Rejection of low RSSI signals for bias estimation, RSSI rejection threshold = -85
+ Scatter rejection threshold of wifi position is set to 10m

Release 1.4.7 - WiFi adjustment
+ Use different scatter rejection threshold for BLE and WiFi
+ Scatter rejection threshold adaptation depends on AP count in DB
  AP count <  50: 10m
  AP count >= 50: 20m
Adjustment dispersion of WiFI initializer depends on AP count in DB
  AP count >=  50: 36m2
  AP count <  50: 144m2

Release 1.4.8 - WiFi adjustment
+ Enable proximity in stops with double sigma
+ Enable WiFi in stops with double sigma
+ Bagfix in tpn position integration (epic fail)
+ Mixed filter prediction model type in depending on mixed solution dispersion:
  if (sigma_mix < 2.): kAisleVenue
  if (sigma_mix >= 2.): kMallVenue

Release 1.4.9
+ Update IRPL wrapper interface
+ No performance changing

Release 1.4.10
+ Add support actual cell_size in LKH algorithms
+ Add "Factory" venue type
+ Add lkh-model adjustment for Factory venues
+ Add temporary lkh-model adjustment for Mall venues
+ Fix bug of particle count limits utilization for mag and mixed filters

Release 1.5.1
+ Add real floor number support. It can be adjusted with floors_count, floor_shift, floor_zero_enable settings of venue structure.
+ Add floor converter for accurate real floor numbers to logical fingerprint numbers conversion and vice versa
+ Parsing of mode_of_transit entity
+ Correction of fpp_api_functions wrapper interface
+ BLP algorithms refactoring and bug fixing.
+ Add random seeds support for pseudorandom normal distribution generator and interface function for enabling the random seeds
+ Bug fix in lkh weight calculation (division by 2)
+ Chancel temporary lkh-model adjustment for Mall venue
+ Bug fixes in InitializerInArea and Initializer

Release 1.6.0 Android P adaptation
+ Disable particle number control when not in tracking
+ Initializer in area: uniform distribution of start  heading
+ Increase allowed wifi solution delay for start up to 20 sec
+ Particle injection from wifi position: p_inj=0.1 for first 30 wifi updates, and 0.01 after

Release 1.6.1 Android P adaptation
+ Cancel injection in tracking
+ Disable injection from BLP, BFP
+ Disable MagSatrtPF data for injection
+ Fix bug of tracking flag settings during injection
+ Review injection logic - no performace changing
+ Bug fix in proximity updater logging

Release 1.6.2 Android P adaptation
+ Bug fix: enable wifi injection after mag start

Release 1.6.3 Android P adaptation
+ Disable wifi/ble/proximity/frmework_pos queues clearing on FF restart

Release 1.7.0 
+ Magnetic start area limitation added
+ Fixed magnetic biases saving

Release 1.7.1
+ Patch for floor params in fingerprints built by FPBL early v1.1.0
+ Fix bug of floor conversion in setStartPosition and processExternalPosition

Release 1.7.2 - hotfix
+ Using of pfMixed for mag bias obtaining instead pfMFP
+ Bug fix: Eliminating of pf->estimate function using to get pf state in GetMagneticBiasAndCov

RTFPPL v 1.8.0 - release
+ Utilizing multiple beacons simultaneously in proximity update. Enable multi-beacon update for iPhone.
+ Add additional sorting criterion (by BleHash) for proximity beacons selection when RSSI criterion gives equality
+ Add BLE bias estimation with proximity beacons
+ Support beacon height field provided in proximity DB
+ Output height of beacon in proximity callback
+ Magnetic start limitations: maximal particle number limit is 50K, no more 12.5K traversable MFP cells in venue where mag start is allowed
+ Using Transit Mode flag of TPN to improve positioning in portals (escalators, stairs and other)
Console improvement with this release:
+ Multicolored kml-files for multi-floor venues
+ Synchronization of console and real-time CV output for bit-exactness
+ Expand position debug output in console for identity with rt-wrapper

RTFPPL v 1.8.1 - hotfix
+ Transit mode logging in converter log
+ Utilization of floor dispersion in MagInitializerInArea (in unpresice mode after SetStartPosition call)
+ Particle number limitation in MagInitializerInArea (in unpresice mode after SetStartPosition call)
+ Fix bug of wifi injection injection which causes in pilling position away

RTFPPL v 1.8.2 - release - floor flickering elimination
+ New position estimate function with particle floor separation by floor
+ Reduce floor variation in PredictTPN3DLkH_MutiPoint when in tracking
+ Impulse LKH injection on large instant particle rejection
+ Fix disabling of cloud size adaptation when no tracking. Now it is prohibited to decreas particle number when no tracking
Console improvement with this release:
+ Add sub-folders in KML files

RTFPPL v 1.8.3 - release – auto restart of mixed filter
+ Automatic restart logic for mixed filter
+ Mixed position pooling in to the closest proximity beacon during stops - enable proximity updater in stops
+ Extra proximity output from ble_proximity_start in to proximity update
+ Fix bug of crashing rtfppl after weighted estimate() calculation
+ Add altitude output in main rtfppl callback update(const Fppe::Position &position)
+ Ignore covariance matrix recalculation in main rtfppl callback for debug output - when provided Sig is negative
+ Proximity beacons only output in proximity debug log instead all beacons output before
Console improvement with this release:
+ Disable mfp_pf by default
+ Set filtered altitude using instead barometric one

RTFPPL v 1.8.4 - hotfix
+ Patch for extra proximity output from ble_proximity_start in to proximity update

RTFPPL v 1.8.5 - hotfix
+ Additional debug output for venue_rdetection and mixed filter reacquisition
+ Fix bug of proximity DB initialization

RTFPPL v 1.9.0
+ Additional interface for reading Wi-Fi and BLE biases and for setting start values of these biases (setting the absolute value of bias to >= 20 is ignored).

RTFPPL v 1.9.1 - release – proximity improvements
+ PD-filtration of BLE measurements for regular BLP pipeline
+ Bugfix of proximity threshold setting in Proximity classes
+ Add beacon height and Tx power in proximity measurement log
+ Provide BLP solution when no PDR alignment
+ Disable extra proximity output from start pipeline
+ Add separated threshold for proximity loc-list and majoritarian logic solution
+ Set maximal processing distance of proximity updater in stop as 1.5m

RTFPPL v 1.9.2 - hotfix – floor conversion bugfix
+ Bugfix of venue floor parameters parsing from fp4 header
+ Add logging of venue parameters in pf log

RTFPPL v 1.9.3 - hotfix – floor conversion bugfix
+ Bugfix of venue floor parameters with negative/zero floors
+ Patch: disable floor checking for proximity pooling

RTFPPL v 1.9.4 - hotfix – proximity pulling adjustment
+ Disable ble-proximity bias estimation during stops
+ PD-data for proximity bias and start
+ Proximity bias applying bugfix
+ Soft pulling
+ Add received txPower to proximity log

RTFPPL v 1.10.0
+ Add new extended proximity callback which provides beacon information
+ BLE beacon type support
+ Add GetCurrentRealFloor method in floor converter
Console improvement with this release:
+ Support extended proximity interface in console

RTFPPL v 1.10.1 - wifi positioning adjustment for Bridgestone
+ Disable additinal solution scatter limitation if small AP number in wifi location object
+ Set default wifi solution scatter as 25
+ Set default wifi knn level as 6
Console improvement with this release:
+ Add random collor for kml files

RTFPPL v 1.10.2 - blp pulling settings updatefor Bridgestone
+ Set blp pulling parameters: instant pulling up to 2m

RTFPPL v 1.11.0 
+ Platform type setting/reading added to RTFPPL interface
+ Added MFP map matching functionality, but it is turned off by default. CheckParticles3D function is refactored to do less calculations.

RTFPPL v 1.12.0 
+ Add blp pulling interface into RTFPPL to provide setting of pulling type and distance
+ Extension of BLP-updater logging
+ Add multipoint weighted bias estimation for WiFi/BLE bias using KNN-cells of fingerprint
+ Disable WiFi/BLE bias estimation during stops
+ WiFi positioning improvements: add normalization of lkh-metric for KNN-cells; disable n-degree root in lkh-metric calculation
+ Adjustment of parameters of WiFi/BLE bias estimation algorithm
+ Add code for entire hypothesis model for cell LKH calculation, this option is disabled currently

RTFPPL v 1.12.1 
+ Added fix to block restarts during long stops

RTFPPL v 1.13.0
+ Add VDR support. Requires setting pftp_FORKLIFT mode with FPc.setPlatformType function to be enabled
+ Add proximity position for detection output. The position is calculated independent from proximity for positioning
+ Add interface for setting of proximity for detection parameters: peak detector filter, majoritarian logic and cut-off threshold
+ Fix bug in PD data rounding
+ Extend proximity updater logging
+ Updated internal interfaces for prediction model and related functions
+ Internal flag is_step renamed to is_motion

RTFPPL v 1.14.0
+ Add Map Matching updater to particle filter
+ Add vector map parsing to use the one with MM
+ Update API with MM setting and initialization interfaces
+ Update PredictTPN3DLkH_MutiPoint prediction model with LKH applying improvements
+ Set PredictTPN3DLkH_MutiPoint prediction model for all venue types
+ Disable PredictTPN3DLkH_Mixed prediction model
+ Review UpdateMFP3D updater for correct update of LKH particles
+ Utilize transit_mode flag for prediction and update adaptation

RTFPPL v 1.14.1
+ MM update, support doors in map file
+ Fix bug of resetting is_motion flag after PF iteration
+ Add new beacon type bbt_restricted_assistance
+ Disable MM update when pfMixed is not in tracking

RTFPPL v 1.14.2
+ Bugfix of MM map initialization

RTFPPL v 1.14.3
+ Patch in blp-db parsing, using stod()/stol instead std::stringstream for reading operations
+ Patch in WiFi-db parsing, add checking of std::stringstream status after reading operations

RTFPPL v 1.14.4
+ Patch in map matching - now particles are not killed if there is no map for some floors


RTFPPL v 1.15.0
+ Add interface for proximity beacon number calculation on specified floor using blp-db
Console improvement with this release:
+ set detection proximity enable based on venue.json

RTFPPL v 1.15.1 - sprint release
+ Add proximity position uncertainty estimate in RTFPPL proximity outputs
+ Utilize proximity position uncertainty in proximity updater
+ LKH algorithm tunning for MM
+ Add injection from multiple sources for stucks avoiding
+ Add resampling after increasing of particle number
+ Proximity bias bug fix

RTFPPL v 1.15.2
+ Improve floor flickering using transition areas
+ Add propagation and update for portals (transition areas)
+ Add EKF for proximity bias

RTFPPL v 1.15.3
+ Restore TxPower for iPhones from BLP-DB
+ Disable EKF for proximity bias estimation, it is using old estimation algorithm
+ Use proximity majoritarian logic position for proximiti update with iOS with parameters as 1:1:-10
Console improvement with this release:
+ Disable proximity settings reconfiguration in console, It is using default settings now

RTFPPL v 1.15.4
+ Change floor estimation for a elimination of flickering
+ Decrease a noise of move model for particles inside elevator

RTFPPL v 1.15.5
+ fix bug in altitude filter
+ correct logic of moving particles to portal

RTFPPL v 1.15.6 - hotfix
+ Fix bug in UpdateMFP3D: correct particle normalization procedure

RTFPPL v 1.16.0 
+ Fixed bug with extra injection from proximity
+ Fixed bug with wrong BLE prox sigma in case of no pulling
+ Changed proximity mode to multiple update for both iOS and Android (using single update for pulling only)
+ Changed peak detection mode to ON for iOS
+ Added new interface functions to set and get BLE bias and its uncertainty (also using input bias age)
+ Enabled EKF proximity bias estimation
+ Adjusted EKF proximity bias estimator parameters
RTFPPL console: + Added RTFPPL-seed randomization support

RTFPPL v 1.16.1
+ Enabled using of initial heading from SetStartPosition function

RTFPPL v 1.16.2
+ Disable initial heading std from SetStartPosition function, set this parameter as 180deg
+ Enable MMUpdater when mixed position sigma less than 3m and PF not in tracking

RTFPPL v 1.16.3
+ Apply init_normal initializer in setStartPositionWithUncertainties function for position uncertainty up to 5.5 m
+ Enable initial heading std from SetStartPosition function

RTFPPL v 1.16.4
+ Correct portal logic
+ Fixed map matching issue with particles sometimes going through walls by making walls thicker
+ Changed initializers used in case of SetStartPosition (normal or uniform)
+ Set output position validity flag to "false" in case when mixed filter is not in tracking

RTFPPL v 1.16.5
+ Reverted 1.16.3 change "Enable initial heading std from SetStartPosition function" to increase robustness after ZUPT

RTFPPL v 1.16.6
+ Add position consistency checking in EKF proximity bias estimator
+ Update EKF bias managing in FF
+ Clear position queue for proximity bias estimator on restart
+ Add BLE and WiFi bias logging in PF
+ Apply BLE bias for all pipelines including detection proximity, and proximity for start
+ Set cut_off threshold as -15dBm for proximity for start
+ Fix bug of using proximity data for proximity update in Android
RTFPPL console: 
+Add set start position feature update in console

RTFPPL v 1.16.7
+ Fix bug in BleProximityBiasEstimator_EKF initialization.
+ Set default initial BLE proximity bias covariance as 4 (decreased from 9).
+ Enable ring update in proximity updater.

RTFPPL v 1.17.0
+ Add base collaboration functionality. Collaboration datatypes and API. Initialization and injection with collaboration data.
+ Disable collaboration data using by default. 
+ Disable injection from collaboration data until injection logic update.
+ Fix bug in num_beacons_in_update calculation in updateBleProximity
+ Disable empty proximity scans processing
+ Enable Venue Detection logging in pf_log when venue detection callback is not set
+ Changed initializer and injector selection logic and added uncertainty increase over time for initializer/injector start position.

RTFPPL v 1.17.1
+ Rejection of repeated measurements from one BLE in proximity scans. A measurement with max RSSI is only processed.

RTFPPL v 1.18.0
+ Add floor transition ability without barometer based on wireless positions
+ Add interface of enable/disable baro data usage
+ Portal updater modification: use combined rejection criterion instead vertical speed threshold
+ Portal updater modification: prohibition floor changing outside of portal
+ Portal updater code refactoring
+ Allow height updates from proximity during stops
+ Add protection from barometer drift - after 5 minutes without new wifi/ble/proximity position user height is frozen in motion model until wifi/ble/proximity position appears
+ Proximity bias estimate adjustment: set cutting_level to -90bBm, distance range to 0.5-5m
+ Bugfix in floor transition with WiFi code
+ Set 5 sec injection ban after any injection was applied.
+ Add logging for process_beacons_proximity function

RTFPPL v 1.18.1 hotfix
+ Fixed bug of BLP Tx correction utilization provided with BLP DB
+ Proximity start cutoff threshold set as -30 dBm to decrease TTFF

RTFPPL v 1.18.2 hotfix
+ Add stop intervals detection for VDR with velocity threshold as 0.25m
+ Disable UKF in stops for VDR
RTFPPL console: 
+ Add platform type setting from setting json files
+ Add support debug settings and start position from settins venue.json

RTFPPL v 1.18.3
+ Update fingerprint initialization functions. Add error handling
+ Sonar Cube critical errors bugfixes

RTFPPL v 1.18.4 - hotfix
+ Fix bug of RTFPPL version return in C-interface
