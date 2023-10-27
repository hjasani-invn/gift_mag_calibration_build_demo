/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \defgroup        PF
* @{
* \file            ff_config.h
* \brief           Fusion filter global configuration parameters
* \author          D. Churikov, M. Frolov
* \date            28.11.2014
*/

#ifndef FF_CONFIG_H
#define FF_CONFIG_H

#define PERSONALIZATION_ENABLED 0 ///< zero value disables step length adaptation by default

#define AUTO_ADJUSTMENT_IN_ENVIRONMENT_ENABLED  0 ///< zero value disables environment state changing

#define USE_MAGNETIC_BIAS_FROM_RBF  1 ///< non-zero enables obtaining magnetic bias from RBF

#define BARO_ENABLED 0 ///< zero disables using baro by default

//#1 filter settings
#define PF1_ENABLED             1 ///< enables Filter #1

#define PF1_PARTICLE_COUNT      2000  ///< Filter #1 default particles count

#define PF1_WIFI_ENABLED        0     ///< Filter #1 WiFi updater default value
#define PF1_BLE_ENABLED         0     ///< Filter #1 BLE updater default value
#define PF1_MAGNETIC_ENABLED    0     ///< Filter #1 magnitude MFP updater default value (obsolete)
#define PF1_3D_MAGNETIC_ENABLED 0     ///< Filter #1 3D-vector MFP updater default value
#define PF1_3DIFF_MAGNETIC_ENABLED 0  ///< Filter #1 differential MFP updater default value (experimental)
#define PF1_COMPASS_ENABLED     0     ///< Filter #1 compass (2D MFP) updater default value (obsolete)

#define PF1_SIGMA_WIFI          20           ///< Filter #1 default WiFi updater std [m]
#define PF1_SIGMA_BLE           20           ///< Filter #1 default BLE updater std  [m]
#define PF1_SIGMA_BLE_PROXIMITY 1.5           ///< Filter #1 minimal proximity updater std  [m]
#define PF1_SIGMA_COMPASS       (M_PI / 3)   ///< Filter #1 default compass updater std (obsolete) [rad]
#define PF1_COMPASS_WEIGHT      0.8          ///< Filter #1 compass updater weighting constant (obsolete)
#define PF1_SIGMA_PDR_H         (2*M_PI/180) ///< Filter #1 prediction model heading std in tracking mode [rad]
#define PF1_SIGMA_PDR_H_INIT    (6*M_PI/180) ///< Filter #1 prediction model heading std in non-tracking mode
#define PF1_SIGMA_PDR_L2H_RATIO (30./170.)   ///< Filter #1 step length std, equal to 30 cm for 170 height 
// WARNING: need to be 10 cm for current personalization settings
// BUT it leads to problems with all particles dying to mapmatching if a step goes into obstacle
#define PF1_SIGMA_PDR_L2H_RATIO_I (30./170.) ///< Filter #1 step length std equal to 30 cm for 170 height
#define PF1_SIGMA_PDR_P         0.1      ///< Filter #1 additive position noise in tracking mode [m]
#define PF1_SIGMA_PDR_P_INIT    0.2      ///< Filter #1 additive position noise in non-tracking mode [m]
#define PF1_MFP_WEIGHT_THRESHOLD  (0.2)  ///< Filter #1 MFP weight limith (obsolete)
#define PF1_MM_WEIGHT           0.0      ///< Filter #1 MM weight limith

#define PF1_REINIT_THRESHOLD_INSTANCE  (0.90) ///< second reinitialization threshold, 90% particles dies
#define PF1_REINIT_THRESHOLD_AVERAGE  (0.75)  ///< first reinitialization threshold 

//#2 filter settings
#define PF2_ENABLED             1        ///< enables Filter #2
#define PF2_PARTICLE_COUNT    1000       ///< Filter #2 default particles count

#define PF2_WIFI_ENABLED        1        ///< Filter #2 WiFi updater default value
#define PF2_BLE_ENABLED         1        ///< Filter #2 BLE updater default value
#define PF2_MAGNETIC_ENABLED    <UNUSED> ///< reserved
#define PF2_COMPASS_ENABLED     <UNUSED> ///< reserved

#define PF2_SIGMA_WIFI          20       ///< Filter #2 default WiFi updater std [m]
#define PF2_SIGMA_BLE           5        ///< Filter #2 default BLE updater std [m]
#define PF2_SIGMA_COMPASS       <UNUSED> ///< reserved
//#define PF2_COMPASS_CORRECTION  DEFAULT_COMPASS_CORRECTION
#define PF2_COMPASS_WEIGHT      <UNUSED>     ///< reserved
#define PF2_SIGMA_PDR_H         (6*M_PI/180) ///< Filter #2 prediction model heading std [rad]
//#define PF1_SIGMA_PDR_L         0.3
#define PF2_SIGMA_PDR_L2H_RATIO   (20./170.) ///< Filter #2 step length std, equal to 20 cm for 170 height
#define PF2_SIGMA_PDR_P         1            ///< Filter #2 additive noise std [m]
#define PF2_MFP_WEIGHT_THRESHOLD  <UNUSED>   ///< reserved
#define PF2_MM_WEIGHT           0.75         ///< Filter #2 MM weight threshold

// Empirical settings
#define WIFI_SPIKE_PATCH         1           ///< enables wifi position spike detection [m]
#define WIFI_SPIKE_THRESHOLD    20           ///< defines spike threshold  [m]
#define WIFI_MAX_SPIKE_COUNT    5            ///< defines maximum serial spikes count


#define PF_MAX_PARTICLES_NUMBER 50000		 ///< maximal particle number alowed in PF
#define PF_MAX_ALLOWED_BAROMETER_TIME 300.0  ///< maximum allowed seconds for barometer height change (since last wi-fi, proximity or BFP position was detected)

#endif //FF_CONFIG_H

/**@} */