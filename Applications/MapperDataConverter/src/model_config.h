#ifndef MODEL_CONFIG_H
#define MODEL_CONFIG_H

#include <string>
#include "pdr_rt\Personalization.h"
//#include "mc_bias.h"
#include <cstddef>


// in/out data params
#define DEFAULT_OUTPUT_PATH ".\\"
#define DEFAULT_WORKING_PATH ".\\"
#define MODEL_CONFIG   "IndoorDemo.cfg"

// indoor components
#define MODEL_WIFI_ENABLED  1
#define MODEL_BLE_ENABLED   0
#define MODEL_MAGNETIC_MAGNITUDE_ENABLED 0 //deprecated
#define MODEL_MAGNETIC_3DIFF_ENABLED 0 //3d differential (magnetic B)
#define MODEL_MAGNETIC_3D_ENABLED  1 //3d-compass (magnetic A)
#define MODEL_MAPMATCHING_ENABLED  1
#define MODEL_MAPMATCHING_3D_ENABLED  1
#define MODEL_GPS_ENABLED 0

// additional fusion params
#define MODEL_BARO_ENABLED 1
#define MODEL_PERSONALIZATION_ENABLED 0
#define MODEL_MAGNETIC_CALIBRATION_ENABLED 1
#define MODEL_ADJUST_WFP_PARAMS 1
#define MODEL_ADJUST_MFP_PARAMS 1

// user params
#define MODEL_PDR_USER_HEIGHT  175

// plan
#define MODEL_OFFICE_MAP		0
#define MODEL_SPIRIT_MULTI		0
#define MODEL_AFIMALL		0
#define MODEL_PANASONIC_SAEDO		0
#define MODEL_AEON_HIGASHI		0
#define MODEL_MFP		0
#define MODEL_DPLAZA_4		0
#define MODEL_TARGET		0
#define MODEL_ST2		0
#define MODEL_LALAPORT    1
#define MODEL_GALLERIA_2    0
#define MODEL_VALLEY    0
#define MODEL_VALLEY_1    0

// initialization params
#define MODEL_START_BY_WIFI 1

// track params
#define MODEL_MULTY_COLOR_TRACK   0
#define MODEL_TRACK_COLOR         249
#define MODEL_PARTICLE_COLOR      252
#define MODEL_WIFI_COLOR          250
#define MODEL_TRACK2_COLOR        252
#define MODEL_PARTICLE2_COLOR     245





#ifdef _CONSOLE
#define MODEL_TRACK_NAME    "Track_0_04.trk"
#define MODEL_TRACK2_NAME   "Track2_0_04.trk"
#endif

#if (MODEL_MFP)
//#   define DEFAULT_INPUT_PATH  "c:\\VSS_SK\\HIOL\\Tools\\Utils\\mfp\\t1\\"
#   define DEFAULT_INPUT_PATH  "c:\\VSS_SK\\HIOL\\Tools\\Utils\\mfp\\"

#   define MODEL_MAP_WIFI "none.wifi3"
#   define MODEL_MAP_MM   "model_mfp.map"
//#   define MODEL_MAP_MM   "SPirit_Floor6.map"
#   define MODEL_MAP_BLE  "none.wifi3"

//#   define MODEL_PDR_TRACK  "c:\\VSS_SK\\HIOL\\Tools\\Utils\\mfp\\model_45.pdr"
#   define MODEL_PDR_TRACK  "c:\\VSS_SK\\HIOL\\Tools\\Utils\\mfp\\model_none.pdr"
//#   define MODEL_PDR_TRACK  "none.pdr"

#   define MODEL_MAP_SIZE_X_M 128
#   define MODEL_MAP_SIZE_Y_M 64
#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M*100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M*100)

#   define MODEL_MM_CEIL_M 0.1
#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)

#   define MODEL_VIS_DOWNSCALE 15
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 2

//#   define MODEL_MAP_MFP  "c:\\VSS_SK\\HIOL\\Tools\\Utils\\mfp\\mg_cross1.mfp3"
#   define MODEL_MAP_MFP  "c:\\VSS_SK\\HIOL\\Tools\\Utils\\mfp\\mg_cross1-3.mfp3"
//#   define MODEL_MAP_MFP  "c:\\VSS_SK\\HIOL\\Tools\\Utils\\mfp\\office.mfp3"
#   define MODEL_MFP_CEIL_M 1.0
#   define MODEL_MG_B_X 10
#   define MODEL_MG_B_Y 10
#   define MODEL_MG_B_Z 10
#   define MODEL_MG_B_SIG 10


#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#if MODEL_START_BY_WIFI == 0
#   define MODEL_START_POS_X  128.0
#   define MODEL_START_POS_Y  64.0
#   define MODEL_START_POS_Z  0.0
#   define MODEL_START_H  0
#endif

#   define MODEL_LOC_FRAME_lat0       55.724253553756192
#   define MODEL_LOC_FRAME_lon0       37.653916182652893
#   define MODEL_LOC_FRAME_alfa_lat   0.000000162179509
#   define MODEL_LOC_FRAME_alfa_lon  -0.000000271000819
#   define MODEL_LOC_FRAME_heading  -31.327662731809170

#endif

#if (MODEL_SPIRIT_MULTI)
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\test_20_08\\1st\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\test_20_08\\2nd\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\test_20_08\\3rd(1_4rc2)\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\test_20_08\\4th\\"
#   define DEFAULT_INPUT_PATH "c:\\Vectors\\tests_25_08\\reinit\\"

#   define MODEL_MAP_WIFI "Spirit_Floor1-6.wifi3"
#   define MODEL_MAP_BLE  "Spirit_Floor1-6.bea3"
#   define MODEL_MAP_MFP  "Spirit_Floor1-6.mfp3"
#   define MODEL_MAP_MM   "Spirit_Floor1-6.map"

#   define MODEL_MAP_SIZE_X_M 50
#   define MODEL_MAP_SIZE_Y_M 12
#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M*100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M*100)

#   define MODEL_MM_CEIL_M 0.1
#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)

#   define MODEL_VIS_DOWNSCALE 5
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 5

#   define MODEL_MFP_CEIL_M 1.0
#   define MODEL_MG_B_X -4.0
#   define MODEL_MG_B_Y 3
#   define MODEL_MG_B_Z 26
#   define MODEL_MG_B_SIG 100*100
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_LOC_FRAME_lat0       55.724253553756192
#   define MODEL_LOC_FRAME_lon0       37.653916182652893
#   define MODEL_LOC_FRAME_alfa_lat   0.000000162179509
#   define MODEL_LOC_FRAME_alfa_lon  -0.000000271000819
#   define MODEL_LOC_FRAME_heading  -31.327662731809170

#endif

#if (MODEL_LALAPORT)

//#		define DEFAULT_INPUT_PATH  "C:\\Vectors\\Galleria\\10_12_11_27_50\\"
//#		define DEFAULT_INPUT_PATH  "C:\\Vectors\\Galleria\\10_12_13_16_45\\"

//#		define DEFAULT_INPUT_PATH "c:\\Vectors\\Galleria\\Ruslan\\11_02_06_11_32\\"
//#		define DEFAULT_INPUT_PATH "c:\\Vectors\\Galleria\\Ruslan\\11_02_06_22_50\\"
//#		define DEFAULT_INPUT_PATH "c:\\Vectors\\Galleria\\Ruslan\\11_02_12_08_37\\"
//#		define DEFAULT_INPUT_PATH "c:\\Vectors\\Galleria\\Ruslan\\track_003\\"


//#define DEFAULT_INPUT_PATH "v:\\Demonstrations\\2015_10_30_Alibaba\\tracks\\new_tracks\\track_003\\"
//#define DEFAULT_INPUT_PATH "c:\\Churikov\\Tasks\\HIOL\\Demonstrations\\2015_11_06_Lalaport\\tracks\\mc_001\\"
#define DEFAULT_INPUT_PATH "C:\\Vectors\\Demonstrations\\2015_11_06_Lalaport\\tracks\\mc_001\\"
//#define DEFAULT_INPUT_PATH "v:\\Demonstrations\\2015_11_06_Lalaport\\tracks\\track_001\\"
//#define DEFAULT_INPUT_PATH "C:\\Deliveries\\Galleria\\Logs\\3november\\track_005\\"

// #define DEFAULT_INPUT_PATH "C:\\Deliveries\\Galleria\\Logs\\3november\\track_006\\"
//#define DEFAULT_INPUT_PATH "C:\\Deliveries\\Galleria\\Logs\\3november\\track_007\\"

#   define MODEL_MAP_WIFI "lalaport_1_2f.wifi3"
#   define MODEL_MAP_BLE  "none3.bea3"
#   define MODEL_MAP_MFP  "lalaport_1_2f.mfp3"
//#   define MODEL_MAP_MM   "lalaport_1_2f.map"
#   define MODEL_MAP_MM   "Navigation_1_2_3.map"

#   define MODEL_MAP_SIZE_X_M 274
#   define MODEL_MAP_SIZE_Y_M 182
//#   define MODEL_MAP_SIZE_X_M 20
//#   define MODEL_MAP_SIZE_Y_M 13
#   define MODEL_MFP_CEIL_M 2.5

#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M * 100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M * 100)
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_MM_CEIL_M 0.1

#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)
#   define MODEL_VIS_DOWNSCALE 25 // 2 for Aisle
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 2

#   define MODEL_LOC_FRAME_lat0       5.0
#   define MODEL_LOC_FRAME_lon0       0.75
#   define MODEL_LOC_FRAME_alfa_lat   (-0.01745329)
#   define MODEL_LOC_FRAME_alfa_lon  0.01745329
#   define MODEL_LOC_FRAME_heading   0

#if MODEL_START_BY_WIFI == 0
#   define MODEL_START_POS_X  4
#   define MODEL_START_POS_Y  1.0
#   define MODEL_START_POS_Z  0.0
#   define MODEL_START_H  M_PI/2
#endif

#endif


#if (MODEL_GALLERIA_2)

//#		define DEFAULT_INPUT_PATH  "C:\\Vectors\\Galleria\\10_12_11_27_50\\"
#		define DEFAULT_INPUT_PATH  "C:\\Vectors\\Galleria\\10_12_13_16_45\\"

#   define MODEL_MAP_WIFI "galleria_f2.wifi3"
#   define MODEL_MAP_BLE  "galleria_f2.bea3"
#   define MODEL_MAP_MFP  "galleria_f2.mfp3"
#   define MODEL_MAP_MM   "galleria_f2.map"

#   define MODEL_MAP_SIZE_X_M 219
#   define MODEL_MAP_SIZE_Y_M 180
//#   define MODEL_MAP_SIZE_X_M 20
//#   define MODEL_MAP_SIZE_Y_M 13
#   define MODEL_MFP_CEIL_M 2.5

#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M * 100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M * 100)
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_MM_CEIL_M 0.1

#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)
#   define MODEL_VIS_DOWNSCALE 20 // 2 for Aisle
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 0

#   define MODEL_LOC_FRAME_lat0       5.0
#   define MODEL_LOC_FRAME_lon0       0.75
#   define MODEL_LOC_FRAME_alfa_lat   (-0.01745329)
#   define MODEL_LOC_FRAME_alfa_lon  0.01745329
#   define MODEL_LOC_FRAME_heading   0

#if MODEL_START_BY_WIFI == 0
#   define MODEL_START_POS_X  4
#   define MODEL_START_POS_Y  1.0
#   define MODEL_START_POS_Z  0.0
#   define MODEL_START_H  M_PI/2
#endif

#endif

#if (MODEL_VALLEY)

//#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_003\\"
// 1 floor track
//#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_004\\"
// bad small track, only 1 floor (crossing escalators)
//#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_005\\"
// 1 floor only track (bottom to center-right)
//#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_006\\"
// long "horizontal" track around escalator, from right to left, only 1 floor
#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_007\\"
// long track , only 1 floor

#   define MODEL_MAP_WIFI "_valley.wifi3"
#   define MODEL_MAP_BLE  "valley.bea3"
//#   define MODEL_MAP_MFP  "valley.mfp3"
#   define MODEL_MAP_MFP  "2_5_valley.mfp3"
#   define MODEL_MAP_MM   "valley.map"

#   define MODEL_MAP_SIZE_X_M 99
#   define MODEL_MAP_SIZE_Y_M 78

// !!!!!!!! don't forget to change
//#   define MODEL_MFP_CEIL_M 1.0
#   define MODEL_MFP_CEIL_M 2.5

#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M * 100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M * 100)
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_MM_CEIL_M 0.1

#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)
#   define MODEL_VIS_DOWNSCALE 10 // 2 for Aisle
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 1

#   define MODEL_LOC_FRAME_lat0       5.0
#   define MODEL_LOC_FRAME_lon0       0.75
#   define MODEL_LOC_FRAME_alfa_lat   (-0.01745329)
#   define MODEL_LOC_FRAME_alfa_lon  0.01745329
#   define MODEL_LOC_FRAME_heading   0

#if MODEL_START_BY_WIFI == 0
#   define MODEL_START_POS_X  4
#   define MODEL_START_POS_Y  1.0
#   define MODEL_START_POS_Z  0.0
#   define MODEL_START_H  M_PI/2
#endif

#endif

#if (MODEL_VALLEY_1)


//#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_003\\"
// 1 floor track
//#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_004\\"
// bad small track, only 1 floor (crossing escalators)
//#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_005\\"
// 1 floor only track (bottom to center-right)
//#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_006\\"
// long "horizontal" track around escalator, from right to left, only 1 floor
#		define DEFAULT_INPUT_PATH  "C:\\Deliveries\\Valley\\Logs\\Navigator\\track_007\\"
// long track , only 1 floor

//#   define MODEL_MAP_WIFI "valley_1.wifi3"
#   define MODEL_MAP_WIFI "none.wifi3"
#   define MODEL_MAP_BLE  "valley.bea3"
//#   define MODEL_MAP_MFP  "valley_1.mfp3"
#   define MODEL_MAP_MFP  "2_5_valley_1.mfp3"
#   define MODEL_MAP_MM   "valley_1.map"

#   define MODEL_MAP_SIZE_X_M 99
#   define MODEL_MAP_SIZE_Y_M 78

// !!!!!!!! don't forget to change
#   define MODEL_MFP_CEIL_M 2.5
//#   define MODEL_MFP_CEIL_M 2.5

#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M * 100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M * 100)
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_MM_CEIL_M 0.1

#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)
#   define MODEL_VIS_DOWNSCALE 10 // 2 for Aisle
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 0

#   define MODEL_LOC_FRAME_lat0       5.0
#   define MODEL_LOC_FRAME_lon0       0.75
#   define MODEL_LOC_FRAME_alfa_lat   (-0.01745329)
#   define MODEL_LOC_FRAME_alfa_lon  0.01745329
#   define MODEL_LOC_FRAME_heading   0

#if MODEL_START_BY_WIFI == 0
#   define MODEL_START_POS_X  4
#   define MODEL_START_POS_Y  1.0
#   define MODEL_START_POS_Z  0.0
#   define MODEL_START_H  M_PI/2
#endif

#endif

#if (MODEL_DPLAZA_4)
#   define DEFAULT_INPUT_PATH "c:\\Vectors\\tests_25_08\\reinit\\"

#   define MODEL_MAP_WIFI "derb_plaza_4.wifi3"
#   define MODEL_MAP_BLE  "derb_plaza_4.bea3"
#   define MODEL_MAP_MFP  "derb_plaza_4.mfp3"
#   define MODEL_MAP_MM   "derb_plaza_4.map"

#   define MODEL_MAP_SIZE_X_M 27
#   define MODEL_MAP_SIZE_Y_M 19
#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M*100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M*100)

#   define MODEL_MM_CEIL_M 0.1
#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)

#   define MODEL_VIS_DOWNSCALE 5
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 0

#   define MODEL_MFP_CEIL_M 1.0
#   define MODEL_MG_B_X -4.0
#   define MODEL_MG_B_Y 3
#   define MODEL_MG_B_Z 26
#   define MODEL_MG_B_SIG 100*100
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_LOC_FRAME_lat0       55.724253553756192
#   define MODEL_LOC_FRAME_lon0       37.653916182652893
#   define MODEL_LOC_FRAME_alfa_lat   0.000000162179509
#   define MODEL_LOC_FRAME_alfa_lon  -0.000000271000819
#   define MODEL_LOC_FRAME_heading  -31.327662731809170

#endif

#if (MODEL_TARGET)
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\tests_25_08\\reinit\\"
#   define DEFAULT_INPUT_PATH "V:\\Sensors\\Pedestrian\\Target\\10_03_14_07\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\Target\\10_03_13_51\\"

#   define MODEL_MAP_WIFI "target.wifi3"
#   define MODEL_MAP_BLE  "none.bea3"
#   define MODEL_MAP_MFP  "target.mfp3"
#   define MODEL_MAP_MM   "target.map"

#   define MODEL_MAP_SIZE_X_M 58
#   define MODEL_MAP_SIZE_Y_M 35
#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M*100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M*100)

#   define MODEL_MM_CEIL_M 0.1
#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)

#   define MODEL_VIS_DOWNSCALE 5
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 0

#   define MODEL_MFP_CEIL_M 1.0
#   define MODEL_MG_B_X -4.0
#   define MODEL_MG_B_Y 3
#   define MODEL_MG_B_Z 26
#   define MODEL_MG_B_SIG 100*100
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_LOC_FRAME_lat0       55.724253553756192
#   define MODEL_LOC_FRAME_lon0       37.653916182652893
#   define MODEL_LOC_FRAME_alfa_lat   0.000000162179509
#   define MODEL_LOC_FRAME_alfa_lon  -0.000000271000819
#   define MODEL_LOC_FRAME_heading  -31.327662731809170

#endif

#if (MODEL_ST2)
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\tests_25_08\\reinit\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\ST\\10_15_13_35_54\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\ST\\10_15_08_07_06\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\ST\\10_14_14_49_10\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\ST\\track_021\\"
#   define DEFAULT_INPUT_PATH "c:\\Vectors\\ST\\Long\\track_003\\"

//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\Target\\10_03_13_51\\"

#   define MODEL_MAP_WIFI "st_2.wifi3"
#   define MODEL_MAP_BLE  "none.wifi3"
#   define MODEL_MAP_MFP  "st_2.mfp3"
#   define MODEL_MAP_MM   "st_2.map"

#   define MODEL_MAP_SIZE_X_M 23
#   define MODEL_MAP_SIZE_Y_M 19
#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M*100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M*100)

#   define MODEL_MM_CEIL_M 0.1
#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)

#   define MODEL_VIS_DOWNSCALE 5
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 0

#   define MODEL_MFP_CEIL_M 1.0
#   define MODEL_MG_B_X -4.0
#   define MODEL_MG_B_Y 3
#   define MODEL_MG_B_Z 26
#   define MODEL_MG_B_SIG 100*100
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_LOC_FRAME_lat0       55.724253553756192
#   define MODEL_LOC_FRAME_lon0       37.653916182652893
#   define MODEL_LOC_FRAME_alfa_lat   0.000000162179509
#   define MODEL_LOC_FRAME_alfa_lon  -0.000000271000819
#   define MODEL_LOC_FRAME_heading  -31.327662731809170

#endif

#if (MODEL_OFFICE_MAP)
//#   define DEFAULT_INPUT_PATH  "C:\\Vectors\\For_AISLE\\track_05\\"
//#   define DEFAULT_INPUT_PATH  "C:\\Vectors\\Test_08_07\\track_01\\"
//#   define DEFAULT_INPUT_PATH  "C:\\Vectors\\Test_08_07\\track_02_for_RC\\"
//#   define DEFAULT_INPUT_PATH  "C:\\Vectors\\test_30_07\\1\\in\\"
#   define DEFAULT_INPUT_PATH  "C:\\Vectors\\test_30_07\\2\\in\\"
//#  define DEFAULT_INPUT_PATH "c:\\Vectors\\Test_08_07\\small_test\\"
//#   define DEFAULT_INPUT_PATH "c:\\Vectors\\test_20_08\\"

//#   define MODEL_MAP_WIFI "ble.wifi2"

#   define MODEL_MAP_WIFI "Spirit_Floor6.wifi3"
#   define MODEL_MAP_BLE  "Spirit_Floor6.bea3"
#   define MODEL_MAP_MFP  "Spirit_Floor6.mfp3"
#   define MODEL_MAP_MM   "Spirit_Floor6.map"

#   define MODEL_MAP_SIZE_X_M 40
#   define MODEL_MAP_SIZE_Y_M 12
#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M*100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M*100)

#   define MODEL_MM_CEIL_M 0.1
#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)

#   define MODEL_VIS_DOWNSCALE 5
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 0

#if 1
//#   define MODEL_MAP_MFP  "Spirit_Floor6.mfp3"
#   define MODEL_MFP_CEIL_M 1.0
#   define MODEL_MG_B_X -4.0
#   define MODEL_MG_B_Y 3
#   define MODEL_MG_B_Z 26
#   define MODEL_MG_B_SIG 100*100
#else
#   define MODEL_MAP_MFP  "Spirit_Floor6_250.mfp"
#   define MODEL_MFP_CEIL_M 2.5
#endif
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#if MODEL_START_BY_WIFI == 0
#   define MODEL_START_POS_X  6.3
#   define MODEL_START_POS_Y  4.4
#   define MODEL_START_POS_Z  0.0
#   define MODEL_START_H  0
#endif

#   define MODEL_LOC_FRAME_lat0       55.724253553756192
#   define MODEL_LOC_FRAME_lon0       37.653916182652893
#   define MODEL_LOC_FRAME_alfa_lat   0.000000162179509
#   define MODEL_LOC_FRAME_alfa_lon  -0.000000271000819
#   define MODEL_LOC_FRAME_heading  -31.327662731809170

#endif





#if (MODEL_AFIMALL)
//#   define DEFAULT_INPUT_PATH  "v:\\Sensors\\Pedestrian\\Afimall2\\M0003\\e3\\"
//#   define DEFAULT_INPUT_PATH  "v:\\Sensors\\Pedestrian\\Afimall2\\M0003\\t5\\"
//#   define DEFAULT_INPUT_PATH  "v:\\Sensors\\Pedestrian\\Afimall2\\T0003\\"
//#   define DEFAULT_INPUT_PATH  "v:\\Sensors\\Pedestrian\\AfeeMall\\T3002\\"
#   define DEFAULT_INPUT_PATH  "v:\\Sensors\\Pedestrian\\AfeeMall\\T2600\\"
//#   define DEFAULT_INPUT_PATH  "c:\\tmp\\T3002\\"
//#   define MODEL_MAP_WIFI "Afimall_mix.wifi3"
//#   define MODEL_MAP_MFP  "Afimall_mix.mfp3"
//#   define MODEL_MAP_WIFI "Afimall.wifi3"
//#   define MODEL_MAP_MFP  "Afimall_smoothed.mfp3"
//#   define MODEL_MAP_MM   "Afimall.map"
#   define MODEL_MAP_WIFI "Afimall2-3.wifi3"
#   define MODEL_MAP_BLE "not_avaliable.wifi3"
#   define MODEL_MAP_MFP  "Afimall2-3.mfp3"
#   define MODEL_MAP_MM   "Afimall2-3.map"


#   define MODEL_MAP_SIZE_X_M 420
#   define MODEL_MAP_SIZE_Y_M 130
#   define MODEL_MFP_CEIL_M 2.5

#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M * 100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M * 100)
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_MM_CEIL_M 0.2

#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)
#   define MODEL_VIS_DOWNSCALE 24
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 1

//e1
//#   define MODEL_START_POS_X   (52.8*5)
//#   define MODEL_START_POS_Y  (10.3 * 5)
//#   define MODEL_START_POS_Z  0.0
//#   define MODEL_START_H  -1.9055
//e2
//#   define MODEL_START_POS_X   (243.00)
//#   define MODEL_START_POS_Y  (35.5)
//#   define MODEL_START_POS_Z  1.0
//#   define MODEL_START_H  1.8789
//e3
//#   define MODEL_START_POS_X   (256.75)
//#   define MODEL_START_POS_Y  (35.3)
//#   define MODEL_START_POS_Z  1.0
//#   define MODEL_START_H  1.2416

//t1
//#   define MODEL_START_POS_X   (250.7)
//#   define MODEL_START_POS_Y  (32.35)
//#   define MODEL_START_POS_Z  1.0
//#   define MODEL_START_H  -2.9055
//t2
//#   define MODEL_START_POS_X   (158.9)
//#   define MODEL_START_POS_Y  (54.0)
//#   define MODEL_START_POS_Z  1.0
//#   define MODEL_START_H  -0.4324
//t3
//#   define MODEL_START_POS_X   (261.9)
//#   define MODEL_START_POS_Y  (33.94)
//#   define MODEL_START_POS_Z  1.0
//#   define MODEL_START_H   2.9196
//t4
//#   define MODEL_START_POS_X   (155.85)
//#   define MODEL_START_POS_Y  (48.82)
//#   define MODEL_START_POS_Z  1.0
//#   define MODEL_START_H   0.58
//t5
#   define MODEL_START_POS_X   (150.707028)
#   define MODEL_START_POS_Y  (44.308219)
#   define MODEL_START_POS_Z  1.0
#   define MODEL_START_H   -0.3279


#   define MODEL_LOC_FRAME_lat0       55.747538788755740
#   define MODEL_LOC_FRAME_lon0       37.536895870323946
#   define MODEL_LOC_FRAME_alfa_lat   0.000000158865719
#   define MODEL_LOC_FRAME_alfa_lon  -0.000000280180342
#   define MODEL_LOC_FRAME_heading   -59.693942054014855


#endif

#if (MODEL_PANASONIC_SAEDO)
#   define DEFAULT_INPUT_PATH  "c:\\Churikov\\Tasks\\HIOL\\Demonstrations\\Panasonic1\\tracks\\T01\\"
#   define MODEL_MAP_WIFI "panasonic-saedo.wifi3"
#   define MODEL_MAP_BLE  "panasonic-saedo.bea3"
#   define MODEL_MAP_MFP  "panasonic-saedo.mfp3"
#   define MODEL_MAP_MM   "panasonic-saedo.map"

#   define MODEL_MAP_SIZE_X_M 118
#   define MODEL_MAP_SIZE_Y_M 46
#   define MODEL_MFP_CEIL_M 2.0

#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M * 100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M * 100)
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_MM_CEIL_M 0.1

#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)
#   define MODEL_VIS_DOWNSCALE 5
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 1


#   define MODEL_LOC_FRAME_lat0       55.738319006448805
#   define MODEL_LOC_FRAME_lon0       37.522771060466766
#   define MODEL_LOC_FRAME_alfa_lat   1.5731304288731103e-007
#   define MODEL_LOC_FRAME_alfa_lon  -2.7849554655055437e-007
#   define MODEL_LOC_FRAME_heading   -62.707


#if MODEL_START_BY_WIFI == 0
#   define MODEL_START_POS_X  -50
#   define MODEL_START_POS_Y  -31
#   define MODEL_START_POS_Z  2.0
#   define MODEL_START_H  0
#endif

#endif

#if (MODEL_AEON_HIGASHI)
#   define DEFAULT_INPUT_PATH  "c:\\Churikov\\Tasks\\HIOL\\Demonstration\\AEON\\Tracks\\T03\\"
#   define MODEL_MAP_WIFI "aeon_higashi.wifi3"
#   define MODEL_MAP_BLE  "aeon_higashi.bea3"
#   define MODEL_MAP_MFP  "aeon_higashi.mfp3"
#   define MODEL_MAP_MM   "aeon_higashi.map"

#   define MODEL_MAP_SIZE_X_M 115
#   define MODEL_MAP_SIZE_Y_M 102
#   define MODEL_MFP_CEIL_M 2.5

#   define MODEL_MAP_SIZE_X_SM (MODEL_MAP_SIZE_X_M * 100)
#   define MODEL_MAP_SIZE_Y_SM (MODEL_MAP_SIZE_Y_M * 100)
#   define MODEL_MFP_CEIL_SM   (MODEL_MFP_CEIL_M   * 100)

#   define MODEL_MM_CEIL_M 0.1

#   define MODEL_MM_SCALE_FACTOR (MODEL_MM_CEIL_M * 100)
#   define MODEL_VIS_DOWNSCALE 10
#   define MODEL_MAP_FLOOR_MIN 0
#   define MODEL_MAP_FLOOR_MAX 1


#   define MODEL_LOC_FRAME_lat0       55.738319006448805
#   define MODEL_LOC_FRAME_lon0       37.522771060466766
#   define MODEL_LOC_FRAME_alfa_lat   1.5731304288731103e-007
#   define MODEL_LOC_FRAME_alfa_lon  -2.7849554655055437e-007
#   define MODEL_LOC_FRAME_heading   -62.707


#if MODEL_START_BY_WIFI == 0
#   define MODEL_START_POS_X  -50
#   define MODEL_START_POS_Y  -31
#   define MODEL_START_POS_Z  2.0
#   define MODEL_START_H  0
#endif

#endif


class ModelConfig
{
    public:
        ModelConfig() {};
        ~ModelConfig() {};

        // model configuration
        static bool LoadConfig( char *cfg_file );
        static bool SaveConfig( char *cfg_file );
        static void DefaultConfig();

        // personalizer settings
    public:
        static void SetPersonaParams( PF::tPersoneParams *newPersoneParams );
        static PF::tPersoneParams GetPersonaParams();
        static void SetPersonalyzerEnabled( bool newPersonalyzerEnabled );
        static bool GetPersonalyzerEnabled();
    private:
        static PF::tPersoneParams PersoneParams;
        static bool PersonalyzerEnabled;

        // model folder settings
    public:
        static bool SetInputPath( char * str );
        static bool SetWorkingPath( char * str );
        static bool SetOutputPath( char * str );
        static std::string GetOutputPath();
        static std::string InputPath;
        static char fAutoPath;

    private:
        static std::string WorkingPath;
        static std::string OutputPath;
        static void GenerateAutoPath( char *cur_out_path );

        // magnetic params
    public:
        //static PF::tMagneticSensorParams GetMagneticSensorParams();
        //static void SetMagneticSensorParams( PF::tMagneticSensorParams Params );
        static bool IsFixMagneticBias();
        static void SetFixMagneticBias( bool value );
        static bool GetMagneticBias( int64_t *pTime, double *pBias );
        static void SetMagneticBias( int64_t Time, double *pBias );
        static void GetMagneticBiasCov( double *pBiasCov );
        static void SetMagneticBiasCov( double *pBiasCov );

    private:
        static double MagneticBias[3];
        static double MagneticBiasCov[3];
        static long long MagneticBiasTime;
        static bool bFixMagneticBias;
        static bool bMagneticBiasValid;
};

#endif //MODEL_CONFIG_H

