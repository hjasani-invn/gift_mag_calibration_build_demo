/**
* \file fpeDataTypes.h
* \brief Fingerprint builder/positioning libraries API, base data types
* \author D Churikov (dchurikov@invensense.com)
* \version 0.1
* \copyright TDK, all rights reserved
* \date November 14, 2019
*/

/*! \C Shared data types
* \section
* \image
*/

#ifndef C_FPE_DATA_TYPES_H
#define C_FPE_DATA_TYPES_H

#include <stdint.h>

/** BLE beacon type enumeration type*/
typedef enum BleBeaconTypeTag
{
    bbt_unknown = 0,                    /**< unknown */
    bbt_proximity = 1,                  /**< general proximity beacon*/
    bbt_crowdsoursing = 2,              /**< crowdsoursing beacon*/
    bbt_crowdsoursing_assistance = 3,   /**< crowdsoursing assistance beacon*/
    bbt_height_assistance = 4,          /**< height assistance beacon*/
    bbt_bfp = 5,                        /**< bfp beacon*/
    bbt_restricted_area = 6,            /**< beacon for restricted areas designation*/
    bbt_restricted_assistance = 7       /**< assistance beacon for restricted areas designation*/
    //bbt_default = 8                     /**< default beacon*/ // has to be type with higest value
} BleBeaconType;

/**
 * platform type
 */
typedef enum  PlatformTypeTag
{
    pftp_PEDESTRIAN = 0,
    pftp_FORKLIFT = 1
}PlatformType;

/**
 * BLP pulling type
 */
typedef enum  ePullingTypeTag
{
    pt_nonePulling = 0,
    pt_softPulling = 1,
    pt_instantPulling = 2
}ePullingType;

/** BLE beacon data structure*/
typedef struct BleBeaconDataTag
{
    uint16_t major;         /**< beacon major number */
    uint16_t minor;         /**< beacon minor number */
    uint8_t uuid[16];       /**< beacon uuid 128 bit value */
    int8_t txPower;         /**< beacon tx power level [dbm] on 1m distance, as received from beacon*/
    uint16_t advInterval;   /**< advertisement interval [msec] - reserved*/

    BleBeaconType beacon_type;     /**<  */

    double lattitude;       /**< beacon location lattitude [deg] [-90..+90]*/
    double longitude;       /**< beacon location longitude [deg] [-180..+180] */
    double elevation;       /**< elevation of beacon over floor level*/
    int16_t floor;          /**< discrete floor number*/

    double azimuth;         /**< azimuth of ble radiation pattern median [deg] [-180..+180] */

    int8_t rxPower;         /**< beacon rx power level [dBm]*/
    double distance;        /**< estimate of distance to beacon[m]*/

    uint32_t reserved1;     /**< reserved*/
    double rms;             /**< horizontal position uncertainty, udefined if rms <= 0*/

    bool is_valid;          /**< position validity flag */
}BleBeaconData;

/** Portal type enumeration*/
typedef enum PortalTypeTag    // 0 - no portal, 1 - elevator, 2 - stairs, 3 - escalator, 4 - conveyer
{
    k_NoPortal = 0,
    k_Elevator = 1,
    k_Stairs = 2,
    k_Escalator = 3,
    k_Conveyor = 4,
} PortalType;

typedef enum OperationSystemTypeTag
{
    OS_UNKNOWN = 0,
    OS_ANDROID = 1,
    OS_IOS = 2,
    OS_LINUX = 3
} OperationSystemType;

#endif //C_FPE_DATA_TYPES_H
