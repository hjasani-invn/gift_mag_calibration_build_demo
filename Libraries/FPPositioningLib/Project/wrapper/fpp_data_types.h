/*
 * fpp_data_types.h
 *
 *  Created on: Oct 28, 2016
 *      Author: melhoushi
 */

#ifndef FPL_DATA_TYPES_H_
#define FPL_DATA_TYPES_H_

#include "CFppe.h"
#include "CMagData.h"
#include "CTpnData.h"
#include "Venue.h"

typedef CReturnStatus FppReturnStatusEnum;
#define FPP_STATUS_SUCCESS STATUS_SUCCESS
#define FPP_STATUS_UNKNOWN_ERROR STATUS_UNKNOWN_ERROR

typedef CVersionNumber FppApiVersionStruct;
typedef CTpnOutput FppTppMessageStruct;
typedef CWiFiScanResult FppWiFiMessageStruct;
typedef CBleScanResult FppBleMessageStruct;
typedef CPosition FppOutputMessageStruct;
typedef Venue FppVenueStruct;
typedef CMagneticCalibrationParam FppMagneticCalibrationParam;

typedef struct IplFppPositionUpdateTag
{
	CIPositionCallbackObject* m_pCbkFpObj;
	FppOutputMessageStruct m_oPosSolution;
	bool m_bIsUpdated;
	uint32_t counter;

	FILE* outputstream;	/// stream for log file
	char fname[1024];
	FILE* outputstream_dbg;
	char fname_dbg[1024];
} IplFppPositionUpdate;

typedef CIFPEngine* FppSessionHandle;
typedef IplFppPositionUpdate FppCallbackHandle;
typedef CILogger* FppLogger;

typedef enum FppMapVersionTag{
	NOT_FOUND = 0,
	VERSION3 = 3,
	VERSION4 = 4
}FppMapVersionEnum;

typedef struct FppSettingsStructTag {
	bool magEnable;
	bool wifiEnable;
	bool bleEnable;
	bool blpEnable;
	bool mmEnable;


	/** Venue information, defines transformation to local frame*/
	uint64_t id;                /**< venue identifier */
	double origin_lattitude;    /**< bottom left corner lattitude [deg] [-90..+90] */
	double origin_longitude;    /**< bottom left corner longitude [deg] [-180..+180] */
	double origin_altitude;     /**< zero floor altitude [m] */
	double origin_azimuth;      /**< venue x-axix rotation to true north [rad] [-180..+180] */
	uint16_t floors_count;      /**< total floor number in venue */
	double size_x;              /**< x axis venue size [m] */
	double size_y;              /**< y axis venue size [m] */
	double alfa;                /**< axis transforamtion param (scale factor) */
	double beta;                /**< axis transforamtion param (scale factor) */

	char* fp_magnetic_base_file;
	char* fp_wifi_base_file;
	char* fp_ble_base_file;
	char* fp_blp_base_file;
	char* fp_mm_base_file;

	bool b_set_bias_cov;
	double bias_cov_matrix[3][3]; /**< mag field covariance matrix [uT^2], column order\n
								   * cov(mX, mX), cov(mY, mX), cov(mZ, mX)\n
								   * cov(mX, mY), cov(mY, mY), cov(mZ, mY)\n
								   * cov(mX, mZ), cov(mY, mZ), cov(mZ, mZ) */
	double mX;          /**<  magnetic field on x-axis [uT] */
	double mY;          /**<  magnetic field on y-axis [uT] */
	double mZ;          /**<  magnetic field on z-axis [uT] */

	double mag_grid_size;
	FppMapVersionEnum wifi_map_version;
	FppMapVersionEnum mag_map_version;
	FppMapVersionEnum ble_map_version;
	FppMapVersionEnum blp_map_version;
	FppMapVersionEnum mm_map_version;
} FppSettingsStruct;

typedef struct FppMagMapStructTag
{
   unsigned char* data_pointer_;
   unsigned long data_length_;
} FppMagMapStruct;

typedef struct FppWiFiMapStructTag
{
   unsigned char* data_pointer_;
   unsigned long data_length_;
} FppWiFiMapStruct;

typedef struct FppBleMapStructTag
{
   unsigned char* data_pointer_;
   unsigned long data_length_;
} FppBleMapStruct;

typedef struct FppBlpMapStructTag
{
   unsigned char* data_pointer_;
   unsigned long data_length_;
} FppBlpMapStruct;


#endif /* FPL_DATA_TYPES_H_ */
