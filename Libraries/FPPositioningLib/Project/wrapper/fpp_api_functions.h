/*
 * fpp_api_functions.h
 *
 *  Created on: Oct 28, 2016
 *      Author: melhoushi
 */

#ifndef FPP_API_FUNCTIONS_H_
#define FPP_API_FUNCTIONS_H_

#include <sys/stat.h>

#include "fpp_data_types.h"
#include "fpp_callback_api_functions.h"

FppLogger pf_log = NULL;
FILE *pf_log_file = NULL;

#define GET_FPL_VERSION_PLAIN_NUMBER(major, minor, build)   ((major)*1000000 + (minor)*1000 + (build))
#define ENABLE_MAP_MATCHING_FPL_VERSION (GET_FPL_VERSION_PLAIN_NUMBER (1,14,0))

void fpp_get_api_version(FppApiVersionStruct* version_struct_pointer)
{
	FppSessionHandle temp_handle = FPEngine_new();

	FppApiVersionStruct version = FPEngine_getVersionNumber(temp_handle);
	version_struct_pointer->major = version.major;
	version_struct_pointer->minor = version.minor;
	version_struct_pointer->build = version.build;
	version_struct_pointer->releaseId = version.releaseId;

	FPEngine_free(temp_handle);
}

FppSessionHandle fpp_create_session()
{
	FppSessionHandle fpp_handle = FPEngine_new();
	FPEngine_restart(fpp_handle);

	return fpp_handle;
}
static char *get_file_content_helper(const char *fname, unsigned long*fsize)
{
	struct stat file_buf;
	stat(fname, &file_buf);
	size_t mapSizeInBytes = file_buf.st_size;
	*fsize = mapSizeInBytes;
	if (mapSizeInBytes == 0) return NULL;
	char *pMap = (char *)malloc(mapSizeInBytes);
	FILE *pF = fopen(fname, "rb");
	bool fp_success = (pF != 0);
	//    _ASSERT( pF );

	if (pF)
	{
		if (fread(pMap, mapSizeInBytes, 1, pF) == 0)
		{
			//_ASSERT( 0 );
			fp_success = false;
			printf("[WARNING] FPP: Failed to open map file: %s\n", fname);
			free(pMap);
			pMap = NULL;
		}
		fclose(pF);
	}
	else
	{
		free(pMap);
		pMap = NULL;
	}
	
	return pMap;
}
FppReturnStatusEnum fpp_init_session(FppSessionHandle fpp_handle, 
	FppMagMapStruct* fpp_mag_map_data, FppCallbackHandle* fpp_mag_callback_handle, 
	FppWiFiMapStruct* fpp_wifi_map_data, FppCallbackHandle* fpp_wifi_callback_handle, 
	FppBleMapStruct* fpp_ble_map_data, FppCallbackHandle* fpp_ble_callback_handle, 
	FppBlpMapStruct* fpp_blp_map_data, FppCallbackHandle* fpp_blp_callback_handle, 
	FppCallbackHandle* fpp_mixed_callback_handle,
	const FppSettingsStruct* fpp_init, const char* log_dir_path)
{
	FppReturnStatusEnum overall_status = FPP_STATUS_UNKNOWN_ERROR;
	FppReturnStatusEnum mag_status = FPP_STATUS_UNKNOWN_ERROR;
	FppReturnStatusEnum wifi_status = FPP_STATUS_UNKNOWN_ERROR;
	FppReturnStatusEnum ble_status = FPP_STATUS_UNKNOWN_ERROR;
	FppReturnStatusEnum blp_status = FPP_STATUS_UNKNOWN_ERROR;
	FppReturnStatusEnum mm_status = FPP_STATUS_UNKNOWN_ERROR;
	uint8_t count_init_success = 0;

	FPEngine_setUpdateMFP(fpp_handle, fpp_init->magEnable);
	FPEngine_setUpdateWiFi(fpp_handle, fpp_init->wifiEnable);
	FPEngine_setUpdateBLE(fpp_handle, fpp_init->bleEnable);
	FPEngine_setUpdateBLEProximity(fpp_handle, fpp_init->blpEnable);

	FPEngine_setMagFilterEnabled(fpp_handle, false);
   	FPEngine_setMixedFilterEnabled(fpp_handle, true);

	FppApiVersionStruct version = FPEngine_getVersionNumber(fpp_handle);
	// Debug log file
#if IRL_ENABLED_YES == IRL_DEBUG_FILES
	if ( FPEngine_getLogsCount( fpp_handle ) > 0 )
	{
		char pf_log_path[1024];
		char short_name[50];
		char ext[] = ".txt";
		char *name = short_name;
		bool succes = false;
		int name_size = FPEngine_getLogDescription( fpp_handle, short_name, sizeof( short_name ), 0 );

		if ( ( name_size + sizeof( ext ) ) < sizeof( short_name ) )
		{
			name = strcat( short_name, ext );
		}
		strcpy(pf_log_path, log_dir_path);
		strcat(pf_log_path, name);
		pf_log_file = fopen(pf_log_path, "w");
		if(pf_log_file != NULL)
		{
			pf_log = CLogger_new( pf_log_file );
			succes = FPEngine_setLogger( fpp_handle, pf_log, 0 );
		}

		if(succes == false)
		{
			printf("[WARNING] FPP: Failure calling FPEngine_setLogger\n");
		}
	}
#endif

    // Venue parameters for target Central :
	FppVenueStruct venue = {0};
	venue.id = fpp_init->id;
	venue.origin_lattitude=fpp_init->origin_lattitude;
	venue.origin_longitude=fpp_init->origin_longitude;
	venue.origin_altitude=fpp_init->origin_altitude;
	venue.origin_azimuth=fpp_init->origin_azimuth;
	venue.floors_count=fpp_init->floors_count;
	venue.floor_height=5;
	venue.size_x=fpp_init->size_x;
	venue.size_y=fpp_init->size_y;
	venue.alfa=fpp_init->alfa;
	venue.beta=fpp_init->beta;
	overall_status = FPEngine_setVenueParams(fpp_handle, &venue);
	if (overall_status == FPP_STATUS_UNKNOWN_ERROR)
	{
		printf("[WARNING] FPP: Failed to set Venue Params\n");
	}
	int maxfloors = venue.floors_count - 1;
	int minfloors = 0;

	if (overall_status == FPP_STATUS_SUCCESS)
	{
		double min_p = 0.001; //validity threshold

		// Magnetic FP Initialization
		if(fpp_init->magEnable)
		{
			// Bias covariance matrix from file
			bool b_set_bias_cov = fpp_init->b_set_bias_cov;
			FppMagneticCalibrationParam bias_cov = {0};
			bias_cov.mX = fpp_init->mX;
			bias_cov.mY = fpp_init->mY;
			bias_cov.mZ = fpp_init->mZ;
			memcpy(bias_cov.covarianceMatrix, fpp_init->bias_cov_matrix, sizeof(bias_cov.covarianceMatrix));
			// Open Magnetic File Map
			struct stat mag_file_buf;
			stat(fpp_init->fp_magnetic_base_file, &mag_file_buf);
			size_t mfpMapSizeInBytes = mag_file_buf.st_size;
			char *pMfpMap = (char *)malloc(mfpMapSizeInBytes);
			FILE *pF = fopen(fpp_init->fp_magnetic_base_file, "rb");
			bool mag_fp_success = (pF != 0);
			//    _ASSERT( pF );
			if (pF)
			{
				if (fread(pMfpMap, mfpMapSizeInBytes, 1, pF) == 0)
				{
					//_ASSERT( 0 );
					mag_fp_success = false;
					printf("[WARNING] FPP: Failed to open FPP magnetic map file: %s\n", fpp_init->fp_magnetic_base_file);
				}
				fclose(pF);
			}
			if(mag_fp_success == true)
			{
				if(fpp_init->mag_map_version==VERSION4)
				{
					mag_status = FPEngine_initializeMFP_Ex(fpp_handle, pMfpMap, mfpMapSizeInBytes);
				}else{
					mag_status = FPEngine_initializeMFP(fpp_handle, pMfpMap, mfpMapSizeInBytes, venue.size_x, venue.size_y, fpp_init->mag_grid_size, minfloors, maxfloors);
				}
				if (mag_status == FPP_STATUS_SUCCESS)
				{
					if (b_set_bias_cov)
					{
						FPEngine_setMagneticBias(fpp_handle, &bias_cov);
					}

					FPEngine_setPositionCallbackMFP( fpp_handle, fpp_mag_callback_handle->m_pCbkFpObj );
					printf("FPP: Successfully initialized MFP Engine\n");

					count_init_success++;
				}
				else
				{
					printf("[WARNING] FPP: Failed to initialize MFP Engine\n");
					FPEngine_free(fpp_handle);
				}
			}
			else
			{
				mag_status = FPP_STATUS_UNKNOWN_ERROR;
			}
		}

		// WiFi FP Initialization
		if (fpp_init->wifiEnable)
		{
			// Open WiFi File Map
			struct stat wifi_file_buf;
			stat(fpp_init->fp_wifi_base_file, &wifi_file_buf);
			size_t wfpMapSizeInBytes = wifi_file_buf.st_size;
			char *pWfpMap = (char *)malloc(wfpMapSizeInBytes);
			FILE *pF = fopen(fpp_init->fp_wifi_base_file, "rb");
			bool wifi_fp_success = (pF != 0);
			//    _ASSERT( pF );
			if (pF)
			{
				if (fread(pWfpMap, wfpMapSizeInBytes, 1, pF) == 0)
				{
					//_ASSERT( 0 );
					wifi_fp_success = false;
					printf("[WARNING] FPP: Failed to open FPP WiFi map file: %s\n", fpp_init->fp_wifi_base_file);
				}
				fclose(pF);
			}
			if (wifi_fp_success == true)
			{
				wifi_status = FPEngine_initializeWiFI_Ex(fpp_handle, pWfpMap, wfpMapSizeInBytes);

				if (wifi_status == FPP_STATUS_SUCCESS)
				{
					FPEngine_setPositionCallbackWiFi(fpp_handle, fpp_wifi_callback_handle->m_pCbkFpObj);
					printf("FPP: Successfully initialized WiFi Engine\n");
					count_init_success++;
				}
				else
				{
					printf("FPP: Failed to initialize WiFi Engine\n");
					FPEngine_free(fpp_handle);
				}
			}
			else
			{
				wifi_status = FPP_STATUS_UNKNOWN_ERROR;
			}
		}
		// BLE FP Initialization
		if (fpp_init->bleEnable)
		{
			// Open BLE File Map
			struct stat ble_file_buf;
			stat(fpp_init->fp_ble_base_file, &ble_file_buf);
			size_t bfpMapSizeInBytes = ble_file_buf.st_size;
			char *pBfpMap = (char *)malloc(bfpMapSizeInBytes);
			FILE *pF = fopen(fpp_init->fp_ble_base_file, "rb");
			bool ble_fp_success = (pF != 0);
			//    _ASSERT( pF );
			if (pF)
			{
				if (fread(pBfpMap, bfpMapSizeInBytes, 1, pF) == 0)
				{
					//_ASSERT( 0 );
					ble_fp_success = false;
					printf("[WARNING] FPP: Failed to open FPP BLE map file: %s\n", fpp_init->fp_ble_base_file);
				}
				fclose(pF);
			}
			if (ble_fp_success == true)
			{
				if(fpp_init->mag_map_version==VERSION4)
				{
					ble_status = FPEngine_initializeBLE_Ex(fpp_handle, pBfpMap, bfpMapSizeInBytes);
				}else{
					ble_status = FPEngine_initializeBLE(fpp_handle, pBfpMap, bfpMapSizeInBytes, min_p);
				}
				

				if (ble_status == FPP_STATUS_SUCCESS)
				{
					FPEngine_setPositionCallbackBLE(fpp_handle, fpp_ble_callback_handle->m_pCbkFpObj);
					printf("FPP: Successfully initialized BLE Engine\n");
					count_init_success++;
				}
				else
				{
					printf("[WARNING] FPP: Failed to initialize BLE Engine\n");
				}
			}
			else
			{
				ble_status = FPP_STATUS_UNKNOWN_ERROR;
			}
		}
		// BLE FP Initialization
		if (fpp_init->blpEnable)
		{
			// Open BLE File Map
			struct stat blp_file_buf;
			stat(fpp_init->fp_blp_base_file, &blp_file_buf);
			size_t blpMapSizeInBytes = blp_file_buf.st_size;
			char *pBlpMap = (char *)malloc(blpMapSizeInBytes);
			FILE *pF = fopen(fpp_init->fp_blp_base_file, "rb");
			bool blp_fp_success = (pF != 0);
			//    _ASSERT( pF );
			if (pF)
			{
				if (fread(pBlpMap, blpMapSizeInBytes, 1, pF) == 0)
				{
					//_ASSERT( 0 );
					blp_fp_success = false;
					printf("[WARNING] FPP: Failed to open FPP BLP map file: %s\n", fpp_init->fp_ble_base_file);
				}
				fclose(pF);
			}
			if (blp_fp_success == true)
			{
				ble_status = FPEngine_initializeBLEProximity(fpp_handle, pBlpMap, blpMapSizeInBytes);
				if (ble_status == FPP_STATUS_SUCCESS)
				{
					FPEngine_setPositionCallbackBLEProximity(fpp_handle, fpp_blp_callback_handle->m_pCbkFpObj);
					printf("FPP: Successfully initialized BLP Engine\n");
					count_init_success++;
				}
				else
				{
					printf("[WARNING] FPP: Failed to initialize BLP Engine\n");
				}
			}
			else
			{
				ble_status = FPP_STATUS_UNKNOWN_ERROR;
			}
		}
		// map maptching

		if (GET_FPL_VERSION_PLAIN_NUMBER(version.major, version.minor,
										 version.build) >= ENABLE_MAP_MATCHING_FPL_VERSION)
		{
			if (fpp_init->mmEnable)
			{
				unsigned long fsize;
				char *pMMMap = get_file_content_helper(fpp_init->fp_mm_base_file, &fsize);
				if (pMMMap == NULL || fsize == 0)
				{
					printf("FPL venue map size is zero, disabling map matching");
					FPEngine_setUpdateMapMatching(fpp_handle, false);
					mm_status = FPP_STATUS_UNKNOWN_ERROR;
				}
				else
				{
					mm_status = FPEngine_initializeMapMatching(fpp_handle, pMMMap, fsize);
					if(mm_status == FPP_STATUS_SUCCESS){
						FPEngine_setUpdateMapMatching(fpp_handle, true);
						printf("FPP: Successfully initialized MM Engine\n");
						count_init_success++;
					}
					else
					{
						printf("[WARNING] FPP: Failed to initialize MM Engine\n");
					}
					
				}
			}
			else
			{

				FPEngine_setUpdateMapMatching(fpp_handle, false);
				printf("[WARNING] FPP: disable MM Engine\n");
			}
		}

		// Mixed Callback
		if (count_init_success > 0)
		{
			FPEngine_setPositionCallbackMixed(fpp_handle, fpp_mixed_callback_handle->m_pCbkFpObj);
		}

	}
	
	if (count_init_success == 0)
	{
		FPEngine_free(fpp_handle);
		overall_status = FPP_STATUS_UNKNOWN_ERROR;
	}

	return overall_status;
}

void fpp_set_start_position(FppSessionHandle fpp_handle, const FppOutputMessageStruct* position)
{
	FPEngine_setStartPosition(fpp_handle, position);
	FPEngine_restart(fpp_handle);
}

void fpp_process_tpp(FppSessionHandle fpp_handle, const FppTppMessageStruct* fpp_tpp)
{
	FPEngine_processTpnOutput(fpp_handle, fpp_tpp);
}

void fpp_process_wifi(FppSessionHandle fpp_handle, const FppWiFiMessageStruct* fpp_wifi)
{
	FPEngine_processWiFi(fpp_handle, fpp_wifi);
}

void fpp_process_ble(FppSessionHandle fpp_handle, const FppBleMessageStruct* fpp_ble)
{
	FPEngine_processBLE(fpp_handle, fpp_ble);
}

static FppOutputMessageStruct fppe_get_position(FppCallbackHandle* update_struct)
{
	update_struct->m_bIsUpdated = false;
	return update_struct->m_oPosSolution;
}

bool fppe_is_updated(FppCallbackHandle* update_struct)
{
	return update_struct->m_bIsUpdated;
}

bool fppe_get_solution(FppCallbackHandle* fppe_callback_handle, FppOutputMessageStruct *fppe_pos)
{
	FppOutputMessageStruct newPosition = fppe_get_position(fppe_callback_handle);

    fppe_pos->timestamp = newPosition.timestamp;
    fppe_pos->is_valid = newPosition.is_valid;
    fppe_pos->lattitude = newPosition.lattitude;
    fppe_pos->longitude = newPosition.longitude;
    fppe_pos->altitude = newPosition.altitude;
    fppe_pos->covariance_lat_lon[0][0] = newPosition.covariance_lat_lon[0][0];
    fppe_pos->covariance_lat_lon[1][0] = newPosition.covariance_lat_lon[1][0];
    fppe_pos->covariance_lat_lon[0][1] = newPosition.covariance_lat_lon[0][1];
    fppe_pos->covariance_lat_lon[1][1] = newPosition.covariance_lat_lon[1][1];
    fppe_pos->azimuth = newPosition.azimuth;
    fppe_pos->azimuth_std = newPosition.azimuth_std;
    fppe_pos->floor_number = newPosition.floor_number;
    fppe_pos->floor_std = newPosition.floor_std;

    fppe_callback_handle->m_bIsUpdated = false;

    return fppe_pos->is_valid;
}

void fpp_delete_session(FppSessionHandle fpp_handle)
{
	if (fpp_handle != NULL)
	{
		FPEngine_free(fpp_handle);
		fpp_handle = NULL;
	}

	if (pf_log != NULL)
	{
		CLogger_free(pf_log);
		pf_log = NULL;
	}

	if (pf_log_file != NULL)
	{
		fclose(pf_log_file);
		pf_log_file = NULL;
	}

}


#endif /* FPP_API_FUNCTIONS_H_ */
