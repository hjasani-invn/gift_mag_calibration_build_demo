/*
* fpp_callback_api_functions.h
*
*  Created on: Dec 21, 2016
*      Author: melhoushi
*/

#ifndef FPP_CALLBACK_API_FUNCTIONS_H_
#define FPP_CALLBACK_API_FUNCTIONS_H_

#include <sys/stat.h>

#include "fpp_data_types.h"

void func_update_vars(void* object_handle, double x, double y, double f, double h, double s, double t, const CParticle* p, int n)
{
	FppCallbackHandle* fpp_callback_handle = (FppCallbackHandle*)object_handle;

#if IRL_ENABLED_YES == IRL_DEBUG_FILES
	if (fpp_callback_handle->outputstream_dbg)
	{
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", x);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", y);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", f);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", h);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", s);
		fprintf(fpp_callback_handle->outputstream_dbg, "%25.4f", t);
	}
#endif
}

void func_update_struct_ptr(void* object_handle, const CPosition* pos)
{
	FppCallbackHandle* fpp_callback_handle = (FppCallbackHandle*)object_handle;

	fpp_callback_handle->m_oPosSolution = *pos;
	fpp_callback_handle->m_bIsUpdated = true;

#if IRL_ENABLED_YES == IRL_DEBUG_FILES
	if (fpp_callback_handle->outputstream)
	{
		fprintf(fpp_callback_handle->outputstream, "\t<Placemark>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t\t<ExtendedData><SchemaData schemaUrl=\"#%s\">", fpp_callback_handle->fname);
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t\t\t<SimpleData name=\"number\">");
		fprintf(fpp_callback_handle->outputstream, "%d", fpp_callback_handle->counter++);
		fprintf(fpp_callback_handle->outputstream, "</SimpleData>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t\t\t<SimpleData name=\"latitude\">");
		fprintf(fpp_callback_handle->outputstream, "%8.8f", pos->lattitude);
		fprintf(fpp_callback_handle->outputstream, "</SimpleData>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t\t\t<SimpleData name=\"longitude\">");
		fprintf(fpp_callback_handle->outputstream, "%8.8f", pos->longitude);
		fprintf(fpp_callback_handle->outputstream, "</SimpleData>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t\t\t<SimpleData name=\"timestamp\">");
		fprintf(fpp_callback_handle->outputstream, "%8.8f", pos->timestamp);
		fprintf(fpp_callback_handle->outputstream, "</SimpleData>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t\t\t<SimpleData name=\"cov_lat\">");
		fprintf(fpp_callback_handle->outputstream, "%8.8f", pos->covariance_lat_lon[0][0]);
		fprintf(fpp_callback_handle->outputstream, "</SimpleData>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t\t\t<SimpleData name=\"cov_lon\">");
		fprintf(fpp_callback_handle->outputstream, "%8.8f", pos->covariance_lat_lon[1][1]);
		fprintf(fpp_callback_handle->outputstream, "</SimpleData>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t\t\t<SimpleData name=\"cov_lat_lon\">");
		fprintf(fpp_callback_handle->outputstream, "%8.8f", pos->covariance_lat_lon[1][0]);
		fprintf(fpp_callback_handle->outputstream, "</SimpleData>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t\t</SchemaData></ExtendedData>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t\t\t<Point><coordinates>");
		fprintf(fpp_callback_handle->outputstream, "%8.8f", pos->longitude);
		fprintf(fpp_callback_handle->outputstream, ",");
		fprintf(fpp_callback_handle->outputstream, "%8.8f", pos->lattitude);
		fprintf(fpp_callback_handle->outputstream, "</coordinates></Point>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fprintf(fpp_callback_handle->outputstream, "\t</Placemark>");
		fprintf(fpp_callback_handle->outputstream, "\n");
	}

	if (fpp_callback_handle->outputstream_dbg)
	{
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4d", pos->timestamp);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.8f", pos->lattitude);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.8f", pos->longitude);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", pos->azimuth);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", pos->altitude);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15d", pos->floor_number);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", pos->covariance_lat_lon[0][0]);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", pos->covariance_lat_lon[0][1]);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", pos->covariance_lat_lon[1][0]);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", pos->covariance_lat_lon[1][1]);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", pos->azimuth_std);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15.4f", pos->floor_std);
		fprintf(fpp_callback_handle->outputstream_dbg, "%15d", pos->is_valid);
		fprintf(fpp_callback_handle->outputstream_dbg, "\n");
	}
#endif

}

FppCallbackHandle* fpp_create_callback_handle(const char* log_name, const char* log_dir_path)
{
	FppCallbackHandle* fpp_callback_handle = (FppCallbackHandle*)calloc(sizeof(FppCallbackHandle), sizeof(FppCallbackHandle)); 
	fpp_callback_handle->m_pCbkFpObj = NULL;

	CIPositionUpdate mfp_update_struct;
	mfp_update_struct.update_by_vars = &func_update_vars;
	mfp_update_struct.update_by_struct = &func_update_struct_ptr;

	fpp_callback_handle->m_bIsUpdated = false;
	fpp_callback_handle->m_oPosSolution.is_valid = false;
	fpp_callback_handle->m_oPosSolution.timestamp = 0;
	fpp_callback_handle->m_oPosSolution.lattitude = 0.0;
	fpp_callback_handle->m_oPosSolution.longitude = 0.0;
	fpp_callback_handle->m_oPosSolution.covariance_lat_lon[0][0] = 0.0;
	fpp_callback_handle->m_oPosSolution.covariance_lat_lon[0][1] = 0.0;
	fpp_callback_handle->m_oPosSolution.covariance_lat_lon[1][1] = 0.0;
	fpp_callback_handle->m_oPosSolution.covariance_lat_lon[1][0] = 0.0;
	fpp_callback_handle->m_oPosSolution.altitude = 0.0;
	fpp_callback_handle->m_oPosSolution.floor_std = 0.0;
	fpp_callback_handle->m_oPosSolution.azimuth = 0.0;
	fpp_callback_handle->m_oPosSolution.floor_number = 0;
	fpp_callback_handle->m_oPosSolution.azimuth_std = 0.0;

	mfp_update_struct.object_handle = fpp_callback_handle;

	fpp_callback_handle->counter = 0;

	// Initialize callback structure
	fpp_callback_handle->m_pCbkFpObj = PositionCallbackObject_new(mfp_update_struct);

	// Log file
#if IRL_ENABLED_YES == IRL_DEBUG_FILES
	char dbg_log_path[1024];
	strcpy(dbg_log_path, log_dir_path);
	strcat(dbg_log_path, log_name);
	strcat(dbg_log_path, ".txt");
	fpp_callback_handle->outputstream_dbg = fopen(dbg_log_path, "w");
	strcpy(fpp_callback_handle->fname_dbg, log_name);
	strcat(fpp_callback_handle->fname_dbg, ".txt");

	char log_path[1024];
	strcpy(log_path, log_dir_path);
	strcat(log_path, log_name);
	strcat(log_path, ".kml");
	fpp_callback_handle->outputstream = fopen(log_path, "w");
	strcpy(fpp_callback_handle->fname, log_name);
	strcat(fpp_callback_handle->fname, ".kml");
	if (fpp_callback_handle->outputstream)
	{
		fprintf(fpp_callback_handle->outputstream, "<?xml version=\"1.0\" encoding=\"utf-8\" ?>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "<Document><Folder><name>%s</name>", log_dir_path);
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "<Schema name=\"%s\" id=\"%s\">", fpp_callback_handle->fname, fpp_callback_handle->fname);
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"Name\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"Description\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"number\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"latitude\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"longitude\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"timestamp\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"cov_lat\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"cov_lon\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "\t<SimpleField name=\"cov_lat_lon\" type=\"string\"></SimpleField>");
		fprintf(fpp_callback_handle->outputstream, "\n");
		fprintf(fpp_callback_handle->outputstream, "</Schema>");
		fprintf(fpp_callback_handle->outputstream, "\n");
	}
#endif

	return fpp_callback_handle;
}

void fpp_delete_callback_handle(FppCallbackHandle* fpp_callback_handle)
{
	fpp_callback_handle->m_bIsUpdated = false;
	fpp_callback_handle->m_oPosSolution.is_valid = false;
	if (fpp_callback_handle->m_pCbkFpObj != NULL)
	{
		PositionCallbackObject_free(fpp_callback_handle->m_pCbkFpObj);
	}
	fpp_callback_handle->m_pCbkFpObj = NULL;
#if IRL_ENABLED_YES == IRL_DEBUG_FILES
	if (fpp_callback_handle->outputstream)
	{
		fprintf(fpp_callback_handle->outputstream, "</Folder></Document></kml>");
		fprintf(fpp_callback_handle->outputstream, "\n");

		fclose(fpp_callback_handle->outputstream);
		fpp_callback_handle->outputstream = NULL;
	}
	if (fpp_callback_handle->outputstream_dbg)
	{
		fclose(fpp_callback_handle->outputstream_dbg);
		fpp_callback_handle->outputstream_dbg = NULL;
	}
#endif

	free(fpp_callback_handle);
}

#endif