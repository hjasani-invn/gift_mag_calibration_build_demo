#define _CRT_SECURE_NO_WARNINGS
#include "CFppe.h"
#include "Data.h"
#include "string.h"
#include <sys/stat.h> //TODO replace
#include <assert.h>
#include <stdlib.h>  

typedef struct CIPositionUpdateObjectTag
{
	CIPositionCallbackObject* pos_cbk_obj;

	FILE* outputstream;	/// stream for log file
	FILE* outputstream_dbg;
	char fname[1024];
	uint32_t counter;
} CIPositionUpdateObject;

void func_update_vars(void* object_handle, double x, double y, double f, double h, double s, double t, const CParticle* p, int n)
{
	CIPositionUpdateObject* p_update_object = (CIPositionUpdateObject*)object_handle;

	fprintf(p_update_object->outputstream, "%15.4f", x);
	fprintf(p_update_object->outputstream, "%15.4f", y);
	fprintf(p_update_object->outputstream, "%15.4f", f);
	fprintf(p_update_object->outputstream, "%15.4f", h);
	fprintf(p_update_object->outputstream, "%15.4f", s);
	fprintf(p_update_object->outputstream, "%25.4f", t);
	fprintf(p_update_object->outputstream, "\n");
}

void func_update_struct_ptr(void* object_handle, const CPosition* pos)
{
	CIPositionUpdateObject* p_update_object = (CIPositionUpdateObject*)object_handle;

	fprintf(p_update_object->outputstream, "\t<Placemark>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t\t<ExtendedData><SchemaData schemaUrl=\"#%s\">", p_update_object->fname);

	fprintf(p_update_object->outputstream, "\t\t\t<SimpleData name=\"number\">");
	fprintf(p_update_object->outputstream, "d", p_update_object->counter++);
	fprintf(p_update_object->outputstream, "</SimpleData>");
	fprintf(p_update_object->outputstream, "\n");

	fprintf(p_update_object->outputstream, "\t\t\t<SimpleData name=\"latitude\">");
	fprintf(p_update_object->outputstream, "%8.8f", pos->lattitude);
	fprintf(p_update_object->outputstream, "</SimpleData>");
	fprintf(p_update_object->outputstream, "\n");

	fprintf(p_update_object->outputstream, "\t\t\t<SimpleData name=\"longitude\">");
	fprintf(p_update_object->outputstream, "%8.8f", pos->longitude);
	fprintf(p_update_object->outputstream, "</SimpleData>");
	fprintf(p_update_object->outputstream, "\n");

	fprintf(p_update_object->outputstream, "\t\t\t<SimpleData name=\"timestamp\">");
	fprintf(p_update_object->outputstream, "%8.8f", pos->timestamp);
	fprintf(p_update_object->outputstream, "</SimpleData>");
	fprintf(p_update_object->outputstream, "\n");

	fprintf(p_update_object->outputstream, "\t\t\t<SimpleData name=\"cov_lat\">");
	fprintf(p_update_object->outputstream, "%8.8f", pos->covariance_lat_lon[0][0]);
	fprintf(p_update_object->outputstream, "</SimpleData>");
	fprintf(p_update_object->outputstream, "\n");

	fprintf(p_update_object->outputstream, "\t\t\t<SimpleData name=\"cov_lon\">");
	fprintf(p_update_object->outputstream, "%8.8f", pos->covariance_lat_lon[1][1]);
	fprintf(p_update_object->outputstream, "</SimpleData>");
	fprintf(p_update_object->outputstream, "\n");

	fprintf(p_update_object->outputstream, "\t\t\t<SimpleData name=\"cov_lat_lon\">");
	fprintf(p_update_object->outputstream, "%8.8f", pos->covariance_lat_lon[1][0]);
	fprintf(p_update_object->outputstream, "</SimpleData>");
	fprintf(p_update_object->outputstream, "\n");

	fprintf(p_update_object->outputstream, "\t\t</SchemaData></ExtendedData>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t\t\t<Point><coordinates>");
	fprintf(p_update_object->outputstream, "%8.8f", pos->longitude);
	fprintf(p_update_object->outputstream, ",");
	fprintf(p_update_object->outputstream, "%8.8f", pos->lattitude);
	fprintf(p_update_object->outputstream, "</coordinates></Point>");
	fprintf(p_update_object->outputstream, "\n");

	fprintf(p_update_object->outputstream, "\t</Placemark>");
	fprintf(p_update_object->outputstream, "\n");
}

CIPositionUpdateObject* CIPositionUpdateObject_new(const char* output_log_file, const char* output_log_name_dbg)
{
	CIPositionUpdateObject* p_update_object = (CIPositionUpdateObject*)malloc(sizeof(CIPositionUpdateObject));
	
	p_update_object->outputstream = fopen(output_log_file, "w");

	if (p_update_object->outputstream == NULL)
	{
		free(p_update_object);
		return NULL;
	}

	fprintf(p_update_object->outputstream, "<?xml version=\"1.0\" encoding=\"utf-8\" ?>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "<Document><Folder><name>%s</name>", output_log_file);
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "<Schema name=\"%s\" id=\"%s\">", output_log_file, output_log_file);
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"Name\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"Description\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"number\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"latitude\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"longitude\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"timestamp\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"cov_lat\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"cov_lon\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "\t<SimpleField name=\"cov_lat_lon\" type=\"string\"></SimpleField>");
	fprintf(p_update_object->outputstream, "\n");
	fprintf(p_update_object->outputstream, "</Schema>");
	fprintf(p_update_object->outputstream, "\n");

	if (strlen(output_log_name_dbg) > 0)
	{
		p_update_object->outputstream_dbg = fopen(output_log_name_dbg, "w");
	}

	strcpy(p_update_object->fname, output_log_file);
	p_update_object->counter = 0;

	return p_update_object;
}

void CIPositionUpdateObject_free(CIPositionUpdateObject* p_update_object)
{
	fprintf(p_update_object->outputstream, "</Folder></Document></kml>");
	fprintf(p_update_object->outputstream, "\n");
	fclose(p_update_object->outputstream);
	fclose(p_update_object->outputstream_dbg);

	free(p_update_object);
	p_update_object = NULL;
}

int main()
{
    FILE *log_file_0 = 0;
    CILogger *log_obj_0 = 0;
    CTpnOutput tpn_data;
	CIPositionUpdateObject* mfp_update_object = CIPositionUpdateObject_new("mag_out.kml", "mag_out_dbg.log");
	CIPositionUpdate mfp_update_struct = { &func_update_vars, &func_update_struct_ptr, mfp_update_object };
	CIPositionCallbackObject *pos_cbk_obj = PositionCallbackObject_new(mfp_update_struct);
    CIFPEngine* fppe_obj = FPEngine_new();

    FPEngine_restart( fppe_obj );

    if ( FPEngine_getLogsCount( fppe_obj ) > 0 )
    {
        char short_name[50];
        char ext[] = "_c.txt";
        char *name = short_name;
        int name_size = FPEngine_getLogDescription( fppe_obj, short_name, sizeof( short_name ), 0 );

        if ( ( name_size + sizeof( ext ) ) < sizeof( short_name ) )
        {
            name = strcat( short_name, ext );
        }

        log_file_0 = fopen( name, "w" );
        log_obj_0 = CLogger_new( log_file_0 );

        bool succes = FPEngine_setLogger( fppe_obj, log_obj_0, 0 );

    }

    Venue venue;
    // venue parameters
    venue.id = 1;
    venue.size_x = 142;
    venue.size_y = 110;
    venue.floors_count = 1;
    // venue frame parameters
    venue.origin_lattitude = 37.303872359106229; // [-90;90]
    venue.origin_longitude = -121.865964103907100; // [-180;180]
    venue.origin_azimuth = -49.533150237103442;     // [-180;180]
    venue.origin_altitude = 0;
    venue.alfa = 0;   // 0 - for default scaling
    venue.beta = 0;   // 0 - for default scaling
    CReturnStatus status = FPEngine_setVenueParams( fppe_obj, &venue );

    char mfp_file[] = "fp/target_sj.mfp3";

    struct stat file_buf;
    stat( mfp_file, &file_buf );
    size_t mfpMapSizeInBytes = file_buf.st_size;
    char *pMfpMap = ( char * )malloc( mfpMapSizeInBytes );

    FILE *pF = fopen( mfp_file, "rb" );
    assert( pF );

    fread( pMfpMap, mfpMapSizeInBytes, 1, pF );
    fclose( pF );

    double mag_cell_size = 1;
    FPEngine_initializeMFP( fppe_obj, pMfpMap, mfpMapSizeInBytes, venue.size_x, venue.size_y, mag_cell_size, 0, venue.floors_count - 1 );
    FPEngine_setPositionCallbackMFP( fppe_obj, pos_cbk_obj );

    FPEngine_setUpdateMFP( fppe_obj, true );
    FPEngine_setUpdateBLE( fppe_obj, false );
    FPEngine_setUpdateWiFi( fppe_obj, false );

    for ( size_t i = 0;  i < sizeof( tpn_plain_data ) / sizeof( *tpn_plain_data ); ++i )
    {
        tpn_data.timestamp = tpn_plain_data[i][0];

        tpn_data.position.lattitude = tpn_plain_data[i][1];
        tpn_data.position.longitude = tpn_plain_data[i][2];
        tpn_data.position.user_heading = tpn_plain_data[i][3];
        tpn_data.position.sigma_north = tpn_plain_data[i][4];
        tpn_data.position.sigma_east = tpn_plain_data[i][5];
        tpn_data.position.sigma_user_heading = tpn_plain_data[i][6];
        tpn_data.position.misalignment = tpn_plain_data[i][7];
        tpn_data.position.sigma_misalignment = tpn_plain_data[i][8];
        tpn_data.position.floor = tpn_plain_data[i][9];
        tpn_data.position.altitude = tpn_plain_data[i][10];
        tpn_data.position.sigma_altitude = tpn_plain_data[i][11];
        tpn_data.position.navigation_phase = tpn_plain_data[i][12];
        tpn_data.position.fidgeting_flag = tpn_plain_data[i][13];
        tpn_data.position.is_valid = tpn_plain_data[i][14] != 0;

        tpn_data.attitude.orientation_id = tpn_plain_data[i][15];
        tpn_data.attitude.roll = tpn_plain_data[i][16];
        tpn_data.attitude.pitch = tpn_plain_data[i][17];
        tpn_data.attitude.heading = tpn_plain_data[i][18];
        tpn_data.attitude.sigma_roll = tpn_plain_data[i][19];
        tpn_data.attitude.sigma_pitch = tpn_plain_data[i][20];
        tpn_data.attitude.sigma_heading = tpn_plain_data[i][21];
        tpn_data.attitude.is_valid = tpn_plain_data[i][22] != 0;

        tpn_data.pdr.stride_length = tpn_plain_data[i][23];
        tpn_data.pdr.is_valid = tpn_plain_data[i][24] != 0;

        tpn_data.mag_meas.mX = tpn_plain_data[i][25];
        tpn_data.mag_meas.mY = tpn_plain_data[i][26];
        tpn_data.mag_meas.mZ = tpn_plain_data[i][27];
        tpn_data.mag_meas.sigma_mX = tpn_plain_data[i][28];
        tpn_data.mag_meas.sigma_mY = tpn_plain_data[i][29];
        tpn_data.mag_meas.sigma_mZ = tpn_plain_data[i][30];
        tpn_data.mag_meas.covarianceMatrix[0][0] = tpn_plain_data[i][31];
        tpn_data.mag_meas.covarianceMatrix[0][1] = tpn_plain_data[i][32];
        tpn_data.mag_meas.covarianceMatrix[0][2] = tpn_plain_data[i][33];
        tpn_data.mag_meas.covarianceMatrix[1][0] = tpn_plain_data[i][34];
        tpn_data.mag_meas.covarianceMatrix[1][1] = tpn_plain_data[i][35];
        tpn_data.mag_meas.covarianceMatrix[1][2] = tpn_plain_data[i][36];
        tpn_data.mag_meas.covarianceMatrix[2][0] = tpn_plain_data[i][37];
        tpn_data.mag_meas.covarianceMatrix[2][1] = tpn_plain_data[i][38];
        tpn_data.mag_meas.covarianceMatrix[2][2] = tpn_plain_data[i][39];
        tpn_data.mag_meas.level_of_calibration = tpn_plain_data[i][40];
        tpn_data.mag_meas.is_valid = tpn_plain_data[i][41] != 0;


        FPEngine_processTpnOutput( fppe_obj, &tpn_data );
    }

    if ( fppe_obj ) FPEngine_free( fppe_obj );

    if ( pos_cbk_obj ) PositionCallbackObject_free( pos_cbk_obj );
	
	if (mfp_update_object) CIPositionUpdateObject_free(mfp_update_object);

    if ( log_obj_0 ) CLogger_free( log_obj_0 );

    if ( log_file_0 ) fclose( log_file_0 );

    return 0;
}
