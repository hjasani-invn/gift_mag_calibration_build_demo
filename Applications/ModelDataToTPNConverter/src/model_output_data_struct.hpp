#ifndef MODEL_OUTPUT_DATA_STRUCT_HPP
#define MODEL_OUTPUT_DATA_STRUCT_HPP
#include <stdint.h>

struct model_output_data
{
    static const int number_of_fields = 25;
	int64_t time_tag; // ms
	double  geo_lat;  // deg
	double  geo_lon;  // deg
	double  sigma_n;  // m - uncertainty in North direction
	double  sigma_e;  // m - uncertainty in East direction
	int16_t floor;
	double  height;   // m - height from sea level
	double  sigma_height; // m
	double  misalignment; // deg - Misalignment angle
	double  user_heading; //deg
	double  sigma_user_heading; //deg
	double  roll; // deg
	double  pitch; // deg
	double  heading; // deg - device heading
	double  roll_std; // deg
	double  pitch_std; // deg
	double  heading_std; // deg - device heading
	int8_t     frame_flag; // -1, 0, 1
	double  mag_x; // mG - Magnetic vector in BF
	double  mag_y; // mG
	double  mag_z; // mG
  bool    mag_valid; // magnetic data valid
  int8_t	navigation_phase; // Navigation phase flag :
	                          // 0 – navigation is not available
		                      // 1 – navigation is available
	//
	int8_t  fidgeting_flag;   // optional parameter can absent
	double stride_length;
};


#endif
