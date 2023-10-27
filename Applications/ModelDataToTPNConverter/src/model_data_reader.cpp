/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @project     RTFPPL
* @file        tpn_data_reader.cpp
* @author      D Churikov
* @date        13 Apr 2016
* @brief       TPN data reader
*/
/*****************************************************************************/

#include <vector>
#include <sstream>
#include <iostream>

#include"model_data_reader.hpp"
//---------------------------------------------------------------------------
bool model_data_reader::read_next_line(model_output_data &output)
{
  bool res = true;

	std::string line;
	if (is->eof())
	{
		res = false;
		return res;
	}
	std::getline(*is, line);

	std::stringstream buf(line);
	std::string token;
	std::vector<std::string> result;

	const char delim = ',';
	while (std::getline(buf, token, delim))
	{
		result.push_back(token);
	}
	uint16_t size_of_result = result.size();

  if (size_of_result < ( + model_output_data::number_of_fields)) // '+' operation is neccesary to avoid odr-using of internal class constant
  {
		res = false;
		return res;
	}

	int16_t tmp;

	std::stringstream ss;
	ss << result[0];
	ss >> output.time_tag;
	ss.str("");
	ss.clear();

	ss << result[1];
	ss >> output.geo_lat;
	ss.str("");
	ss.clear();

	ss << result[2];
	ss >> output.geo_lon;
	ss.str("");
	ss.clear();

	ss << result[3];
	ss >> output.sigma_n;
	ss.str("");
	ss.clear();

	ss << result[4];
	ss >> output.sigma_e;
	ss.str("");
	ss.clear();

	ss << result[5];
	ss >> output.floor;
        output.floor++;
	ss.str("");
	ss.clear();

	ss << result[6];
	ss >> output.height;
	ss.str("");
	ss.clear();

	ss << result[7];
	ss >> output.sigma_height;
	ss.str("");
	ss.clear();

	ss << result[8];
	ss >> output.misalignment;
	ss.str("");
	ss.clear();

	ss << result[9];
	ss >> output.user_heading;
	ss.str("");
	ss.clear();

	ss << result[10];
	ss >> output.sigma_user_heading;
	ss.str("");
	ss.clear();

	ss << result[11];
	ss >> output.roll;
	ss.str("");
	ss.clear();

	ss << result[12];
	ss >> output.pitch;
	ss.str("");
	ss.clear();

	ss << result[13];
	ss >> output.heading;
	ss.str("");
	ss.clear();

	ss << result[14];
	ss >> output.roll_std;
	ss.str("");
	ss.clear();

	ss << result[15];
	ss >> output.pitch_std;
	ss.str("");
	ss.clear();

	ss << result[16];
	ss >> output.heading_std;
	ss.str("");
	ss.clear();

	ss << result[17];
	ss >> tmp;
	output.frame_flag = tmp;
	ss.str("");
	ss.clear();

	ss << result[18];
	ss >> output.mag_x;
	ss.str("");
	ss.clear();

	ss << result[19];
	ss >> output.mag_y;
	ss.str("");
	ss.clear();

	ss << result[20];
	ss >> output.mag_z;
	ss.str("");
	ss.clear();

	ss << result[21];
	ss >> tmp;
	output.navigation_phase = tmp;
	ss.str("");
	ss.clear();


	// optional parameter
	if (size_of_result < model_output_data::number_of_fields)
	{
      output.fidgeting_flag = 0; // default value
		return res;
	}

	ss << result[22];
	ss >> tmp;
	output.fidgeting_flag = tmp;
	ss.str("");
	ss.clear();

	ss << result[23];
	ss >> output.stride_length;
	ss.str("");
	ss.clear();

  ss << result[24];
  ss >> output.mag_valid;
  ss.str("");
  ss.clear();

    return res;
}

//---------------------------------------------------------------------------
/*
bool tpn_data_reader::find_preambule()
{
    uint8_t buffer[3];
    buffer[0] = buffer[1] = buffer[2] = 0;

    while ( !is->eof() )
    {
        is->read( ( char* )&buffer[2], 1 );

        if ( ( buffer[0] == 0x54 ) && ( buffer[1] == 0x50 ) && ( buffer[2] == 0x49 ) )
            return true;

        buffer[0] = buffer[1];
        buffer[1] = buffer[2];
    }

    return false;
}
*/
