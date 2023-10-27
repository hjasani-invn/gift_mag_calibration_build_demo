#ifndef SETTINGSPARSER_HPP
#define SETTINGSPARSER_HPP

#include "Fppe.hpp"
#include "jsoncons/json.hpp"
#include "VenueEx.hpp"

using jsoncons::json;
using jsoncons::json_deserializer;

static	void parseVenueParams( json venueParams, VenueEx &venue );

void parseSettings( std::string settings_file, // input JSON file
                    std::string &name,						  // output
                    std::string &fp_bases_folder,
                    std::string &in_data_folder,
                    std::string &out_log_folder,
                    VenueEx &venue,
                    bool   &magEnable,
                    bool   &wifiEnable,
                    bool   &bleEnable,
                    bool   &bleProximityEnable,
                    double &magneticCellsize,
                    double *magneticFPsizes,
                    Fppe::Position &startPosition
                  )
{

    json args = json::parse_file( settings_file );

    // JSON file parsing
    {
        try
        {
            name = args["name"].as<std::string>();
            fp_bases_folder = args["folder_fp"].as<std::string>();
            in_data_folder = args["folder_in"].as<std::string>();
            out_log_folder = args["folder_out"].as<std::string>();
            //std::cout << name << ", " << fp_bases_folder << ", " << in_data_folder << ", " << out_log_folder << std::endl;

            json venueParams = args["venue"];
            //std::cout << venueJSON.size() << "    " << venueJSON << std::endl;
            parseVenueParams( venueParams, venue );

            magneticCellsize = args["magnetic_cellsize"].as<double>();
            magEnable = args["magnetic_enable"].as<bool>();
            wifiEnable = args["wifi_enable"].as<bool>();
            bleEnable = args["ble_enable"].as<bool>();
            bleProximityEnable = args["ble_proximity_enable"].as<bool>();

            if ( args.count( "magnetic_fp_sizes" ) > 0 )
            {
                json sizes = args["magnetic_fp_sizes"];
                magneticFPsizes[0] = sizes[0].as<double>();
                magneticFPsizes[1] = sizes[1].as<double>();
                magneticFPsizes[2] = sizes[2].as<double>();
            }
            else
                magneticFPsizes[0] = magneticFPsizes[1] = magneticFPsizes[2] = -1;

            if ( args.count( "start_position" ) > 0 )
            {
                json start_position = args["start_position"];
                startPosition.lattitude = start_position["lattitude"].as<double>();
                startPosition.longitude = start_position["longitude"].as<double>();
                startPosition.altitude = start_position["altitude"].as<double>();
                startPosition.floor_number = start_position["floor_number"].as<int>();
                startPosition.azimuth = start_position["heading"].as<double>();

                json covariance_lat_lon = start_position["covariance_lat_lon"];
                json covariance_line0 = covariance_lat_lon[0];
                json covariance_line1 = covariance_lat_lon[1];

                startPosition.covariance_lat_lon[0][0] = covariance_line0[0].as<double>();
                startPosition.covariance_lat_lon[0][1] = covariance_line0[1].as<double>();;
                startPosition.covariance_lat_lon[1][0] = covariance_line1[0].as<double>();;
                startPosition.covariance_lat_lon[1][1] = covariance_line1[1].as<double>();;;

                startPosition.azimuth_std = start_position["heading_std"].as<double>();
                startPosition.floor_std = 0;
                startPosition.timestamp = 0;
                startPosition.is_valid = true;
            }
        }
        catch ( const std::exception& e )
        {
            std::cerr << e.what() << std::endl;
        }
    }
}

static	void parseVenueParams( json venueParams, VenueEx &venue )
{
    venue.id = venueParams["id"].as<int>();
    venue.origin_lattitude = venueParams["origin_lattitude"].as<double>();
    venue.origin_longitude = venueParams["origin_longitude"].as<double>();
    venue.origin_altitude = venueParams["origin_altitude"].as<double>();
    venue.origin_azimuth = venueParams["origin_azimuth"].as<double>();
    venue.floors_count = venueParams["floors_count"].as<int>();

	venue.floor_height = 5.0;

	if (venueParams.has_member("floor_height"))
	{
		venue.floor_height = venueParams["floor_height"].as<double>();
	}
	
    venue.plan_size_x = venue.size_x = venueParams["size_x"].as<double>();
    venue.plan_size_y = venue.size_y = venueParams["size_y"].as<double>();

    if (venueParams.has_member("plan_size_x"))
        venue.plan_size_x = venueParams["plan_size_x"].as<double>();
    if (venueParams.has_member("plan_size_y"))
        venue.plan_size_y = venueParams["plan_size_y"].as<double>();

    venue.alfa = venueParams["alfa"].as<double>();
    venue.beta = venueParams["beta"].as<double>();
}

#endif //SETTINGSPARSER_HPP
