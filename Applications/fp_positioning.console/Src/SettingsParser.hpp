#ifndef SETTINGSPARSER_HPP
#define SETTINGSPARSER_HPP

#include "Fppe.hpp"
#include "jsoncons/json.hpp"
#include "fpVenue.hpp"

using jsoncons::json;
using jsoncons::json_deserializer;

static	void parseVenueParams(json venueParams, BaseVenue &venue);

/** RTFPPL settings structure*/
struct RtfpplSettings
{
    PlatformType platform_type;     /**< type of platform  */

    RtfpplSettings::RtfpplSettings() :
        platform_type(PlatformType::pftp_PEDESTRIAN)
    {};
};

void parseSettings( std::string settings_file, // input JSON file
                    std::string &name,
                    std::string &fp_bases_folder,
                    std::string &in_data_folder,
                    std::string &out_log_folder,
                    BaseVenue &venue,
                    bool   &magEnable,
                    bool   &wifiEnable,
                    bool   &bleEnable,
                    bool   &bleProximityEnable,
                    bool   &mmEnable,
                    bool   &collaborationEnable,
                    double &magneticCellsize,
                    double *magneticFPsizes
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

            if (args.has_member("magnetic_cellsize"))
                magneticCellsize = args["magnetic_cellsize"].as<double>();
            
            if (args.has_member("magnetic_enable"))
                magEnable = args["magnetic_enable"].as<bool>();
            if (args.has_member("wifi_enable") == true)
                wifiEnable = args["wifi_enable"].as<bool>();
            if (args.has_member("WiFi_enable") == true)
                wifiEnable = args["WiFi_enable"].as<bool>();
            if (args.has_member("ble_enable") == true)
                bleEnable = args["ble_enable"].as<bool>();
            if (args.has_member("BLE_enable") == true)
                bleEnable = args["BLE_enable"].as<bool>();
            if (args.has_member("collaboration_enable") == true)
                collaborationEnable = args["collaboration_enable"].as<bool>();
            if (args.has_member("ble_collaboration") == true)
                collaborationEnable = args["ble_collaboration"].as<bool>();
            if (args.has_member("ble_proximity_enable"))
                bleProximityEnable = args["ble_proximity_enable"].as<bool>();
            if (args.has_member("mm_enable") == true)
                mmEnable = args["mm_enable"].as<bool>();

            if (args.count("magnetic_fp_sizes") > 0)
            {
                json sizes = args["magnetic_fp_sizes"];
                magneticFPsizes[0] = sizes[0].as<double>();
                magneticFPsizes[1] = sizes[1].as<double>();
                magneticFPsizes[2] = sizes[2].as<double>();
            }
            else
            {
                magneticFPsizes[0] = magneticFPsizes[1] = magneticFPsizes[2] = -1;
            }
        }
        catch ( const std::exception& e )
        {
            std::cerr << "json parsing error: ";
            std::cerr << e.what() << std::endl;
        }
    }
}

//static	void parseVenueParams( json venueParams, Venue &venue )
static	void parseVenueParams(json venueParams, BaseVenue &venue)
{
    venue.id = venueParams["id"].as<int>();
    venue.origin_lattitude = venueParams["origin_lattitude"].as<double>();
    venue.origin_longitude = venueParams["origin_longitude"].as<double>();
    venue.origin_altitude = venueParams["origin_altitude"].as<double>();
    venue.origin_azimuth = venueParams["origin_azimuth"].as<double>();

    venue.floors_count = 1;
    venue.floor_shift = 1;
    venue.floor_height = 5.0;
    venue.floor_zero_enable = false;

    if (venueParams.has_member("floors_count"))
        venue.floors_count = venueParams["floors_count"].as<int>();
    if (venueParams.has_member("floor_shift"))
        venue.floor_shift = venueParams["floor_shift"].as<int>();
    if (venueParams.has_member("floor_zero_enable"))
        venue.floor_zero_enable = venueParams["floor_zero_enable"].as<bool>();
    if (venueParams.has_member("floor_height"))
        venue.floor_height = venueParams["floor_height"].as<int>();
    if (venueParams.has_member("floor_height"))
        venue.floor_height = venueParams["floor_height"].as<int>();

    venue.size_x = venueParams["size_x"].as<double>();
    venue.size_y = venueParams["size_y"].as<double>();
    venue.alfa = venueParams["alfa"].as<double>();
    venue.beta = venueParams["beta"].as<double>();
}

bool  parseDebugSettings(std::string settings_file, Fppe::Position &startPosition, RtfpplSettings &rfppl_settings)
{
    bool result = true;

    // JSON file parsing
    try
    {
        json args = json::parse_file(settings_file);

        if (args.count("rtfppl_settings") > 0)
        {
            json rtfppl_settings_json = args["rtfppl_settings"];

            if (rtfppl_settings_json.has_member("platform_type"))
                rfppl_settings.platform_type = (PlatformType) rtfppl_settings_json["platform_type"].as<int>();
        }
        if (args.count("start_position") > 0)
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
            //startPosition.floor_std = start_position["floor_std"].as<double>();
            startPosition.floor_std = start_position["floor_number_std"].as<double>();
            startPosition.timestamp = start_position["timestamp"].as<double>();
            startPosition.is_valid = start_position["enabled"].as<bool>();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "debug settings-json parsing error: ";
        std::cerr << e.what() << std::endl;
        result = false;
    }
    
    return result;
}

#endif //SETTINGSPARSER_HPP
