#ifndef SETTINGSPARSER_HPP
#define SETTINGSPARSER_HPP

#include "Fpbl.hpp"
#include "jsoncons/json.hpp"
using jsoncons::json;
using jsoncons::json_deserializer;

static	void parseVenueParams(json venueParams, Venue &venue);
static	void parseGridParams(json gridParams, Fpbl::Grid &grid);

void parseSettings(std::string settings_file, // input JSON file
    std::string &name,						  // output
    std::string &in_data_folder,
    std::string &grid_folder,
    std::string &fp_bases_folder,
    std::string &out_log_folder,
    Venue &venue,
    bool   &magEnable,
    bool   &wifiEnable,
    bool   &bleEnable,
    int32_t &min_mag_measurements,
    int32_t &min_wifi_measurements,
    int32_t &min_ble_measurements,
    double  &mag_interpolation_interval,
    double  &wifi_interpolation_interval,
    double  &ble_interpolation_interval,
    Fpbl::Grid &mag_grid,
    Fpbl::Grid &wifi_grid,
    Fpbl::Grid &ble_grid,
    int &data_format
    )
{
	json args = json::parse_file(settings_file);

	// JSON file parsing
	{
		try
		{
			name = args["name"].as<std::string>();
			in_data_folder = args["folder_in"].as<std::string>();
			grid_folder = args["folder_grid"].as<std::string>();
			fp_bases_folder = args["folder_fp"].as<std::string>();
			out_log_folder = args["folder_out"].as<std::string>();
			//std::cout << name << ", " << fp_bases_folder << ", " << in_data_folder << ", " << out_log_folder << std::endl;

			json venueParams = args["venue"];
			//std::cout << venueJSON.size() << "    " << venueJSON << std::endl;
			parseVenueParams(venueParams, venue);

      magEnable = args["magnetic_enable"].as<bool>();
			wifiEnable = args["wifi_enable"].as<bool>();
			bleEnable = args["ble_enable"].as<bool>();
      min_mag_measurements = args["magnetic_min_meas_num"].as<int>();
      min_wifi_measurements = args["wifi_min_meas_num"].as<int>();
      min_ble_measurements = args["ble_min_meas_num"].as<int>();
      mag_interpolation_interval = args["magnetic_interpolatesize"].as<double>();
      wifi_interpolation_interval = args["wifi_interpolatesize"].as<double>();
      ble_interpolation_interval = args["ble_interpolatesize"].as<double>();
      mag_grid.size = args["magnetic_cellsize"].as<double>();
      ble_grid.size = args["wifi_cellsize"].as<double>();
      wifi_grid.size = args["ble_cellsize"].as<double>();

			data_format = args["data_format"].as<int>();
		
			json magneticGridParams = args["magnetic_grid"];
			parseGridParams(magneticGridParams, mag_grid);

			json wifiGridParams = args["wifi_grid"];
			parseGridParams(wifiGridParams, wifi_grid);

			json bleGridParams = args["ble_grid"];
			parseGridParams(bleGridParams, ble_grid);
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << std::endl;
		}
	}
}

static	void parseVenueParams(json venueParams, Venue &venue)
	{
			venue.id = venueParams["id"].as<int>();
			venue.origin_lattitude = venueParams["origin_lattitude"].as<double>();
			venue.origin_longitude = venueParams["origin_longitude"].as<double>();
			venue.origin_altitude = venueParams["origin_altitude"].as<double>();
			venue.origin_azimuth = venueParams["origin_azimuth"].as<double>();
			venue.floors_count = venueParams["floors_count"].as<int>();
			venue.size_x = venueParams["size_x"].as<double>();
			venue.size_y = venueParams["size_y"].as<double>();
			venue.alfa = venueParams["alfa"].as<double>();
			venue.beta = venueParams["beta"].as<double>();
			venue.magX = venueParams["magX"].as<double>();
			venue.magY = venueParams["magY"].as<double>();
			venue.magZ = venueParams["magZ"].as<double>();
	}

static	void parseGridParams(json gridParams, Fpbl::Grid &grid)
{
		grid.type = (Fpbl::CellType)gridParams["celltype"].as<int>();
		grid.size = gridParams["cellsize"].as<double>();
		{
			json min_sizes = gridParams["min"].as_vector<double>();
      int length = min_sizes.size();
			grid.min.x = min_sizes[0].as<double>();
			grid.min.y = min_sizes[1].as<double>();
			grid.min.floor = min_sizes[2].as<double>();
		}
		{
			json max_sizes = gridParams["max"];
			grid.max.x = max_sizes[0].as<double>();
			grid.max.y = max_sizes[1].as<double>();
			grid.max.floor = max_sizes[2].as<double>();
		}
}

#endif //SETTINGSPARSER_HPP
