#ifndef SETTINGSPARSER_HPP
#define SETTINGSPARSER_HPP

#include <map>
#include "Fpbl.hpp"
#include "fpeDataTypes.h"
#include "fpVenue.hpp"
#include "jsoncons/json.hpp"
using jsoncons::json;
using jsoncons::json_deserializer;

//#define DEFAULT_WIFI_SCAN_PERIOD   4000 // ms; scaning period of BLE sensor
//#define DEFAULT_BLE_SCAN_PERIOD   4000 // ms; scaning period of BLE sensor


struct bleProximityData
{
    bool      enable;
    double    max_position_uncertainty;
    double    skip_time;
    double    sigma;
    double    max_distance;
    bool      ble_enable_prox_only;
};


struct jsonVenue : public  MFPVenue, public WFPVenue, public BFPVenue
{
    jsonVenue();

    uint32_t wifi_scan_period; // ms, wifi scaning period
    uint32_t ble_scan_period; // ms, ble scaning period
    double   blp_height;
    BleBeaconType blp_type;

    std::vector <std::pair <uint64_t, uint64_t>>  wifi_ignore_list; // first - MAC template (0xAABBCC112233), first - MAC mask (0xFFFFFF000000)
    std::vector <std::pair <uint64_t, uint64_t>>  wifi_white_list; // first - MAC template (0xAABBCC112233), first - MAC mask (0xFFFFFF000000)
    std::vector <uint64_t>  ble_ignore_list;
    bleProximityData        ble_proximity_data;
};

static	void parseVenueParams(json venueParams, jsonVenue &venue);

static	void parseGridParams(json gridParams, Fpbl::Grid &grid);
static	void parseMagValidators(json magValidatorsObj, std::vector<std::string> &mag_validators);
static	void parseFPGridParams(json gridParams, Fpbl::FPGrid &grid);
static	void parseInputData(json settings, std::string &name,
    std::string &in_data_folder, std::string &in_mag_grid_file, 
    std::string &in_wifi_grid_file, std::string &in_ble_grid_file, std::string &in_portals_grid_file, std::string &in_cs_grid_file);
static	void parseOutputData(json settings, std::string &name, std::string &out_data_folder,
    std::string &out_mag_grid_file, std::string &out_wifi_grid_file, std::string &out_ble_grid_file,
    std::string &out_mag_fp_file, std::string &out_wifi_fp_file, std::string &out_ble_fp_file,
    int &mag_fp_format, int &wifi_fp_format, int &ble_fp_format);
static	void parseFileMasksParams(json fileMasksParams, std::map<std::string, std::string> &file_masks);
static	void parseProxParams(json proxData, bleProximityData &proxParams);


void parseSettings(std::string settings_file, // input JSON file
	std::string &name,						  // output
	std::string &in_data_folder,
	std::string &in_mag_grid_file,
	std::string &in_wifi_grid_file,
	std::string &in_ble_grid_file,
    std::string &in_portals_grid_file,
	std::string &in_cs_grid_file,
    std::string &grid_folder,
	std::string &fp_bases_folder,
	std::string &out_data_folder,
	std::string &out_mag_grid_file,
	std::string &out_wifi_grid_file,
	std::string &out_ble_grid_file,
	std::string &out_mag_fp_file,
	std::string &out_wifi_fp_file,
	std::string &out_ble_fp_file,
  std::string &in_ble_fprox_file,
  int &mag_fp_format,
	int &wifi_fp_format,
	int &ble_fp_format,
	jsonVenue &venue,
	bool   &magEnable,
	bool   &wifiEnable,
  bool   &bleEnable,
  bool   &bleProxEnable,
	Fpbl::Grid &mag_grid,
	Fpbl::Grid &wifi_grid,
	Fpbl::Grid &ble_grid,
	Fpbl::FPGrid &mag_fp,
	Fpbl::FPGrid &wifi_fp,
	Fpbl::FPGrid &ble_fp,
	std::string  &default_mag_validators,
	std::vector<std::string> &mag_validators,
  std::map<std::string, std::string> &file_masks
  );

#endif //SETTINGSPARSER_HPP
