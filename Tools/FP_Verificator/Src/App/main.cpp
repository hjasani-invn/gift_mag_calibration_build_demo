
// C includes
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

// Cpp includes
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <stdint.h>
#include <regex>
#include <vector>
//#include <list>
#include <iterator>

#include "dirent.h"

#include "Fpbl.hpp"
#include "CmdReader.hpp"
#include "SettingsParser.hpp"

#include "CreateIRLPositions.hpp"
#include "CalculateNumberOfBleMeasuments.hpp"
#include "CalculateNumberOfMagMeasuments.hpp"
#include "CalculateNumberOfWiFiMeasuments.hpp"
#include "CalculateCoverage.hpp"

#ifdef _WIN32
#define  SLASH "\\"
#else
#define  SLASH "/"
#endif

using namespace FPVerificator;

bool load_mag_grid_file( const std::string &fname, Fpbl::MagneticGrid     &magnetic_grid);

bool load_wifi_grid_file(const std::string &fname, Fpbl::WiFiGrid     &wifi_grid);

bool load_ble_grid_file(const std::string &fname, Fpbl::BleGrid     &ble_grid);

bool load_irl_map_grid_file(const std::string &fname, char** pBuffer, uint32_t &fileSize);

bool save_position_occupancy(const std::string &fname, std::vector<OccupancyOfPosition> poslist);

int main(const int argc, const char** argv)
{
    Venue venue;

    std::string arg;
    std::stringstream ss;
    std::string name;

    std::string in_data_folder;
    std::string grid_folder;
    std::string fp_bases_folder;
    std::string out_log_folder;

    Fpbl::Grid magnetic_grid;
    Fpbl::Grid wifi_grid;
    Fpbl::Grid ble_grid;

    bool   magEnable;
    bool   wifiEnable;
    bool   bleEnable;
    int32_t min_mag_measurements;
    int32_t min_wifi_measurements;
    int32_t min_ble_measurements;
    double  mag_interpolation_interval;
    double  wifi_interpolation_interval;
    double  ble_interpolation_interval;

    bool success = true;

    std::string settings_file; // must be in JSON format
    // command line parsing
    success &= setOptionFromCmd((const int)argc, (const char **)argv, "--settings", &settings_file);

    int tmp;
    parseSettings(settings_file, // input JSON file	with settings
        name,					 // output
        in_data_folder,
        grid_folder,
        fp_bases_folder,
        out_log_folder,
        venue,
        magEnable,
        wifiEnable,
        bleEnable,
        min_mag_measurements,
        min_wifi_measurements,
        min_ble_measurements,
        mag_interpolation_interval,
        wifi_interpolation_interval,
        ble_interpolation_interval,
        magnetic_grid,
        wifi_grid,
        ble_grid,
        tmp
        );

    double mag_grid_size = magnetic_grid.size;
    double wifi_grid_size = wifi_grid.size;
    double ble_grid_size = ble_grid.size;

    DIR *dir_pointer;
    struct dirent *entry;
  
    dir_pointer = opendir(grid_folder.c_str());    
    
    std::vector<std::string> fileslist;
    while ((entry = readdir(dir_pointer)))
    {
        if (entry->d_type != DT_DIR)
        {
            std::string file_name(entry->d_name);
            {
                fileslist.push_back(file_name);
            }

        }

    }
    std::sort(fileslist.begin(), fileslist.end());

    std::regex mag_grid_mask = std::regex("(.*)(.maggrid)");
    std::regex wifi_grid_mask = std::regex("(.*)(.wifigrid)");
    std::regex ble_grid_mask = std::regex("(.*)(.blegrid)");
    std::regex imm_map_grid_mask = std::regex("(.*)(.dat)");

    std::string wifiGridFileName = "";
    std::string bleGridFileName = "";
    std::string magGridFileName = "";
    std::string immMapGridFileName = "";

    for (auto it = fileslist.begin(); it != fileslist.end(); it++)
    {
        std::string filename = *it;
        if (std::regex_match(filename, wifi_grid_mask))
            wifiGridFileName = grid_folder + SLASH + filename;
        if (std::regex_match(filename, ble_grid_mask))
            bleGridFileName = grid_folder + SLASH + filename;
        if (std::regex_match(filename, mag_grid_mask))
            magGridFileName = grid_folder + SLASH + filename;
        if (std::regex_match(filename, imm_map_grid_mask))
            immMapGridFileName = grid_folder + SLASH + filename;
    } 

    char *pBuffer = NULL;
    uint32_t fileSize;
    load_irl_map_grid_file(immMapGridFileName, &pBuffer, fileSize);

    std::vector<OccupancyOfPosition> positions;
    positions.clear();

    create_irl_positions(venue, pBuffer, fileSize,
        mag_interpolation_interval, positions);

    if (magEnable)
    {
        Fpbl::MagneticGrid magneticGrid; // created once, then updated for each track
        load_mag_grid_file(magGridFileName, magneticGrid);

        calculate_number_of_mag_mesuments(magneticGrid, mag_grid_size, positions);

        double  coverage;
        coverage = calculate_coverage(positions, min_mag_measurements);
        std::cout << "magnetic coverage = " << coverage << std::endl;

        save_position_occupancy(out_log_folder + SLASH + "occupancy.mag", positions);
    }
    if (wifiEnable)
    {
        Fpbl::WiFiGrid wifiGrid; // created once, then updated for each track
        load_wifi_grid_file(wifiGridFileName, wifiGrid);

        calculate_number_of_wifi_mesuments(wifiGrid, wifi_grid_size, positions);

        double  coverage;
        coverage = calculate_coverage(positions, min_wifi_measurements);
        std::cout << "wifi coverage = " <<coverage << std::endl;

        save_position_occupancy(out_log_folder + SLASH + "occupancy.wifi", positions);
    }
    if (bleEnable)
    {
        Fpbl::BleGrid bleGrid; // created once, then updated for each track
        load_ble_grid_file(bleGridFileName, bleGrid);

        calculate_number_of_ble_mesuments(bleGrid, ble_grid_size, positions);

        double  coverage;
        coverage = calculate_coverage(positions, min_ble_measurements);
        std::cout << "ble coverage = " << coverage << std::endl;

        save_position_occupancy(out_log_folder + SLASH + "occupancy.ble", positions);
    }

    exit(0);

}