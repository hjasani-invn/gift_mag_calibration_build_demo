// FP_builder.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <regex>
#include "FP_builder.hpp"
#include "CmdReader.hpp"
#include "SettingsParser.hpp"
#include "WiFiIgnoreListParser.hpp"
#include "BLPIgnoreListParser.hpp"
#include "DatasetsListProcessor.hpp"

//#include <unsupported/Eigen/FFT>

int main( int argc, char* argv[] )
{
  	jsonVenue venue;

    std::string arg;
    std::stringstream ss;
    std::string name;

    bool   magEnable;
    bool   wifiEnable;
    bool   bleEnable;
    bool   bleProxEnable;

    Fpbl::Grid magnetic_grid;
    Fpbl::Grid wifi_grid;
    Fpbl::Grid ble_grid;

    Fpbl::FPGrid magnetic_fp;
    Fpbl::FPGrid wifi_fp;
    Fpbl::FPGrid ble_fp;

    std::string in_data_folder;
    std::string in_mag_grid_file;
    std::string in_wifi_grid_file;
    std::string in_ble_grid_file;
    std::string in_portals_grid_file;
	std::string in_cs_grid_file;
    std::string grid_folder;
    std::string fp_bases_folder;
    std::string out_data_folder;
    std::string out_mag_grid_file;
    std::string out_wifi_grid_file;
    std::string out_ble_grid_file;
    std::string out_mag_fp_file;
    std::string out_wifi_fp_file;
    std::string out_ble_fp_file;
    std::string in_ble_fprox_file;

	std::string out_remag_grid_file;
	std::string out_remag_fp_file;
	std::string out_rebias_file;
	std::string validation_log_name = "validation_results.csv", revalidation_log_name;

    int mag_fp_format;
    int wifi_fp_format;
    int ble_fp_format;

    std::vector<std::string> mag_validators;
    std::string default_mag_validators;
    std::map<std::string, std::string> file_masks;

    bool success = true;

    // command line parsing
    std::string settings_file; // must be in JSON format
    success &= setOptionFromCmd((const int)argc, (const char **)argv, "--settings", &settings_file);

    std::string input_list_file; // must be in JSON format
    success &= setOptionFromCmd((const int)argc, (const char **)argv, "--input_list", &input_list_file);

    bool wifi_self_healing_mode = false;
    success &= setOptionFromCmd((const int)argc, (const char **)argv, "--self_healing", &wifi_self_healing_mode);

    std::string ignore_list_file;
    success &= setOptionFromCmd((const int)argc, (const char **)argv, "--ignore_list", &ignore_list_file);

    bool xblp_detection_mode = false;
    std::string blp_detection_mode = "";
    success &= setOptionFromCmd((const int)argc, (const char **)argv, "--xblp_detection", &blp_detection_mode);
    if(blp_detection_mode == "on")
        xblp_detection_mode = true;

    std::cout << "FPBL console. FPBL version:"
        << int(Fpbl::getVersionNumber().major)
        << "." << int(Fpbl::getVersionNumber().minor)
        << "." << int(Fpbl::getVersionNumber().build)
        << "." << int(Fpbl::getVersionNumber().releaseId)
        << std::endl;
    std::cout << "settings file    " << settings_file << std::endl;
    parseSettings(settings_file, // input JSON file	with settings
        name, // output
        in_data_folder,
        in_mag_grid_file,
        in_wifi_grid_file,
        in_ble_grid_file,
        in_portals_grid_file,
		in_cs_grid_file,
        grid_folder,
        fp_bases_folder,
        out_data_folder,
        out_mag_grid_file,
        out_wifi_grid_file,
        out_ble_grid_file,
        out_mag_fp_file,
        out_wifi_fp_file,
        out_ble_fp_file,
        in_ble_fprox_file,
        mag_fp_format,
        wifi_fp_format,
        ble_fp_format,
        venue,
        magEnable,
        wifiEnable,
        bleEnable,
        bleProxEnable,
        magnetic_grid,
        wifi_grid,
        ble_grid,
        magnetic_fp,
        wifi_fp,
        ble_fp,
        default_mag_validators,
        mag_validators,
        file_masks
        );

    // ignore lists managing
    if (ignore_list_file.length() == 0)
    {
        ignore_list_file = settings_file;
    }
    std::cout << "ignore/white list file: " << ignore_list_file << std::endl;
    if (ignore_list_file.length() != 0)
    {
        venue.wifi_ignore_list.clear();
        parseWiFiIgnoreList(ignore_list_file, venue.wifi_ignore_list);
        std::cout << "  " << venue.wifi_ignore_list.size() << " items in WiFi black list" << std::endl;

        venue.wifi_white_list.clear();
        parseWiFiWhiteList(ignore_list_file, venue.wifi_white_list);
        std::cout << "  " << venue.wifi_white_list.size() << " items in WiFi ignore list" << std::endl;

        venue.ble_ignore_list.clear();
        parseBLEIgnoreList(ignore_list_file, venue.ble_ignore_list);
        std::cout << "  " << venue.ble_ignore_list.size() << " items in BLE black list" << std::endl;
    }

    FPBuilderConsole::FP_builder FPb(
        name, 
        venue,
        magnetic_grid, 
        wifi_grid, 
        ble_grid,
        magnetic_fp, 
        wifi_fp, 
        ble_fp,
        bleProxEnable,
        default_mag_validators,
        mag_validators,
        mag_fp_format,
        wifi_fp_format,
        ble_fp_format,
        wifi_self_healing_mode,
        xblp_detection_mode,
        file_masks
        );

    std::vector<std::vector<std::string>>  dataset_list;
    if (input_list_file != "")
    {
        std::cout << "Obtain datasets from " << input_list_file << " list " << ENDL;
        parseDatasetsList(input_list_file, "datasets_list", dataset_list);
    }
    else if (in_data_folder != "")
    {
        std::vector<std::string> dataset_list_tmp;
        std::string in_file_mask = file_masks["in_file_mask"];
        std::regex  irl_file_mask = std::regex(in_file_mask);
        std::cout << "Search for  " << in_file_mask << "  datasets " << ENDL;
        std::cout << "in folder: " << in_data_folder << ENDL;
        FindDatasetsRecursive(in_data_folder, irl_file_mask, dataset_list_tmp);
        std::sort(dataset_list_tmp.begin(), dataset_list_tmp.end(), [](std::string a, std::string b) { return a.compare(b) < 0;});
        
        for (std::string dataset : dataset_list_tmp)
        {
            std::vector<std::string > dataset_vect;
            dataset_vect.push_back(dataset);
            dataset_list.push_back(dataset_vect);
        }
    }
    std::cout << dataset_list.size() << " datasets has been found" << ENDL;

    bool tst = FPb.ProcessDatasetsAndBuildFingerprints(
        grid_folder, 
        fp_bases_folder,
        in_mag_grid_file, 
        in_wifi_grid_file, 
        in_ble_grid_file,
        in_portals_grid_file,
		in_cs_grid_file,
        out_data_folder,
        out_mag_grid_file, 
        out_wifi_grid_file, 
        out_ble_grid_file,
        out_mag_fp_file, 
        out_wifi_fp_file, 
        out_ble_fp_file,
        in_ble_fprox_file,
        dataset_list
        );

#if ENABLE_SECOND_PARSE == 1
	out_rebias_file = "_mag_out_by_recalib_history.txt";
	bool retst = false;
	for (int i = 0; i < NUMBER_OF_RECALIB_ITERATIONS; i++)
	{
		out_remag_grid_file = "re" + out_mag_grid_file;
		out_remag_fp_file = "re" + out_mag_fp_file;
		revalidation_log_name = "re" + validation_log_name;
		out_rebias_file = std::to_string(i) + out_rebias_file;

		retst = FPb.reparse_folders(in_data_folder, out_data_folder, in_portals_grid_file, in_cs_grid_file, out_data_folder, out_mag_grid_file,
			out_mag_fp_file, out_remag_grid_file, out_remag_fp_file, validation_log_name, revalidation_log_name, out_rebias_file, Fpbl::NormalMode, Fpbl::kSurveyData);

		out_mag_grid_file = out_remag_grid_file;
		out_mag_fp_file = out_remag_fp_file;
		validation_log_name = revalidation_log_name;
		out_rebias_file = "_mag_out_by_recalib_history.txt";
	}
#endif

    return 0;
}
