#include "SettingsParser.hpp"

jsonVenue::jsonVenue()
{
    this->id = 0;
    this->venue_type = VenueType::kDefaultVenue;

    this->origin_lattitude = 0;
    this->origin_longitude = 0;
    this->origin_altitude = 0;
    this->origin_azimuth = 0;
    this->size_x = 0;
    this->size_y = 0;
    this->alfa = 0;
    this->beta = 0;

    this->floor_height = 5;
    this->floors_count = 1;
    this->floor_shift = 1;
    this->floor_zero_enable = false;

    //MFPVenue
    ((MFPVenue*)this)->celltype = 0;
    ((MFPVenue*)this)->cell_size = 1;
    magX = 0;
    magY = 0;
    magZ = 0;

    //WFPVenue
    ((WFPVenue*)this)->celltype = 0;
    ((WFPVenue*)this)->cell_size = 5;
    ((WFPVenue*)this)->APCount = 150;
    ((WFPVenue*)this)->minAPCount = 5;
    ((WFPVenue*)this)->minProbMetric = 0.01;

    //BFPVenue
    ((BFPVenue*)this)->celltype = 0;
    ((BFPVenue*)this)->cell_size = 5;
    ((BFPVenue*)this)->APCount = 150;
    ((BFPVenue*)this)->minAPCount = 3;
    ((BFPVenue*)this)->minProbMetric = 0.01;

    wifi_scan_period = 3000; // default wifi scan period 3 sec (as provided by Samsung Galaxy S6)
    ble_scan_period = 4000; // default BLE scan period 4 sec
    blp_height = 2.0;
    blp_type = bbt_unknown;
    wifi_ignore_list.clear();
    wifi_white_list.clear();
    ble_ignore_list.clear();

    ble_proximity_data = {};
}

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
    )
{
    // set default values
    /**/
    in_data_folder = "";
    fp_bases_folder = ".";
    out_data_folder = ".";
    grid_folder = ".";

    out_mag_grid_file = name;
    out_mag_fp_file = name;

    out_wifi_grid_file = name;
    out_wifi_fp_file = name;

    out_ble_grid_file = name;
    out_ble_fp_file = name;

    in_ble_fprox_file = "";

    mag_fp_format = 3;
    wifi_fp_format = 3;
    ble_fp_format = 3;

    in_mag_grid_file = "none.maggrid";
    in_wifi_grid_file = "none.wifigrid";
    in_ble_grid_file = "none.blegrid";
    in_portals_grid_file = "none.portalsgrid";
	in_cs_grid_file = "none.csgrid";

    default_mag_validators = "none";
    mag_validators.clear();

    venue.id = 0;
    venue.origin_lattitude = 0;
    venue.origin_longitude = 0;
    venue.origin_altitude = 0;
    venue.origin_azimuth = 0;
    venue.floors_count = 1;
    venue.size_x = 0;
    venue.size_y = 0;
    venue.alfa = 1;
    venue.beta = -1;
    venue.floors_count = 1;
    venue.floor_height = 5;
    venue.floor_shift = 1;
    venue.floor_zero_enable = false;

    magEnable = false;
    wifiEnable = false;
    bleEnable = false;
    bleProxEnable = false;

    mag_grid.size = 1.0;
    wifi_grid.size = 5.0;
    ble_grid.size = 5.0;

    // mag grid
    mag_grid.enable = true;
    mag_grid.size = 1.0;
    mag_grid.type = CellType::CELL_SQUARE;
    mag_grid.min.x = 0;
    mag_grid.min.y = 0;
    mag_grid.min.floor = 0;
    mag_grid.max.x = 0;
    mag_grid.max.y = 0;
    mag_grid.max.floor = 0;
    mag_grid.max_position_uncertainty = 4;

    // mag FP
    mag_fp.grid = mag_grid;
    mag_fp.min_data_count = 20;
    mag_fp.max_bssid_count = 0;
    mag_fp.min_ig = 0;
    mag_fp.stdEstimatorType = Fpbl::eUnknownStdEstimator;

    // WiFi grid
    wifi_grid.enable = false;
    wifi_grid.size = 5.0;
    wifi_grid.type = CellType::CELL_SQUARE;
    wifi_grid.min.x = 0;
    wifi_grid.min.y = 0;
    wifi_grid.min.floor = 0;
    wifi_grid.max.x = 0;
    wifi_grid.max.y = 0;
    wifi_grid.max.floor = 0;
    wifi_grid.max_position_uncertainty = 10;

    // WiFi FP
    wifi_fp.grid = wifi_grid;
    wifi_fp.min_data_count = 5;
    wifi_fp.max_bssid_count = 1000;
    wifi_fp.min_ig = 0.25;
    wifi_fp.stdEstimatorType = Fpbl::eUnknownStdEstimator;

    // BLE grid
    ble_grid.enable = false;
    ble_grid.size = 5.0;
    ble_grid.type = CellType::CELL_SQUARE;
    ble_grid.min.x = 0;
    ble_grid.min.y = 0;
    ble_grid.min.floor = 0;
    ble_grid.max.x = 0;
    ble_grid.max.y = 0;
    ble_grid.max.floor = 0;
    ble_grid.max_position_uncertainty = 10;

    // BLE FP
    ble_fp.grid = ble_grid;
    ble_fp.min_data_count = 5;
    ble_fp.max_bssid_count = 1000;
    ble_fp.min_ig = 0.25;
    ble_fp.stdEstimatorType = Fpbl::eUnknownStdEstimator;
    /**/

    file_masks.clear();
    file_masks.insert(std::pair< std::string, std::string >("in_file_mask", "(irl_.*)(.dat)"));

    json settings = json::parse_file(settings_file);
    // JSON file parsing
    {
        try
        {
            /*   names list
            for (auto it_member = settings.members().begin(); it_member != settings.members().end(); it_member++)
            {
            std::cout << it_member->name() << std::endl;
            }
            */

            // General settings
            if (settings.has_member("name"))
                name = settings["name"].as<std::string>();
            else
                name = "tmp";

            // Folders: old FPBL/RTFPPL format
            if (settings.has_member("folder_in"))
                in_data_folder = settings["folder_in"].as<std::string>();

            if (settings.has_member("folder_fp"))
                fp_bases_folder = settings["folder_fp"].as<std::string>();

            if (settings.has_member("folder_out"))
                out_data_folder = settings["folder_out"].as<std::string>();

            if (settings.has_member("folder_grid"))
                grid_folder = settings["folder_grid"].as<std::string>();

            // Folders: new FPBL format
            parseOutputData(settings, name, out_data_folder, out_mag_grid_file, out_wifi_grid_file, out_ble_grid_file,
                out_mag_fp_file, out_wifi_fp_file, out_ble_fp_file, mag_fp_format, wifi_fp_format, ble_fp_format);

            parseInputData(settings, name, in_data_folder, in_mag_grid_file, in_wifi_grid_file, in_ble_grid_file, in_portals_grid_file, in_cs_grid_file);


            // mag validators
            if (settings.has_member("mag_validators"))
            {
                json magValidators = settings["mag_validators"].as_vector<std::string>();
                parseMagValidators(magValidators, mag_validators);
            }
            if (settings.has_member("default_mag_validators"))
            {
                default_mag_validators = settings["default_mag_validators"].as<std::string>();
            }

            // venue params
            if (settings.has_member("venue"))
            {
                json venueParams = settings["venue"];
                //std::cout << venueJSON.size() << "    " << venueJSON << std::endl;
                parseVenueParams(venueParams, venue);
            }

            // grid params

            if (settings.has_member("ble_proximity_enable"))
                bleProxEnable = settings["ble_proximity_enable"].as<bool>();
            else if (settings.has_member("BLE_proximity_enable"))
                bleProxEnable = settings["BLE_proximity_enable"].as<bool>();

            // grid cell size: old FPBL/RTFPPL format
            if (settings.has_member("magnetic_cellsize"))
                ((MFPVenue*)&venue)->cell_size = settings["magnetic_cellsize"].as<double>();
            else
                ((MFPVenue*)&venue)->cell_size = 1.0;

            if (settings.has_member("wifi_cellsize"))
                ((WFPVenue*)&venue)->cell_size = settings["wifi_cellsize"].as<double>();
            else
                ((WFPVenue*)&venue)->cell_size = 5.0;

            if (settings.has_member("ble_cellsize"))
                ((BFPVenue*)&venue)->cell_size = settings["ble_cellsize"].as<double>();
            else
                ((BFPVenue*)&venue)->cell_size = 5.0;

            /*if (settings.has_member("data_format"))
            data_format = settings["data_format"].as<int>();
            else
            data_format = 2;
            */

            // mag grid and FP params settings
            if (settings.has_member("magnetic_enable"))
                mag_fp.grid.enable = mag_grid.enable = magEnable = settings["magnetic_enable"].as<bool>();

            { // set grid params and region by default
                mag_grid.size = ((MFPVenue*)&venue)->cell_size;
                mag_grid.min.x = 0;
                mag_grid.min.y = 0;
                mag_grid.min.floor = 0;
                mag_grid.max.x = venue.size_x;
                mag_grid.max.y = venue.size_y;
                mag_grid.max.floor = venue.floors_count - 1;
            }
            if (settings.has_member("magnetic_grid"))
            {
                json magneticGridParams = settings["magnetic_grid"];
                parseGridParams(magneticGridParams, mag_grid);
            }
            
            mag_fp.grid = mag_grid;//it is set as default for MFP 
            if (settings.has_member("magnetic_fingerprint"))
            {
                json fpParams = settings["magnetic_fingerprint"];
                parseFPGridParams(fpParams, mag_fp);
                if (fpParams.has_member("fp_grid"))
                {
                    json gridParams = fpParams["fp_grid"];
                    parseGridParams(gridParams, mag_fp.grid);
                }
            }


            // WiFi grid and FP params settings
            if (settings.has_member("wifi_enable"))
                wifi_fp.grid.enable = wifi_grid.enable = wifiEnable = settings["wifi_enable"].as<bool>();
            else if (settings.has_member("WiFi_enable"))
                wifi_fp.grid.enable = wifi_grid.enable = wifiEnable = settings["WiFi_enable"].as<bool>();

            { // set grid params and region by default 
                wifi_grid.size = ((WFPVenue*)&venue)->cell_size;
                wifi_grid.min.x = 0;
                wifi_grid.min.y = 0;
                wifi_grid.min.floor = 0;
                wifi_grid.max.x = venue.size_x;
                wifi_grid.max.y = venue.size_y;
                wifi_grid.max.floor = venue.floors_count - 1;
            }
            if (settings.has_member("wifi_grid"))
            {
                json wifiGridParams = settings["wifi_grid"];
                parseGridParams(wifiGridParams, wifi_grid);
            }
            
            wifi_fp.grid = wifi_grid;//it is set as default for WFP 
            if  (settings.has_member("wifi_fingerprint"))
            {
                json fpParams = settings["wifi_fingerprint"];
                parseFPGridParams(fpParams, wifi_fp);
                if (fpParams.has_member("fp_grid"))
                {
                    json gridParams = fpParams["fp_grid"];
                    parseGridParams(gridParams, wifi_fp.grid);
                }
            }

            // BLE grid and FP params settings
            if (settings.has_member("ble_enable"))
                ble_fp.grid.enable = ble_grid.enable = bleEnable = settings["ble_enable"].as<bool>();
            else if (settings.has_member("BLE_enable"))
                ble_fp.grid.enable = ble_grid.enable = bleEnable = settings["BLE_enable"].as<bool>();

            { // set grid params and region by default  
                ble_grid.size = ((BFPVenue*)&venue)->cell_size;
                ble_grid.min.x = 0;
                ble_grid.min.y = 0;
                ble_grid.min.floor = 0;
                ble_grid.max.x = venue.size_x;
                ble_grid.max.y = venue.size_y;
                ble_grid.max.floor = venue.floors_count - 1;
            }
            if (settings.has_member("ble_grid"))
            {
                json bleGridParams = settings["ble_grid"];
                parseGridParams(bleGridParams, ble_grid);
            }
            ble_fp.grid = ble_grid;//it is set as default for BFP 
            if  (settings.has_member("ble_fingerprint"))
            {
                json fpParams = settings["ble_fingerprint"];
                parseFPGridParams(fpParams, ble_fp);
                if (fpParams.has_member("fp_grid"))
                {
                    json gridParams = fpParams["fp_grid"];
                    parseGridParams(gridParams, ble_fp.grid);
                }
            }

            if (settings.has_member("file_masks"))
            {
                json fileMasksParams = settings["file_masks"];
                parseFileMasksParams(fileMasksParams, file_masks);
            }

            if (settings.has_member("in_ble_fprox_file"))
            {
                in_ble_fprox_file = settings["in_ble_fprox_file"].as<std::string>();
            }

            bleProximityData proxParams = { false, 1e6, 0, 5, 10, false };
            if (settings.has_member("ble_proximity_data"))
            {
                json proxData = settings["ble_proximity_data"];
                parseProxParams(proxData, proxParams);
            }
            venue.ble_proximity_data = proxParams;

        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}

static	void parseVenueParams(json venueParams, jsonVenue &venue)
{
    if (venueParams.has_member("id"))
        venue.id = venueParams["id"].as<int>();
    if (venueParams.has_member("type"))
    {
        int tmp = venueParams["type"].as<int>();
        venue.venue_type = (VenueType)tmp;
    }
    venue.origin_lattitude = venueParams["origin_lattitude"].as<double>();
    venue.origin_longitude = venueParams["origin_longitude"].as<double>();
    venue.origin_altitude = venueParams["origin_altitude"].as<double>();
    venue.origin_azimuth = venueParams["origin_azimuth"].as<double>();

    if (venueParams.has_member("floors_count"))
        venue.floors_count = venueParams["floors_count"].as<int>();
    if (venueParams.has_member("floor_shift"))
        venue.floor_shift = venueParams["floor_shift"].as<int>();
    if (venueParams.has_member("floor_zero_enable"))
        venue.floor_zero_enable = venueParams["floor_zero_enable"].as<bool>();
    if (venueParams.has_member("floor_height"))
        venue.floor_height = venueParams["floor_height"].as<double>();

    venue.size_x = venueParams["size_x"].as<double>();
    venue.size_y = venueParams["size_y"].as<double>();
    venue.alfa = venueParams["alfa"].as<double>();
    venue.beta = venueParams["beta"].as<double>();
    if (venueParams.has_member("magX"))
        venue.magX = venueParams["magX"].as<double>();
    else
        venue.magX = 0;
    if (venueParams.has_member("magY"))
        venue.magY = venueParams["magY"].as<double>();
    else
        venue.magY = 0;
    if (venueParams.has_member("magZ"))
        venue.magZ = venueParams["magZ"].as<double>();
    else
        venue.magZ = 0;
}

static	void parseMagValidators(json magValidatorsObj, std::vector<std::string> &mag_validators)
{
    mag_validators.clear();
    int length = magValidatorsObj.size();
    std::string validator;

    for (int i = 0; i < length; i++)
    {
        validator = magValidatorsObj[i].as<std::string>();
        mag_validators.push_back(validator);
    }

}

static void parseGridParams(json gridParams, Fpbl::Grid &grid)
{
    if (gridParams.has_member("enable"))
        grid.enable = gridParams["enable"].as<bool>();

    if (gridParams.has_member("celltype"))
        grid.type = (CellType)gridParams["celltype"].as<int>();

    if (gridParams.has_member("cellsize"))
        grid.size = gridParams["cellsize"].as<double>();

    if (gridParams.has_member("min"))
    {
        json min_sizes = gridParams["min"];
        if (min_sizes.size() >= 2)
        {
            grid.min.x = min_sizes[0].as<double>();
            grid.min.y = min_sizes[1].as<double>();
        }
        if (min_sizes.size() == 3)
            grid.min.floor = min_sizes[2].as<double>();
    }

    if (gridParams.has_member("max"))
    {
        json max_sizes = gridParams["max"];
        if (max_sizes.size() >= 2)
        {
            grid.max.x = max_sizes[0].as<double>();
            grid.max.y = max_sizes[1].as<double>();
        }
        if (max_sizes.size() == 3)
            grid.max.floor = max_sizes[2].as<double>();
    }

    if (gridParams.has_member("max_position_uncertainty"))
        grid.max_position_uncertainty = gridParams["max_position_uncertainty"].as<double>();

    if (gridParams.has_member("disabled_floors"))
    {
        int n;
        json disabled_floors = gridParams["disabled_floors"];
        if ((n = disabled_floors.size()) > 0)
        {
            for (int i = 0; i < n; i++)
            {
                //disabled_floors[i];
            }

        }

    }
}

static	void parseFPGridParams(json gridParams, Fpbl::FPGrid &grid)
{
    if (gridParams.has_member("min_data_count"))
        grid.min_data_count = gridParams["min_data_count"].as<int>();

    if (gridParams.has_member("max_bssid_count"))
        grid.max_bssid_count = gridParams["max_bssid_count"].as<int>();

    if (gridParams.has_member("min_ig"))
        grid.min_ig = gridParams["min_ig"].as<double>();

    if (gridParams.has_member("disabled_floors"))
    {
        int n;
        json disabled_floors = gridParams["disabled_floors"];
        if ((n = disabled_floors.size()) > 0)
        {
            for (int i = 0; i < n; i++)
            {
                //disabled_floors[i];
            }

        }
    }
    if (gridParams.has_member("mag_std_estimator"))
    {
        std::string mag_est = gridParams["mag_std_estimator"].as<std::string>();

        if (mag_est == "robust")
            grid.stdEstimatorType = Fpbl::eRobustStdEstimator;
        else
        {
            if (mag_est == "quantile")
                grid.stdEstimatorType = Fpbl::eQuantileStdEstimator;
            else
            {
                if (mag_est == "combine")
                    grid.stdEstimatorType = Fpbl::eCombineStdEstimator;
            }
        }
    }
}

static	void parseInputData(json settings, std::string &name,
    std::string &in_data_folder, std::string &in_mag_grid_file, 
    std::string &in_wifi_grid_file, std::string &in_ble_grid_file, std::string &in_portals_grid_file, std::string &in_cs_grid_file)
{
    if (settings.has_member("input_data"))
    {
        json input_data = settings["input_data"];
        if (input_data.has_member("folder_in"))
            in_data_folder = input_data["folder_in"].as<std::string>();

        if (input_data.has_member("mag_grid_file"))
            in_mag_grid_file = input_data["mag_grid_file"].as<std::string>();

        if (input_data.has_member("wifi_grid_file"))
            in_wifi_grid_file = input_data["wifi_grid_file"].as<std::string>();

        if (input_data.has_member("ble_grid_file"))
            in_ble_grid_file = input_data["ble_grid_file"].as<std::string>();
        if (input_data.has_member("portals_grid_file"))
            in_portals_grid_file = input_data["portals_grid_file"].as<std::string>();

		if (input_data.has_member("crowdsource_grid_file"))
			in_cs_grid_file = input_data["crowdsource_grid_file"].as<std::string>();
    }
}

static	void parseOutputData(json settings, std::string &name, std::string &out_data_folder,
    std::string &out_mag_grid_file, std::string &out_wifi_grid_file, std::string &out_ble_grid_file,
    std::string &out_mag_fp_file, std::string &out_wifi_fp_file, std::string &out_ble_fp_file,
    int &mag_fp_format, int &wifi_fp_format, int &ble_fp_format)
{
    if (settings.has_member("output_data"))
    {
        json output_data = settings["output_data"];
        if (output_data.has_member("folder_out"))
            out_data_folder = output_data["folder_out"].as<std::string>();

        if (output_data.has_member("mag_grid_file"))
            out_mag_grid_file = output_data["mag_grid_file"].as<std::string>();
        else
            out_mag_grid_file = name;
        if (output_data.has_member("wifi_grid_file"))
            out_wifi_grid_file = output_data["wifi_grid_file"].as<std::string>();
        else
            out_wifi_grid_file = name;
        if (output_data.has_member("ble_grid_file"))
            out_ble_grid_file = output_data["ble_grid_file"].as<std::string>();
        else
            out_ble_grid_file = name;
        if (output_data.has_member("mag_fp_file"))
            out_mag_fp_file = output_data["mag_fp_file"].as<std::string>();
        else
            out_mag_fp_file = name;
        if (output_data.has_member("wifi_fp_file"))
            out_wifi_fp_file = output_data["wifi_fp_file"].as<std::string>();
        else
            out_wifi_fp_file = name;
        if (output_data.has_member("ble_fp_file"))
            out_ble_fp_file = output_data["ble_fp_file"].as<std::string>();
        else
            out_ble_fp_file = name;
        if (output_data.has_member("mag_fp_format"))
            mag_fp_format = output_data["mag_fp_format"].as<int>();

        if (output_data.has_member("wifi_fp_format"))
            wifi_fp_format = output_data["wifi_fp_format"].as<int>();

        if (output_data.has_member("ble_fp_format"))
            ble_fp_format = output_data["ble_fp_format"].as<int>();
    }
    else
    {
        out_mag_grid_file = name;
        out_wifi_grid_file = name;
        out_ble_grid_file = name;
        out_mag_fp_file = name;
        out_wifi_fp_file = name;
        out_ble_fp_file = name;
    }
}

static	void parseFileMasksParams(json fileMasksParams, std::map<std::string, std::string> &file_masks)
{
    /*   names and values list */
#if 0
    for (auto it_member = fileMasksParams.members().begin(); it_member != fileMasksParams.members().end(); it_member++)
    {
        std::cout << it_member->name() << "   " << it_member->value() << std::endl;
        file_masks[it_member->name()] = it_member->value().as<std::string>();
    }
    for (auto it = file_masks.begin(); it != file_masks.end(); ++it)
        std::cout << it->first << "   " << it->second << std::endl;;
#endif
    if (fileMasksParams.has_member("in_file_mask"))
    {
        std::string  in_file_mask = fileMasksParams["in_file_mask"].as<std::string>();
        //file_masks.insert(std::pair< std::string, std::string >("in_file_mask", in_file_mask));
        file_masks["in_file_mask"] = in_file_mask;
    }
}

static	void parseProxParams(json proxData, bleProximityData &proxParams)
{
    if (proxData.has_member("enable"))
    {
        proxParams.enable = proxData["enable"].as<bool>();
    }
    else
        proxParams.enable = true;
    if (proxData.has_member("max_position_uncertainty"))
    {
        proxParams.max_position_uncertainty = proxData["max_position_uncertainty"].as<float>();
    }
    else
        proxParams.max_position_uncertainty = 1e6;
    if (proxData.has_member("skip_time"))
    {
        proxParams.skip_time = proxData["skip_time"].as<float>();
    }
    else
        proxParams.skip_time = 0;
    if (proxData.has_member("sigma"))
    {
        proxParams.sigma = proxData["sigma"].as<float>();
    }
    else
        proxParams.sigma = 1000;
    if (proxData.has_member("max_distance"))
    {
        proxParams.max_distance = proxData["max_distance"].as<float>();
    }
    else
        proxParams.max_distance = 1e6;
    if (proxData.has_member("ble_enable_prox_only"))
    {
        proxParams.ble_enable_prox_only = proxData["ble_enable_prox_only"].as<bool>();
    }
    else
        proxParams.ble_enable_prox_only = false;
}

