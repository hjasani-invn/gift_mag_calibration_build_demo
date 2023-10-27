
#include <iomanip>
#include <sys/stat.h>
#include "FP_builder.hpp"

#include "tpn_data_reader.hpp"
#include "stringMac_to_intMac.h"
#include "DatasetsListProcessor.hpp"

#include "fpHeader.hpp"


#include <eigen/LU>
#include <eigen/Geometry>
#include <fstream>
#include <stdlib.h>
#include <algorithm>

#include "FloorConverter.h"
#include "BleHash.h"

#define NOMINMAX // blockage of min/max redefinition
#include "dirent.h"
#undef NOMINMAX

const double max_position_uncertainty_mfp_default_ivl = 2.;
const double max_position_uncertainty_wfp_default_ivl = 4.;
const double max_position_uncertainty_bfp_default_ivl = 4.;
const double max_position_uncertainty_mfp_default_survey = 4.;
const double max_position_uncertainty_wfp_default_survey = 10.;
const double max_position_uncertainty_bfp_default_survey = 10.;
const double max_position_uncertainty_mfp_default_retail = 2.;
const double max_position_uncertainty_wfp_default_retail = 4.;
const double max_position_uncertainty_bfp_default_retail = 4.;

bool use_ble_exception = false;
const int ble_exception_count = 1;
uint16_t ble_exception_list[ble_exception_count][2] =
{
    0, 0
};

namespace FPBuilderConsole
{
    FP_builder::FP_builder(std::string name,
        jsonVenue &ven,
        Fpbl::Grid &mag_grid,
        Fpbl::Grid &wifi_grid,
        Fpbl::Grid &ble_grid,
        Fpbl::FPGrid &mag_fp,
        Fpbl::FPGrid &wifi_fp,
        Fpbl::FPGrid &ble_fp,
        bool   &bleProxEnable,
        std::string &default_mag_validators,
        std::vector<std::string> &mag_validators,
        int mag_fp_format,
        int wifi_fp_format,
        int ble_fp_format,
        bool wifi_self_healing,
        bool xblp_detection_mode,
        std::map<std::string, std::string> &file_masks
        )
    {
        FP_builder::wifi_file_mask = std::regex( "(wifi_)(.*)(log)" );
        //FP_builder::wifi_file_mask = std::regex("(NONE_wifi)(.*)");
        FP_builder::ble_file_mask = std::regex("(ble_)(.*)(log)");
        //FP_builder::ble_file_mask = std::regex("(NONE_ble)(.*)");

        //FP_builder::pos_att_file_mask = std::regex( "(fpbl_pos)(.*)" );
        //FP_builder::mg_file_mask = std::regex( "(fpbl_mag_20)(.*)" );
        FP_builder::pos_att_file_mask = std::regex("(NONE_pos)(.*)");
        FP_builder::mg_file_mask = std::regex("(NONE_mg)(.*)");

        FP_builder::mag_out_file = "mag_out_by_fdmc.txt";

        //FP_builder::mag_out_file_mask = std::regex("(mag_out.*)(.txt)");
        //FP_builder::mag_out_file_mask = std::regex("(mag_out)(.txt)");
        //FP_builder::mag_out_file_mask = std::regex( "mag_out_by_fdmc.txt" );
		FP_builder::mag_out_file_mask = std::regex("(mag_out\\.txt)|(mag_out_by_fdmc.*)"); // !
        //FP_builder::mbias_file_mask = std::regex( "default.mbias" );
        FP_builder::mbias_file_mask = std::regex("OFF_default.mbias");

        FP_builder::default_in_file_mask = file_masks["in_file_mask"];
        FP_builder::irl_file_mask = std::regex(default_in_file_mask);
        //FP_builder::irl_file_mask = std::regex("(irl_.*)(.dat)");
        //FP_builder::irl_file_mask = std::regex( "(nav.*)(.dat)" );
        //FP_builder::irl_file_mask = std::regex( "(tpn.*)(.dat)" );
        //FP_builder::irl_file_mask = std::regex("(.*)(TppOutput\.dat)");
        //if (wifi_self_healing)
        //{
        //    FP_builder::irl_file_mask = std::regex( "(nav.*)(.dat)" );
        //}

        //FP_builder::venue_grid = grid;

		FP_builder::remag_out_file = WEIGHT_RECALIBRATION_METHOD == UNWEIGHTED ? "mag_out_by_recalib0.txt" : "mag_out_by_recalib_w" + std::string(1, 'a' + WEIGHT_RECALIBRATION_METHOD - 1) + ".txt";

		FP_builder::re_mag_out_file_mask = std::regex(remag_out_file);

        //INTEGRATION EXAMLE: start of initialisation  (1)
        this->mag_grid = mag_grid;    // magnetic grid parameters
        this->wifi_grid = wifi_grid;  // wifi grid parameters
        this->ble_grid = ble_grid;    // ble grid parameters
        this->mag_fp = mag_fp;    // magnetic finger print parameters
        this->wifi_fp = wifi_fp;  // wifi finger print parameters
        this->ble_fp = ble_fp;    // ble finger print parameters
        this->FPname = name;                // fingerprint name
        this->venue = ven;                  // venue parameters
        this->bleProxEnable = bleProxEnable;
        this->bleEnableProxOnly = venue.ble_proximity_data.ble_enable_prox_only;   // false;       // default value
        this->default_mag_validators = default_mag_validators;
        this->mag_validators = mag_validators;

        this->mag_fp_format = mag_fp_format;
        this->wifi_fp_format = wifi_fp_format;
        this->ble_fp_format = ble_fp_format;
        this->default_blp_height = ven.blp_height;
        this->default_blp_type = ven.blp_type;
        //this->ble_scan_period = BLE_SCAN_PERIOD;
        this->ble_scan_period = ven.ble_scan_period;

        this->pos_accuracy.clear();
        this->start_delay = -100;

		this->pDbmfp = NULL;

        this->wifi_self_healing_mode = wifi_self_healing;
        this->xblp_detection_mode = xblp_detection_mode;

        //INTEGRATION EXAMLE: end of initialisation
    }

    FP_builder::~FP_builder() {}
#if 0
    void FindDatasetsRecursive(std::string folder_name, std::regex irl_file_mask, std::vector<std::string> &dataset_list)
    {
        DIR *dir_pointer = opendir(folder_name.c_str());
        struct dirent *entry;

        while ((dir_pointer != 0) && (entry = readdir(dir_pointer)))
        {
            if (entry->d_type == DT_REG)
            {
                std::string file_name(entry->d_name);
                if (std::regex_match(file_name, irl_file_mask))
                {
                    dataset_list.push_back(folder_name);
                }
            }
            else if (entry->d_type == DT_DIR)
            {
                if (std::string(entry->d_name).at(0) != '.')
                {
                    //std::string sub_folder_name = folder_name + SLASH + std::string(entry->d_name);
                    std::string sub_folder_name = folder_name + "/"+ std::string(entry->d_name);
                    FindDatasetsRecursive(sub_folder_name, irl_file_mask, dataset_list);
                }
            }
        }
        closedir(dir_pointer);
    }
#endif

    bool FP_builder::CalculateMagBiasForDataset(std::vector<TpnOutput> &irl_data, TrackPreProcessing::MagBias& magbias)
    {
        TrackPreProcessing::Calibrator_FDMC fdmc;
        magbias = fdmc.CalculateBias(irl_data);

        return true;
    }

	bool FP_builder::ReCalculateMagBiasForDataset(std::vector<TpnOutput> &irl_data, TrackPreProcessing::MagBias& magbias)
	{
		TrackPreProcessing::ReCalibrator recalib(WEIGHT_RECALIBRATION_METHOD);
		magbias = recalib.CalculateBias(irl_data, pDbmfp, venue);

		return true;
	}

    uint32_t FP_builder::GetDatasetID(std::string filename)
    {
        uint32_t dataset_id = 0;
#if 1
        struct stat file_buf;
        stat(filename.c_str(), &file_buf);
        long fileSizeInBytes = file_buf.st_size;
        char *pFile = (char *)malloc(fileSizeInBytes);

        FILE *pF = fopen(filename.c_str(), "rb");
        bool success = (pF != 0);
        if (pF)
        {
            if (fread(pFile, fileSizeInBytes, 1, pF) == 0)
            {
                success = false;
            }
            fclose(pF);
        }
        if (success)
        {
            dataset_id = crc32(pFile, fileSizeInBytes);
            //std::cout << "file " << filename << "  dataset_id  " << dataset_id << std::endl;
        }
        else
        {
            std::cout << "file " << filename << "   can not be opened" << std::endl;
            dataset_id = 0;
        }
        free(pFile);
#else
        dataset_id = crc32(filename.c_str(), filename.size());
#endif
        return dataset_id;
    }

    TrackFiles FP_builder::ProcesDataSet(std::string folder_name, TrackPreProcessing::DataSetParams dataset_params, std::ofstream &validation_log,
        std::ofstream &mag_bias_stream, Fpbl::GridBuilder &Builder, TrackPreProcessing::TrackProcessor &track_processor, Fpbl::RouteMode_t route_type, Fpbl::DataSetType dataset_type,
        bool override_mag_out_file_mask, std::regex sel_mag_out_file_mask)
    {
        // now we are in the track folder
        std::string l2_subdir_name = folder_name;
        struct dirent *l2_subentry;
        TrackFiles track_files;
        uint32_t tpn_dataset_id = 0;
        uint32_t wifi_dataset_id = 0;
        uint32_t ble_dataset_id = 0;

        std::vector<TrackPreProcessing::MagBias> mag_data_list;

        DIR *l2_subdir_pointer = opendir(l2_subdir_name.c_str());

        std::string irl_file("");
        std::string ble_file("");
        std::string wifi_file("");


        while ((l2_subdir_pointer != 0) &&(l2_subentry = readdir(l2_subdir_pointer)))
        {
            if (l2_subentry->d_type == DT_REG)
            {
                std::string file_name(l2_subentry->d_name);

                if (std::regex_match(file_name, wifi_file_mask))
                {
                    //track_files.have_wifi = true;
                    //track_files.wifi = file_name;
                    wifi_file = file_name;
                    //bool result = parse_wifi_file(path_to_wifi_file, Builder);
                    //std::cout << "found wifi file: " << track_files.wifi;
                    //std::cout << (result ? " - processed" : " - rejected") << ENDL;
                }

                if (std::regex_match(file_name, ble_file_mask))
                {
                    track_files.have_ble = true;
                    //track_files.ble = file_name;
                    ble_file = file_name;
                    //bool result = parse_ble_file(path_to_ble_file, Builder);
                    //std::cout << "found ble file: " << track_files.ble;
                    //std::cout << (result ? " - processed" : " - rejected") << ENDL;
                }

                // looking for mag_data files and parsing it, parsing IRL file changes based on the presence of it
				if (override_mag_out_file_mask == false)
					sel_mag_out_file_mask = mag_out_file_mask;
                if (std::regex_match(file_name, sel_mag_out_file_mask))
                {
                    std::cout << "found mag bias file: " << file_name;
                    std::string path_to_mag_out_file = l2_subdir_name + SLASH + file_name;
                    TrackPreProcessing::MagBias mag_bias;
                    if (true == parse_mag_out_file(path_to_mag_out_file, mag_bias))
                    {
                        mag_data_list.push_back(mag_bias);
                        track_files.have_mag_out = true;
                        std::cout << " - parsed " << ENDL;
                    }
                    else
                    {
                        std::cout << " - rejected " << ENDL;
                    }
                }

                if (std::regex_match(file_name, irl_file_mask))
                {
                    //track_files.irl = file_name;
                    irl_file = file_name;
                    /*std::cout << "\nfound irl file: " << track_files.irl << ENDL;
                    long size = GetFileSize(path_to_irl_file);

                    if (size > 0)
                    {
                        track_files.have_irl = true;
                    }*/
                }

#if 0
                // looking for default.mbias and parsing it, parsing IRL file changes based on the presence of it
                if (std::regex_match(file_name, mbias_file_mask))
                {
                    track_files.mbias = file_name;
                    std::string path_to_mbias_file = l2_subdir_name + SLASH + track_files.mbias;
                    std::cout << "found mag-bias file: " << path_to_mbias_file << ENDL;
                    bool result = parse_mbias_file(path_to_mbias_file, mag_data);
                    track_files.have_mag_out = result;
                }
#endif

            }
        }

        closedir(l2_subdir_pointer);

        // mag data processing
        std::string path_to_irl_file = l2_subdir_name + SLASH + irl_file;
        long irl_size = (irl_file.size() > 0) ?  GetFileSize(path_to_irl_file) : 0;
        if (irl_size > 0)
        {
            tpn_dataset_id = GetDatasetID(path_to_irl_file);
            std::cout << "dataset_id: " << tpn_dataset_id << ENDL;
            wifi_dataset_id = tpn_dataset_id;
            ble_dataset_id = tpn_dataset_id;

            track_files.have_irl = true;
            std::vector<TpnOutput> irl_data;
            bool result = false;
            size_t gap_count = 0;
            if (route_type != Fpbl::Unknown)
            {
                result = parse_irl_file(path_to_irl_file, irl_data, dataset_params.calibrated_mag_data);
                gap_count = remove_small_gaps_in_mag_data(irl_data);
            }

            std::cout << "dat-file: " << irl_file << " size=" << irl_size << " bytes";
            std::cout << ", " << irl_data.size() << " packets ";
            std::cout << ", " << "route_type=" << route_type << ENDL;
            std::cout << "number of 1 sample mag gaps fixed: " << gap_count << ENDL;

            track_files.have_pos = irl_data.size() > 0; // to do: place position validation here

#if DEBUG_OUTPUT__IRL_MAG_DATA
            {// debug output
#if 1
                std::string path_to_file = l2_subdir_name + SLASH + "irl_mag_data.csv";
#else
                const int max_path_to_file_length = 150;
                std::string path_to_file = l2_subdir_name + "__" + "irl_mag_data.csv";
                int n;
                while ((n = path_to_file.find("/")) != std::string::npos)
                    path_to_file.replace(n, 1, "_");
                int path_to_file_length = path_to_file.length();
                if (path_to_file_length > max_path_to_file_length)
                {
                    int erase_length = path_to_file_length - max_path_to_file_length;
                    path_to_file.erase(0, erase_length);
                }
                std::cout << path_to_file << ENDL;
#endif
                std::ofstream irl_log(path_to_file, std::ofstream::out | std::ofstream::trunc);
                for (auto irl_item = irl_data.begin(); irl_item != irl_data.end(); irl_item++)
                    irl_item->print(irl_log);
                irl_log.close();
            }
#endif

            //   magnetic bias utilization
            TrackPreProcessing::MagBias mag_data = {};
            if (mag_data_list.size() > 0)
            {
                std::sort(mag_data_list.begin(), mag_data_list.end(),
                    [](const TrackPreProcessing::MagBias &mb1, const TrackPreProcessing::MagBias &mb2) { return mb1.calibration_level > mb2.calibration_level; });

                mag_data = *mag_data_list.begin();

                int cnt = 1;
                auto imag_data = mag_data_list.begin();
                while (++imag_data != mag_data_list.cend() && (imag_data->calibration_level == mag_data.calibration_level))
                {
                    for (int i = 0; i < 3; i++)
                    {
                        mag_data.bias[i] += imag_data->bias[i];
                        for (int j = 0; j < 3; j++)
                        {
                            mag_data.covarianceMatrix[i][j] = std::fmax(mag_data.covarianceMatrix[i][j], imag_data->covarianceMatrix[i][j]);
                        }
                    }
                    cnt++;
                }

                for (int i = 0; i < 3; i++)
                {
                    mag_data.bias[i] /= cnt;
                }
            }
            
            uint8_t irl_data_calibration_level = get_mag_calibration_level(irl_data);
            if ( (false == dataset_params.calibrated_mag_data) || (override_mag_out_file_mask == true))// && (mag_data.calibration_level > 0)) // (mag_data.calibration_level >= irl_data_calibration_level) )
            { // apply mag_bias
				// WARNING - override_mag_out_file_mask condition is used to make sure that biases are applied every time 
                apply_mag_bias(mag_data, irl_data);
            }

#if DEBUG_OUTPUT__IRL_MAG_AND_ORIENTATION_DATA
#if 1
            std::string path_to_mag_orientation_file = l2_subdir_name + SLASH + "irl_mag_orientation_data.csv";
            std::ofstream irl_mag_orientation_log(path_to_mag_orientation_file, std::ofstream::out | std::ofstream::trunc);
#else
            std::string path_to_mag_orientation_file = l2_subdir_name + "__" + "irl_mag_orientation_data.csv";
            {
                const int max_path_to_mag_orientation_file_length = 120;
                int n;
                while ((n = path_to_mag_orientation_file.find("/")) != std::string::npos)
                    path_to_mag_orientation_file.replace(n, 1, "_");
                int path_to_mag_orientation_file_length = path_to_mag_orientation_file.length();
                if (path_to_mag_orientation_file_length > max_path_to_mag_orientation_file_length)
                {
                    int erase_length = path_to_mag_orientation_file_length -
                        max_path_to_mag_orientation_file_length;
                    path_to_mag_orientation_file.erase(0, erase_length);
                }
                std::cout << path_to_mag_orientation_file << ENDL;
            }
            std::ofstream irl_mag_orientation_log(path_to_mag_orientation_file, std::ofstream::out | std::ofstream::trunc);
#endif
#else
            std::ofstream irl_mag_orientation_log("", std::ofstream::out | std::ofstream::trunc);;
#endif
            // magnetic data validation
            std::string data_set_name = track_processor.GenerateDataSetName(path_to_irl_file);
            dataset_params.dataset_name = data_set_name;
            dataset_params.dataset_type_ = dataset_type;
            dataset_params.route_type_ = route_type;
            track_files.have_mg = track_processor.ValidateDataSet(irl_data, venue, dataset_params, irl_mag_orientation_log);

            send_data_to_builder(irl_data, dataset_params.dataset_type_, Builder); // // send IRL data to builder

            std::cout << ">>mag data - " << (track_files.have_mg ? "processed" : "rejected") << ENDL;

            track_processor.PrintValidationReport(validation_log);

#if DEBUG_OUTPUT__IRL_MAG_DATA_AFTER_PREPROCESSING
            {// debug output
#if 1
                std::string path_to_file = l2_subdir_name + SLASH + "irl_mag_data_corrected.csv";
#else
                const int max_path_to_file_length = 150;
                std::string path_to_file = l2_subdir_name + "__" + "irl_mag_data_corrected.csv";
                int n;
                while ((n = path_to_file.find("/")) != std::string::npos)
                    path_to_file.replace(n, 1, "_");
                int path_to_file_length = path_to_file.length();
                if (path_to_file_length > max_path_to_file_length)
                {
                    int erase_length = path_to_file_length - max_path_to_file_length;
                    path_to_file.erase(0, erase_length);
                }
                std::cout << path_to_file << ENDL;
#endif
                std::ofstream irl_log(path_to_file, std::ofstream::out | std::ofstream::trunc);
                for (auto irl_item = irl_data.begin(); irl_item != irl_data.end(); irl_item++)
                    irl_item->print(irl_log);
                irl_log.close();
            }
#endif
        }

        // wifi data processing
        if (track_files.have_pos)
        {
            track_files.have_wifi = false;
            std::vector<WiFiScanResult > wifiMeas; // TO DO: speed up vector using by optimization of vector memory reallocation
            wifiMeas.clear();

            std::cout << ">>wifi data";

            //std::cout << "wifi data";
            bool is_data_avaliable = false;

#if ALLOW_WIFI_DATA_FROM_IRL
            // load WiFi data from IRL file
            is_data_avaliable = parse_irl_file_for_wifi(path_to_irl_file, wifiMeas);
            std::cout << " from " << irl_file << ": " << wifiMeas.size() << " scans";
#endif

#if ALLOW_WIFI_DATA_FROM_MAPPER_LOG
            if ((wifiMeas.size() == 0) && (wifi_file.size() > 0))
            {// try to upload from wifi_xxx.log
                wifi_dataset_id = GetDatasetID(l2_subdir_name + SLASH + wifi_file);
                is_data_avaliable = parse_wifi_file(l2_subdir_name + SLASH + wifi_file, wifiMeas);
                std::cout << "from " << wifi_file << ": " << wifiMeas.size() << " scans";
            }
#endif
            if ((is_data_avaliable) && (wifiMeas.size() > 0))
            {
                size_t sz = 0;
                for (auto wifiScanRes = wifiMeas.begin(); wifiScanRes != wifiMeas.end(); ++wifiScanRes)
                {
                    sz += wifiScanRes->scanWiFi.size();
                }
                std::cout << ", " << sz << " meas";
            }

            wifi_log_data.loaded_data = wifi_logging(wifiMeas);


            if ((is_data_avaliable) && (wifiMeas.size() > 0))
            {
#if WIFI_STUCK_SCANS_REJECTION
                // Remove stuck scans
                size_t stuck_rejected_count = remove_stuck_scans(wifiMeas);
                std::cout << ", " << stuck_rejected_count << " stuck scans rejected";
#endif
                // White list applying
                size_t rejected_count = apply_wifi_list(wifiMeas, venue.wifi_white_list, true);
                std::cout << ", " << rejected_count << " meas ignored by white list ";

                // Ignore list applying
                rejected_count = apply_wifi_list(wifiMeas, venue.wifi_ignore_list, false);
                std::cout << ", " << rejected_count << "  meas ignored by black list ";

                // Clear repeatable wifi messages
                rejected_count = clear_repeatable_measurements(wifiMeas);
                std::cout << ", " << rejected_count << " repeated meas removed";
            }

            // remake scans
            std::vector<WiFiScanResult> wifiScanResults; // TO DO: speed up vector using by optimization of vector memory reallocation
            if (venue.wifi_scan_period > 0)
            {
                make_scans(wifiMeas, wifiScanResults, venue.wifi_scan_period);
            }
            else
            {
                wifiScanResults.assign(wifiMeas.begin(), wifiMeas.end());
            }

            wifi_log_data.processed_data = {};
            // process wifi data
            if ((is_data_avaliable) && (wifiScanResults.size() > 0))
            {
                wifi_log_data.processed_data = wifi_logging(wifiScanResults);

                for (auto wifiScanRes = wifiScanResults.begin(); wifiScanRes != wifiScanResults.end(); ++wifiScanRes)
                {
                    if (wifiScanRes->scanWiFi.size() > 0)
                    {
                        wifiScanRes->dataset_id = wifi_dataset_id;
                        Fpbl::ReturnStatus status = Builder.processWiFiMeasurement(wifiScanRes->timestamp, *wifiScanRes);
                        track_files.have_wifi = track_files.have_wifi || (status == Fpbl::ReturnStatus::STATUS_SUCCESS);
                    }
                }
                if (track_files.have_wifi)      std::cout << " - processed" << ENDL;
                else                            std::cout << " - rejected" << ENDL;
            }
            else                                std::cout << " - not found" << ENDL;

            std::cout << "wifi dataset_id: " << wifi_dataset_id << ENDL;


#if DEBUG_OUTPUT__IRL_WIFI_DATA
            {// debug output
#if 1
                std::string path_to_file = l2_subdir_name + SLASH + "irl_wifi_data.csv";
#else
                const int max_path_to_file_length = 150;
                std::string path_to_file = l2_subdir_name + "__" + "irl_wifi_data.csv";
                int n;
                while ((n = path_to_file.find("/")) != std::string::npos)
                    path_to_file.replace(n, 1, "_");
                int path_to_file_length = path_to_file.length();
                if (path_to_file_length > max_path_to_file_length)
                {
                    int erase_length = path_to_file_length - max_path_to_file_length;
                    path_to_file.erase(0, erase_length);
                }
                std::cout << path_to_file << ENDL;
#endif
                std::ofstream irl_log(path_to_file, std::ofstream::out | std::ofstream::trunc);
                int64_t num = 0;
                for (auto wifiScanRes = wifiScanResults.begin(); wifiScanRes != wifiScanResults.end(); ++wifiScanRes)
                    wifiScanRes->print(irl_log, num++);
                irl_log.close();
            }
#endif
            wifi_log_data.print(wifi_log, path_to_irl_file);
        }

        // process BLE data
        if (track_files.have_pos)
        {
            track_files.have_ble = false;
            std::vector<BleScanResult > bleScanResults;  // TO DO: speed up vector using by optimization of vector memory reallocation
            bleScanResults.clear();
            //std::cout << "ble data";

            bool is_data_avaliable;
#if ALLOW_BLE_DATA_FROM_IRL
            // load BLE data from IRL file
            is_data_avaliable = parse_irl_file_for_ble(path_to_irl_file, bleScanResults);
            if (is_data_avaliable)
            {
                if (xblp_detection_mode)
                {
                    blp_data_statistics(path_to_irl_file, bleScanResults);
                }
                std::cout << ">>ble data from: " << irl_file << ": " << bleScanResults.size() << " scans";
                for (auto bleScanRes = bleScanResults.begin(); bleScanRes != bleScanResults.end(); ++bleScanRes)
                {
                    // process ble data
                    if (bleScanRes->scanBle.size() > 0)
                    {
                        bleScanRes->dataset_id = ble_dataset_id;
                        Fpbl::ReturnStatus status = Builder.processBleMeasurement(bleScanRes->timestamp, *bleScanRes);
                        track_files.have_ble = track_files.have_ble || (status == Fpbl::ReturnStatus::STATUS_SUCCESS);
                    }
                } 
                if (track_files.have_ble)                   std::cout << " - processed" << ENDL;
                else if ((bleScanResults.size() <= 0))      std::cout << " - not found" << ENDL;
                else                                        std::cout << " - rejected" << ENDL;
            }
#endif

#if ALLOW_BLE_DATA_FROM_MAPPER_LOG
            if ((bleScanResults.size() == 0) && (ble_file.size() > 0))
            {
                is_data_avaliable = true;
                std::cout << ">>ble data from ble-log file: " << ble_file;
                track_files.have_ble = parse_ble_file(l2_subdir_name + SLASH + ble_file, bleScanResults); // TODO: refact parse_wifi_file() just for data reading - not for processing
                std::cout << ": " << bleScanResults.size() << " scans";
                //std::cout << (is_data_avaliable ? " - processed" : " - not found") << ENDL;
                if (is_data_avaliable)
                {
                    for (auto bleScanRes = bleScanResults.begin(); bleScanRes != bleScanResults.end(); ++bleScanRes)
                    {
                        // process ble data
                        if (bleScanRes->scanBle.size() > 0)
                        {
                            bleScanRes->dataset_id = ble_dataset_id;
                            Fpbl::ReturnStatus status = Builder.processBleMeasurement(bleScanRes->timestamp, *bleScanRes);
                            track_files.have_ble = track_files.have_ble || (status == Fpbl::ReturnStatus::STATUS_SUCCESS);
                        }
                    }
                    if (track_files.have_ble)                       std::cout << " - processed" << ENDL;
                    else if ((bleScanResults.size() <= 0))          std::cout << " - not found" << ENDL;
                    else                                            std::cout << " - rejected" << ENDL;
                }
                ble_dataset_id = GetDatasetID(l2_subdir_name + SLASH + ble_file);
                std::cout << "ble dataset_id: " << ble_dataset_id;
            }
#endif
            ble_log_data.loaded_data = ble_logging(bleScanResults);
            ble_log_data.processed_data = ble_logging(bleScanResults);

            ble_log_data.print(ble_log, path_to_irl_file);

#if DEBUG_OUTPUT__IRL_BLE_DATA
            {// debug output
#if 1
                std::string path_to_file = l2_subdir_name + SLASH + "irl_ble_data.csv";
#else
                const int max_path_to_file_length = 150;
                std::string path_to_file = l2_subdir_name + "__" + "irl_ble_data.csv";
                int n;
                while ((n = path_to_file.find("/")) != std::string::npos)
                    path_to_file.replace(n, 1, "_");
                int path_to_file_length = path_to_file.length();
                if (path_to_file_length > max_path_to_file_length)
                {
                    int erase_length = path_to_file_length - max_path_to_file_length;
                    path_to_file.erase(0, erase_length);
                }
                std::cout << path_to_file << ENDL;
#endif
                std::ofstream irl_log(path_to_file, std::ofstream::out | std::ofstream::trunc);
                int64_t num = 0;
                for (auto bleScanRes = bleScanResults.begin(); bleScanRes != bleScanResults.end(); ++bleScanRes)
                    bleScanRes->print(irl_log, num++);
                irl_log.close();
            }
#endif
        }

        return track_files;
    }

    // statistics for BLE beacons - UUID is not zero
    bool FP_builder::blp_data_statistics(const std::string file_path, std::vector<BleScanResult> & bleScanResults)
    {
        std::regex rgx(".*(\\d\\d\\d\\d_\\d\\d_\\d\\d_\\d\\d_\\d\\d_\\d\\d).*"); // date_time pattern
        std::smatch match;
        std::string date_time = match[1];

        if (!std::regex_search(file_path.begin(), file_path.end(), match, rgx))
            return false;

        // binary UUID to string
        auto  uuid2str = [](const uint8_t uuid[16])
        {
            std::string str = "";
            std::stringstream ss;
            for (int i = 0; i < 16; i++)
            {
                ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (uint16_t)uuid[i];
                if (i == 3 || i == 5 || i == 7 || i == 9)
                    ss << "-";
            }
            ss >> str;
            ss.str("");
            ss.clear();
            return str;
        };

        // create list of pointers to each single measurement
        //typedef std::pair<std::vector<BleMeasurement>::iterator, std::vector<BleScanResult>::iterator> meas_item;
        typedef std::pair<BSSID, std::vector<BleMeasurement>::iterator> meas_item;
        std::vector<meas_item> all_meas;
         
        for (auto bleScanRes_it = bleScanResults.begin(); bleScanRes_it != bleScanResults.end(); ++bleScanRes_it)
        {
            // process ble data
            if (bleScanRes_it->scanBle.size() > 0)
            {
                for (auto  meas_it = (bleScanRes_it->scanBle).begin(); meas_it != (bleScanRes_it->scanBle).end(); meas_it++)
                {
                    int i;
                    for (i = 0; i < 16; i++)
                    {
                        if (meas_it->uuid[i] != 0)
                            break;
                    }
                    if (i < 16) // uuid is not zero
                    {
                        all_meas.push_back(std::make_pair( meas_it->mac, meas_it));
                    }
                }
            }
        }

        // sort list
        std::sort(all_meas.begin(), all_meas.end(), [](meas_item a, meas_item b) {
            return (a.first < b.first);
        }); 

        auto iter = all_meas.begin();
        if (all_meas.size() == 0)
            return false;
        BSSID mac1 = iter->first;
        BSSID mac2 = mac1;
        while (iter != all_meas.end())
        {
            int count = 0;
            uint64_t min_time = std::numeric_limits<uint64_t>::max();
            uint64_t max_time = 0;
            int min_txPower = std::numeric_limits<int>::max();
            int max_txPower = std::numeric_limits<int>::min();

            ble_statistics_log << file_path << ", ";
            ble_statistics_log << iter->second->mac << ", " << iter->second->major << ", " << iter->second->minor << ", ";
            ble_statistics_log << uuid2str(iter->second->uuid) << ", ";
            while (mac1 == mac2 && iter != all_meas.end())
            {
                count++;
                uint64_t time = iter->second->timestamp;
                if (time < min_time)
                    min_time = time;
                if (time > max_time)
                    max_time = time;
                int8_t tx_power = iter->second->txPower;
                if (tx_power < min_txPower)
                    min_txPower = tx_power;
                if (tx_power > max_txPower)
                    max_txPower = tx_power;
                mac2 = mac1;
                ++iter;
                if(iter != all_meas.end())
                    mac1 = iter->first;
            }
            ble_statistics_log << count << ", " << min_time << ", " << max_time
                               << ", " << (int)min_txPower << ", " << (int)max_txPower
                               << std::endl;
            mac2 = mac1;
        }
        return true;
    }

    bool FP_builder::ProcessDatasetsAndBuildFingerprints(
        std::string grid_folder,
        std::string fp_bases_folder,
        std::string &in_mag_grid_file,
        std::string &in_wifi_grid_file,
        std::string &in_ble_grid_file,
        std::string &in_portals_grid_file,
		std::string &in_cs_grid_file,
        std::string &out_data_folder,
        std::string &out_mag_grid_file,
        std::string &out_wifi_grid_file,
        std::string &out_ble_grid_file,
        std::string &out_mag_fp_file,
        std::string &out_wifi_fp_file,
        std::string &out_ble_fp_file,
        std::string &in_ble_fprox_file,
        std::vector<std::vector<std::string>>  dataset_list)
    {
        if (dataset_list.size() == 0)
        {
            std::cout << "Input data are absent" << ENDL;
            //return false;
        }

        this->out_data_folder = out_data_folder;

        std::ofstream validation_log(out_data_folder + SLASH + "validation_results.csv", std::ofstream::out | std::ofstream::trunc);

        std::string wifi_log_file = out_data_folder + SLASH + "wifi_log.csv";
        wifi_log.open(wifi_log_file, std::ofstream::out | std::ofstream::trunc);
        std::string ble_log_file = out_data_folder + SLASH + "ble_log.csv";
        ble_log.open(ble_log_file, std::ofstream::out | std::ofstream::trunc);
        
        if (xblp_detection_mode)
        { // open this log only for xblp mode
            std::string ble_statistics_file = out_data_folder + SLASH + "blp_scaning_log.csv";
            ble_statistics_log.open(ble_statistics_file, std::ofstream::out | std::ofstream::trunc);
        }

        std::string wifi_header = "dataset name, Total count of loaded scans, Total count of loaded messages, Total count of loaded APs, Count of processed scans, Count of processed messages, Count of processed APs";
        wifi_log_data.print_header(wifi_log, wifi_header);
        std::string ble_header = "dataset name, Total count of loaded scans, Total count of loaded messages, Total count of loaded APs, Count of processed scans, Count of processed messages, Count of processed APs";
        ble_log_data.print_header(ble_log, ble_header);

        std::ofstream mag_bias_stream(out_data_folder + SLASH + "mag_bias_by_spectr-6.csv", std::ofstream::out);

        //INTEGRATION EXAMLE: start of grid initialisation (2)
        Fpbl::MagneticGrid magneticGrid; // created once, then updated for each track
        Fpbl::WiFiGrid     wifiGrid;
        Fpbl::BleGrid      bleGrid;
        Fpbl::PortalsGrid   portalsGrid;
		Fpbl::CSGrid csGrid;
        {
            Fpbl::GridBuilder Builder;
            if (this->wifi_self_healing_mode)
            {
                // set differenent Wi-Fi pos uncertainty in Grid Builder
            }
            Builder.setVenue(venue);
            Builder.createEmptyGridMFP(venue.id, mag_grid, magneticGrid);
            Builder.createEmptyGridWiFi(venue.id, wifi_grid, wifiGrid);
            Builder.createEmptyGridBLE(venue.id, ble_grid, bleGrid);
            Builder.createEmptyGridPortals(venue.id, mag_grid, portalsGrid);
			Builder.createEmptyGridCS(venue.id, mag_grid, csGrid);
        }
        std::string input_grid_file_name;

        // reading magnetic grid
        if (mag_grid.enable)
        {
            input_grid_file_name = in_mag_grid_file;// +".maggrid";  // read maggrid
            std::cout << "Load magnetic grid from " << input_grid_file_name;
            if (load_mag_grid_file(input_grid_file_name, venue, mag_grid, magneticGrid))
                std::cout << "- ok" << ENDL;
            else
                std::cout << " - no data have been loaded" << ENDL;
        }

        // reading portal grid
		// ! currently removed 
        if (mag_grid.enable) // !!! TODO: use unic portal grid settings structure for portal gird
        {
            input_grid_file_name = in_portals_grid_file;// +".portalsgrid";  // read portalsgrid
            std::cout << "Load portals grid from " << input_grid_file_name;
            if (load_portals_grid_file(input_grid_file_name, venue, mag_grid, portalsGrid))
                std::cout << "- ok" << ENDL;
            else
                std::cout << " - no data has been loaded" << ENDL;
#if 0 // debug, to check number of loaded data
            size_t sz_grid_data = 0;
            for (auto grid_cell : portalsGrid)
            {
                sz_grid_data += grid_cell.resultsInPortal.size();
            }
#endif 
        }

		// reading CS grid
		if (mag_grid.enable) 
		{
			input_grid_file_name = in_cs_grid_file;// +".csgrid";  // read csgrid
			std::cout << "Load CS grid from " << input_grid_file_name;
			if (load_cs_grid_file(input_grid_file_name, venue, mag_grid, csGrid))
				std::cout << "- ok" << ENDL;
			else
				std::cout << " - no data has been loaded" << ENDL;
		}

        // reading wifi grid
        if (wifi_grid.enable)
        {
            input_grid_file_name = in_wifi_grid_file;// +".wifigrid";  // read wifigrid
            std::cout << "Load wifi grid from " << input_grid_file_name;
            if (load_wifi_grid_file(input_grid_file_name, venue, wifi_grid, wifiGrid))
                std::cout << "- ok" << ENDL;
            else
                std::cout << " - no data have been loaded" << ENDL;
        }
        // readnig ble grid
        if (ble_grid.enable)
        {
            input_grid_file_name = in_ble_grid_file;// +".blegrid";  // read blegrid
            std::cout << "Load ble grid from " << input_grid_file_name;
            if (load_ble_grid_file(input_grid_file_name, venue, ble_grid, bleGrid))
                std::cout << " - ok" << ENDL;
            else
                std::cout << " - no data have been loaded" << ENDL;
        }
        //INTEGRATION EXAMLE: end of part

        // init track processor
        //TrackPreProcessing::TrackProcessor track_processor(std::string("empty"));
        TrackPreProcessing::TrackProcessor track_processor(default_mag_validators);

        std::cout << ENDL << "default validators:  " << default_mag_validators << ENDL;

        if (xblp_detection_mode)
        {
            if (std::find(mag_validators.begin(), mag_validators.end(), "corr_reject_position_in_stop") == mag_validators.end())
            {
                mag_validators.push_back("corr_reject_position_in_stop");
            }
        }
        if ( mag_validators.size() > 0)
        {
            std::cout << "actual validators:  " << ENDL;
            // for (int i = 0; i < mag_validators.size(); i++)
            //     std::cout << mag_validators[i] << ENDL;
            track_processor.set_mag_validators(mag_validators);
        }
        std::cout << ENDL;

        if(bleEnableProxOnly)
            parse_ble_proximity_db_file(in_ble_fprox_file, ble_proximity_db);

        for (auto dataset_info : dataset_list)
        {
            //if (dbg_count++ > 5)    break;

            Fpbl::GridBuilder Builder; // new builder instance is created for each track
            Fpbl::ReturnStatus status1 = Builder.setVenue(venue);
            TrackPreProcessing::DataSetParams dataset_params;

            std::cout << ENDL << ENDL << "processing track: " << dataset_info[0] << ENDL;

            // by default
            std::string in_file_mask = this->default_in_file_mask;
            std::string mag_validators_set = this->default_mag_validators;
            double max_position_uncertainty_mfp = this->mag_grid.max_position_uncertainty;
            double max_position_uncertainty_wfp = this->wifi_grid.max_position_uncertainty;
            double max_position_uncertainty_bfp = this->ble_grid.max_position_uncertainty;
            dataset_params.dataset_score_ = 100;
            dataset_params.calibrated_mag_data = true;
            Fpbl::RouteMode_t  route_type = Fpbl::NormalMode;
            Fpbl::DataSetType dataset_type = Fpbl::kSurveyData;
            mag_validators_set = default_mag_validators;

            if (dataset_info.size() > 1)
            {
                const std::string name_dataset_score = "dataset_score";
                const std::string name_dataset_type  = "dataset_type";
                const std::string name_route_type = "route_type";
                std::string name = "";
                std::string valuestr = "";
                double value = 0;
                const char delim = '=';

                auto dataset_attr = dataset_info.begin();
                ++dataset_attr;
                while (dataset_attr != dataset_info.end())
                {
                    parse_dataset_attr(*dataset_attr, delim, name, valuestr);
                    std::cout << "dataset attr:  " << name << " = " << valuestr << ENDL;
                    //////////////
                    if(name == name_dataset_type)
                    {
                        if ((valuestr == "ivl_data") || (valuestr == "venue_data"))
                        {
                            in_file_mask = "ivl.*.dat";
                            mag_validators_set = "for_ivl";
                            max_position_uncertainty_mfp = max_position_uncertainty_mfp_default_ivl;
                            max_position_uncertainty_wfp = max_position_uncertainty_wfp_default_ivl;
                            max_position_uncertainty_bfp = max_position_uncertainty_bfp_default_ivl;
                            dataset_params.calibrated_mag_data = true;
                            dataset_type = Fpbl::kIvlData;
                        }
                        if (valuestr == "survey_data")
                        {
                            in_file_mask = "irl.*.dat";
                            mag_validators_set = "for_coursa_survey_tool";
                            max_position_uncertainty_mfp = max_position_uncertainty_mfp_default_survey;
                            max_position_uncertainty_wfp = max_position_uncertainty_wfp_default_survey;
                            max_position_uncertainty_bfp = max_position_uncertainty_bfp_default_survey;
                            dataset_params.calibrated_mag_data = true;
                            dataset_type = Fpbl::kSurveyData;
                        }
                        if (valuestr == "maper_data")
                        {
                            in_file_mask = "tpn.dat";
                            mag_validators_set = "for_mapper";
                            max_position_uncertainty_mfp = max_position_uncertainty_mfp_default_survey;
                            max_position_uncertainty_wfp = max_position_uncertainty_wfp_default_survey;
                            max_position_uncertainty_bfp = max_position_uncertainty_bfp_default_survey;
                            dataset_params.calibrated_mag_data = true;
                            dataset_type = Fpbl::kMapperData;
                        }
                        if (valuestr == "retail_data")
                        {
                            in_file_mask = "irl.*.dat";
                            mag_validators_set = "for_irl";
                            max_position_uncertainty_mfp = max_position_uncertainty_mfp_default_retail;
                            max_position_uncertainty_wfp = max_position_uncertainty_wfp_default_retail;
                            max_position_uncertainty_bfp = max_position_uncertainty_bfp_default_retail;
                            dataset_params.calibrated_mag_data = false;
                            dataset_type = Fpbl::kRetailData;
                        }
                        if (valuestr == "robotic_survey_data")
                        {
                            in_file_mask = "irl.*.dat";
                            mag_validators_set = "for_robo_survey";
                            max_position_uncertainty_mfp = max_position_uncertainty_mfp_default_survey;
                            max_position_uncertainty_wfp = max_position_uncertainty_wfp_default_survey;
                            max_position_uncertainty_bfp = max_position_uncertainty_bfp_default_survey;
                            dataset_params.calibrated_mag_data = true;
                            dataset_type = Fpbl::kRoboticData;
                        }
                    }
                    //////////////
                    if (name == name_dataset_score)
                    {
                        //number_from_string(valuestr, value);
                        value = std::stod(valuestr);
                        dataset_params.dataset_score_ = ((value >= 0) && (value <= 100)) ? value : 100;
                        std::cout << "dataset score:  " << dataset_params.dataset_score_ << ENDL;
                    }
                    //////////////
                    if (name == name_route_type)
                    {
                        route_type = Fpbl::Unknown;
                        //std::cout << "route_type:  " << valuestr;

                        if (valuestr == "single-cell")
                        {
                            route_type = Fpbl::SingleCellMode;
                            mag_validators_set = "for_single-cell";
                        }
                        if (valuestr == "normal")
                        {
                            route_type = Fpbl::NormalMode;
                        }
                        if (valuestr == "elevator")
                        {
                            route_type = Fpbl::ElevatorMode;
                        }
                        if (valuestr == "escalator")
                        {
                            route_type = Fpbl::EscalatorMode;
                        }
                        if (valuestr == "stairs")
                        {
                            route_type = Fpbl::StairsMode;
                        }
                        if (valuestr == "conveyor-belt")
                        {
                            route_type = Fpbl::ConveyorBeltMode;
                        }
                        if (valuestr == "straight-line")
                        {
                            route_type = Fpbl::NormalMode;
                        }
                        if (route_type == Fpbl::Unknown)
                            std::cout << "  (Unknown)";
                        std::cout << ENDL;
                    }
                    //////////////
                    ++dataset_attr;
                }
            }

            std::cout << "mag validators set:  " << mag_validators_set << ENDL;
            track_processor.ClearValidatorList();
            track_processor.SetDefaultValidators(mag_validators_set);

            if (dataset_type != previous_dataset_type)
            {
                track_processor.PrintValidationReportHeader(validation_log);
                previous_dataset_type = dataset_type;
            }

#if ENABLE_MAG_CALIBRATION
            if (false == dataset_params.calibrated_mag_data)
            {
                struct dirent *l2_subentry;
                DIR *l2_subdir_pointer = opendir(dataset_info[0].c_str());

                bool is_present_mag_out = false;
                bool is_present_irl = false;
                std::string irl_file_name;
                while ((l2_subdir_pointer != 0) && (l2_subentry = readdir(l2_subdir_pointer)))
                {
                    if (l2_subentry->d_type == DT_REG)
                    {
                        std::string file_name = std::string(l2_subentry->d_name);
                        if (std::regex_match(std::string(file_name), std::regex(mag_out_file)))
                            is_present_mag_out = true;

                        if (std::regex_match(std::string(file_name), std::regex(irl_file_mask)))
                        {
                            irl_file_name = dataset_info[0] + SLASH + file_name;
                            is_present_irl = GetFileSize(irl_file_name) > 0;
                        }
                    }
                }
                closedir(l2_subdir_pointer);

                if ((is_present_mag_out == false) && (is_present_irl == true))
                {
                    // load irl data
                    std::vector<TpnOutput> irl_data;
                    TrackPreProcessing::MagBias mag_data = {};

                    bool result = parse_irl_file(irl_file_name, irl_data, true);

                    if (result)
                    {
                        result = CalculateMagBiasForDataset(irl_data, mag_data);
                    }

                    if (result)
                    {
                        std::string mag_out_file_name = dataset_info[0] + SLASH + mag_out_file;
                        result = save_mag_out_file(mag_out_file_name, mag_data);
                    }
                }
            }
#endif
            if (xblp_detection_mode)
            {
                if (std::find(mag_validators.begin(), mag_validators.end(), "corr_reject_position_in_stop") == mag_validators.end())
                {
                    mag_validators.push_back("corr_reject_position_in_stop");
                }
            }
            if (mag_validators.size() > 0)
            {
                std::cout << "additional validators:  " << ENDL;
                for (int i = 0; i < mag_validators.size(); i++)
                     std::cout << mag_validators[i] << ENDL;
                track_processor.set_mag_validators(mag_validators);
            }

            FP_builder::irl_file_mask = std::regex(in_file_mask);

            Builder.set_max_uncertainty_mfp(max_position_uncertainty_mfp);
            Builder.set_max_uncertainty_wifi(max_position_uncertainty_wfp);
            Builder.set_max_uncertainty_ble(max_position_uncertainty_bfp);

            if (wifi_self_healing_mode)
            {
                Builder.set_max_uncertainty_wifi(5.0);
            }
            if (xblp_detection_mode)
            {
                Builder.set_max_uncertainty_ble(venue.ble_proximity_data.max_position_uncertainty);
            }

            TrackFiles track_files = ProcesDataSet(dataset_info[0], dataset_params, validation_log, mag_bias_stream, Builder, track_processor, route_type, dataset_type);

            //INTEGRATION EXAMLE: start of grid update (4)
            //Fpbl::ReturnStatus status;
            if ((track_files.have_irl) || (track_files.have_mg && track_files.have_pos))
            {
                if (mag_grid.enable)
                    Builder.updateGridMFP(venue.id, mag_grid, magneticGrid);
            }

            if (track_files.have_wifi)
            {
                if (wifi_grid.enable)
                    Builder.updateGridWiFi(venue.id, wifi_grid, wifiGrid);
            }
            //std::cout << "updateGridBLE";
            if (track_files.have_ble)
            {
                if (ble_grid.enable)
                    Builder.updateGridBLE(venue.id, ble_grid, bleGrid);
            }

            Builder.updateGridPortals(venue.id, mag_grid, portalsGrid);

            //INTEGRATION EXAMLE: end of part

            track_files.have_mg = false;
            track_files.have_pos = false;
            track_files.have_wifi = false;
            track_files.have_ble = false;
            track_files.have_irl = false;
            track_files.have_mag_out = false;
        }

        mag_bias_stream.close();
        wifi_log.close();
        ble_log.close();


        std::cout << ENDL << "Final operations:" << ENDL;
        std::string output_file_name;
        Fpbl::ReturnStatus operation_result;

#if 1 // debug, enable disable 
        size_t sz_grid_data = 0;
        for (auto it_cell = portalsGrid.begin(); it_cell != portalsGrid.end(); ++it_cell)
        {
            sz_grid_data += it_cell->resultsInPortal.size();
#if 0 // sorting for debug only 
            std::vector<FfPosition> resultsInPortal = it_cell->resultsInPortal;

            // sort portals cell
            std::sort(resultsInPortal.begin(), resultsInPortal.end(), [](FfPosition a, FfPosition b) {
                if (a.floor_number != b.floor_number) return (a.floor_number < b.floor_number);
                else if (a.x != b.x)  return (a.x < b.x);
                else if (a.y != b.y)  return (a.y < b.y);
                else if (a.altitude != b.altitude)  return (a.altitude < b.altitude);
                else   return (a.mode_of_transit > b.mode_of_transit);
            });

            it_cell->resultsInPortal = resultsInPortal;
#endif
        }
        //std::cout << "sz_grid_data: " << sz_grid_data << std::endl;
        if (sz_grid_data > 0)
        {
#if 0 // sorting for debug only 
            // sort portals grid
            std::sort(portalsGrid.begin(), portalsGrid.end(), [](Fpbl::PortalData a, Fpbl::PortalData b) {
                if (a.positionInPortal.floor !=  b.positionInPortal.floor) return (a.positionInPortal.floor < b.positionInPortal.floor);
                else if (a.positionInPortal.x !=  b.positionInPortal.x) return (a.positionInPortal.x < b.positionInPortal.x);
                else return (a.positionInPortal.y < b.positionInPortal.y);
            });
#endif
           // saving portal grid
		   // TODO: !!!!! change savePortalsGrid format to make it work
            std::cout << "Portals grid saving - ";
            output_file_name = grid_folder + SLASH + out_wifi_grid_file + ".portalsgrid";   // save portal grid
            savePortalsGrid(output_file_name, portalsGrid);
            std::cout << "done." << ENDL;
        }

        // Magnetic grid and fingerprint
        sz_grid_data = 0;
#if 0 // sorting for debug only
        for (auto it_cell = magneticGrid.begin(); it_cell != magneticGrid.end(); ++it_cell)
        {
            sz_grid_data += it_cell->magData.size();
            std::vector<MagneticData> magData = it_cell->magData;

            // sort magnetic cell
            std::sort(magData.begin(), magData.end(), [](MagneticData a, MagneticData b) {
                if (a.mX != b.mX) return (a.mX > b.mX);
                if (a.mY != b.mY) return (a.mY > b.mY);
                if (a.mZ != b.mZ) return (a.mZ > b.mZ);
                else   return false;
            });

            it_cell->magData = magData;
        }
#endif
        for (auto grid_cell : magneticGrid)
        {
            sz_grid_data += grid_cell.magData.size();
        }
        if (sz_grid_data > 0)
        {
            // saving magnetic grid
            if (mag_grid.enable)
            {
                output_file_name = grid_folder + SLASH + out_mag_grid_file + ".maggrid";  // save maggrid
                std::cout << "Mag grid saving - " << output_file_name << " - ";
                operation_result = saveMagneticGrid(output_file_name, magneticGrid);
                std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;
            }

            std::cout << "MFP build - ";
            Fpbl::MfpBuilder *pMfp = new Fpbl::MfpBuilder();
            {
                Fpbl::eStdEstimatorType stdEstimatorType = mag_fp.stdEstimatorType;
                std::cout << "std estimator type is ";
                if (stdEstimatorType == Fpbl::eRobustStdEstimator)
                    std::cout << "robust" << std::endl;
                else
                {
                    if (stdEstimatorType == Fpbl::eQuantileStdEstimator)
                        std::cout << "quantile" << std::endl;
                    else
                        if (mag_fp.stdEstimatorType == Fpbl::eCombineStdEstimator)
                            std::cout << "combine" << std::endl;
                        else
                        {
                            std::cout << "unknown: set default (robust)" << std::endl;
                            stdEstimatorType = Fpbl::eRobustStdEstimator;
                        }
                }
                pMfp->setStdEstimatorType(stdEstimatorType);
                pMfp->setDefaultMagdata(venue.magX, venue.magY, venue.magZ);
            }
            Fpbl::MfpBuilder::LocalDB dbmfp(mag_fp);
            operation_result = pMfp->buildFingerprint(magneticGrid, &dbmfp); // build MFP
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;

            std::cout << "MFP update with portals - ";
            operation_result = pMfp->updateFingerprint(portalsGrid, &dbmfp); // build MFP
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;
     		operation_result = pMfp->updateFingerprintForCrowdsourced(csGrid, &dbmfp);

            output_file_name = fp_bases_folder + SLASH + out_mag_fp_file + ".mfp" + std::to_string(mag_fp_format);  // save MFP DB
            std::cout << "MFP db saving - " << output_file_name << " - ";
            operation_result = saveMagneticDB(output_file_name, dbmfp, mag_fp_format);
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;

            output_file_name = fp_bases_folder + SLASH + out_mag_fp_file + ".mfp" + std::to_string(4);  // save MFP DB
            std::cout << "MFP db saving - " << output_file_name << " - ";
            operation_result = saveMagneticDB(output_file_name, dbmfp, 4);
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;
        }

        // WIFI grid and fingerprint
        sz_grid_data = 0;
        for (auto grid_cell : wifiGrid)
        {
            sz_grid_data += grid_cell.wifiData.size();
        }
        if (sz_grid_data > 0)
        {
            // saving WiFi grid
            if (wifi_grid.enable)
            {
                std::cout << "WiFi grid saving - ";
                output_file_name = grid_folder + SLASH + out_wifi_grid_file + ".wifigrid"; // save wifi grid
                saveWiFiGrid(output_file_name, wifiGrid);
                std::cout << "done." << ENDL;
            }
            //output_file_name = grid_folder + SLASH + out_wifi_grid_file + ".wifi3.mat"; // save wifi grid debug file
            //saveWiFiGridForMatlab( output_file_name, wifiGrid );

            std::cout << "WFP build - ";
            Fpbl::WiFiBuilder *pWiFi = new Fpbl::WiFiBuilder(wifi_fp);
            Fpbl::WiFiBuilder::LocalDB dbwifi;
            operation_result = pWiFi->buildFingerprint(wifiGrid, &dbwifi); // build WFP
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;

            //output_file_name = fp_bases_folder + SLASH + out_wifi_fp_file + ".wifi3"; // save WFP DB
            output_file_name = fp_bases_folder + SLASH + out_wifi_fp_file + ".wifi" + std::to_string(mag_fp_format); // save WFP DB
            std::cout << "WFP db saving - " << output_file_name << " - ";
            operation_result = saveWiFiDb(output_file_name, dbwifi, wifi_fp_format);
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;

            output_file_name = fp_bases_folder + SLASH + out_wifi_fp_file + ".wifi" + std::to_string(4); // save WFP DB
            std::cout << "WFP db saving - " << output_file_name << " - ";
            operation_result = saveWiFiDb(output_file_name, dbwifi, 4);
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;
        }

        // BLE grid and fingerprint
        sz_grid_data = 0;
        for (auto grid_cell : bleGrid)
        {
            sz_grid_data += grid_cell.bleData.size();
        }
        
        if (sz_grid_data > 0)
        {
            // saving BLE grid
            if (ble_grid.enable)
            {
                std::cout << "BLE grid saving - ";
                output_file_name = grid_folder + SLASH + out_ble_grid_file + ".blegrid";   // save ble grid
                saveBleGrid(output_file_name, bleGrid);
                std::cout << "done." << ENDL;
            }

            std::cout << "BFP build - ";
            Fpbl::BleBuilder *pBle = new Fpbl::BleBuilder(ble_fp);
            Fpbl::BleBuilder::LocalDB dbble;
            operation_result = pBle->buildFingerprint(bleGrid, &dbble); // build BFP
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;

            //output_file_name = fp_bases_folder + SLASH + out_ble_fp_file + ".ble3";  // save BFP DB
            output_file_name = fp_bases_folder + SLASH + out_ble_fp_file + ".ble" + std::to_string(mag_fp_format);  // save BFP DB
            std::cout << "BFP db saving - " << output_file_name << " - ";
            operation_result = saveBleDb(output_file_name, dbble, ble_fp_format);
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;

            output_file_name = fp_bases_folder + SLASH + out_ble_fp_file + ".ble" + std::to_string(4);  // save BFP DB
            std::cout << "BFP db saving - " << output_file_name << " - ";
            operation_result = saveBleDb(output_file_name, dbble, 4);
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;
        }
#endif
#if 1
        if (bleProxEnable)
        {
            std::cout << "BLE proximity db reading - " << in_ble_fprox_file << ENDL;

            //std::vector<std::string> ble_proximity_db;
            parse_ble_proximity_db_file(in_ble_fprox_file, ble_proximity_db);

            output_file_name = fp_bases_folder + SLASH + out_ble_fp_file + ".blp" + std::to_string(3);  // save BFP DB
            std::cout << "BLE proximity db format 3 saving - " << output_file_name << " - ";
            operation_result = saveBleProximityDb(output_file_name, &ble_proximity_db, 3);
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;
            output_file_name = fp_bases_folder + SLASH + out_ble_fp_file + ".blp" + std::to_string(4);  // save BFP DB
            std::cout << "BLE proximity db format 4 saving - " << output_file_name << " - ";
            operation_result = saveBleProximityDb(output_file_name, &ble_proximity_db, 4);
            std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;
            //std::cout << std::endl;
        }
#endif

        return true;
    }

    long FP_builder::GetFileSize(std::string filename)
    {
        struct stat stat_buf;
        int rc = stat(filename.c_str(), &stat_buf);
        return rc == 0 ? stat_buf.st_size : -1;
    }

    // load grids

    bool FP_builder::load_mag_grid_file(const std::string &fname, const BaseVenueType &venue, const Fpbl::Grid &mag_grid_param, Fpbl::MagneticGrid  &magnetic_grid)
    {
        std::ifstream infile(fname);
        if (infile.bad() || infile.fail())
            return false;

        std::string line;

        Fpbl::MagneticMeasurementAndDataSetType  mag_data;

        Fpbl::MagneticCell  mag_cell;
        double  x_prev, y_prev;
        int16_t floor_prev;
        double  x, y;
        int16_t floor;
        std::stringstream ss;
        long counter = 0;

        x_prev = -1.0;
        y_prev = -1.0;
        floor_prev = -1;

        double cell_size = mag_grid_param.size; // grid.size parameter is actually the size of cell
        uint32_t cellXQuantity = (uint32_t)ceil(venue.size_x / cell_size);
        uint32_t cellYQuantity = (uint32_t)ceil(venue.size_y / cell_size);
        uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;
        int64_t positionInGrid = -1;

        while (std::getline(infile, line))
        {
            if (line == "")
                continue;

            counter++;

            ss << line;

            ss >> x;
            ss >> y;
            ss >> floor;

            if ((x != x_prev) || (y != y_prev) || (floor != floor_prev))
            {
                if (mag_cell.magData.size() > 0)
                {
                    int32_t x_pos = (int32_t)(mag_cell.coordinates.x / venue.size_x * (venue.size_x / cell_size));
                    int32_t y_pos = (int32_t)(mag_cell.coordinates.y / venue.size_y * (venue.size_y / cell_size));

                    positionInGrid = y_pos + x_pos * cellYQuantity + mag_cell.coordinates.floor * gridCellsQuantity;

                    if (positionInGrid < 0 || positionInGrid >= magnetic_grid.size())
                        continue;

                    magnetic_grid[(std::vector<int>::size_type)positionInGrid] = mag_cell;
                }

                // new coordinates
                mag_cell.coordinates.x = x;
                mag_cell.coordinates.y = y;
                mag_cell.coordinates.floor = floor;

                mag_cell.magData.clear();
            }

            ss >> mag_data.mX;
            ss >> mag_data.mY;
            ss >> mag_data.mZ;
            ss >> mag_data.covarianceMatrix[0][0];
            ss >> mag_data.covarianceMatrix[0][1];
            ss >> mag_data.covarianceMatrix[0][2];
            ss >> mag_data.covarianceMatrix[1][0];
            ss >> mag_data.covarianceMatrix[1][1];
            ss >> mag_data.covarianceMatrix[1][2];
            ss >> mag_data.covarianceMatrix[2][0];
            ss >> mag_data.covarianceMatrix[2][1];
            ss >> mag_data.covarianceMatrix[2][2];
            mag_data.dataset_type = Fpbl::kSurveyData;
            {
                std::string  str;
                ss >> str;
                try
                {
                    int dataset_type = std::stoi(str);
                    mag_data.dataset_type = (Fpbl::DataSetType)dataset_type;
                }
                catch (const std::invalid_argument& e) {
                }
            }
            mag_data.timestamp = 0;

            ss.str("");
            ss.clear();

            mag_cell.magData.push_back(mag_data);

            x_prev = x;
            y_prev = y;
            floor_prev = floor;

        }
        if (mag_cell.magData.size() > 0)
        {
            int32_t x_pos = (int32_t)(mag_cell.coordinates.x / venue.size_x * (venue.size_x / cell_size));
            int32_t y_pos = (int32_t)(mag_cell.coordinates.y / venue.size_y * (venue.size_y / cell_size));

            positionInGrid = y_pos + x_pos * cellYQuantity + mag_cell.coordinates.floor * gridCellsQuantity;

            if (positionInGrid >= 0 && positionInGrid < magnetic_grid.size())
            {
                magnetic_grid[(std::vector<int>::size_type)positionInGrid] = mag_cell;
            }
        }

        return true;
    }

	bool FP_builder::load_wifi_grid_file(const std::string &fname, const BaseVenueType &venue, const Fpbl::Grid &wifi_grid_param, Fpbl::WiFiGrid  &wifi_grid)
    {
        std::ifstream infile(fname);
        if (infile.bad() || infile.fail())
            return false;

        std::string line;

        WiFiScanResult  wifi_scan_result;
        WiFiMeasurement wifiMeasurement;
        Fpbl::WifiCell  wifi_cell;
        double  x_prev, y_prev;
        int16_t floor_prev;
        int64_t time_prev = 0;
        int64_t tmp_scan_time = 0;
        double  x, y;
        int16_t floor;
        std::string  str;
        std::stringstream ss;
        int16_t rssi;
        x_prev = -1.0;
        y_prev = -1.0;
        floor_prev = -1;
        uint32_t dataset_id = 0;
        uint32_t dataset_id_prev = 0;

        double cell_size = wifi_grid_param.size; // grid.size parameter is actually the size of cell
        uint32_t cellXQuantity = (uint32_t)ceil(venue.size_x / cell_size);
        uint32_t cellYQuantity = (uint32_t)ceil(venue.size_y / cell_size);
        uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;
        int64_t positionInGrid = -1;

        while (std::getline(infile, line))
        {
            if (line == "")
                continue;

            ss << line;

            ss >> x;
            ss >> y;
            ss >> floor;

            if ((x_prev != -1) && ((x != x_prev) || (y != y_prev) || (floor != floor_prev)))
            {
                wifi_cell.wifiData.push_back(wifi_scan_result);
                if (wifi_cell.wifiData.size() > 0)
                {
                    int32_t x_pos = (int32_t)(wifi_cell.coordinates.x / venue.size_x * (venue.size_x / cell_size));
                    int32_t y_pos = (int32_t)(wifi_cell.coordinates.y / venue.size_y * (venue.size_y / cell_size));

                    positionInGrid = y_pos + x_pos * cellYQuantity + wifi_cell.coordinates.floor * gridCellsQuantity;

                    if (positionInGrid < 0 || positionInGrid >= wifi_grid.size())
                        continue;

                    wifi_grid[(std::vector<int>::size_type)positionInGrid] = wifi_cell;
                }
                wifi_scan_result.scanWiFi.clear();
                wifi_cell.wifiData.clear();
            }

            ss >> str;
            wifiMeasurement.mac = stringMac_to_intMac(str);

            ss >> rssi;
            wifiMeasurement.rssi = (int8_t)rssi;
            ss >> wifiMeasurement.frequency;
            ss >> wifiMeasurement.timestamp;
            ss >> tmp_scan_time;
            ss >> dataset_id;

            if (time_prev == 0)
                time_prev = tmp_scan_time;
            if ((time_prev > 0) && ((dataset_id != dataset_id_prev) || (tmp_scan_time != time_prev)))
            {
                // ready to push back scan result, since we got to the new scan
                // wifi_cell.wifiData.push_back( wifi_scan_result [t = t1] )
                // wifi_cell.wifiData.push_back( wifi_scan_result [t = t2] )
                // ...
                // wifi_cell.wifiData.push_back( wifi_scan_result [t = tN] )

                if (wifi_scan_result.scanWiFi.size() > 0)
                {
                    wifi_cell.wifiData.push_back(wifi_scan_result);
                }
                wifi_scan_result.scanWiFi.clear();
            }

            wifi_scan_result.timestamp = tmp_scan_time;
            wifi_scan_result.dataset_id = dataset_id;

            wifi_scan_result.scanWiFi.push_back(wifiMeasurement);
            wifi_cell.coordinates.x = x;
            wifi_cell.coordinates.y = y;
            wifi_cell.coordinates.floor = floor;

            time_prev = tmp_scan_time;
            dataset_id_prev = dataset_id;
            x_prev = x;
            y_prev = y;
            floor_prev = floor;

            ss.str("");
            ss.clear();
        } /* while */

        if (wifi_scan_result.scanWiFi.size() > 0)
        {
            wifi_cell.wifiData.push_back(wifi_scan_result); // pushing the leftover scan result to cell
        }
        // assigning the leftover data in a cell to the grid
        if (wifi_cell.wifiData.size() > 0)
        {
            int32_t x_pos = (int32_t)(wifi_cell.coordinates.x / venue.size_x * (venue.size_x / cell_size));
            int32_t y_pos = (int32_t)(wifi_cell.coordinates.y / venue.size_y * (venue.size_y / cell_size));

            positionInGrid = y_pos + x_pos * cellYQuantity + wifi_cell.coordinates.floor * gridCellsQuantity;

            if (positionInGrid >= 0 && positionInGrid < wifi_grid.size())
            {
                wifi_grid[(std::vector<int>::size_type)positionInGrid] = wifi_cell;

                //std::cout << counter << "\t" << wifi_cell.wifiData.size() << ENDL;
                //std::cout << wifi_cell.coordinates.x << "\t" << wifi_cell.coordinates.y << "\t" << wifi_cell.wifiData.size() << ENDL;
            }
        }
        wifi_scan_result.scanWiFi.clear();
        wifi_cell.wifiData.clear();

#if 0
        std::ofstream f_grid("wifi_grid_loded.txt", std::ios::out);
        int i, j, k;
        for (auto pos_it = wifi_grid.cbegin(); pos_it != wifi_grid.cend(); ++pos_it)
        {
            for (j = 0; j < pos_it->wifiData.size(); j++)
            {
                for (k = 0; k < pos_it->wifiData[j].scanWiFi.size(); k++)
                {
                    f_grid << pos_it->coordinates.x << "\t";
                    f_grid << pos_it->coordinates.y << "\t";
                    f_grid << pos_it->coordinates.floor << "\t";

                    std::stringstream ss;
                    std::string mac_s;

                    ss << std::hex << /*std::uppercase <<*/
                        std::fixed << std::setw(2) << std::setfill('0') <<
                        ((pos_it->wifiData[j].scanWiFi[k].mac >> 48) & 0xFF) << ":" <<
                        std::fixed << std::setw(2) << std::setfill('0') <<
                        ((pos_it->wifiData[j].scanWiFi[k].mac >> 40) & 0xFF) << ":" <<
                        std::fixed << std::setw(2) << std::setfill('0') <<
                        ((pos_it->wifiData[j].scanWiFi[k].mac >> 32) & 0xFF) << ":" <<
                        std::fixed << std::setw(2) << std::setfill('0') <<
                        ((pos_it->wifiData[j].scanWiFi[k].mac >> 24) & 0xFF) << ":" <<
                        std::fixed << std::setw(2) << std::setfill('0') <<
                        ((pos_it->wifiData[j].scanWiFi[k].mac >> 16) & 0xFF) << ":" <<
                        std::fixed << std::setw(2) << std::setfill('0') <<
                        ((pos_it->wifiData[j].scanWiFi[k].mac >> 8) & 0xFF) << ":" <<
                        std::fixed << std::setw(2) << std::setfill('0') <<
                        ((pos_it->wifiData[j].scanWiFi[k].mac >> 0) & 0xFF);
                    ss >> mac_s;
                    f_grid << mac_s << "\t";
                    //
                    f_grid << (int16_t)pos_it->wifiData[j].scanWiFi[k].rssi << "\t";
                    f_grid << pos_it->wifiData[j].scanWiFi[k].frequency;

                    f_grid << "\t";
                    f_grid << pos_it->wifiData[j].scanWiFi[k].timestamp; // time of measurement, each scanWiFi[k] has "WiFiMeasurement" type

                    f_grid << "\t";
                    f_grid << pos_it->wifiData[j].timestamp; // time of scan, each wifiData[j] is struct: "time + WiFi_meas_vector"

                    f_grid << "\t";
                    f_grid << pos_it->wifiData[j].dataset_id;

                    f_grid << std::endl;
                }
            }

        }
#endif
        return true;
    }

    bool FP_builder::load_ble_grid_file(const std::string &fname, const BaseVenueType &venue, const Fpbl::Grid &ble_grid_param, Fpbl::BleGrid  &ble_grid)
    {
        std::ifstream infile(fname);
        if (infile.bad() || infile.fail())
            return false;

        std::string line;

        BleScanResult  ble_scan_result;
        BleMeasurement bleMeasurement;
        Fpbl::BleCell  ble_cell;
        double  x_prev, y_prev;
        int16_t floor_prev;
        int64_t time_prev = 0;
        double  x, y;
        int16_t floor;
        std::string  str;
        std::stringstream ss;
        int16_t rssi;

        ble_scan_result.timestamp = 0;
        x_prev = -1.0;
        y_prev = -1.0;
        floor_prev = -1;
        uint32_t dataset_id = 0;
        uint32_t dataset_id_prev = 0;

        double cell_size = ble_grid_param.size; // grid.size parameter is actually the size of cell
        uint32_t cellXQuantity = (uint32_t)ceil(venue.size_x / cell_size);
        uint32_t cellYQuantity = (uint32_t)ceil(venue.size_y / cell_size);
        uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;
        int64_t positionInGrid = -1;

        while (std::getline(infile, line))
        {
            if (line == "")
                continue;

            ss << line;

            ss >> x;
            ss >> y;
            ss >> floor;

            if ((x_prev != -1) && ((x != x_prev) || (y != y_prev) || (floor != floor_prev))) // this executes for each new cell in the grid file, except for cell number 1
            {
                if (ble_scan_result.scanBle.size() > 0)
                {
                    ble_scan_result.timestamp = static_cast<int64_t>((ble_scan_result.scanBle.back().timestamp + 0.5* ble_scan_period) / ble_scan_period) * ble_scan_period;
                    ble_cell.bleData.push_back(ble_scan_result);
                }

                if (ble_cell.bleData.size() > 0)
                {
                    int32_t x_pos = (int32_t)(ble_cell.coordinates.x / venue.size_x * (venue.size_x / cell_size));
                    int32_t y_pos = (int32_t)(ble_cell.coordinates.y / venue.size_y * (venue.size_y / cell_size));

                    positionInGrid = y_pos + x_pos * cellYQuantity + ble_cell.coordinates.floor * gridCellsQuantity;

                    if (positionInGrid < 0 || positionInGrid >= ble_grid.size())
                        continue;

                    ble_grid[(std::vector<int>::size_type)positionInGrid] = ble_cell;
                }
                ble_scan_result.scanBle.clear();
                ble_scan_result.timestamp = 0;
                ble_scan_result.dataset_id = 0;
                ble_cell.bleData.clear();
            }

            bleMeasurement.timestamp = 0;

            ss >> bleMeasurement.mac;
            ss >> rssi;
            bleMeasurement.rssi = (int8_t)rssi;
            ss >> bleMeasurement.frequency;
            ss >> bleMeasurement.timestamp;
            ss >> dataset_id;

            // set default
            bleMeasurement.major = 0xffff;
            bleMeasurement.minor = 0xffff;
            memset(&bleMeasurement.uuid[0], 0xff, sizeof(bleMeasurement.uuid));
            bleMeasurement.txPower = -59;

            //int64_t scan_timestamp = static_cast<int64_t>((bleMeasurement.timestamp + 0.5* ble_scan_period) / ble_scan_period) * ble_scan_period;
            int64_t scan_timestamp = bleMeasurement.timestamp;

            if ( (time_prev > 0) && ((dataset_id != dataset_id_prev) || (scan_timestamp != time_prev)) )
            {
                // ready to push back scan result, since we got to the new scan
                // ble_cell.bleData.push_back( ble_scan_result [t = t1] )
                // ble_cell.bleData.push_back( ble_scan_result [t = t2] )
                // ...
                // ble_cell.bleData.push_back( ble_scan_result [t = tN] )

                if (ble_scan_result.scanBle.size() > 0)
                {
                    ble_scan_result.timestamp = static_cast<int64_t>((ble_scan_result.scanBle.back().timestamp + 0.5* ble_scan_period) / ble_scan_period) * ble_scan_period;
                    ble_cell.bleData.push_back(ble_scan_result);
                }
                ble_scan_result.scanBle.clear();
            }

            ble_scan_result.scanBle.push_back(bleMeasurement);
            ble_scan_result.dataset_id = dataset_id;

            time_prev = scan_timestamp;
            dataset_id_prev = dataset_id;
            ble_cell.coordinates.x = x;
            ble_cell.coordinates.y = y;
            ble_cell.coordinates.floor = floor;

            x_prev = x;
            y_prev = y;
            floor_prev = floor;

            ss.str("");
            ss.clear();
        }  /* while */

        if (ble_scan_result.scanBle.size() > 0)
        {
            ble_scan_result.timestamp = static_cast<int64_t>((ble_scan_result.scanBle.back().timestamp + 0.5* ble_scan_period) / ble_scan_period) * ble_scan_period;
            ble_scan_result.dataset_id = dataset_id;
            ble_cell.bleData.push_back(ble_scan_result);
        }
        if (ble_cell.bleData.size() > 0)
        {
            int32_t x_pos = (int32_t)(ble_cell.coordinates.x / venue.size_x * (venue.size_x / cell_size));
            int32_t y_pos = (int32_t)(ble_cell.coordinates.y / venue.size_y * (venue.size_y / cell_size));

            positionInGrid = y_pos + x_pos * cellYQuantity + ble_cell.coordinates.floor * gridCellsQuantity;

            if (positionInGrid >= 0 && positionInGrid < ble_grid.size())
            {
                ble_grid[(std::vector<int>::size_type)positionInGrid] = ble_cell;
            }
        }
        ble_scan_result.scanBle.clear();
        ble_cell.bleData.clear();
#if 0
        {
            std::ofstream f_grid("ble_grid_loded.txt", std::ios::out);
            uint32_t i, j, k;

            for (i = 0; i < ble_grid.size(); i++)
            {
                if (ble_grid[i].bleData.size() > 0)
                {
                    for (j = 0; j < ble_grid[i].bleData.size(); j++)
                    {
                        for (k = 0; k < ble_grid[i].bleData[j].scanBle.size(); k++)
                        {
                            f_grid << ble_grid[i].coordinates.x << "\t";
                            f_grid << ble_grid[i].coordinates.y << "\t";
                            f_grid << ble_grid[i].coordinates.floor << "\t";

                            f_grid << ble_grid[i].bleData[j].scanBle[k].mac << "\t";

                            f_grid << (int16_t)ble_grid[i].bleData[j].scanBle[k].rssi << "\t";
                            f_grid << ble_grid[i].bleData[j].scanBle[k].frequency;

                            f_grid << "\t";
                            f_grid << ble_grid[i].bleData[j].scanBle[k].timestamp; // time of measurement, each scanBLe[k] has "BleMeasurement" type

                            f_grid << "\t";
                            f_grid << ble_grid[i].bleData[j].dataset_id;

                            f_grid << ENDL;
                        }
                    }
                }
            }

            f_grid.close();
        }
#endif

        return true;
    }

#if 0
    bool FP_builder::load_portals_grid_file(const std::string &fname, const BaseVenueType &venue, const Fpbl::Grid &mag_grid_param, Fpbl::PortalsGrid  &portals_grid)
    {
        std::ifstream infile(fname);
        if (infile.bad() || infile.fail())
            return false;

        std::string line;
        std::string  str;
        std::stringstream ss;
        double  x_prev, y_prev;
        int16_t floor_prev;
        int64_t time_prev = 0;
        double  x, y;
        int16_t floor_number;
        int16_t mode;
        int8_t mode_of_transit;
        double altitude;
        std::string prefix;

        Fpbl::PortalData portalData = {};
        FfPosition resultsInPortal = {};

        x_prev = -1.0;
        y_prev = -1.0;
        floor_prev = -1;
        uint32_t counter = 0;

        double   cell_size = mag_grid_param.size; // grid.size parameter is actually the size of cell
        uint32_t cellXQuantity = (uint32_t)ceil(venue.size_x / cell_size);
        uint32_t cellYQuantity = (uint32_t)ceil(venue.size_y / cell_size);
        uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;
        int64_t  positionInGrid = -1;

        bool file_start = true;
        bool is_crowdsourced = false;
        double mag_x = 0.0; double mag_y = 0.0; double mag_z = 0.0;

        std::vector<FfPosition> parsed_resultsInPortal;

        while (std::getline(infile, line))
        {
            if (line == "")
                continue;

            ss << line;

            ss >> prefix;

            if (prefix.compare("cell") == 0)
            {
                if (file_start == true)
                    file_start = false;
                else
                {
                    // append current parsed data to previous cell, if it exists
                    // then clear data vector
                    // NOTE: all of the above is previously parsed data (from previous cell)
                    for (std::vector<FfPosition>::iterator data_item = parsed_resultsInPortal.begin(); data_item != parsed_resultsInPortal.end(); ++data_item)
                    {
                        portalData.resultsInPortal.push_back(*data_item);
                    }
                    parsed_resultsInPortal.clear();

                    // then put portalData in a grid
                    int32_t x_pos = (int32_t)(portalData.positionInPortal.x / venue.size_x * (venue.size_x / cell_size));
                    int32_t y_pos = (int32_t)(portalData.positionInPortal.y / venue.size_y * (venue.size_y / cell_size));
                    positionInGrid = y_pos + x_pos * cellYQuantity + portalData.positionInPortal.floor * gridCellsQuantity;
                    if (positionInGrid < 0 || positionInGrid >= portals_grid.size())
                        continue;

                    portals_grid[(std::vector<int>::size_type)positionInGrid] = portalData;
                    portalData = {};
                }

                // then parse current cell object and write it to grid (updating existing cell)
                // data vector will be added when next cell is found, or after the end of file
                ss >> x;
                ss >> y;
                ss >> floor_number;
                ss >> is_crowdsourced;
                ss >> mag_x;
                ss >> mag_y;
                ss >> mag_z;

                portalData.positionInPortal.x = x;
                portalData.positionInPortal.y = y;
                portalData.positionInPortal.floor = floor_number;
                portalData.positionInPortal.is_crowdsourced = is_crowdsourced;
                portalData.positionInPortal.mag_x = mag_x;
                portalData.positionInPortal.mag_y = mag_y;
                portalData.positionInPortal.mag_z = mag_z;
                portalData.positionInPortal.is_valid = true;

            }

            if (prefix.compare("data") == 0)
            {
                ss >> x;
                ss >> y;
                ss >> floor_number;
                ss >> mode;
                mode_of_transit = (int8_t)mode;
                ss >> altitude;

                resultsInPortal = {};
                resultsInPortal.x = x;
                resultsInPortal.y = y;
                resultsInPortal.floor_number = floor_number;
                resultsInPortal.mode_of_transit = mode_of_transit;
                resultsInPortal.altitude = altitude;
                resultsInPortal.altitude_std = 1;
                resultsInPortal.is_valid = true;

                parsed_resultsInPortal.push_back(resultsInPortal);
            }

            ss.str("");
            ss.clear();
        }

        // add last cell to grid after EOF
        for (std::vector<FfPosition>::iterator data_item = parsed_resultsInPortal.begin(); data_item != parsed_resultsInPortal.end(); ++data_item)
        {
            portalData.resultsInPortal.push_back(*data_item);
        }
        parsed_resultsInPortal.clear();

        // then put portalData in a grid
        int32_t x_pos = (int32_t)(portalData.positionInPortal.x / venue.size_x * (venue.size_x / cell_size));
        int32_t y_pos = (int32_t)(portalData.positionInPortal.y / venue.size_y * (venue.size_y / cell_size));
        positionInGrid = y_pos + x_pos * cellYQuantity + portalData.positionInPortal.floor * gridCellsQuantity;
        if ((positionInGrid < 0 || positionInGrid >= portals_grid.size()) == false)
            portals_grid[(std::vector<int>::size_type)positionInGrid] = portalData;

        portalData = {};

        return true;
    }
#else
    bool FP_builder::load_portals_grid_file(const std::string &fname, const BaseVenueType &venue, const Fpbl::Grid &mag_grid_param, Fpbl::PortalsGrid  &portals_grid)
    {
        std::ifstream infile(fname);
        if (infile.bad() || infile.fail())
            return false;

        std::string line;
        std::string  str;
        std::stringstream ss;
        double  x_prev, y_prev;
        int16_t floor_prev;
        int64_t time_prev = 0;
        double  x, y;
        int16_t floor_number;
        int16_t mode;
        int8_t mode_of_transit;
        double altitude;
        std::string prefix;

        Fpbl::PortalData portalData = {};
        FfPosition resultsInPortal = {};

        x_prev = -1.0;
        y_prev = -1.0;
        floor_prev = -1;
        uint32_t counter = 0;

        double   cell_size = mag_grid_param.size; // grid.size parameter is actually the size of cell
        uint32_t cellXQuantity = (uint32_t)ceil(venue.size_x / cell_size);
        uint32_t cellYQuantity = (uint32_t)ceil(venue.size_y / cell_size);
        uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;
        int64_t  positionInGrid = -1;

        bool file_start = true;
        bool is_crowdsourced = false;
        double mag_x = 0.0; double mag_y = 0.0; double mag_z = 0.0;

        std::vector<FfPosition> parsed_resultsInPortal;

        int16_t floor_number_prev = -1;

        while (std::getline(infile, line))
        {
            if (line == "")
                continue;

            ss << line;

            ss >> x;
            ss >> y;
            ss >> floor_number;
            ss >> mode;
            mode_of_transit = (int8_t)mode;
            ss >> altitude;

            resultsInPortal = {};
            resultsInPortal.x = x;
            resultsInPortal.y = y;
            resultsInPortal.floor_number = floor_number;
            resultsInPortal.mode_of_transit = mode_of_transit;
            resultsInPortal.altitude = altitude;
            resultsInPortal.altitude_std = 1;
            resultsInPortal.is_valid = true;

            parsed_resultsInPortal.push_back(resultsInPortal);

            if ((x != x_prev || y != y_prev || floor_number != floor_number_prev) && !file_start)
            {
                // append current parsed data to previous cell, if it exists
                // then clear data vector
                // NOTE: all of the above is previously parsed data (from previous cell)
                if (parsed_resultsInPortal.size() > 1)
                {
                    for (std::vector<FfPosition>::iterator data_item = parsed_resultsInPortal.begin(); data_item != parsed_resultsInPortal.end() - 1; ++data_item)
                    {
                        portalData.resultsInPortal.push_back(*data_item);
                    }

                    // int count = parsed_resultsInPortal.size();
                    portalData.positionInPortal.x = parsed_resultsInPortal[0].x;
                    portalData.positionInPortal.y = parsed_resultsInPortal[0].y;
                    portalData.positionInPortal.floor = parsed_resultsInPortal[0].floor_number;
                    portalData.positionInPortal.mode_of_transit = parsed_resultsInPortal[0].mode_of_transit;
                    portalData.positionInPortal.altitude = parsed_resultsInPortal[0].altitude;
                    portalData.positionInPortal.is_crowdsourced = false;
                    portalData.positionInPortal.mag_x = 0;
                    portalData.positionInPortal.mag_y = 0;
                    portalData.positionInPortal.mag_z = 0;
                    portalData.positionInPortal.is_valid = true;

                    //parsed_resultsInPortal.clear();
                    // erase all items except last
                    parsed_resultsInPortal.erase(parsed_resultsInPortal.begin(), parsed_resultsInPortal.end() - 1);

                    // then put portalData in a grid
                    int32_t x_pos = (int32_t)(portalData.positionInPortal.x / venue.size_x * (venue.size_x / cell_size));
                    int32_t y_pos = (int32_t)(portalData.positionInPortal.y / venue.size_y * (venue.size_y / cell_size));
                    positionInGrid = y_pos + x_pos * cellYQuantity + portalData.positionInPortal.floor * gridCellsQuantity;
                    if (!(positionInGrid < 0 || positionInGrid >= portals_grid.size()))
                        portals_grid[(std::vector<int>::size_type)positionInGrid] = portalData;
                }
                portalData = {};
            }

            file_start = false;
            x_prev = x;
            y_prev = y;
            floor_number_prev = floor_number;

            ss.str("");
            ss.clear();
        }

        // add last cell to grid after EOF
        if (parsed_resultsInPortal.size() > 0)
        {
            for (std::vector<FfPosition>::iterator data_item = parsed_resultsInPortal.begin(); data_item != parsed_resultsInPortal.end(); ++data_item)
            {
                portalData.resultsInPortal.push_back(*data_item);
            }

            portalData.positionInPortal.x = parsed_resultsInPortal[0].x;
            portalData.positionInPortal.y = parsed_resultsInPortal[0].y;
            portalData.positionInPortal.floor = parsed_resultsInPortal[0].floor_number;
            portalData.positionInPortal.is_crowdsourced = false;
            portalData.positionInPortal.mag_x = 0;
            portalData.positionInPortal.mag_y = 0;
            portalData.positionInPortal.mag_z = 0;
            portalData.positionInPortal.is_valid = true;
                                                          
            parsed_resultsInPortal.clear();

            // then put portalData in a grid
            int32_t x_pos = (int32_t)(portalData.positionInPortal.x / venue.size_x * (venue.size_x / cell_size));
            int32_t y_pos = (int32_t)(portalData.positionInPortal.y / venue.size_y * (venue.size_y / cell_size));
            positionInGrid = y_pos + x_pos * cellYQuantity + portalData.positionInPortal.floor * gridCellsQuantity;
            if ((positionInGrid < 0 || positionInGrid >= portals_grid.size()) == false)
                portals_grid[(std::vector<int>::size_type)positionInGrid] = portalData;
        }
        portalData = {};

        return true;
    }
#endif

	bool FP_builder::load_cs_grid_file(const std::string &fname, const BaseVenueType &venue, const Fpbl::Grid &mag_grid_param, Fpbl::CSGrid &cs_grid)
	{
		// TODO: add loading code, also check portal grid loading code (probably will need to alter portal grid saving)
		std::ifstream infile(fname);
		if (infile.bad() || infile.fail())
			return false;

		std::string line;
		std::string  str;
		std::stringstream ss;
		double  x_prev, y_prev;
		int16_t floor_prev;
		int64_t time_prev = 0;
		double  x, y;
		int16_t floor_number;
		int16_t mode;
		int8_t mode_of_transit;
		double altitude;
		std::string prefix;

		Fpbl::CSData csData;

		x_prev = -1.0;
		y_prev = -1.0;
		floor_prev = -1;
		uint32_t counter = 0;

		double   cell_size = mag_grid_param.size; // grid.size parameter is actually the size of cell
		uint32_t cellXQuantity = (uint32_t)ceil(venue.size_x / cell_size);
		uint32_t cellYQuantity = (uint32_t)ceil(venue.size_y / cell_size);
		uint64_t gridCellsQuantity = (uint64_t)cellXQuantity * cellYQuantity;
		int64_t  positionInGrid = -1;

		bool file_start = true;
		bool is_crowdsourced = false;
		double mag_x = 0.0; double mag_y = 0.0; double mag_z = 0.0;
		
		while (std::getline(infile, line))
		{
			if (line == "")
				continue;

			ss << line;

			ss >> prefix;

			if (prefix.compare("cell") == 0)
			{
				if (file_start == true)
					file_start = false;
				else
				{
					// then put portalData in a grid
					int32_t x_pos = (int32_t)(csData.position.x / venue.size_x * (venue.size_x / cell_size));
					int32_t y_pos = (int32_t)(csData.position.y / venue.size_y * (venue.size_y / cell_size));
					positionInGrid = y_pos + x_pos * cellYQuantity + csData.position.floor * gridCellsQuantity;
					if (positionInGrid < 0 || positionInGrid >= cs_grid.size())
						continue;

					cs_grid[(std::vector<int>::size_type)positionInGrid] = csData;
					csData = {};
				}

				// then parse current cell object and write it to grid (updating existing cell)
				// data vector will be added when next cell is found, or after the end of file
				ss >> x;
				ss >> y;
				ss >> floor_number;
				ss >> is_crowdsourced;
				ss >> mag_x;
				ss >> mag_y;
				ss >> mag_z;

				csData.position.x = x;
				csData.position.y = y;
				csData.position.floor = floor_number;
				csData.position.is_crowdsourced = is_crowdsourced;
				csData.position.mag_x = mag_x;
				csData.position.mag_y = mag_y;
				csData.position.mag_z = mag_z;
				csData.position.is_valid = true;
			}

			ss.str("");
			ss.clear();
		}

		// then put portalData in a grid
		int32_t x_pos = (int32_t)(csData.position.x / venue.size_x * (venue.size_x / cell_size));
		int32_t y_pos = (int32_t)(csData.position.y / venue.size_y * (venue.size_y / cell_size));
		positionInGrid = y_pos + x_pos * cellYQuantity + csData.position.floor * gridCellsQuantity;
		if ((positionInGrid < 0 || positionInGrid >= cs_grid.size()) == false)
			cs_grid[(std::vector<int>::size_type)positionInGrid] = csData;

		csData = {};

		return true;
	}

    // save grids
//#define MAG_PRECISION_OUTPUT
    Fpbl::ReturnStatus FP_builder::saveMagneticGrid(const std::string &fname, Fpbl::MagneticGrid  &magneticGrid)
    {
        Fpbl::ReturnStatus result = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;

        std::ofstream f_grid(fname.c_str(), std::ios::out);

        if (f_grid.is_open())
        {
            uint32_t i, j;
#ifdef MAG_PRECISION_OUTPUT
            int width = 15;
            int precision = 8;
#else
            int width = 8;
            int precision = 3;
#endif

            for (i = 0; i < magneticGrid.size(); i++)
            {
                if (magneticGrid[i].magData.size() > 0)
                {
                    for (j = 0; j < magneticGrid[i].magData.size(); j++)
                    {
                        {
                            f_grid << std::fixed << std::setw(8) << std::left << std::setprecision(2) /*<< std::defaultfloat*/ << magneticGrid[i].coordinates.x << "\t";
                            f_grid << std::fixed << std::setw(8) << std::left << std::setprecision(2) << magneticGrid[i].coordinates.y << "\t";
                            f_grid << magneticGrid[i].coordinates.floor << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].mX << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].mY << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].mZ << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[0][0] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[0][1] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[0][2] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[1][0] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[1][1] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[1][2] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[2][0] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[2][1] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].covarianceMatrix[2][2] << "\t";
                            f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].dataset_type << "\t";
                            //f_grid << std::fixed << std::setw(width) << std::right << std::setprecision(precision) << magneticGrid[i].magData[j].timestamp << "\t";
                            f_grid << ENDL;
                        }
                    }
                }
            }
            f_grid.close();
            result = Fpbl::ReturnStatus::STATUS_SUCCESS;
        }

        return result;
    }

    bool save_mfp3( std::ofstream &fs, const Fpbl::MfpBuilder::LocalDB &dbmfp)
    {
        for (auto it = dbmfp.cbegin(); (it != dbmfp.cend() && fs.good()); ++it)
        {
            Fpbl::MfpBuilder::DBRecord rec = (*it);
            fs.write(reinterpret_cast<char*>(&rec), sizeof(rec));
        }

        bool result = fs.good();
        return result;
    }

    bool  save_mfp4(std::ofstream &fs, const Fpbl::MfpBuilder::LocalDB &dbmfp, const MFPVenue &mfpVenue)
    {
        Mfp4Header mfp4_header;

        mfp4_header.initialize();
        mfp4_header.setDataSize (dbmfp.size_in_bytes());                     /// FP size in bytes

		mfp4_header.setBuiderVersion(VersionNumber(Fpbl::getVersionNumber()));
        mfp4_header.setFpBuidNumber(1); // !!! DEBUG VALUE
        std::time_t t = std::time(nullptr);
        mfp4_header.setFpBuildTime(*std::gmtime(&t));
        mfp4_header.setVenue(mfpVenue);
        // to do: calc crc

        // write header
        mfp4_header.write(fs);
        bool result = fs.good();

        // write data
        result &= save_mfp3(fs, dbmfp);

        return result;
    }

    Fpbl::ReturnStatus FP_builder::saveMagneticDB(const std::string &fname, Fpbl::MfpBuilder::LocalDB dbmfp, int mag_fp_format)
    {
        Fpbl::ReturnStatus result = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
        std::ofstream fs(fname.c_str(), std::ios::out | std::ios::binary);

        if (fs.is_open())
        {

            if (mag_fp_format == 3) // TODO: add mag_fp_format
            {
                result = save_mfp3(fs, dbmfp) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
            }
            else if (mag_fp_format == 4)
            {
                MFPVenue mfp_venue = venue;
                result = save_mfp4(fs, dbmfp, mfp_venue) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
            }

            fs.close();
        }
        return result;
    }

    void FP_builder::saveWiFiGrid(const std::string &fname, Fpbl::WiFiGrid  &wifiGrid)
    {
        std::ofstream f_grid(fname.c_str(), std::ios::out);
        uint32_t i, j, k;

        for (i = 0; i < wifiGrid.size(); i++)
        {
            if (wifiGrid[i].wifiData.size() > 0)
            {
                for (j = 0; j < wifiGrid[i].wifiData.size(); j++)
                {
                    for (k = 0; k < wifiGrid[i].wifiData[j].scanWiFi.size(); k++)
                    {
                        f_grid << wifiGrid[i].coordinates.x << "\t";
                        f_grid << wifiGrid[i].coordinates.y << "\t";
                        f_grid << wifiGrid[i].coordinates.floor << "\t";

                        std::stringstream ss;
                        std::string mac_s;

                        ss << std::hex << /*std::uppercase <<*/
                            std::fixed << std::setw(2) << std::setfill('0') <<
                            ((wifiGrid[i].wifiData[j].scanWiFi[k].mac >> 48) & 0xFF) << ":" <<
                            std::fixed << std::setw(2) << std::setfill('0') <<
                            ((wifiGrid[i].wifiData[j].scanWiFi[k].mac >> 40) & 0xFF) << ":" <<
                            std::fixed << std::setw(2) << std::setfill('0') <<
                            ((wifiGrid[i].wifiData[j].scanWiFi[k].mac >> 32) & 0xFF) << ":" <<
                            std::fixed << std::setw(2) << std::setfill('0') <<
                            ((wifiGrid[i].wifiData[j].scanWiFi[k].mac >> 24) & 0xFF) << ":" <<
                            std::fixed << std::setw(2) << std::setfill('0') <<
                            ((wifiGrid[i].wifiData[j].scanWiFi[k].mac >> 16) & 0xFF) << ":" <<
                            std::fixed << std::setw(2) << std::setfill('0') <<
                            ((wifiGrid[i].wifiData[j].scanWiFi[k].mac >> 8) & 0xFF) << ":" <<
                            std::fixed << std::setw(2) << std::setfill('0') <<
                            ((wifiGrid[i].wifiData[j].scanWiFi[k].mac >> 0) & 0xFF);
                        ss >> mac_s;
                        f_grid << mac_s << "\t";
                        //
                        f_grid << (int16_t)wifiGrid[i].wifiData[j].scanWiFi[k].rssi << "\t";
                        f_grid << wifiGrid[i].wifiData[j].scanWiFi[k].frequency;

                        f_grid << "\t";
                        f_grid << wifiGrid[i].wifiData[j].scanWiFi[k].timestamp; // time of measurement, each scanWiFi[k] has "WiFiMeasurement" type

                        f_grid << "\t";
                        f_grid << wifiGrid[i].wifiData[j].timestamp; // time of scan, each wifiData[j] is struct: "time + WiFi_meas_vector"

                        f_grid << "\t";
                        f_grid << wifiGrid[i].wifiData[j].dataset_id;

                        f_grid << ENDL;
                    }
                }
            }
        }

        f_grid.close();
    }

    void FP_builder::saveWiFiGridForMatlab(const std::string &fname, Fpbl::WiFiGrid  &wifiGrid)
    {

        std::ofstream f_grid(fname.c_str(), std::ios::out);

        uint32_t i, j, k;
        //int width = 15;
        std::string delimiter = "\t";
        std::vector<uint64_t> mac_addresses;
        std::vector<WiFiMeasurement> wifiMeasurements;
        std::vector<int16_t> rssies;


        for (uint32_t i = 0; i < wifiGrid.size(); i++)
        {
            if (wifiGrid[i].wifiData.size() > 0)
            {
                for (uint32_t j = 0; j < wifiGrid[i].wifiData.size(); j++)
                {
                    for (uint32_t k = 0; k < wifiGrid[i].wifiData[j].scanWiFi.size(); k++)
                    {
                        uint64_t mac_address = wifiGrid[i].wifiData[j].scanWiFi[k].mac;
                        if (std::find(mac_addresses.begin(), mac_addresses.end(), mac_address) == mac_addresses.end())
                        {
                            mac_addresses.push_back(mac_address);
                        }
                    }
                }
            }
        }

        std::sort(mac_addresses.begin(), mac_addresses.end());
        for (auto mac_it = mac_addresses.begin(); mac_it != mac_addresses.end(); ++mac_it)
            f_grid << *mac_it << delimiter;
        f_grid << ENDL;

        int point_number = 0;
        for (i = 0; i < wifiGrid.size(); i++)
        {
            if (wifiGrid[i].wifiData.size() > 0)
            {
                wifiMeasurements.clear();

                for (j = 0; j < wifiGrid[i].wifiData.size(); j++)
                {

                    for (k = 0; k < wifiGrid[i].wifiData[j].scanWiFi.size(); k++)
                    {
                        wifiMeasurements.push_back(wifiGrid[i].wifiData[j].scanWiFi[k]);
                    }

                    auto wifi_it = wifiMeasurements.begin();
                    for (auto mac_it = mac_addresses.begin(); mac_it != mac_addresses.end(); ++mac_it)
                    {
                        for (wifi_it = wifiMeasurements.begin(); wifi_it != wifiMeasurements.end(); ++wifi_it)
                        {
                            if (*mac_it == wifi_it->mac)
                            {
                                f_grid << (int16_t)(wifi_it->rssi) << delimiter;
                                break;
                            }

                        }
                        if (wifi_it == wifiMeasurements.end())
                        {
                            f_grid << "-100" << delimiter;
                        }
                    }

                    f_grid << point_number << delimiter;
                    f_grid << wifiGrid[i].coordinates.x << delimiter;
                    f_grid << wifiGrid[i].coordinates.y;
                    f_grid << ENDL;

                }
                point_number++;
            }
        }

        f_grid.close();
    }

    bool save_wfp3(std::ofstream &f_db, const Fpbl::WiFiBuilder::LocalDB db)
    {
        f_db.precision(6);
        int width = 15;

        Fpbl::WiFiBuilder::GMixture hist;

        f_db << ENDL;
        f_db << std::right;

        for (Fpbl::WiFiBuilder::LocalDB::const_iterator db_it = db.begin(); db_it != db.end(); ++db_it)
        {
            //DB loop
            const Fpbl::CoordinatesInGrid  loc = (*db_it).location;
            f_db << std::fixed;
            f_db.precision(3);
            f_db << "POINT " << loc.x << " " << loc.y << " " << (double)loc.floor << ENDL;

            //const std::map<Fpbl::BSSID, Fpbl::WiFiBuilder::GMixture>  fp = (*db_it).fingerprintGM;
            const  Fpbl::WiFiBuilder::APListGM  fp = (*db_it).fingerprintGM;

            for (Fpbl::WiFiBuilder::APListGM::const_iterator ap_it = fp.begin(); ap_it != fp.end(); ++ap_it)
            {
                // AP loop
                const BSSID bssid = (*ap_it).first;
                f_db << "AP_MAC ";
                f_db.width(width);
                f_db.right;
                f_db << bssid;
                double tmp;
                hist = (*ap_it).second;
                if (hist.w1 >= hist.w2)
                {
                    f_db << std::fixed;

                    f_db.width(width);
                    f_db.precision(4);
                    //f_db << hist.w1;
                    tmp = floor(hist.w1 * 10000 + 0.5) / 10000;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.mu1;
                    tmp = floor(hist.mu1 * 100 + 0.5) / 100;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.sig1;
                    tmp = floor(hist.sig1 * 100 + 0.5) / 100;
                    f_db << tmp;

                    f_db.width(width);
                    f_db.precision(4);
                    //f_db << hist.w2;
                    tmp = floor(hist.w2 * 10000 + 0.5) / 10000;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.mu2;
                    tmp = floor(hist.mu2 * 100 + 0.5) / 100;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.sig2;
                    tmp = floor(hist.sig2 * 100 + 0.5) / 100;
                    f_db << tmp;

					f_db.width(width);
					f_db.precision(2);
					tmp = hist.scan_count;
					f_db << tmp;

                    f_db << ENDL;
                }
                else
                {
                    f_db << std::fixed;

                    f_db.width(width);
                    f_db.precision(4);
                    //f_db << hist.w2;
                    tmp = floor(hist.w2 * 10000 + 0.5) / 10000;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.mu2;
                    tmp = floor(hist.mu2 * 100 + 0.5) / 100;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.sig2;
                    tmp = floor(hist.sig2 * 100 + 0.5) / 100;
                    f_db << tmp;

                    f_db.width(width);
                    f_db.precision(4);
                    //f_db << hist.w1;
                    tmp = floor(hist.w1 * 10000 + 0.5) / 10000;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.mu1;
                    tmp = floor(hist.mu1 * 100 + 0.5) / 100;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.sig1;
                    tmp = floor(hist.sig1 * 100 + 0.5) / 100;
                    f_db << tmp;

					f_db.width(width);
					f_db.precision(2);
					tmp = hist.scan_count;
					f_db << tmp;

                    f_db << ENDL;
                }
            }
        }
        bool result = f_db.good();
        return result;
    }

    static bool  save_wfp4_header(std::ofstream &fs, const Fpbl::WiFiBuilder::LocalDB dbwfp, const WFPVenue &wfpVenue)
    {
        Wfp4Header wfp4_header;

        wfp4_header.initialize();
        wfp4_header.setDataSize(-1); /// It is not used now

		wfp4_header.setBuiderVersion(VersionNumber(Fpbl::getVersionNumber()));
        wfp4_header.setFpBuidNumber(1); // !!! DEBUG VALUE
        std::time_t t = std::time(nullptr);
        wfp4_header.setFpBuildTime(*std::gmtime(&t));

        wfp4_header.setVenue(wfpVenue);

        // write header
        wfp4_header.write(fs);
        bool result = fs.good();

        return result;
    }

    Fpbl::ReturnStatus FP_builder::saveWiFiDb(const std::string &fname, Fpbl::WiFiBuilder::LocalDB dbwfp, int wifi_fp_format)
    {
        Fpbl::ReturnStatus result = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
        std::ofstream fs(fname.c_str(), std::ios::out);

        if (fs.is_open())
        {

            if (wifi_fp_format == 3) // TODO: add mag_fp_format
            {
                fs << "WIFI_DB VER 3" << ENDL;
                fs << std::right;
                result = save_wfp3(fs, dbwfp) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
            }
            else if (wifi_fp_format == 4)
            {
                WFPVenue wfp_venue = venue;
                fs.close();
                fs.open(fname.c_str(), std::ios::out | std::ios::binary);
                if (fs.is_open())
                {
                    result = save_wfp4_header(fs, dbwfp, wfp_venue) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
                }
                if ((result == Fpbl::ReturnStatus::STATUS_SUCCESS) && (fs.is_open()))
                {
                    fs.close();
                    fs.open(fname.c_str(), std::ios::out | std::ios::app);
                    result = save_wfp3(fs, dbwfp) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
                }
            }

            fs.close();
        }
        return result;
    }

    void FP_builder::saveBleGrid(const std::string &fname, Fpbl::BleGrid  &bleGrid)
    {
        std::ofstream f_grid(fname.c_str(), std::ios::out);
        uint32_t i, j, k;

        for (i = 0; i < bleGrid.size(); i++)
        {
            if (bleGrid[i].bleData.size() > 0)
            {
                for (j = 0; j < bleGrid[i].bleData.size(); j++)
                {
                    for (k = 0; k < bleGrid[i].bleData[j].scanBle.size(); k++)
                    {
                        f_grid << bleGrid[i].coordinates.x << "\t";
                        f_grid << bleGrid[i].coordinates.y << "\t";
                        f_grid << bleGrid[i].coordinates.floor << "\t";

                        f_grid << bleGrid[i].bleData[j].scanBle[k].mac << "\t";

                        f_grid << (int16_t)bleGrid[i].bleData[j].scanBle[k].rssi << "\t";
                        f_grid << bleGrid[i].bleData[j].scanBle[k].frequency;

                        f_grid<< "\t";
                        f_grid << bleGrid[i].bleData[j].scanBle[k].timestamp; // time of measurement, each scanBLe[k] has "BleMeasurement" type

                        f_grid << "\t";
                        f_grid << bleGrid[i].bleData[j].dataset_id;

                        f_grid << ENDL;
                    }
                }
            }
        }

        f_grid.close();
    }

    void FP_builder::savePortalsGrid(const std::string &fname, Fpbl::PortalsGrid  &portalsGrid)
    {
        std::ofstream f_grid(fname.c_str(), std::ios::out);

        uint32_t i, j;

        for (i = 0; i < portalsGrid.size(); i++)
        {
            if (portalsGrid[i].positionInPortal.is_crowdsourced)
            {
                f_grid << "crowdsourced cell: ";
                f_grid << portalsGrid[i].positionInPortal.x << " " << portalsGrid[i].positionInPortal.y << " " << portalsGrid[i].positionInPortal.floor << " ";
                f_grid << portalsGrid[i].positionInPortal.is_crowdsourced << " ";
                f_grid << portalsGrid[i].positionInPortal.mag_x << " " << portalsGrid[i].positionInPortal.mag_y << " " << portalsGrid[i].positionInPortal.mag_z << ENDL;
            }
            if (portalsGrid[i].resultsInPortal.size() > 0)
            {
                for (j = 0; j < portalsGrid[i].resultsInPortal.size(); j++)
                {
                    if (portalsGrid[i].resultsInPortal[j].altitude_std > 0)
                    {
                        //f_grid << portalsGrid[i].positionInPortal.x << " ";
                        //f_grid << portalsGrid[i].positionInPortal.y << " ";
                        //f_grid << portalsGrid[i].positionInPortal.floor << " -- ";
                        //f_grid << "data ";
                        f_grid << portalsGrid[i].resultsInPortal[j].x << " ";
                        f_grid << portalsGrid[i].resultsInPortal[j].y << " ";
                        f_grid << portalsGrid[i].resultsInPortal[j].floor_number << " ";
                        f_grid << (int)portalsGrid[i].resultsInPortal[j].mode_of_transit << " ";
                        f_grid << portalsGrid[i].resultsInPortal[j].altitude << " ";
                        f_grid << ENDL;
                    }
                }
            }
        }

        f_grid.close();
    }

	void FP_builder::saveCSGrid(const std::string &fname, Fpbl::CSGrid  &cs_grid)
	{

	}

    bool  save_bfp3(std::ofstream &f_db, const Fpbl::BleBuilder::LocalDB db)
    {
        f_db.precision(6);
        int width = 15;

        Fpbl::BleBuilder::GMixture hist;

        f_db << ENDL;
        f_db << std::right;
        for (Fpbl::BleBuilder::LocalDB::const_iterator db_it = db.begin(); db_it != db.end(); ++db_it)
        {
            //DB loop
            const Fpbl::CoordinatesInGrid  loc = (*db_it).location;
            f_db << std::fixed;
            f_db.precision(2);
            f_db << "POINT " << loc.x << " " << loc.y << " " << (double)loc.floor << ENDL;

            //const std::map<Fpbl::BSSID, Fpbl::WiFiBuilder::GMixture>  fp = (*db_it).fingerprintGM;
            const  Fpbl::BleBuilder::APListGM  fp = (*db_it).fingerprintGM;

            for (Fpbl::BleBuilder::APListGM::const_iterator ap_it = fp.begin(); ap_it != fp.end(); ++ap_it)
            {
                // AP loop
                const BSSID bssid = (*ap_it).first;
                f_db << "AP_MAC ";
                f_db.width(width);
                f_db.right;
                //f_db << std::hex;
                f_db << bssid;
                //f_db << std::dec;
                double tmp;

                hist = (*ap_it).second;
                if (hist.w1 >= hist.w2)
                {
                    f_db << std::fixed;

                    f_db.width(width);
                    f_db.precision(4);
                    //f_db << hist.w1;
                    tmp = floor(hist.w1 * 10000 + 0.5) / 10000;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.mu1;
                    tmp = floor(hist.mu1 * 100 + 0.5) / 100;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.sig1;
                    tmp = floor(hist.sig1 * 100 + 0.5) / 100;
                    f_db << tmp;

                    f_db.width(width);
                    f_db.precision(4);
                    //f_db << hist.w2;
                    tmp = floor(hist.w2 * 10000 + 0.5) / 10000;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.mu2;
                    tmp = floor(hist.mu2 * 100 + 0.5) / 100;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.sig2;
                    tmp = floor(hist.sig2 * 100 + 0.5) / 100;
                    f_db << tmp;

					f_db.width(width);
					f_db.precision(2);
					tmp = hist.scan_count;
					f_db << tmp;

                    f_db << ENDL;
                }
                else
                {
                    f_db << std::fixed;

                    f_db.width(width);
                    f_db.precision(4);
                    //f_db << hist.w2;
                    tmp = floor(hist.w2 * 10000 + 0.5) / 10000;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.mu2;
                    tmp = floor(hist.mu2 * 100 + 0.5) / 100;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.sig2;
                    tmp = floor(hist.sig2 * 100 + 0.5) / 100;
                    f_db << tmp;

                    f_db.width(width);
                    f_db.precision(4);
                    //f_db << hist.w1;
                    tmp = floor(hist.w1 * 10000 + 0.5) / 10000;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.mu1;
                    tmp = floor(hist.mu1 * 100 + 0.5) / 100;
                    f_db << tmp;
                    f_db.width(width);
                    f_db.precision(2);
                    //f_db << hist.sig1;
                    tmp = floor(hist.sig1 * 100 + 0.5) / 100;
                    f_db << tmp;

					f_db.width(width);
					f_db.precision(2);
					tmp = hist.scan_count;
					f_db << tmp;

                    f_db << ENDL;
                }

            }
        }
        bool result = f_db.good();
        return result;
    }

    bool  save_bfp4_header(std::ofstream &fs, const Fpbl::BleBuilder::LocalDB dbbfp, const BFPVenue &bfpVenue)
    {
        Bfp4Header bfp4_header;

        bfp4_header.initialize();
        bfp4_header.setDataSize(-1); /// It is not used now

		bfp4_header.setBuiderVersion(VersionNumber(Fpbl::getVersionNumber()));
        bfp4_header.setFpBuidNumber(1); // !!! DEBUG VALUE
        std::time_t t = std::time(nullptr);
        bfp4_header.setFpBuildTime(*std::gmtime(&t));

        bfp4_header.setVenue(bfpVenue);

        // write header
        bfp4_header.write(fs);

        bool result = fs.good();

        return result;
    }

    Fpbl::ReturnStatus   FP_builder::saveBleDb(const std::string &fname, Fpbl::BleBuilder::LocalDB dbbfp, int ble_fp_format)
    {
        Fpbl::ReturnStatus result = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
        std::ofstream fs(fname.c_str(), std::ios::out);
        if (fs.is_open())
        {

            if (ble_fp_format == 3) // TODO: add mag_fp_format
            {
                //fs << "BLE_DB VER 3" << ENDL;
                fs << "WIFI_DB VER 3" << ENDL;
                fs << std::right;
                result = save_bfp3(fs, dbbfp) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
            }
            else if (ble_fp_format == 4)
            {
                BFPVenue bfp_venue = venue;
                fs.close();
                fs.open(fname.c_str(), std::ios::out | std::ios::binary);
                if (fs.is_open())
                {
                    result = save_bfp4_header(fs, dbbfp, bfp_venue) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
                }
                if ((result == Fpbl::ReturnStatus::STATUS_SUCCESS) && (fs.is_open()))
                {
                    fs.close();
                    fs.open(fname.c_str(), std::ios::out | std::ios::app);
                    result = save_bfp3(fs, dbbfp) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
                }
            }

            fs.close();
        }
        return result;
    }

    bool  save_bprox3(std::ofstream &f_db, void* dbbprox)
    {
        std::vector<std::string> *db = static_cast< std::vector<std::string> * >(dbbprox);
        for (auto it = db->begin(); it != db->end(); ++it)
        {
            f_db << *it << ENDL;
        }
        return true;
    }

    bool  save_bprox4_header(std::ofstream &fs, void* dbbprox, BaseVenue venue)
    {
        BProxVenue bproxVenue = venue;
        Blp4Header blp4_header;

        blp4_header.initialize();
        blp4_header.setDataSize(-1); /// It is not used now

		blp4_header.setBuiderVersion(VersionNumber(Fpbl::getVersionNumber()));
        blp4_header.setFpBuidNumber(1); // !!! DEBUG VALUE

        std::time_t t = std::time(nullptr);
        blp4_header.setFpBuildTime(*std::gmtime(&t));

        blp4_header.setVenue(bproxVenue);

        // write header
        blp4_header.write(fs);
        bool result = fs.good();


        return result;
    }

    Fpbl::ReturnStatus   FP_builder::saveBleProximityDb(const std::string &fname, void* dbbprox, int ble_fp_format)
    {
        Fpbl::ReturnStatus result = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
        std::ofstream fs(fname.c_str(), std::ios::out);

        if (fs.is_open())
        {

            if (ble_fp_format == 3) // TODO: add mag_fp_format
            {
                fs << "BLP_DB VER 3" << ENDL;
                fs << std::right;
                result = save_bprox3(fs, dbbprox) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
            }
            else if (ble_fp_format == 4)
            {
                BProxVenue blp_venue = static_cast <BaseVenue>(venue);
                fs.close();
                fs.open(fname.c_str(), std::ios::out | std::ios::binary);
                if (fs.is_open())
                {
                    result = save_bprox4_header(fs, dbbprox, blp_venue) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
                }
                if ((result == Fpbl::ReturnStatus::STATUS_SUCCESS) && (fs.is_open()))
                {
                    fs.close();
                    fs.open(fname.c_str(), std::ios::out | std::ios::app);
                    result = save_bprox3(fs, dbbprox) ? Fpbl::ReturnStatus::STATUS_SUCCESS : Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
                }
            }

            fs.close();
        }
        if (fs.good())
            result = Fpbl::ReturnStatus::STATUS_SUCCESS;
        return result;
    }

#if 0
    bool FP_builder::parse_pos_att_file(std::string file_path, Fpbl::GridBuilder &Builder)
    {
        std::ifstream infile(file_path);
        std::string line;
        const char delim = ',';

        while (std::getline(infile, line))
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;

            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }

            uint64_t timestamp;
            Fpbl::Position position;
            Fpbl::Attitude attitude;

            std::stringstream ss;
            ss << result[0];
            ss >> timestamp;
            ss.str("");
            ss.clear();
            ss << result[1];
            ss >> position.isValid;
            ss.str("");
            ss.clear();
            ss << result[2];
            ss >> position.x;
            ss.str("");
            ss.clear();
            ss << result[3];
            ss >> position.y;
            ss.str("");
            ss.clear();
            ss << result[4];
            ss >> position.altitude;
            ss.str("");
            ss.clear();
            ss << result[5];
            ss >> position.floorNumber;
            ss.str("");
            ss.clear();
            ss << result[6];
            ss >> position.covarianceXY[0][0];
            ss.str("");
            ss.clear();
            ss << result[7];
            ss >> position.covarianceXY[0][1];
            ss.str("");
            ss.clear();
            ss << result[8];
            ss >> position.covarianceXY[1][0];
            ss.str("");
            ss.clear();
            ss << result[9];
            ss >> position.covarianceXY[1][1];
            ss.str("");
            ss.clear();
            ss << result[10];
            ss >> position.altitudeStd;
            ss.str("");
            ss.clear();
            ss << result[11];
            ss >> position.floorStd;
            ss.str("");
            ss.clear();
            // position parsing done
            ss << result[12];
            ss >> attitude.isValid;
            ss.str("");
            ss.clear();
            ss << result[13];
            ss >> attitude.quaternion[0];
            ss.str("");
            ss.clear();
            ss << result[14];
            ss >> attitude.quaternion[1];
            ss.str("");
            ss.clear();
            ss << result[15];
            ss >> attitude.quaternion[2];
            ss.str("");
            ss.clear();
            ss << result[16];
            ss >> attitude.quaternion[3];
            ss.str("");
            ss.clear();
            ss << result[17];
            ss >> attitude.covarianceQuaternion[0][0];
            ss.str("");
            ss.clear();
            ss << result[18];
            ss >> attitude.covarianceQuaternion[0][1];
            ss.str("");
            ss.clear();
            ss << result[19];
            ss >> attitude.covarianceQuaternion[0][2];
            ss.str("");
            ss.clear();
            ss << result[20];
            ss >> attitude.covarianceQuaternion[0][3];
            ss.str("");
            ss.clear();
            ss << result[21];
            ss >> attitude.covarianceQuaternion[1][0];
            ss.str("");
            ss.clear();
            ss << result[22];
            ss >> attitude.covarianceQuaternion[1][1];
            ss.str("");
            ss.clear();
            ss << result[23];
            ss >> attitude.covarianceQuaternion[1][2];
            ss.str("");
            ss.clear();
            ss << result[24];
            ss >> attitude.covarianceQuaternion[1][3];
            ss.str("");
            ss.clear();
            ss << result[25];
            ss >> attitude.covarianceQuaternion[2][0];
            ss.str("");
            ss.clear();
            ss << result[26];
            ss >> attitude.covarianceQuaternion[2][1];
            ss.str("");
            ss.clear();
            ss << result[27];
            ss >> attitude.covarianceQuaternion[2][2];
            ss.str("");
            ss.clear();
            ss << result[28];
            ss >> attitude.covarianceQuaternion[2][3];
            ss.str("");
            ss.clear();
            ss << result[29];
            ss >> attitude.covarianceQuaternion[3][0];
            ss.str("");
            ss.clear();
            ss << result[30];
            ss >> attitude.covarianceQuaternion[3][1];
            ss.str("");
            ss.clear();
            ss << result[31];
            ss >> attitude.covarianceQuaternion[3][2];
            ss.str("");
            ss.clear();
            ss << result[32];
            ss >> attitude.covarianceQuaternion[3][3];
            ss.str("");
            ss.clear();
            // attitude parsing done

            Fpbl::ReturnStatus status1 = Builder.processDevicePosition(timestamp, position);
            Fpbl::ReturnStatus status2 = Builder.processDeviceAttitude(timestamp, attitude);
        }
        return true;
    }
#endif

    bool FP_builder::parse_mg_file(std::string file_path, Fpbl::GridBuilder &Builder)
    {
        std::ifstream infile(file_path);
        std::string line;
        const char delim = ',';

        while (std::getline(infile, line))
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;

            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }

            uint64_t timestamp;
            MagneticVector mag_vector;

            std::stringstream ss;
            ss << result[0];
            ss >> timestamp;
            ss.str("");
            ss.clear();
            // NOTE: value number [1] is skipped, unused
            ss << result[2];
            ss >> mag_vector.mX;
            ss.str("");
            ss.clear();
            ss << result[3];
            ss >> mag_vector.mY;
            ss.str("");
            ss.clear();
            ss << result[4];
            ss >> mag_vector.mZ;
            ss.str("");
            ss.clear();
            // magnetic data parsing done

            Fpbl::ReturnStatus status = Builder.processMFPMeasurement(timestamp, mag_vector);
        }
        return true;
    }

    bool FP_builder::parse_wifi_file(std::string file_path, std::vector<WiFiScanResult > & wifiScanResults)
    {
        std::ifstream infile(file_path);
        std::string line;
        const char delim = ',';

        uint64_t timestamp;
        int16_t  rssi;

        uint64_t wifiMeasurementNumberPrev = 0;
        WiFiMeasurement wifiMeasurement;
        WiFiScanResult wifiScan;
        std::string macStr;

        uint64_t wifiMeasurementNumber = 0;

        while (std::getline(infile, line))
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;

            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }

            std::stringstream ss;
            ss << result[0];
            ss >> timestamp;
            wifiScan.timestamp = timestamp;
            ss.str("");
            ss.clear();

            // NOTE: value number [1] is skipped, unused
            ss << result[1];
            ss >> wifiMeasurementNumber;
            ss.str("");
            ss.clear();

            ss << result[2];
            ss >> macStr;
            wifiMeasurement.mac = string_to_mac(macStr);
            ss.str("");
            ss.clear();

            ss << result[4];
            ss >> wifiMeasurement.frequency;
            ss.str("");
            ss.clear();

            ss << result[6];
            ss >> rssi;
            wifiMeasurement.rssi = (int8_t)rssi;
            ss.str("");
            ss.clear();

            if (result.size() >= 8)
            {
                ss << result[7];
                ss >> wifiMeasurement.timestamp;
                ss.str("");
                ss.clear();
            }
            else
                wifiMeasurement.timestamp = wifiScan.timestamp;
            // wifi data parsing done
            if (wifiMeasurementNumber != wifiMeasurementNumberPrev)
            {
                if (wifiScan.scanWiFi.size() > 0)
                    wifiScanResults.push_back(wifiScan);

                wifiScan.scanWiFi.clear();
                wifiScan.timestamp = timestamp;

                wifiMeasurementNumberPrev = wifiMeasurementNumber;
            }
            wifiScan.scanWiFi.push_back(wifiMeasurement);
        }
        if (wifiScan.scanWiFi.size() > 0)
            wifiScanResults.push_back(wifiScan);
        return true;
    }

    bool FP_builder::parse_ble_file(std::string file_path, std::vector<BleScanResult>  & bleScanResults)
    {
        std::ifstream infile(file_path);
        std::string line;
        const char delim = ',';

        uint64_t timestamp, scan_time;
        int16_t  rssi;
        uint16_t  major, minor;
        int16_t txPower;

        uint64_t bleMeasurementNumberPrev = 0;
        BleMeasurement bleMeasurement;
        BleScanResult bleScan;
        std::string macStr;

        uint64_t bleMeasurementNumber = 0;

        while (std::getline(infile, line))
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;


            //std::getline(buf, token, delim); // ATENTION: skip timestamp of BLE measurement
            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }

            //if (!((result.size() == (12-1)) || (result.size() == (13-1))))
            //{
            //    continue;
            //}

            if (!((result.size() == 12) || (result.size() == 13)))
                continue;
            if (result.size() == 13)
                result.erase(result.begin());


            std::stringstream ss;
            ss << result[0];
            ss >> timestamp;
            bleMeasurement.timestamp = timestamp;
            ss.str("");
            ss.clear();

             scan_time = timestamp;

            int field_shift;
            if (result.size() == (13 - 1))
            {// data from Mapper v1.6.10 and early
                ss << result[1];
                ss >> bleMeasurementNumber;
                ss.str("");
                ss.clear();

                field_shift = 0;
            }
            else
            {// data from Mapper v1.6.12 and late; it does not contain "measurement number" field
                bleMeasurementNumber = static_cast<int64_t>((timestamp + 0.5* ble_scan_period) / ble_scan_period) * ble_scan_period;
                field_shift = 1;
            }

            ss << result[2 - field_shift];
            ss >> macStr;
            bleMeasurement.hasMAC = false;

            if (/* &macStr == NULL || */ macStr == "")
            {
                bleMeasurement.mac = 0;
            }
            else
            {
                bleMeasurement.mac = string_to_mac(macStr);

                if (bleMeasurement.mac != 0)
                    bleMeasurement.hasMAC = true;
            }

            ss.str("");
            ss.clear();

            ss << result[4 - field_shift];
            ss >> bleMeasurement.frequency;
            ss.str("");
            ss.clear();

            ss << result[6 - field_shift];
            ss >> rssi;
            bleMeasurement.rssi = (int8_t)rssi;
            ss.str("");
            ss.clear();

            ss << result[8 - field_shift];
            ss >> major;
            bleMeasurement.major = major;
            ss.str("");
            ss.clear();

            ss << result[9 - field_shift];
            ss >> minor;
            bleMeasurement.minor = minor;
            ss.str("");
            ss.clear();

            //bleMeasurement.mac = ((uint64_t)major << 16) + minor;

            ss << result[10 - field_shift];
            ss >> txPower;
            bleMeasurement.txPower = (int8_t)txPower;
            ss.str("");
            ss.clear();
            // wifi data parsing done

            if (bleMeasurementNumber != bleMeasurementNumberPrev)
            {
                if (bleScan.scanBle.size() > 0)
                {
                    bleScan.timestamp = static_cast<int64_t>((bleScan.scanBle.back().timestamp + 0.5* ble_scan_period) / ble_scan_period) * ble_scan_period;
                    //Fpbl::ReturnStatus status = Builder.processBleMeasurement(bleScan.timestamp, bleScan);
                    bleScanResults.push_back(bleScan);
                }

                bleScan.scanBle.clear();
                //bleScan.timestamp = scan_time;

                bleMeasurementNumberPrev = bleMeasurementNumber;
            }

            bool process_ble_message = true; // (bleMeasurement.major != 0) && (bleMeasurement.minor != 0);
            //process_ble_message &= (bleMeasurement.rssi > -100);
            if (use_ble_exception && process_ble_message)
            {
                for (int i = 0; i < ble_exception_count; i++)
                {
                    process_ble_message &= !((bleMeasurement.major == ble_exception_list[i][0]) && (bleMeasurement.minor == ble_exception_list[i][1]));
                }
            }

            if (process_ble_message)
            {
                bleScan.scanBle.push_back(bleMeasurement);
            }
            else
            {
                //std::cout << ENDL;
                //std::cout << "reject ble exception:" << bleMeasurement.major << ", " << bleMeasurement.minor << "," << (int)bleMeasurement.rssi;
            }
        }

        if (bleScan.scanBle.size() > 0)
        {
            bleScan.timestamp = static_cast<int64_t>((bleScan.scanBle.back().timestamp + 0.5* ble_scan_period) / ble_scan_period) * ble_scan_period;
            //Fpbl::ReturnStatus status = Builder.processBleMeasurement(bleScan.timestamp, bleScan);
            bleScanResults.push_back(bleScan);
        }

        return true;
    }

    bool FP_builder::parse_mbias_file(std::string file_path, TrackPreProcessing::MagBias &mag_data)
    {
        std::ifstream infile(file_path);
        std::string line;
        std::vector<std::string> result;

        while (std::getline(infile, line))
        {
            result.push_back(line);
        }

        std::stringstream ss;
        ss << result[1];
        ss >> mag_data.bias[0];
        ss.str("");
        ss.clear();

        ss << result[2];
        ss >> mag_data.bias[1];
        ss.str("");
        ss.clear();

        ss << result[3];
        ss >> mag_data.bias[2];
        ss.str("");
        ss.clear();

        double std_mbias_X = 0;
        double std_mbias_Y = 0;
        double std_mbias_Z = 0;

        ss << result[4];
        ss >> std_mbias_X;
        ss.str("");
        ss.clear();

        ss << result[5];
        ss >> std_mbias_Y;
        ss.str("");
        ss.clear();

        ss << result[6];
        ss >> std_mbias_Z;
        ss.str("");
        ss.clear();

        mag_data.covarianceMatrix[0][0] = std_mbias_X * std_mbias_X;
        mag_data.covarianceMatrix[1][1] = std_mbias_Y * std_mbias_Y;
        mag_data.covarianceMatrix[2][2] = std_mbias_Z * std_mbias_Z;

        return true;
    }

    bool FP_builder::parse_ble_proximity_db_file(std::string file_path, std::vector<std::string> & ble_proximity_db)
    {
        //std::regex uuid_mask = std::regex("[0-9,a-f,A-F]{8}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{12}");
        std::string uuid_symbols("-0123456789abcdefABCDEF");
        const char uuid_delim = '-';

        ble_proximity_db.clear();

        std::ifstream infile(file_path);
        std::string line;
        const char delim = ',';
        std::string newline;
        FloorConverter floorConverter;
        floorConverter.SetFloorsParams(
            venue.floors_count,        /**< total floor number in venue */
            venue.floor_height,        /**< floor height [m] */
            venue.floor_shift,        /**< floor shift*/
            venue.floor_zero_enable
            );

        ble_proximity_hashes_list.clear();

        while (std::getline(infile, line))
        {
            size_t pos_r = line.find('\r');  // it needs for Linux
            if (pos_r != std::string::npos)
                line.erase(pos_r, 1);

            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;

            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }
            if (! ( (result.size() == 7) || (result.size() == 8) || (result.size() == 9) || (result.size() == 10) ) )
            {
                continue;
            }
            // remove the spaces
            //std::string uuid = std::regex_replace(std::string(result[0]), std::regex("[ ]"), "");
            std::string uuid = result[0];
            uuid.erase(std::remove(uuid.begin(), uuid.end(), ' '), uuid.end());

            //bool match_uuid = std::regex_match(uuid, uuid_mask);
            //if (! match_uuid )
            //    continue;

            // remove the '-'
            //uuid.erase(std::remove(uuid.begin(), uuid.end(), '-'), uuid.end());

            std::vector<bool> uuid_test;
            for (auto it = uuid.begin(); it != uuid.end(); ++it)
            {
                bool find_succes = (find_if(uuid_symbols.begin(), uuid_symbols.end(), [it](const char c) { return (*it == c); }) != uuid_symbols.end());
                if (find_succes)
                    uuid_test.push_back(find_succes);
            }
            //if (uuid_test.size() != uuid.size())
            if (uuid_test.size() != 36) // uuid length with '-'
                continue;

            int16_t real_floor = std::stoi(result[6]);
            floorConverter.SetCurrentRealFloor(real_floor);
            int16_t logical_floor = 0;
            floorConverter.GetCurrentLogicalFloor(&logical_floor);

            uint8_t uuid_array[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
            string_to_ble(uuid, uuid_array);
            uint16_t major = std::stoi(result[1]);
            uint16_t minor = std::stoi(result[2]);
            uint64_t hash = getBleHash(major, minor, uuid_array);

            if (find_if(venue.ble_ignore_list.begin(), venue.ble_ignore_list.end(), [hash](uint64_t blp_ignored_hash) { return (blp_ignored_hash == hash); }) != venue.ble_ignore_list.end())
                continue;
            //std::cout << "hash = " << hash << std::endl;
            ble_proximity_hashes_list.push_back(hash);

            newline = uuid + ", " + result[1] + ", " + result[2] + ", " + result[4] + ", " + result[5] + ", " + std::to_string(logical_floor) + ", " + result[3];

            if ((result.size() >= 8))
            {
                if (result[7] != "")
                    newline = newline + ", " + result[7];
                else
                    newline = newline + ", " + std::to_string(default_blp_height);
            }
            else
                newline = newline + ", " + std::to_string(default_blp_height);

            if ((result.size() >= 9))
            {
                if (result[8] != "")
                    newline = newline + ", " + result[8];
                else
                    newline = newline + ", " + std::to_string(default_blp_type);
            }
            else
                newline = newline + ", " + std::to_string(default_blp_type);

			if ((result.size() >= 10))
			{
				if (result[9] != "")
					newline = newline + ", " + result[9]; // TxPower_correction
			}

            ble_proximity_db.push_back(newline);
        }
        // ble proximity db parsing done

        return true;

    }

    // output of magnetic calibration result
    bool FP_builder::save_mag_out_file(std::string file_path, TrackPreProcessing::MagBias &magbias)
    {
        std::ofstream outfile(file_path);

        outfile << (uint16_t)magbias.calibration_level;
        outfile << "," << std::fixed << std::setw(12) << std::right << std::setprecision(3) << magbias.bias[0];
        outfile << "," << std::setw(12) << magbias.bias[1];
        outfile << "," << std::setw(12) << magbias.bias[2];
        outfile << "," << std::setw(12) << magbias.covarianceMatrix[0][0];
        outfile << "," << std::setw(12) << magbias.covarianceMatrix[1][1];
        outfile << "," << std::setw(12) << magbias.covarianceMatrix[2][2];
        outfile.close();

        return true;
    }

    bool FP_builder::parse_mag_out_file(std::string file_path, TrackPreProcessing::MagBias &mag_data)
    {
        std::ifstream infile(file_path);
        std::string line;
        const char delim = ',';
        bool result = true;

        if (std::getline(infile, line))
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> data;

            memset(mag_data.covarianceMatrix, 0, sizeof(mag_data.covarianceMatrix));

            while (std::getline(buf, token, delim))
            {
                data.push_back(token);
            }

            std::stringstream ss;
            result = data.size() >= 4;
            if (result == true)
            {
                // calibration level
                unsigned int ui_val;
                ss << data[0] << ENDL;
                ss >> ui_val;
                mag_data.calibration_level = (uint8_t)ui_val;
                ss.str("");
                ss.clear();

                // mab bias parsing
                for (int i = 0; i < 3; i++)
                {
                    ss << data[1 + i];
                    ss >> mag_data.bias[i];
                    ss.str("");
                    ss.clear();
                }
            }

            // cavariance matrix parsing
            if ((result == true) && (data.size() == 7))
            {
                for (int i = 0; i < 3; i++)
                {
                    ss << data[i + 4];
                    ss >> mag_data.covarianceMatrix[i][i];
                    ss.str("");
                    ss.clear();
                }
            }
            else if ((result == true) && (data.size() == 13))
            {
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                    {
                        ss << data[4 + i + j * 3];
                        ss >> mag_data.covarianceMatrix[i][j];
                        ss.str("");
                        ss.clear();
                    }
            }
            else
            {
                result = false;
            }
        }
        return true;
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------
    /** parses IRL format file and fills irl_data vector
    * \param[in] file_path - path to binary TPN/IRL file
    * \param[in] have_mag_bias is mag_bias file present or not
    * \param[in] mag_bias structure contains magnetic bias data
    * \param[out] irl_data - output vector in TpnOutput format
    * \return status code */
    bool FP_builder::parse_irl_file(std::string file_path, std::vector<TpnOutput> &irl_data, 
        bool read_calibrated_mag_data)
    {
        static int irl_counter = 0;
        irl_counter++;

        double first_timestamp = -100;

        Fpbl::ReturnStatus status;

        tpn_data_reader tpn_reader;
        std::ifstream is_irl_binary_stream(file_path, std::ios_base::in | std::ios_base::binary);
        tpn_packet_parser tpn_parser;
        tpn_packet_header_data_t header_data;

        tpn_reader.setStream(&is_irl_binary_stream);

        if (tpn_parser.get_packet_header_data(header_data))
        {
            //std::cout << "header: ";
            //std::cout << header_data.packet_id_ << "  " << header_data.epoch_number_ << "  " << header_data.number_of_entities_;
        }

        int packet_count = 0;
        int transit_count = 0;
#if 0
        std::cout << file_path;// << std::endl;
        //std::cout << " , ";// << std::endl;
#endif
#if (DEBUG_OUTPUT__HEADING_LOG)
        std::fstream old_headings_log;
        old_headings_log.open( "headings_OLD.log", std::ofstream::out | std::ofstream::trunc );

        std::fstream new_headings_log;
        new_headings_log.open( "headings_NEW.log", std::ofstream::out | std::ofstream::trunc );
#endif

        if (tpn_reader.is_state_correct())
        {
            while (tpn_reader.read_next_packet(tpn_parser))
            {
                packet_count++;
                // parse paket
                tpn_packet_header_data_t header_data;

                if (tpn_parser.get_packet_header_data(header_data))
                {
                    //std::cout << "packet " << packet_count << ": ";
                    //std::cout << header_data.packet_id_ << "  " << header_data.epoch_number_ << "  " << header_data.number_of_entities_;
                }

                tpn_entity_time_t e_time;
                tpn_entity_position_t e_pos;
                tpn_entity_position_std_t e_pos_std;
                tpn_entity_attitude_standard_deviation_t e_att_std;
                tpn_entity_magnetometer_data_t e_mag;
                tpn_entity_grp_07_t e_grp_07;
                tpn_entity_internal_dbg_t e_dbg;
                tpn_entity_floor_number_t e_floor;
                tpn_entity_device_heading_t e_dev_heading;
                tpn_entity_stride_information_t e_stride_inf;
                tpn_entity_attitude_t e_att;
                tpn_entity_mode_of_transit e_transit_mode;

                bool f_time = tpn_parser.get_entity_time(e_time);
                bool f_pos = tpn_parser.get_entity_position(e_pos);
                bool f_pos_std = tpn_parser.get_entity_position_std(e_pos_std);
                bool f_att_std = tpn_parser.get_entity_attitude_standard_deviation(e_att_std);
                bool f_mag = tpn_parser.get_entity_magnetic_data(e_mag);
                bool f_grp_7 = tpn_parser.get_entity_grp_07(e_grp_07);
                bool f_dbg = tpn_parser.get_entity_internal_dbg(e_dbg);
                bool f_floor = tpn_parser.get_entity_floor_number(e_floor);
                bool f_dev_heading = tpn_parser.get_entity_device_heading(e_dev_heading);
                bool f_stride_inf = tpn_parser.get_entity_stride_information(e_stride_inf);
                bool f_att = tpn_parser.get_entity_attitude(e_att);
                bool f_mode_of_transit = tpn_parser.get_entity_mode_of_transit(e_transit_mode);

                // special logig is required for e_baro_data processing due to 1Hz samplerate
                tpn_entity_barometer_data e_baro_data = {};
                bool f_baro_data = tpn_parser.get_entity(e_baro_data);

                tpn_entity_barometer_height e_baro_height = {};
                bool f_baro_height = tpn_parser.get_entity(e_baro_height);
                //std::cout << "e_floor  " << e_floor.floor_number_ << "   barometer_filter_height_  " << e_dbg.barometer_filter_height_ << "   e_pos.height_    " << e_pos.height_ << "   e_baro_data.height_    " << e_baro_data.height_ << std::endl;
                //std::cout  << "   e_pos.height_    " << e_pos.height_ << std::endl;
                //std::cout << "   barometer_filter_height_  " << e_dbg.barometer_filter_height_ << "   " << e_dbg.barometer_filter_height_standard_deviation_ << std::endl;
                //std::cout << "f_baro  " << f_baro << "   e_baro_data.height_    " << e_baro_data.height_ << "     " << e_baro_data.height_standard_deviation_ << std::endl;
                //std::cout << "f_baro_height  " << f_baro_height << "   e_baro_height  " << e_baro_height.barometer_filter_height_ << "       " << e_baro_height.barometer_filter_height_standard_deviation_ << std::endl;

                if (f_time && f_pos && f_pos_std && f_mag && f_grp_7 && f_floor && /*f_dev_heading && */f_att_std)
                //if ( e_grp_07.m_nav_phase > 0 )
                {
                    TpnOutput irlOutput = {};
                    irlOutput.timestamp = e_time.timetag_;

                    if (first_timestamp < 0)
                    {
                        first_timestamp = irlOutput.timestamp;
                    }

                    // transit mode
                    //irlOutput.position.mode_of_transit = (f_mode_of_transit) ? (e_transit_mode.mode) : (0); // do not use transit mode from tpn

                    // checking if user is moving or has stopped
                    irlOutput.pdr.stride_length = (f_stride_inf) ? (e_stride_inf.stride_distance_) : (0);
                    irlOutput.pdr.is_valid = f_stride_inf;

                    if (f_mode_of_transit && ((e_transit_mode.mode == kStairs) || (e_transit_mode.mode == kEscalatorStanding)))
                    {   // special case for stairs and escalators
                        irlOutput.pdr.is_valid = true;
                        irlOutput.pdr.stride_length = 1; // any value > 0 works
                    }

                    // position data
                    irlOutput.position.lattitude = e_pos.latitude_;
                    irlOutput.position.longitude = e_pos.longitude_;
#if 1
                    irlOutput.position.sigma_north = e_pos_std.position_north_standard_deviation_;
                    irlOutput.position.sigma_east = e_pos_std.position_east_standard_deviation_;
#else
                    if ((packet_count > 100) && (packet_count < 1000))
                    {
                        irlOutput.position.sigma_north = packet_count % 7;
                        irlOutput.position.sigma_east = packet_count % 7;
                    }
                    else
                    {
                        irlOutput.position.sigma_north = 10;
                        irlOutput.position.sigma_east = 10;
                    }
#endif
                    //std::cout << "sigma_north = " << irlOutput.position.sigma_north << std::endl;
                    //std::cout << "sigma_east = " << irlOutput.position.sigma_east << std::endl;

                    if (wifi_self_healing_mode)
                    {
                        if (irlOutput.timestamp - first_timestamp < 30.0)
                            irlOutput.position.sigma_north = irlOutput.position.sigma_east = 1000.0; // not using data from first 30 seconds

                        if (irlOutput.position.sigma_north < 0.01)
                            irlOutput.position.sigma_north = 1000.0;
                        if (irlOutput.position.sigma_east < 0.01)
                            irlOutput.position.sigma_east = 1000.0; // initially sigma is set to zero, we shouldn't use this data (most likely previous condition takes care of it)
                    }
                    else
                        if (xblp_detection_mode)
                        {
                            if (irlOutput.timestamp - first_timestamp < venue.ble_proximity_data.skip_time)
                                //irlOutput.position.sigma_north = irlOutput.position.sigma_east = 1000.0; // not using data from skip_time 30 seconds
                                continue;

                            if (irlOutput.position.sigma_north < 0.01)
                                irlOutput.position.sigma_north = venue.ble_proximity_data.sigma;
                            if (irlOutput.position.sigma_east < 0.01)
                                irlOutput.position.sigma_east = venue.ble_proximity_data.sigma; // initially sigma is set to zero, we shouldn't use this data (most likely previous condition takes care of it)
                        }

#if DEBUG_INPUT__USE_ROUTE_IRL_DATA
                    // set small position dispersion for rout data
                    irlOutput.position.sigma_north = 1.;
                    irlOutput.position.sigma_east = 1.;
#endif

//                    irlOutput.position.altitude = e_pos.height_;
                    if (f_baro_data)
                    {
                        irlOutput.position.altitude = e_baro_data.height_;
                        irlOutput.position.sigma_altitude = 1;
                    }
                    else
                    {
                        irlOutput.position.altitude = 0;
                        irlOutput.position.sigma_altitude = -1;
                    }
                    irlOutput.position.floor = e_floor.floor_number_;
                    //irlOutput.position.floor -= 1; // ACHTUNG: the IRL floor numbering is from 1
                    // !!!!!!!! Comment the above line if building from converted Mapper data!

                    //irlOutput.position.sigma_altitude = e_pos_std.height_standard_deviation_;

                    if (f_dev_heading)
                    {
                        irlOutput.position.misalignment = e_dev_heading.misalignment_angle_;
                        irlOutput.position.user_heading = e_dev_heading.platform_heading_;
                        irlOutput.position.sigma_user_heading = e_att_std.heading_standard_deviation_; // warning: it needs to clarify input entity and field for IRL
                    }
                    else
                    {
                        irlOutput.position.misalignment = 0.;
                        irlOutput.position.user_heading = 0.;
                        irlOutput.position.sigma_user_heading = -100.; // indicates invalidity of misalignment, user_heading and itself
                    }

                    irlOutput.position.is_valid = true;

                    if (f_dbg)
                    {
                        irlOutput.position.fidgeting_flag = e_dbg.walking_fidgeting_flag;
                        irlOutput.position.navigation_phase = e_grp_07.m_nav_phase;
                        // attitude data
                        irlOutput.attitude.roll = e_dbg.m_att_filter_roll;
                        irlOutput.attitude.pitch = e_dbg.m_att_filter_pitch;
                        // attitude data
                        irlOutput.attitude.roll = e_dbg.m_att_filter_roll;
                        irlOutput.attitude.pitch = e_dbg.m_att_filter_pitch;
                        irlOutput.attitude.heading = e_att.heading_;
#if DEBUG_OUTPUT__HEADING_LOG
                        old_headings_log << e_time.timetag_ << " , " << e_dbg.m_att_filter_heading << ENDL;
                        new_headings_log << e_time.timetag_ << " , " << e_att.heading_ << ENDL;
#endif
                        // ************************************************************

                        irlOutput.attitude.sigma_roll = e_dbg.m_att_filter_roll_std;
                        irlOutput.attitude.sigma_pitch = e_dbg.m_att_filter_pitch_std;
                        irlOutput.attitude.sigma_heading = e_dbg.m_att_filter_heading_std;

                        irlOutput.attitude.orientation_id = e_dbg.m_orientation_on_pitch;

                        irlOutput.attitude.is_valid = true;
                    }
                    else
                        irlOutput.attitude.is_valid = false;

                    // IRL magnetic data
                    irlOutput.mag_meas.is_valid = false;

                    const double uT_to_mG = 10.0;
                    if (false == read_calibrated_mag_data )
                    {
                        if ((bool)e_mag.raw_data_available_)
                        {
                            irlOutput.mag_meas.mX = e_mag.raw_data_x_;// -mag_bias.bias[0] * uT_to_mG;
                            irlOutput.mag_meas.mY = e_mag.raw_data_y_;// -mag_bias.bias[1] * uT_to_mG;
                            irlOutput.mag_meas.mZ = e_mag.raw_data_z_;// -mag_bias.bias[2] * uT_to_mG;
                            irlOutput.mag_meas.level_of_calibration = 0;
                            irlOutput.mag_meas.is_valid = true;

                            const double default_mag_meas_sigma = 0.5 * uT_to_mG; // mGaus
                            irlOutput.mag_meas.sigma_mX = irlOutput.mag_meas.sigma_mY = irlOutput.mag_meas.sigma_mZ = default_mag_meas_sigma; // WARNING: temporary assignment due to absence meassurement sigma in current TPN/IRL output

                            //irlOutput.mag_meas.level_of_calibration = 0;// mag_bias.calibration_level;
                            for (int i = 0; i < 3; i++)
                                for (int j = 0; j < 3; j++)
                                {
                                    irlOutput.mag_meas.covarianceMatrix[i][j] = ( i == j ) ? 1e6 * uT_to_mG *uT_to_mG : 0; // set large dispersion to prevent using uncalibrated data
                                }
                        }
                    }
                    else // reading calibratd data
                    {
                        if ((bool)e_mag.calibrated_data_available_ && (e_mag.calibrated_data_accuracy_flag_ >= 2))
                        {
                            irlOutput.mag_meas.mX = e_mag.calibrated_data_x_;
                            irlOutput.mag_meas.mY = e_mag.calibrated_data_y_;
                            irlOutput.mag_meas.mZ = e_mag.calibrated_data_z_;
                            irlOutput.mag_meas.level_of_calibration = e_mag.calibrated_data_accuracy_flag_;// mag_bias.calibration_level;
                            irlOutput.mag_meas.is_valid = true;

                            const double default_mag_meas_sigma = 0.5 * uT_to_mG; // mGaus
                            irlOutput.mag_meas.sigma_mX = irlOutput.mag_meas.sigma_mY = irlOutput.mag_meas.sigma_mZ = default_mag_meas_sigma; // WARNING: temporary assignment due to absence meassurement sigma in current TPN/IRL output

                            irlOutput.mag_meas.sigma_mX = default_mag_meas_sigma; // WARNING: this assignment is due to absence meassurement sigma in current TPN/IRL output
                            irlOutput.mag_meas.sigma_mY = default_mag_meas_sigma; // WARNING: this assignment is due to absence meassurement sigma in current TPN/IRL output
                            irlOutput.mag_meas.sigma_mZ = default_mag_meas_sigma; // WARNING: this assignment is due to absence meassurement sigma in current TPN/IRL output

                            double default_mag_bias_sigma = 5 * uT_to_mG; //mGaus

                            for (int i = 0; i < 3; i++)
                                for (int j = 0; j < 3; j++)
                                {
                                    irlOutput.mag_meas.covarianceMatrix[i][j] = ( i == j ) ? default_mag_bias_sigma * default_mag_bias_sigma : 0; // WARNING: this assignment is due to absence meassurement sigma in current TPN/IRL output
                                }
                        }
                    }

                    irl_data.push_back(irlOutput);
                }
            }
            status = Fpbl::ReturnStatus::STATUS_SUCCESS;

#if 0 // debug output to check raw/calibrated data utilization
            // attention: this debug code uses same condition as the parsing code above
            //            check the condition equality if this code is used
            if ( have_mag_bias == true ) // we have mag_out.txt for this IRL track
                std::cout << "raw_mag, b=" << mag_bias.bias[0] << "," << mag_bias.bias[1] << "," << mag_bias.bias[2];
            else
                std::cout << "clb_mag";
            std::cout << "\n";
#endif

        }
        else
        {
            std::cout << "Can't open IRL data file\n";
            status = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

#if DEBUG_OUTPUT__HEADING_LOG
        old_headings_log.close();
        new_headings_log.close();
#endif

        return (status == Fpbl::ReturnStatus::STATUS_SUCCESS);
    }

    uint8_t FP_builder::get_mag_calibration_level(const std::vector<TpnOutput> &irl_data)
    {
        uint8_t mag_calibration_level(std::numeric_limits<uint8_t>::max());

        for (auto it = irl_data.begin(); it != irl_data.end(); ++it)
        {
            mag_calibration_level = (it->mag_meas.is_valid) ? std::min(it->mag_meas.level_of_calibration, mag_calibration_level) : mag_calibration_level;
           /* if ((it->mag_meas.is_valid) && 
            {
                mag_calibration_level = std::min(it->mag_meas.level_of_calibration, mag_calibration_level);
                mag_calibration_level = std::min(3, 2);
            }*/
        }
        return 0;
    }

    bool FP_builder::apply_mag_bias(const TrackPreProcessing::MagBias mag_bias, std::vector<TpnOutput> &irl_data)
    {
        const double uT_to_mG = 10.0;

        for (auto it = irl_data.begin(); it != irl_data.end(); ++it)
        {
            it->mag_meas.mX -= mag_bias.bias[0] * uT_to_mG;
            it->mag_meas.mY -= mag_bias.bias[1] * uT_to_mG;
            it->mag_meas.mZ -= mag_bias.bias[2] * uT_to_mG;

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    it->mag_meas.covarianceMatrix[i][j] = mag_bias.covarianceMatrix[i][j] * uT_to_mG * uT_to_mG;

            it->mag_meas.level_of_calibration = mag_bias.calibration_level;
        }
        return true;
    }

    //--------------------------------------------------------------------------------------------
    /** parses IRL format file and fills WiFi message array
    * \param[in] file_name - TPN/IRL file name
    * \param[out] wifiScanResults - output vector of WiFiScanResult structures
    * \return status code */
    bool FP_builder::parse_irl_file_for_wifi(std::string file_path, std::vector<WiFiScanResult > & wifiScanResults)
    {
        Fpbl::ReturnStatus status;

        std::ifstream is_irl_binary_stream(file_path, std::ios_base::in | std::ios_base::binary);
        tpn_data_reader tpn_reader(&is_irl_binary_stream);
        tpn_packet_parser tpn_parser;
        tpn_packet_header_data_t header_data;

        int packet_count = 0;

        if (tpn_reader.is_state_correct())
        {
            while (tpn_reader.read_next_packet(tpn_parser))
            {
                packet_count++;
                // parse paket
                tpn_packet_header_data_t header_data;

                if (tpn_parser.get_packet_header_data(header_data))
                {
                    //std::cout << "packet " << packet_count << ": ";
                    //std::cout << header_data.packet_id_ << "  " << header_data.epoch_number_ << "  " << header_data.number_of_entities_;
                }

                tpn_entity_time_t e_time;
                tpn_entity_wifi_scan_t e_wifi;

                bool f_time = tpn_parser.get_entity_time(e_time);

                if (f_time)
                {
                    WiFiScanResult irlWiFi = {};
                    irlWiFi.timestamp = (int64_t)(e_time.timetag_ * 1.e3);

                    uint16_t wifi_number = 0;

                    while (tpn_parser.get_entity(e_wifi, wifi_number))
                    {
                            WiFiMeasurement wifi_measurement;
                            wifi_measurement.timestamp = e_wifi.timestamp;
                            wifi_measurement.frequency = e_wifi.frequency;
                            wifi_measurement.mac = e_wifi.mac;
                            wifi_measurement.rssi = e_wifi.rssi;
                            irlWiFi.scanWiFi.push_back(wifi_measurement);
                            wifi_number++;
                    }

                    if (irlWiFi.scanWiFi.size() > 0)
                    {
                            wifiScanResults.push_back(irlWiFi);
                    }
                }
            }
            status = Fpbl::ReturnStatus::STATUS_SUCCESS;
        }

        else
        {
            std::cout << "Can't open IRL data file\n";
            status = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        if (is_irl_binary_stream.is_open())
            is_irl_binary_stream.close();

        return (status == Fpbl::ReturnStatus::STATUS_SUCCESS);
    }

    //--------------------------------------------------------------------------------------------
    /** parses IRL format file and fills irl_data vector
    * \param[in] file_name - TPN/IRL file name
    * \param[out] bleScanResults - output vector of bleScanResults structures
    * \return status code */
    bool FP_builder::parse_irl_file_for_ble(std::string file_path, std::vector<BleScanResult > & bleScanResults)
    {
        double first_timestamp = -100;

        Fpbl::ReturnStatus status;

        std::ifstream is_irl_binary_stream(file_path, std::ios_base::in | std::ios_base::binary);
        tpn_data_reader tpn_reader(&is_irl_binary_stream);

        tpn_packet_parser tpn_parser;
        tpn_packet_header_data_t header_data;

        int packet_count = 0;
        status = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;

        if (tpn_reader.is_state_correct())
        {
            while (tpn_reader.read_next_packet(tpn_parser))
            {
                packet_count++;
                // parse packet
                tpn_packet_header_data_t header_data;

                if (tpn_parser.get_packet_header_data(header_data))
                {
                    //std::cout << "packet " << packet_count << ": ";
                    //std::cout << header_data.packet_id_ << "  " << header_data.epoch_number_ << "  " << header_data.number_of_entities_;
                }

                tpn_entity_time_t e_time;
                tpn_entity_ble_scan_t e_ble;

                bool f_time = tpn_parser.get_entity_time(e_time);

                if (f_time)
                {
                    BleScanResult irlBLE = {};
                    irlBLE.timestamp = (int64_t)(e_time.timetag_ * 1.e3);

                    if (first_timestamp < 0)
                    {
                        first_timestamp = irlBLE.timestamp;
                    }

                    if (xblp_detection_mode)
                    {
                        if (irlBLE.timestamp - first_timestamp < venue.ble_proximity_data.skip_time * 1.e3)
                        {
                            continue;
                        }
                    }

                    uint16_t ble_number = 0;
                    bool new_ble_entity = true;

                    while (new_ble_entity)
                    {
                        new_ble_entity = tpn_parser.get_entity(e_ble, ble_number);

                        if (new_ble_entity)
                        {
                            // fill one BleMeasurement element
                            BleMeasurement ble_measurement;

                            ble_measurement.timestamp = e_ble.timestamp;
                            ble_measurement.frequency = e_ble.frequency;
                            ble_measurement.hasMAC = e_ble.has_MAC;
                            ble_measurement.mac = e_ble.mac;
                            ble_measurement.major = e_ble.major;
                            ble_measurement.minor = e_ble.minor;
                            ble_measurement.txPower = e_ble.tx_power;
                            ble_measurement.rssi = e_ble.rssi;

                            // WARNING: UUID curently is 128-bit number (16-byte arry) in big endiam format
                            memcpy(ble_measurement.uuid, &(e_ble.uuid_l), sizeof(e_ble.uuid_l));
                            memcpy(ble_measurement.uuid + sizeof(e_ble.uuid_l), &(e_ble.uuid_h), sizeof(e_ble.uuid_h));

                            /*
                            uint8_t uuid_be[16], uuid_le[16];

                            uint64_t uuid_l_be = e_ble.uuid_l; // big endian value
                            uint64_t uuid_h_be = e_ble.uuid_h; // big endian value
                            for (int i = 0; i <= 7; i++)
                            {
                            uuid_be[i] = (uint64_t)0xFF & uuid_h_be;
                            uuid_be[i + 8] = (uint64_t)0xFF & uuid_l_be;
                            uuid_le[7 - i] = (uint64_t)0xFF & uuid_l_be;          uuid_l_be >>= 8;
                            uuid_le[7 - i + 8] = (uint64_t)0xFF & uuid_h_be;      uuid_h_be >>= 8;
                            }

                            uint64_t uuid_l_le = e_ble.uuid_l;
                            uint64_t uuid_h_le = e_ble.uuid_h;
                            for (int i = 0; i <= 7; i++)
                            {
                            uuid_le[i] = (uint64_t)0xFF & uuid_l_le;
                            uuid_le[i + 8] = (uint64_t)0xFF & uuid_h_le;
                            uuid_be[7 - i] = (uint64_t)0xFF & uuid_h_le;          uuid_h_le >>= 8;
                            uuid_be[7 - i + 8] = (uint64_t)0xFF & uuid_l_le;      uuid_l_le >>= 8;
                            }
                            */

                            if (bleEnableProxOnly)
                            {
                                uint64_t hash = getBleHash(ble_measurement.major, ble_measurement.minor, ble_measurement.uuid);
                                if (find_if(ble_proximity_hashes_list.begin(), ble_proximity_hashes_list.end(), [hash](uint64_t blp_hash) { return (blp_hash == hash); }) != ble_proximity_hashes_list.end())
                                {
                                    //std::cout << "major = " << ble_measurement.major << "   minor = " << ble_measurement.minor << std::endl;
                                    irlBLE.scanBle.push_back(ble_measurement);
                                }
                            }
                            else
                                irlBLE.scanBle.push_back(ble_measurement);

                            ble_number++;
                        }
                    }

                    if (irlBLE.scanBle.size() > 0)
                    {
                        bleScanResults.push_back(irlBLE);
                    }
                }
            }
            status = Fpbl::ReturnStatus::STATUS_SUCCESS;
        }
        else
        {
            std::cout << "Can't open IRL data file\n";
        }

        if (is_irl_binary_stream.is_open())
            is_irl_binary_stream.close();

        return (status == Fpbl::ReturnStatus::STATUS_SUCCESS);
    }

    //--------------------------------------------------------------------------------------------
    void FP_builder::send_data_to_builder(std::vector<TpnOutput> irl_data, Fpbl::DataSetType dataset_type, Fpbl::GridBuilder &Builder)
    {
        for (std::vector<TpnOutput>::const_iterator irl_data_item = irl_data.begin(); irl_data_item != irl_data.end(); ++irl_data_item)
        {
            Fpbl::ReturnStatus status = Builder.processIrlOutput(*irl_data_item, dataset_type);
        }
    }
    
    //--------------------------------------------------------------------------------------------
    BSSID FP_builder::string_to_mac(std::string const &s)
    {
        uint64_t last = -1;
        uint64_t band = 0;
        uint64_t mac = 0;
        uint64_t a[8];
        int rc = 7;

        int cnt = std::count(s.begin(), s.end(), ':');

        const char delim = ':';

        std::stringstream buf(s);
        std::string token;
        std::vector<std::string> result;

        while (std::getline(buf, token, delim))
        {
            result.push_back(token);
        }

        uint16_t size = result.size();

        if (size != 8 && size != 7 && size != 6)
            return 0;

        for (int i = 0; i < 8; i++)
        {
            a[i] = 0;
        }

        mac = 0;

        for (int i = 0; i < size; i++)
        {
            std::stringstream ss;
            ss << result[size - 1 - i];
            ss >> std::hex >> a[i];
            ss.str("");
            ss.clear();

            mac |= a[i] << i * 8;
        }

        return mac;
    }

    //--------------------------------------------------------------------------------------------
    void FP_builder::string_to_ble(std::string const &s, uint8_t *ble)
    {
        uint32_t part1 = 0;
        uint16_t part2 = 0;
        uint16_t part3 = 0;
        uint16_t part4 = 0;
        uint64_t part5 = 0;

        uint8_t  tmp[16];

        memset(ble, 0, 16);
        memset(tmp, 0, 16);
        const char delim = '-';

        std::stringstream buf(s);
        std::string token;
        std::vector<std::string> result;

        while (std::getline(buf, token, delim))
        {
            result.push_back(token);
        }

        if (result.size() != 5)
            return;

        std::stringstream ss;

        ss << result[0];
        ss >> std::hex >> part1;
        ss.str("");
        ss.clear();

        *(uint32_t *)(tmp + 12) = part1;

        ss << result[1];
        ss >> std::hex >> part2;
        ss.str("");
        ss.clear();

        *(uint16_t *)(tmp + 10) = part2;

        ss << result[2];
        ss >> std::hex >> part3;
        ss.str("");
        ss.clear();

        *(uint16_t *)(tmp + 8) = part3;

        ss << result[3];
        ss >> std::hex >> part4;
        ss.str("");
        ss.clear();

        *(uint16_t *)(tmp + 6) = part4;

        ss << result[4];
        ss >> std::hex >> part5;
        ss.str("");
        ss.clear();

        *(uint64_t *)tmp |= part5;

        // big endian
        for (int i = 0; i < 16; i++)
        {
            ble[15 - i] = tmp[i];
        }
        /*
        std::cout << std::hex << *(uint64_t *)(tmp + 8) << *(uint64_t *)tmp << "      ";
        std::cout << *(uint64_t *)(ble + 8) << *(uint64_t *)ble << "      ";
        std::cout << std::dec << std::endl;
        */
    }

    // the method removes stuck WiFi meassurement from WiFi scans
    // @return count ot removed meassurements
    size_t FP_builder::remove_stuck_scans(std::vector<WiFiScanResult > & wifiMeas)
    {
        const uint64_t k_min_scan_timestamp_interval = 250;
        size_t rejected_count = 0;
        if (wifiMeas.size() > 1)
        {
            for (auto scan_it1 = wifiMeas.begin(), scan_it2 = wifiMeas.begin() + 1; scan_it2 != wifiMeas.end(); scan_it1++, scan_it2++)
            {
                if ((scan_it2->timestamp - scan_it1->timestamp) < k_min_scan_timestamp_interval)
                {
                    // empty stuck scans - it is a marker of stuck scans
                    scan_it1->scanWiFi.clear();
                    scan_it2->scanWiFi.clear();
                    rejected_count++;
                }
            }

            wifiMeas.erase(std::remove_if(wifiMeas.begin(), wifiMeas.end(),
                [](WiFiScanResult scan_it) {   return (scan_it.scanWiFi.size() == 0); }),
                wifiMeas.end());
        }
        return rejected_count;
    }

    // the method removes repeatable WiFi meassurement from WiFi scans
    // @return count ot removed meassurements
    size_t FP_builder::clear_repeatable_measurements(std::vector<WiFiScanResult > & wifiScanResults)
    {
        // create list of pointers to each single measurement
        typedef std::pair<std::vector<WiFiMeasurement>::iterator, std::vector<WiFiScanResult>::iterator> meas_item;
         std::vector<meas_item> all_meas;

        for (auto scan_it = wifiScanResults.begin(); scan_it != wifiScanResults.end(); scan_it++)
        {
            for (auto meas_it = scan_it->scanWiFi.begin(); meas_it != scan_it->scanWiFi.end(); meas_it++)
            {
                    all_meas.push_back(std::make_pair(meas_it, scan_it));
            }
        }

        // sort list
        std::sort(all_meas.begin(), all_meas.end(), [](meas_item a, meas_item b) {
            if (a.first->timestamp != b.first->timestamp)   return (a.first->timestamp < b.first->timestamp);
            else if (a.first->mac != b.first->mac)          return (a.first->mac < b.first->mac);
            else                                            return (a.second->timestamp > b.second->timestamp);
        });

        // erase repeatable values
        auto is_equal = [](meas_item a, meas_item b) {
            return (a.first->timestamp == b.first->timestamp) && (a.first->mac == b.first->mac)/* && (a.first->rssi > b.first->rssi)*/;
        };

        size_t rejected_count(0), i(0), sz = all_meas.size();
        for (i = 0; (sz > 0) && (i < (sz - 1)); i++)
        {
#if 0
            { // debug output
                std::cout << all_meas.at(i).first->timestamp;
                std::cout << ", " << all_meas.at(i).first->mac;
                std::cout << ", " << (int)all_meas.at(i).first->rssi;
                std::cout << ", " << all_meas.at(i).second->timestamp;
                std::cout << ENDL;
            }
#endif
            if (is_equal(all_meas.at(i), all_meas.at(i + 1)))
            {
                all_meas[i].first->frequency = 0; // mark for erase
                rejected_count++;
            }
            //i++;
        }

        //for (auto scan_it : wifiScanResults)
        for (auto scan_it = wifiScanResults.begin(); scan_it != wifiScanResults.end(); scan_it++)
        {
           scan_it->scanWiFi.erase(std::remove_if(scan_it->scanWiFi.begin(), scan_it->scanWiFi.end(),
                [](WiFiMeasurement meas)    {   return (meas.frequency == 0); }),
                scan_it->scanWiFi.end());
        }

        return rejected_count;
    }

    // the method regroup wifi messages in scans with specified scan period
    // @return count ot removed meassurements
    std::vector<WiFiScanResult > FP_builder::make_scans(const std::vector<WiFiScanResult > & wifiMeas, std::vector<WiFiScanResult > & wifiScans, int64_t scan_period)
    {
        wifiScans.clear();

        // create list of pointers to each single measurement
        typedef std::vector<WiFiMeasurement>::const_iterator meas_item;
        std::vector<meas_item> all_meas;
        for (auto scan_it = wifiMeas.begin(); scan_it != wifiMeas.end(); scan_it++)
        {
            for (auto meas_it = scan_it->scanWiFi.begin(); meas_it != scan_it->scanWiFi.end(); meas_it++)
            {
                all_meas.push_back(meas_it);
            }
        }

        // sort list
        std::sort(all_meas.begin(), all_meas.end(), [](meas_item a, meas_item b) {
            return (a->timestamp < b->timestamp);
        });

        // make scans
        WiFiScanResult scan;
        scan.timestamp = 0;
        scan.scanWiFi.clear();
        for (auto meas_it = all_meas.begin(); meas_it != all_meas.end(); meas_it++)
        {
            int64_t scan_time = static_cast<int64_t>(((*meas_it)->timestamp + 0.5* scan_period) / scan_period) * scan_period;

            if ((scan.timestamp != scan_time))
            {
                if (scan.scanWiFi.size() > 0)
                {
                    wifiScans.push_back(scan);
                }
                scan.timestamp = scan_time;
                scan.scanWiFi.clear();
            }

            scan.scanWiFi.push_back(**meas_it);
        }

        if (scan.scanWiFi.size() > 0)
        {
            wifiScans.push_back(scan);
        }

#if 0
            { // debug output
                for (auto scan_it = wifiScans.begin(); scan_it != wifiScans.end(); scan_it++)
                {
                    for (auto meas_it = scan_it->scanWiFi.begin(); meas_it != scan_it->scanWiFi.end(); meas_it++)
                    {
                        std::cout << meas_it->timestamp;
                        std::cout << ", " << meas_it->mac;
                        std::cout << ", " << (int)meas_it->rssi;
                        std::cout << ", " << scan_it->timestamp;
                        std::cout << ENDL;
                    }
                }
            }
#endif

        return wifiScans;
    }

    // the method regroup wifi messages in scans with specified scan period
    // @return count ot removed meassurements
    size_t FP_builder::apply_wifi_list(std::vector<WiFiScanResult > & wifiScanResults, std::vector <std::pair<uint64_t, uint64_t>> list, bool white_list_request)
    {
        size_t rejected_count = 0;
        for (auto scan_it = wifiScanResults.begin(); scan_it != wifiScanResults.end(); scan_it++)
        {
            for (auto meas_it = scan_it->scanWiFi.begin(); meas_it != scan_it->scanWiFi.end(); meas_it++)
            {
                auto mac_address = meas_it->mac;
                auto found_item = find_if(list.begin(), list.end(),
                    [mac_address](std::pair<uint64_t, uint64_t> list_item) {
                    uint64_t mask_lower = (list_item.first & list_item.second); // list_item.first - template, // list_item.second - mask
                    uint64_t mask_upper = (mask_lower | (~list_item.second));
                    return bool((mac_address >= mask_lower) && (mac_address <= mask_upper));});
                bool mac_is_out_of_range = found_item == list.end();
                if ((list.size()) && (mac_is_out_of_range == white_list_request))
                {
                    meas_it->mac = 0;
                    rejected_count++;
                }
            }
            scan_it->scanWiFi.erase(std::remove_if(scan_it->scanWiFi.begin(), scan_it->scanWiFi.end(),
                [](WiFiMeasurement meas) {   return (meas.mac == 0); }),
                scan_it->scanWiFi.end());
        }
        return rejected_count;
    }


    // the method regroup wifi messages in scans with specified scan period
    // @return count ot removed meassurements
    WiFiLoggingData::WiFiData FP_builder::wifi_logging(const std::vector<WiFiScanResult > & wifiMeas)
    {
        WiFiLoggingData::WiFiData data;
        std::map < BSSID, int> APs_map;
        APs_map.clear();

        data.count_of_scans = wifiMeas.size();
        data.count_of_messages = 0;

        for (auto wifiScanRes = wifiMeas.begin(); wifiScanRes != wifiMeas.end(); ++wifiScanRes)
        {
            data.count_of_messages += wifiScanRes->scanWiFi.size();

            for (auto wifiScan = wifiScanRes->scanWiFi.begin(); wifiScan != wifiScanRes->scanWiFi.end(); ++wifiScan)
            {
                BSSID AP = wifiScan->mac;
                APs_map.insert(std::pair< BSSID, int >(AP, 0));
            }
        }

        data.count_of_APs = APs_map.size();

        return data;
    }

    WiFiLoggingData::WiFiData FP_builder::ble_logging(const std::vector<BleScanResult > bleScanResults)
    {
        WiFiLoggingData::WiFiData data;
        std::map < BSSID, int> APs_map;
        APs_map.clear();

        data.count_of_scans = bleScanResults.size();
        data.count_of_messages = 0;

        for (auto bleScanRes = bleScanResults.begin(); bleScanRes != bleScanResults.end(); ++bleScanRes)
        {
            data.count_of_messages += bleScanRes->scanBle.size();

            for (auto bleScan = bleScanRes->scanBle.begin(); bleScan != bleScanRes->scanBle.end(); ++bleScan)
            {
                BSSID AP = bleScan->mac;
                APs_map.insert(std::pair< BSSID, int >(AP, 0));
            }
        }

        data.count_of_APs = APs_map.size();

        return data;
    }

    void FP_builder::parse_dataset_attr(std::string &attr, const char delim, std::string &name, std::string &value)
    {
        // remove all spaces from string
        attr.erase(remove_if(attr.begin(), attr.end(), isspace), attr.end());
        std::stringstream buf(attr);
        std::getline(buf, name, delim);
        std::getline(buf, value, '\n');
    }

	bool FP_builder::reparse_folders(std::string in_data_folder, std::string grid_folder, std::string &in_portals_grid_file, std::string &in_cs_grid_file, std::string fp_bases_folder,
        std::string &out_mag_grid_file, std::string &out_mag_fp_file, std::string &out_remag_grid_file,
        std::string &out_remag_fp_file, std::string validation_log_name, 
        std::string revalidation_log_name, std::string out_rebias_file, Fpbl::RouteMode_t route_type, Fpbl::DataSetType dataset_type)
	{
		DIR *dir_pointer;

		std::cout << "Starting Recalbiration Process:" << std::endl;
		std::cout << "================================" << std::endl;
		std::cout << "================================" << std::endl;
		std::cout << std::endl;

		validation_log_name = fp_bases_folder + SLASH + validation_log_name;
		revalidation_log_name = fp_bases_folder + SLASH + revalidation_log_name;

		std::ifstream validation_log(validation_log_name, std::ifstream::in);
		std::ofstream revalidation_log(revalidation_log_name, std::ofstream::out | std::ofstream::trunc);

		TrackFiles track_files;
		track_files.have_mg = false;
		track_files.have_pos = false;
		track_files.have_wifi = false;
		track_files.have_ble = false;
		track_files.have_irl = false;
		track_files.have_mag_out = false;

		track_files.pos = "";
		track_files.mg = "";
		track_files.wifi = "";
		track_files.ble = "";
		track_files.irl = "";
		track_files.mag_out = "";

		std::ofstream remag_bias_stream(in_data_folder + SLASH + "remag_bias_by_spectr-6.csv", std::ofstream::out);

		//INTEGRATION EXAMLE: start of grid initialisation (2)
		Fpbl::MagneticGrid remagneticGrid; // created once, then updated for each track
		Fpbl::PortalsGrid portalsGrid;
		Fpbl::CSGrid csGrid;
		{
			Fpbl::GridBuilder Builder;
			Builder.setVenue(venue);
			Builder.createEmptyGridMFP(venue.id, mag_grid, remagneticGrid);
			Builder.createEmptyGridPortals(venue.id, mag_grid, portalsGrid);
			Builder.createEmptyGridCS(venue.id, mag_grid, csGrid);
		}

		// reading portal grid
		//if (mag_grid.enable) 
		//{
		//	std::string input_grid_file_name = in_portals_grid_file;// +".portalsgrid";  // read portalsgrid
		//	std::cout << "Load portals grid from " << input_grid_file_name;
		//	if (load_portals_grid_file(input_grid_file_name, venue, mag_grid, portalsGrid))
		//		std::cout << "- ok" << ENDL;
		//	else
		//		std::cout << " - no data have been loaded" << ENDL;
		//}

		// reading CS grid
		if (mag_grid.enable)
		{
			std::string input_grid_file_name = in_cs_grid_file;// +".csgrid";  // read csgrid
			std::cout << "Load CS grid from " << input_grid_file_name;
			if (load_cs_grid_file(input_grid_file_name, venue, mag_grid, csGrid))
				std::cout << "- ok" << ENDL;
			else
				std::cout << " - no data has been loaded" << ENDL;
		}

		// init track processor
		TrackPreProcessing::TrackProcessor track_processor(default_mag_validators);
		if ((default_mag_validators == "empty") && (mag_validators.size() > 0))
		{
			std::cout << "actual validators:  " << std::endl;
			// for (int i = 0; i < mag_validators.size(); i++)
			//     std::cout << mag_validators[i] << std::endl;
			track_processor.set_mag_validators(mag_validators);
		}
		else
		{
			std::cout << std::endl << "default validators:  " << default_mag_validators << std::endl;
		}

		std::cout << std::endl;
		track_processor.ReadValidationReportHeader(validation_log);
		track_processor.PrintValidationReportHeader(revalidation_log);

		std::string output_file_name;
		if (pDbmfp == NULL)
		{
			std::cout << "MFP db loading..." << std::endl << std::endl;
			output_file_name = fp_bases_folder + SLASH + out_mag_fp_file + ".mfp" + std::to_string(mag_fp_format);  // save MFP DB
			pDbmfp = new Fpbl::MfpBuilder::LocalDB(mag_fp);
			loadMagneticDB(output_file_name, *pDbmfp, mag_fp_format);
		}

		// parse all folders
		while (track_processor.ReadValidationReport(validation_log))
		{
			Fpbl::GridBuilder Builder; // new builder instance is created for each track
			Fpbl::ReturnStatus status1 = Builder.setVenue(venue);

			std::string irl_file_name = track_processor.GetDataSetName();
			std::vector<TpnOutput> irl_data;
			TrackPreProcessing::MagBias mag_bias;

			size_t folder_path_index;
			std::string folder_path = irl_file_name;
			folder_path_index = irl_file_name.find_last_of("/\\");
			folder_path = folder_path.substr(0, folder_path_index);

			bool result = parse_irl_file(irl_file_name, irl_data, true); // reading calibrated mag data

			if (result /*&& track_processor.ReadDataSetValidity() == false*/)
			{
				std::cout << std::endl << "re-processing track: " << irl_file_name << std::endl;
#if ENABLE_MAG_RECALIBRATION
				{
					result = ReCalculateMagBiasForDataset(irl_data, mag_bias);

					std::string mag_bias_out_file_name = folder_path + SLASH + remag_out_file;
					result = save_mag_out_file(mag_bias_out_file_name, mag_bias);

					mag_bias_out_file_name = folder_path + SLASH + out_rebias_file; // saving all biases to check progress
					result = save_mag_out_file(mag_bias_out_file_name, mag_bias);
				}
#endif

				TrackPreProcessing::DataSetParams dataset_params;
				dataset_params.dataset_score_ = 100;
				dataset_params.calibrated_mag_data = true; // WARNING: this has to be false for IRL data or if we switch to uncalibrated mag meas

				// change mag_out_file_mask according to mode chosen
				TrackFiles track_files = ProcesDataSet(folder_path, dataset_params, revalidation_log, remag_bias_stream, Builder, track_processor, route_type, dataset_type, true, re_mag_out_file_mask);

				//INTEGRATION EXAMLE: start of grid update (4)
				//Fpbl::ReturnStatus status;
				if ((track_files.have_irl) || (track_files.have_mg && track_files.have_pos))
				{
					if (mag_grid.enable)
						Builder.updateGridMFP(venue.id, mag_grid, remagneticGrid);
				}
				
				Builder.updateGridPortals(venue.id, mag_grid, portalsGrid);

				//INTEGRATION EXAMLE: end of part

				track_files.have_mg = false;
				track_files.have_pos = false;
				track_files.have_wifi = false;
				track_files.have_ble = false;
				track_files.have_irl = false;
				track_files.have_mag_out = false;
			}
		}

		remag_bias_stream.close();

		std::cout << std::endl << "Final operations:" << std::endl;

		// saving magnetic grid
		if (mag_grid.enable)
		{
			std::cout << "Mag grid saving - ";
			output_file_name = grid_folder + SLASH + out_remag_grid_file + ".maggrid";  // save maggrid
			saveMagneticGrid(output_file_name, remagneticGrid);
			std::cout << "done." << std::endl;
		}

		std::cout << "MFP build - ";
		Fpbl::MfpBuilder *pMfp = new Fpbl::MfpBuilder();
		pDbmfp = new Fpbl::MfpBuilder::LocalDB(mag_fp);
		pMfp->buildFingerprint(remagneticGrid, pDbmfp); // build MFP
		std::cout << "done." << std::endl;

		std::cout << "MFP update with portals - ";
		Fpbl::ReturnStatus operation_result = pMfp->updateFingerprint(portalsGrid, pDbmfp); // build MFP
		std::cout << ((operation_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;
		operation_result = pMfp->updateFingerprintForCrowdsourced(csGrid, pDbmfp);
		
		std::cout << "MFP db saving - ";
		output_file_name = fp_bases_folder + SLASH + out_remag_fp_file + ".mfp" + std::to_string(mag_fp_format);  // save MFP DB
		Fpbl::ReturnStatus save_file_result = saveMagneticDB(output_file_name, *pDbmfp, mag_fp_format);
		std::cout << "done." << std::endl;
		
		output_file_name = fp_bases_folder + SLASH + out_remag_fp_file + ".mfp" + std::to_string(4);  // save MFP DB
		std::cout << "MFP db formt 4 saving - " << output_file_name << " - ";
		save_file_result = saveMagneticDB(output_file_name, *pDbmfp, 4);
		std::cout << ((save_file_result == Fpbl::ReturnStatus::STATUS_SUCCESS) ? "done" : "error") << ENDL;

		return true;
	}

	Fpbl::ReturnStatus FP_builder::loadMagneticDB(const std::string &fname, Fpbl::MfpBuilder::LocalDB &dbmfp, int mag_fp_format)
	{
		Fpbl::ReturnStatus result = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
		std::ifstream fs(fname.c_str(), std::ios::in | std::ios::binary);
		Fpbl::MfpBuilder::DBRecord rec;
		// pMfp->initializeFp(&dbmfp); TODO

		uint32_t i = 0;
		while (fs.read(reinterpret_cast<char*>(&rec), sizeof(rec))) {
			assert(i < dbmfp.getCellsCount());
			dbmfp.records[i] = rec;
			i++;
		}

		fs.close();

		if (fs.good() == true)
		{
			result = Fpbl::ReturnStatus::STATUS_SUCCESS;
		}

		return result;
	}

	size_t FP_builder::remove_small_gaps_in_mag_data(std::vector<TpnOutput> &irl_data)
	{
		// adds patching for small gaps in mag data, only if gap size is one measumerent
		size_t N = irl_data.size();
		size_t patched_data_count = 0;

		if (N >= 20)
		{
			for (size_t i = 0; i < N - 1; ++i)
			{
				// if data[i] is not valid, and data[i+1] is valid - patch it
				if ((irl_data[i].mag_meas.is_valid == false) && (irl_data[i + 1].mag_meas.is_valid == true))
				{
					irl_data[i].mag_meas.is_valid = true;
					irl_data[i].mag_meas.mX = irl_data[i + 1].mag_meas.mX;
					irl_data[i].mag_meas.mY = irl_data[i + 1].mag_meas.mY;
					irl_data[i].mag_meas.mZ = irl_data[i + 1].mag_meas.mZ;
					irl_data[i].mag_meas.sigma_mX = irl_data[i + 1].mag_meas.sigma_mX;
					irl_data[i].mag_meas.sigma_mY = irl_data[i + 1].mag_meas.sigma_mY;
					irl_data[i].mag_meas.sigma_mZ = irl_data[i + 1].mag_meas.sigma_mZ;
					irl_data[i].mag_meas.level_of_calibration = irl_data[i + 1].mag_meas.level_of_calibration;
					irl_data[i].mag_meas.covarianceMatrix[0][0] = irl_data[i + 1].mag_meas.covarianceMatrix[0][0];
					irl_data[i].mag_meas.covarianceMatrix[0][1] = irl_data[i + 1].mag_meas.covarianceMatrix[0][1];
					irl_data[i].mag_meas.covarianceMatrix[0][2] = irl_data[i + 1].mag_meas.covarianceMatrix[0][2];

					irl_data[i].mag_meas.covarianceMatrix[1][0] = irl_data[i + 1].mag_meas.covarianceMatrix[1][0];
					irl_data[i].mag_meas.covarianceMatrix[1][1] = irl_data[i + 1].mag_meas.covarianceMatrix[1][1];
					irl_data[i].mag_meas.covarianceMatrix[1][2] = irl_data[i + 1].mag_meas.covarianceMatrix[1][2];

					irl_data[i].mag_meas.covarianceMatrix[2][0] = irl_data[i + 1].mag_meas.covarianceMatrix[2][0];
					irl_data[i].mag_meas.covarianceMatrix[2][1] = irl_data[i + 1].mag_meas.covarianceMatrix[2][1];
					irl_data[i].mag_meas.covarianceMatrix[2][2] = irl_data[i + 1].mag_meas.covarianceMatrix[2][2];
					patched_data_count++;
				}
			}
		}

		return patched_data_count;
	}

} // namespace FPBuilderConsole
