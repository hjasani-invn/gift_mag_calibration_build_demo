# Automated Survey Completeness Analyzer

import os
import json
import MFPcoverage as MFP
import BFPcoverageEx as BFP
import WFPcoverageEx as WFP
import Geo2LocalCoordinateConverter
import cell_availability_by_routes
import create_beacon_placement_area
import formgrid
import MFP_uncertainty
import WFP_uncertainty
import MFP_WFP_uncertainty
import TotalCoverage
import get_file_name
import get_linenumber
import SurveyType


def internal_floor_to_real_floor(i_floor, floor_shift, floor_zero_enable):
    real_floor = i_floor + floor_shift
    if not floor_zero_enable:
        if floor_shift < 0:
            if real_floor >= 0:
                real_floor += 1

    return real_floor


def asca(settings_json, bfp_db4_file, wfp_db4_file, mfp_db4_file, geos_dict, out_folder):
    f = open(settings_json, 'r')
    text = f.read()
    data = json.loads(text)
    venue = data['venue']
    #print (venue)

    beacon_placement_mode = 'mixed'
    if 'proximity_beacons_placement' in data:
        BP_settings = data['proximity_beacons_placement']
        if 'ASCA_beacon_placement_mode' in BP_settings:
            beacon_placement_mode = BP_settings['ASCA_beacon_placement_mode']

    origin_latitude = venue['origin_lattitude']
    origin_longitude = venue['origin_longitude']
    origin_azimuth = venue['origin_azimuth']
    alfa = venue['alfa']
    beta = venue['beta']

    magnetic_cellsize =  data['magnetic_cellsize']
    wifi_cellsize = data['wifi_cellsize']
    ble_cellsize = data['ble_cellsize']
    mag_max_x = venue['size_x']
    mag_max_y = venue['size_y']

    availability_cellsize = min(magnetic_cellsize, wifi_cellsize, ble_cellsize)

    settings = json.load(open(settings_json))
    floor_count = settings['venue']['floors_count']
    floor_shift = 1
    floor_zero_enable = False

    if 'floor_shift' in settings['venue']:
        floor_shift = settings['venue']['floor_shift']
    if 'floor_zero_enable' in settings['venue']:
        floor_zero_enable = settings['venue']['floor_zero_enable']

    #print("out_folder = ", out_folder)
    if not os.path.isdir(out_folder):
        os.mkdir(out_folder)

    # set venue parametres
    Geo2LocalCoordinateConverter.set_geo_param(origin_latitude, origin_longitude, alfa, beta, origin_azimuth)

    total_plot_file = 'total_coverage.png'
    availability_plot_file = 'cell_availability.png'
    wifi_plot_file = 'wifi_coverage.png'
    ble_plot_file = 'ble_coverage.png'
    mfp_plot_file = 'mfp_coverage.png'
    mag_quality_plot_file = 'mag_quality.png'
    total_quality_plot_file = 'total_quality.png'
    wfp_uncertainty_plot_file = 'wfp_uncertainty.png'
    mfp_wfp_uncertainty_plot_file = 'total_uncertainty.png'
    mfp_uncertainty_plot_file = 'mfp_uncertainty.png'
    survey_type_plot_file = 'survey_type.png'

    total_lat_lon_file = 'total_corners.json'
    wifi_lat_lon_file = 'wifi_corners.json'
    ble_lat_lon_file = 'ble_corners.json'
    mfp_lat_lon_file = 'mfp_corners.json'

    results = []
    gray_percents = []
    red_percents = []
    yellow_percents = []
    green_percents = []
    mfp_uncertainties = []
    wfp_uncertainties = []

    for f in range(0, floor_count):
        real_floor = internal_floor_to_real_floor(f, floor_shift, floor_zero_enable)

        try:
            geo_json_file = geos_dict[str(real_floor)]
        except Exception as ex:
            print("geojson file for floor ", real_floor, " is absent")
            print("file name is ", get_file_name.get_file_name())
            print("line is  ", get_linenumber.get_linenumber())
            print("exception code:")
            print(ex.args)
            results.append(0)
            gray_percents.append(0)
            red_percents.append(0)
            yellow_percents.append(0)
            green_percents.append(0)
            mfp_uncertainties.append(0)
            wfp_uncertainties.append(0)
            continue

        all_points_list, all_sv_points_list, all_cs_points_list = \
            cell_availability_by_routes.cell_availability_by_routes(settings_json, geo_json_file)

        availability_table, availability_sv_table, availability_cs_table, availability = \
            formgrid.formgrid(all_points_list, all_sv_points_list, all_cs_points_list,
                              mag_max_x, mag_max_y, availability_cellsize)

        #print ("=====================")
        #print ("floor = ", real_floor)

        wfp_unc, wfp_unc_smooth, wfp_uncertainty, wifi_unc_result = \
            WFP_uncertainty.get_wfp_uncertainty(wfp_db4_file, mag_max_x, mag_max_y, f, wifi_cellsize)

        if wifi_unc_result == 1:
            WFP_uncertainty.plot_wfp_uncertainty(wfp_unc_smooth, out_folder, wfp_uncertainty_plot_file, real_floor)
            create_beacon_placement_area.create_wfp_quality_image(wfp_unc_smooth, real_floor,
                                                                  wifi_cellsize, out_folder, 'BP_WFP_quality.png')
        else:
            beacon_placement_mode = 'mag'

        result_t, coverage, bf_pnum_meas_per_ap, wf_pnum_meas_per_ap, \
            mfp_coverage, mag_quality_converted, survey_type_matrix, have_mag, have_wifi, have_ble = \
            TotalCoverage.get_total_coverage(bfp_db4_file, wfp_db4_file, mfp_db4_file,
                                             settings_json, f, availability)

        if not os.path.isfile(mfp_db4_file):
            mfp_uncertainty = 0
        else:
            if result_t:
                mfp_uncertainty, mfp_unc_matrix = MFP_uncertainty.mfp_uncertainty(mfp_db4_file, settings_json, f,
                                                                                  mag_quality_converted)

                MFP_uncertainty.plot_mfp_uncertainty(mfp_unc_matrix, mfp_coverage, real_floor,
                                                     out_folder, mfp_uncertainty_plot_file)

                create_beacon_placement_area.create_bp_area(mfp_unc_matrix, f, magnetic_cellsize)

                MFP_WFP_uncertainty.plot_mfp_wfp_uncertainty(mfp_unc_matrix, mfp_coverage, wfp_unc_smooth, out_folder,
                                                             mfp_wfp_uncertainty_plot_file, real_floor, wifi_unc_result)

        if not result_t:
            results.append(0)
            gray_percents.append(0)
            red_percents.append(0)
            yellow_percents.append(0)
            green_percents.append(0)
            mfp_uncertainties.append(0)
            wfp_uncertainties.append(0)
            continue

        availability, black_percent, gray_percent, red_percent, yellow_percent, green_percent = \
            TotalCoverage.plot_total_coverage(coverage, settings_json, out_folder, total_plot_file,
                                              availability_plot_file, total_lat_lon_file, real_floor,
                                              availability_table, availability_sv_table,
                                              availability_cs_table, availability)

        TotalCoverage.create_total_coverage_json(coverage, out_folder, real_floor)

        if (red_percent + yellow_percent + green_percent) * 100 > 95 and (yellow_percent + green_percent) * 100 > 90\
                and green_percent * 100 > 60 and mfp_uncertainty < 10:
            #print ("Full Completeness")
            result = 2
        elif (red_percent + yellow_percent + green_percent) * 100 > 90 and (yellow_percent + green_percent) * 100 > 60\
                and green_percent * 100 > 20 and mfp_uncertainty < 10:
            #print ("Rough Positioning")
            result = 1
        else:
            result = 0

        if result_t:
            if os.path.isfile(mfp_db4_file):
                black_percent_m, gray_percent_m, red_percent_m, yellow_percent_m, green_percent_m = \
                    MFP.plot_mfp_coverage(mfp_coverage, settings_json, out_folder, mfp_plot_file,
                                          mfp_lat_lon_file, real_floor, availability_table)

                MFP.plot_mag_quality(mag_quality_converted, out_folder, mag_quality_plot_file, real_floor)

                TotalCoverage.plot_total_quality(mag_quality_converted, have_wifi, wf_pnum_meas_per_ap, have_ble,
                                                 bf_pnum_meas_per_ap, settings_json, out_folder,
                                                 total_quality_plot_file, real_floor)
            if os.path.isfile(wfp_db4_file):
                black_percent_w, gray_percent_w, red_percent_w, yellow_percent_w, green_percent_w = \
                    WFP.plot_wfp_coverage(wf_pnum_meas_per_ap, availability, settings_json,
                                          out_folder, wifi_plot_file, wifi_lat_lon_file, real_floor)
            if os.path.isfile(bfp_db4_file):
                black_percent_b, gray_percent_b, red_percent_b, yellow_percent_b, green_percent_b = \
                    BFP.plot_bfp_coverage(bf_pnum_meas_per_ap, availability, settings_json,
                                          out_folder, ble_plot_file, ble_lat_lon_file, real_floor)

            # TODO: add plotting of survey_type_matrix here
            SurveyType.plot_survey_type(survey_type_matrix, out_folder, survey_type_plot_file, real_floor)

        results.append(result)
        gray_percents.append(gray_percent)
        red_percents.append(red_percent)
        yellow_percents.append(yellow_percent)
        green_percents.append(green_percent)
        mfp_uncertainties.append(mfp_uncertainty)
        wfp_uncertainties.append(wfp_uncertainty)

    create_beacon_placement_area.combine_results(beacon_placement_mode, out_folder, floor_shift, floor_zero_enable)

    return results, gray_percents, red_percents, yellow_percents, green_percents, mfp_uncertainties, wfp_uncertainties
