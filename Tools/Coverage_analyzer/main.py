import json
import coordinate_converter as cc
import MFPcoverage as MFP
import BFPcoverageEx as BFP
import WFPcoverageEx as WFP
import TotalCoverage as TFP
import sys


if __name__ == "__main__":
    floor = int(sys.argv[1])
    settings_json = sys.argv[2]
    BFP_gridFile = sys.argv[3]
    WFP_gridFile = sys.argv[4]
    MFP_gridFile = sys.argv[5]
    BFP_db3_file = sys.argv[6]
    WFP_db3_file = sys.argv[7]
    out_folder = sys.argv[8]

    settings = json.load(open(settings_json))
    floor_count = settings['venue']['floors_count']

    real_floor = 1
    if floor_count == 1:
        real_floor = floor
        floor = 1

    floor -= 1  # changing number to [0, 1,...]

    if 'BLE_disabled_floors' in settings:
        BLE_disabled_floors = settings['BLE_disabled_floors']

        if floor in BLE_disabled_floors:
            BFP_db3_file = ''
            BFP_gridFile = ''


    total_plot_file = 'total_coverage.png'
    wifi_plot_file = 'wifi_coverage.png'
    ble_plot_file = 'ble_coverage.png'
    mfp_plot_file = 'mfp_coverage.png'

    total_lat_lon_file = 'total_corners.json'
    wifi_lat_lon_file = 'wifi_corners.json'
    ble_lat_lon_file = 'ble_corners.json'
    mfp_lat_lon_file = 'mfp_corners.json'

    result, coverage, BFPnumMeasPerAP, WFPnumMeasPerAP, mfp_coverage, mag_mean_number = TFP.getTotalCoverage(BFP_gridFile, BFP_db3_file, WFP_gridFile, WFP_db3_file, MFP_gridFile, settings_json, floor)

    if floor_count == 1:
        floor = real_floor - 1

    TFP.plot_Total_coverage(coverage, settings_json, out_folder, total_plot_file, total_lat_lon_file, floor)

    if result:
        BFP.plot_BFP_coverage(BFPnumMeasPerAP, settings_json, out_folder, ble_plot_file, ble_lat_lon_file, floor)
        WFP.plot_WFP_coverage(WFPnumMeasPerAP, settings_json, out_folder, wifi_plot_file, wifi_lat_lon_file, floor)
        MFP.plot_MFP_coverage(mfp_coverage, settings_json, out_folder, mfp_plot_file, mfp_lat_lon_file, mag_mean_number, floor)
