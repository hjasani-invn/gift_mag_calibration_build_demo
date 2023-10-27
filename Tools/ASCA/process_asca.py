import os
import re
import json
import ASCA
import maps_list_to_dict
import create_empty_grid


def process_asca(out_folder, input_file_list):
    version = "1.2.0"
    print("ASCA tool, version ", version)

    try:
        os.makedirs(out_folder)
    except OSError:
        print("The directory %s exist" % out_folder)
    else:
        print("Successfully created the directory %s" % out_folder)

    blebase_pattern = '(.*\.ble4)'
    wifibase_pattern = '(.*\.wifi4)'
    magbase_pattern = '(.*\.mfp4)'

    settings_pattern = '(.*venue\.json)'

    map_pattern = '(.*map-?\d+\.geojson)'

    matched_file_list = [f for f in input_file_list if re.match(settings_pattern, f) is not None]
    if len(matched_file_list) > 0:
        settings_json = matched_file_list[0]
    else:
        settings_json = ""
        # print('ASCA failed to find venue.json file in the input folder. Please provide a valid venue.json file.')
        # raise SystemExit

    matched_file_list = [f for f in input_file_list if re.match(blebase_pattern, f) is not None]
    if len(matched_file_list) > 0:
        bfp_db4_file = matched_file_list[0]
    else:
        bfp_db4_file = ""

    matched_file_list = [f for f in input_file_list if re.match(wifibase_pattern, f) is not None]
    if len(matched_file_list) > 0:
        wfp_db4_file = matched_file_list[0]
    else:
        wfp_db4_file = ""

    matched_file_list = [f for f in input_file_list if re.match(magbase_pattern, f) is not None]
    if len(matched_file_list) > 0:
        mfp_db4_file = matched_file_list[0]
    else:
        mfp_db4_file = ""
        # print('ASCA failed to find MFP4 database file in the input folder. Please provide a valid MFP4 file.')
        # return 'ASCA processing failed'

    maps_files = [f for f in input_file_list if re.match(map_pattern, f) is not None]

    geos_dict = maps_list_to_dict.maps_list_to_dict(".", maps_files)

    finish_flag = False

    if not os.path.isfile(bfp_db4_file):
        print('BLE base is not found')
    if not os.path.isfile(wfp_db4_file):
        print('WiFi base is not found')
    if not os.path.isfile(mfp_db4_file):
        print('MFP base is not found, ASCA can\'t continue')
        finish_flag = True

    # create empty grid
    #if not os.path.isdir(out_folder):
    #    os.mkdir(out_folder)

    print('settings file is ', settings_json)
    if not os.path.isfile(settings_json):
        print('settings file is not found, ASCA can\'t continue')
        finish_flag = True
    else:
        if not finish_flag:
            create_empty_grid.create_empty_grid(settings_json, out_folder, 'empty_grid.png', 'empty_grid_coordinates.json')

    if finish_flag:
        print('Finish')
        return 'ASCA processing failed'
        # raise SystemExit

    results, gray_percents, red_percents, yellow_percents, green_percents, mfp_uncertainties, wfp_uncertainties = \
        ASCA.asca(settings_json, bfp_db4_file, wfp_db4_file, mfp_db4_file, geos_dict, out_folder)

    settings = json.load(open(settings_json))
    floor_count = settings['venue']['floors_count']
    floor_shift = 1
    floor_zero_enable = False

    if 'floor_shift' in settings['venue']:
        floor_shift = settings['venue']['floor_shift']
    if 'floor_zero_enable' in settings['venue']:
        floor_zero_enable = settings['venue']['floor_zero_enable']

    for f in range(0, floor_count):
        real_floor = ASCA.internal_floor_to_real_floor(f, floor_shift, floor_zero_enable)

        print("=====================")
        print("floor = ", real_floor)

        print("")
        if results[f] == 2:
            print("Fine Positioning")
        elif results[f] == 1:
            print("Rough Positioning")
        else:
            print("No FingerPrint")
        print("red_percent +  yellow_percent + green_percent = ", (red_percents[f] +
                                                                   yellow_percents[f] + green_percents[f]) * 100)
        print("yellow_percent + green_percent = ", (yellow_percents[f] + green_percents[f]) * 100)
        print("green_percent = ", (green_percents[f]) * 100)
        print("MFP_uncertainty = ", mfp_uncertainties[f])
        print("WFP_uncertainty = ", wfp_uncertainties[f])

    quality = 2
    for r in results:
        print('one floor result:', r)
        if r < quality:
            quality = r

    return 'ASCA processing succeeded'
