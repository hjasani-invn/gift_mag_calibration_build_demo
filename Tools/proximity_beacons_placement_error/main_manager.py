## @ \main_manager
#  startup file
#
import os
import sys
import json
import argparse
import logging
import reader_blp
import reader_ble
import Geo2LocalCoordinateConverter
import dict_merge
import incorrect_beacon_position
import ble_log_reader
import process_twins
import creator_bpl_ignore_list
import process_uncorrect_power

sys.path.insert(0, '..')

if __name__ == "__main__":
    version = "1.0.0"

    try:
        print("Incorrect beacons assesment script, version ", version)

        if sys.platform == "linux" or sys.platform == "linux2":
            # linux
            print('OS: Linux')
        elif sys.platform == "win32":
            # Windows...
            print('OS: Windows')

        parser = argparse.ArgumentParser(description="beacons place tool parameters")

        parser.add_argument("-sv", "--settings_venue", help="json file with venue settings")
        parser.add_argument("-se", "--settings_extra", help="json file with extra settings")
        parser.add_argument("-blp", "--blp_db", help="blp4 proximity beacons data base")
        parser.add_argument("-ble", "--ble_db", help="ble4 fingerprint built for proximity beacons")
        parser.add_argument("-blpl", "--blp_log", help="proximity beacons log file")
        parser.add_argument("-blpi", "--output_ignore_list", help="output proximity beacons ignored list")

        args = parser.parse_args()  # exception with unknown parameters
        #args, unknown = parser.parse_known_args() # ignore unknown parameters

        settings_venue = args.settings_venue
        settings_extra = args.settings_extra
        blp4_file = args.blp_db
        ble4_file = args.ble_db
        blp_log = args.blp_log
        blp_ignored_file = args.output_ignore_list
        output_path = os.path.dirname(blp_ignored_file)

        placement_calc_error = True
        twins_calc = True
        tx_power_error = True

        print(settings_venue)
        print(settings_extra)
        print(blp4_file)
        print(ble4_file)
        print(blp_ignored_file)
        print(blp_log)

        print(os.getcwd())
        # reading extra settings
        if settings_extra == None:
            data_extra = {}
        else:
            f = open(settings_extra, 'r')
            text = f.read()
            data_extra = json.loads(text)

        # reading main settings
        f = open(settings_venue, 'r')
        text = f.read()
        data_main = json.loads(text)

        data = dict_merge.merge(data_main, data_extra)

        # default  incorrect_beacon_detection
        max_position_intolerance = 5
        min_cells_number = 1
        max_cells_number = 50
        max_iter_number = 200
        RSSI_cutoff_threshold = -23
        min_bfp_cell_weight = 0.1
        distance_to_rms = 3

        if 'incorrect_beacon_detection' in data:
            incorrect_beacon_detection = data['incorrect_beacon_detection']
            if 'max_position_intolerance' in incorrect_beacon_detection:
                max_position_intolerance = incorrect_beacon_detection['max_position_intolerance']
            if 'min_cells_number' in incorrect_beacon_detection:
                min_cells_number = incorrect_beacon_detection['min_cells_number']
            if 'max_cells_number' in incorrect_beacon_detection:
                max_cells_number = incorrect_beacon_detection['max_cells_number']
            if 'max_iterations_number' in incorrect_beacon_detection:
                max_iter_number = incorrect_beacon_detection['max_iterations_number']
            if 'RSSI_cutoff_threshold' in incorrect_beacon_detection:
                RSSI_cutoff_threshold = incorrect_beacon_detection['RSSI_cutoff_threshold']
            if 'min_bfp_cell_weight' in incorrect_beacon_detection:
                min_bfp_cell_weight = incorrect_beacon_detection['min_bfp_cell_weight']
            if 'distance_to_rms' in incorrect_beacon_detection:
                distance_to_rms = incorrect_beacon_detection['distance_to_rms']
            if 'blp_position_check_enable' in incorrect_beacon_detection:
                placement_calc_error = incorrect_beacon_detection['blp_position_check_enable']
            if 'twin_detection_enable' in incorrect_beacon_detection:
                twins_calc = incorrect_beacon_detection['twin_detection_enable']
            if 'tx_missmatch_check_enable' in incorrect_beacon_detection:
                tx_power_error = incorrect_beacon_detection['tx_missmatch_check_enable']

        if blp_log == None:
            twins_calc = False
            tx_power_error = False

        # reading venue parameters
        venue = data['venue']

        # venue sizes
        size_x = venue['size_x']
        size_y = venue['size_y']
        # ble_cellsize = venue['ble_cellsize']

        # print(size_x, size_y)

        # geographical binding
        origin_lattitude = venue['origin_lattitude']
        origin_longitude = venue['origin_longitude']
        origin_azimuth = venue['origin_azimuth']
        alfa = venue['alfa']
        beta = venue['beta']

        # set venue parametres
        Geo2LocalCoordinateConverter.SetGeoParam(origin_lattitude, origin_longitude, alfa, beta, origin_azimuth)

        prox_beacons = reader_blp.read_4format(blp4_file)

        all_beacons = reader_ble.read_4format(ble4_file, RSSI_cutoff_threshold)

        ble_prox_only = reader_ble.remove_not_prox(all_beacons, prox_beacons, RSSI_cutoff_threshold)

        if placement_calc_error:
            f_incorrect_beacons = open(os.path.join(output_path, 'incorrect_position_beacons_log.csv'), 'w')
            str1 = ('    major, ' + '    minor, ' + '   ble_prox_hash, ' + '      x_calc,    ' + '   y_calc, ' +
                  'x_expected, ' + 'y_expected,  ' + '    distance, ' +
                  'number of iterations, ' + 'number of cells,' +
                  'tx power error,' + '  rms' + ' , ' + 'assessment' + ' , ' + 'shift of position' + '\n\n')
            f_incorrect_beacons.write(str1)

            blp_ignore_list = incorrect_beacon_position.prepare(ble_prox_only, prox_beacons,
                                                                min_cells_number, max_cells_number,
                                                                min_bfp_cell_weight, max_iter_number,
                                                                distance_to_rms, f_incorrect_beacons)
            f_incorrect_beacons.close()
        else:
            blp_ignore_list = []

        if twins_calc:
            log_data = ble_log_reader.read(blp_log)
            ble_twin_candidates = process_twins.find_twins(log_data)
            ble_twins_changed = process_twins.cross_twins(ble_twin_candidates)

            blp_ignore_list = creator_bpl_ignore_list.add_twins_to_ignore_list(blp_ignore_list, prox_beacons,
                                                                               ble_twins_changed)

            ble_twins_changed_dict = creator_bpl_ignore_list.create_twins_dictionary(ble_twins_changed)
            blp_twins_dict = {'ble_twins_changed': ble_twins_changed_dict}
            json_t_data = json.dumps(blp_twins_dict, sort_keys=False, indent=4, separators=(',', ': '))
            fw = open(os.path.join(output_path, 'ble_twins_changed.json'), 'wt')
            fw.write(json_t_data)
            fw.close()

        if tx_power_error:
            ble_uncorrect_power = process_uncorrect_power.process(log_data, prox_beacons)
            blp_tx_setting_dict = {'ble_tx_setting': ble_uncorrect_power}
            json_p_data = json.dumps(blp_tx_setting_dict, sort_keys=False, indent=4, separators=(',', ': '))
            fw = open(os.path.join(output_path, 'blp_tx_setting.json'), 'wt')
            fw.write(json_p_data)
            fw.close()

            blp_ignore_list = creator_bpl_ignore_list.add_tx_incorrect_to_ignore_list(blp_ignore_list, prox_beacons,
                                                                                      ble_uncorrect_power)

        # remove duplicates
        blp_ignore_list_without_duplicates = []
        for i in blp_ignore_list:
            if i not in blp_ignore_list_without_duplicates:
                blp_ignore_list_without_duplicates.append(i)

        blp_ignored_dict = {'blp_ignore_list_autogenerated': blp_ignore_list_without_duplicates}

        json_i_data = json.dumps(blp_ignored_dict, sort_keys=False, indent=4, separators=(',', ': '))
        fw = open(blp_ignored_file, 'wt')
        fw.write(json_i_data)
        fw.close()

    except Exception as ex:
        print("Exception happened. Exception code:", ex.args)

        log_file_name = 'pbpe_error_log' + '.txt'
        logging.basicConfig(filename=log_file_name, level=logging.INFO,
                            format='%(asctime)s %(levelname)s %(name)s %(message)s')
        logger = logging.getLogger(__name__)
        logger.error(ex, exc_info=True)

        raise SystemExit
