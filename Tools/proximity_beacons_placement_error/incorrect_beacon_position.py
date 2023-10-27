## @package incorrect_beacon_position
#  This module prepare blp ignore list
#
import math
import creator_bpl_ignore_list
import creator_lms_data
import lms_for_level


## prepare blp ignore list
#
# @param[in] ble_prox_only - ble beacons with proximity data
# @param[in] prox_beacons - ble beacons from map
# @param[in] min_cells_number - min cells number that must be
# @param[in] max_cells_number - max cells number that must be
# @param[in] min_bfp_cell_weight - minimal weight of non-empty mode of proximity BFP cell for using proximity BFP data in assessment
# @param[in] max_iter_number - max iteration number of lms process
# @param[in] distance_to_rms - minimal distance to rms relation to solve that beacon is shifted-
# @param[in] f_incorrect_beacons - output file
#
def prepare(ble_prox_only, prox_beacons, min_cells_number, max_cells_number, min_bfp_cell_weight,
            max_iter_number, distance_to_rms, f_incorrect_beacons):
    ble_prox_hashes = []

    for cell in ble_prox_only:
        list_ble = cell['list_ble']
        if len(list_ble) > -1:
            # print(cell)
            for ble in list_ble:
                ble_prox_hash = ble[-1]['mac']
                if ble_prox_hash not in ble_prox_hashes:
                    ble_prox_hashes.append(ble_prox_hash)

    ble_prox_hashes.sort()
    blp_ignore_list = []
    for ble_prox_hash in ble_prox_hashes:
        position_grid, measured_distance, measured_level, major, minor, x_expected, y_expected = \
            creator_lms_data.create(ble_prox_only, prox_beacons, ble_prox_hash, min_bfp_cell_weight)

        tx_power = -77
        tx_power_err = 0
        x0 = x_expected
        y0 = y_expected
        min_delta = 0.01

        #print(len(position_grid))
        if len(position_grid) >= 1: # len(position_grid) <= max_cells_number:
            # LMS
            x_calc, y_calc, delta_x, delta_y, tx_error, delta_tx_error, rms, iter_number, lms_success = \
                lms_for_level.calculate_using_level(position_grid, measured_level,
                                                    tx_power, x0, y0, tx_power_err, 10*max_iter_number, min_delta)

            #print(lms_success)
            distance = math.sqrt((x_expected - x_calc) ** 2 + (y_expected - y_calc) ** 2)

            beacon_is_assessed  = False
            if lms_success and (iter_number <= max_iter_number) and (len(position_grid) >= min_cells_number):
                beacon_is_assessed = True

            beacon_is_shifted = False
            if ((distance / (1 * rms)) > distance_to_rms) and beacon_is_assessed:
                beacon_is_shifted = True

            str1 = ('{:>10}'.format(str(major)) + ', ' + '{:>8}'.format(str(minor)) + ', ' +
                    '{:>14}'.format(str(ble_prox_hash)) + ', ' +
                    '{:14.2f}'.format(x_calc) + ', ' + '{:12.2f}'.format(y_calc) + ', ' +
                    '{:9.2f}'.format(x_expected) + ', ' +
                    '{:7.2f}'.format(y_expected) + ', ' + '{:14.2f}'.format(distance) + ', ' +
                    '{:>14}'.format(str(iter_number)) + ', ' +
                    '{:>15}'.format(str(len(position_grid))) + ', ' + '{:17.2f}'.format(tx_error) + ', ' +
                    '{:10.2f}'.format(rms) + ',      ' +
                    str(beacon_is_assessed) + ',      ' + str(beacon_is_shifted) + '\n')
            f_incorrect_beacons.write(str1)

            if beacon_is_assessed and beacon_is_shifted:  # and (distance > max_position_intolerance):
                blp_ignore_list = creator_bpl_ignore_list.add_to_ignore_list(blp_ignore_list, prox_beacons,
                                                                             ble_prox_hash)
    return blp_ignore_list
