## @package creator_lms_data
#  This module prepares data for lms algorithm
#
import numpy as np

# expected_distance = pow(10, attenuation / 20.);

## prepare data for lms algorithm
#
# @param[in] ble_map_with_prox_only - ble beacons with proximity data
# @param[in] ble_prox_map - ble beacons from map
# @param[in] ble_prox_hash - hash of ble beacon
# @param[in] min_bfp_cell_weight - minimal weight of non-empty mode of proximity BFP cell for using proximity BFP data in assessment
#
def create(ble_map_with_prox_only, ble_prox_map, ble_prox_hash, min_bfp_cell_weight):

    position_grid = []
    measured_distance_grid = []
    measured_level_grid = []

    tx_power = 0
    x_coor = -1
    y_coor = -1
    floor_coor = -1
    major = 0
    minor = 0
    for beacon in ble_prox_map:
        prox_hash = beacon[1]
        #print('prox_hash   ', prox_hash)
        if ble_prox_hash == prox_hash:
            x_coor = beacon[2]
            y_coor = beacon[3]
            floor_coor = beacon[4]
            tx_power = beacon[5]
            major = beacon[0][1]
            minor = beacon[0][2]
            break

    if floor_coor == -1:
        return [], 0, 0, major, minor, x_coor, y_coor

    for cell in ble_map_with_prox_only:
        list_ble = cell['list_ble']
        for ble in list_ble:
            mac = ble[0]['mac']
            if mac == ble_prox_hash:
                x = cell['x']
                y = cell['y']
                floor_ = cell['floor']
                if floor_coor != floor_:
                    continue

                if len(ble) == 2:
                    power_from_map = ble[0]['mu'] * ble[0]['w'] + ble[1]['mu'] * ble[1]['w']
                else:
                    #print(ble[0]['w'])
                    if ble[0]['w'] > min_bfp_cell_weight:
                        power_from_map = ble[0]['mu'] #+ 10.0*math.log10(ble[0]['w'])
                    else:
                        continue
                attenuation = tx_power - power_from_map
                measured_distance = 10 ** (attenuation / 20.)
                if True: #measured_distance > 2:
                    measured_distance_grid.append(measured_distance)
                    position_grid.append([x, y])
                    measured_level_grid.append(power_from_map)


    len_grid = len(measured_distance_grid)
    measured_distance = np.zeros([len_grid, 1])
    measured_level = np.zeros([len_grid, 1])
    for i in range(len_grid):
        measured_distance[i] = measured_distance_grid[i]
        measured_level[i] = measured_level_grid[i]

    return position_grid, measured_distance, measured_level, major, minor, x_coor, y_coor

