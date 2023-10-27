## @package creator_bpl_ignore_list
#  This module contains functions to provide ignore list operations

## the function add item to BLP ignore list
#
# @param[in] blp_ignore_list - BLP ignore list
# @param[in] ble_prox_map - BLP database with beacons info
# @param[in] ble_prox_hash - hash of a beacon to be added
# @param[out] blp_ignore_list - updated ignore list
#
def add_to_ignore_list(blp_ignore_list, ble_prox_map, ble_prox_hash):

    ignored_beacon ={}
    for beacon in ble_prox_map:
        prox_hash = beacon[1]
        #print('prox_hash   ', prox_hash)
        if ble_prox_hash == prox_hash:
            ignored_beacon['uuid'] = beacon[0][0]
            ignored_beacon['major'] = int(beacon[0][1])
            ignored_beacon['minor'] = int(beacon[0][2])
            ignored_beacon['hash'] = prox_hash
            ignored_beacon['rejection_reason'] = 'placement'
            break

    blp_ignore_list.append(ignored_beacon)
    return blp_ignore_list

## the function add twin-items to BLP ignore list
#
# @param[in] blp_ignore_list - BLP ignore list
# @param[in] ble_prox_map - BLP database with beacons info
# @param[in] ble_twins_changed - list of twin/changed beacon hashes, changed items are not added to the list
# @param[out] blp_ignore_list - updated ignore list
#
def add_twins_to_ignore_list(blp_ignore_list, ble_prox_map, ble_twins_changed):

    for ble_twin in ble_twins_changed:
        if ble_twin[1] != 'twin':
            continue
        ble_prox_hash = ble_twin[0][0][1]
        for beacon in ble_prox_map:
            prox_hash = beacon[1]
            #print('prox_hash   ', prox_hash)
            if ble_prox_hash == prox_hash:
                ignored_beacon = {}
                ignored_beacon['uuid'] = beacon[0][0]
                ignored_beacon['major'] = int(beacon[0][1])
                ignored_beacon['minor'] = int(beacon[0][2])
                ignored_beacon['hash'] = prox_hash
                ignored_beacon['rejection_reason'] = 'twin'
                blp_ignore_list.append(ignored_beacon)
                break

    return blp_ignore_list

## the function creates dictionary to output in twin/changed log file
#
# @param[in] ble_twins_changed - list of twin/changed beacon hashes, changed items are not added to the list
# @param[out]  twin dictionary
#
def create_twins_dictionary(ble_twins_changed):

    blp_ignore_dict = []
    for ble_twin in ble_twins_changed:
        ignored_beacon1 = {}
        ignored_beacon2 = {}
        ignored_pair = {}

        ignored_beacon1['mac'] = int(ble_twin[0][0][0])
        ignored_beacon1['hash'] = ble_twin[0][0][1]
        ignored_beacon1['major'] = int(ble_twin[0][0][2])
        ignored_beacon1['minor'] = int(ble_twin[0][0][3])
        ignored_beacon1['uuid'] = ble_twin[0][0][4]
        ignored_beacon1['min_time'] = ble_twin[0][0][5]
        ignored_beacon1['max_time'] = ble_twin[0][0][6]
        ignored_beacon1['time_data'] = ble_twin[0][0][7]

        ignored_beacon2['mac'] = int(ble_twin[0][1][0])
        ignored_beacon2['hash'] = ble_twin[0][1][1]
        ignored_beacon2['major'] = int(ble_twin[0][1][2])
        ignored_beacon2['minor'] = int(ble_twin[0][1][3])
        ignored_beacon2['uuid'] = ble_twin[0][1][4]
        ignored_beacon2['min_time'] = ble_twin[0][1][5]
        ignored_beacon2['max_time'] = ble_twin[0][1][6]
        ignored_beacon2['time_data'] = ble_twin[0][1][7]

        ignored_pair['beacon1'] = ignored_beacon1
        ignored_pair['beacon2'] = ignored_beacon2
        ignored_pair['rejection_reason'] = ble_twin[1]

        blp_ignore_dict.append(ignored_pair)

    return blp_ignore_dict

## the function add beacons with incorrect tx power to BLP ignore list
#
# @param[in] blp_ignore_list - BLP ignore list
# @param[in] ble_prox_map - BLP database with beacons info
# @param[in] ble_twins_changed - list of hashes for beacons with incorrect tx power
# @param[out] blp_ignore_list - updated ignore list
#
def add_tx_incorrect_to_ignore_list(blp_ignore_list, ble_prox_map, ble_incorrect_power):

    for ble in ble_incorrect_power:
        ble_prox_hash = ble['hash']
        for beacon in ble_prox_map:
            prox_hash = beacon[1]
            #print('prox_hash   ', prox_hash)
            if ble_prox_hash == prox_hash:
                ignored_beacon = {}
                ignored_beacon['uuid'] = beacon[0][0]
                ignored_beacon['major'] = int(beacon[0][1])
                ignored_beacon['minor'] = int(beacon[0][2])
                ignored_beacon['hash'] = prox_hash
                ignored_beacon['rejection_reason'] = 'tx_setting'
                blp_ignore_list.append(ignored_beacon)
                break

    return blp_ignore_list

