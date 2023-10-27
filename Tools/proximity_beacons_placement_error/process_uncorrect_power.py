## @package process_uncorrect_power
#  This module find the ble beacons with incorrect tx power
#
import re
import ble_hash
from datetime import datetime
import calendar

## find the beacons with identical UUID, major, minor but different MAC address
#
# @param[in] log_data - data from ble statistics log
# @param[in] ble_prox_map - ble beacons from map
# @param[out] ble_uncorrect_txpower_without_duplicates - beacons for that real tx power is different from value from map
#
def process(log_data, ble_prox_map):

    #print(ble_prox_map)
    ble_data = []
    for i in range(len(log_data) - 1):
        data_set_name = log_data[i][0]
        match = re.search(r'\d\d\d\d_\d\d_\d\d_\d\d_\d\d_\d\d', data_set_name)
        match1 = re.search(r'\d\d\d\d_\d\d_\d\d', data_set_name)
        time_data_set_name = data_set_name[match.start():match.end()]
        data_for_set = data_set_name[match1.start():match1.end()]
        data_for_set1 = data_for_set.replace('_', '-')
        time_for_set = data_set_name[match1.end()+1:match.end()]
        time_for_set1 = time_for_set.replace('_', ':')
        date_time_for_set = data_for_set1 + ' ' + time_for_set1
        d = datetime.fromisoformat(date_time_for_set)
        timestamp1 = calendar.timegm(d.timetuple())

        mac = int(log_data[i][1])
        major = int(log_data[i][2])
        minor = int(log_data[i][3])
        uuid = log_data[i][4]
        txpower = log_data[i][8]
        hash = ble_hash.get_ble_hash(major, minor, uuid)
        min_time = round(float(int(log_data[i][6])))
        max_time = round(float(int(log_data[i][7])))
        min_time += timestamp1
        max_time += timestamp1
        ble_data.append([mac, hash, major, minor, uuid, min_time, max_time, time_data_set_name, timestamp1, txpower])

    #ble_data.sort(key=lambda a: a[1])

    ble_uncorrect_txpower = []
    for ble in ble_data:
        for beacon in ble_prox_map:
            prox_hash = beacon[1]
            prox_uuid = beacon[0][0]
            prox_txpower = beacon[5]
            if ble[1] == prox_hash and ble[4].upper() == prox_uuid.upper() and int(ble[9]) != prox_txpower:
                ignored_beacon = {}
                ignored_beacon['uuid'] = ble[4]
                ignored_beacon['major'] = int(ble[2])
                ignored_beacon['minor'] = int(ble[3])
                ignored_beacon['hash'] = int(ble[1])
                ignored_beacon['mac'] = int(ble[0])
                ignored_beacon['ble_tx_power'] = int(ble[9])
                ignored_beacon['tx_power_from_map'] = int(beacon[5])

                ble_uncorrect_txpower.append(ignored_beacon)
                #ble_uncorrect_txpower.append([ble, beacon])

    # remove duplicates
    ble_uncorrect_txpower_without_duplicates = []
    for i in ble_uncorrect_txpower:
        if i not in ble_uncorrect_txpower_without_duplicates:
            ble_uncorrect_txpower_without_duplicates.append(i)

    return ble_uncorrect_txpower_without_duplicates

