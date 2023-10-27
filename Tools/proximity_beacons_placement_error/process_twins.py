## @package process_twins
#  This module find the twins ble beacons
#
import re
import ble_hash
from datetime import datetime
import calendar

## find the beacons with identical UUID, major, minor but different MAC address
#
# @param[in] log_data - data from ble statistics log
# @param[out] ble_twins_changed - list of beacons that can be twins
#
def find_twins(log_data):

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
        #d1 = datetime.utcfromtimestamp(timestamp1)

        mac = int(log_data[i][1])
        major = int(log_data[i][2])
        minor = int(log_data[i][3])
        uuid = log_data[i][4]
        hash = ble_hash.get_ble_hash(major, minor, uuid)
        #min_time = round(float(int(log_data[i][6])) / 1000)
        #max_time = round(float(int(log_data[i][7])) / 1000)
        min_time = round(float(int(log_data[i][6])))
        max_time = round(float(int(log_data[i][7])))
        min_time += timestamp1
        max_time += timestamp1
        ble_data.append([mac, hash, major, minor, uuid, min_time, max_time, time_data_set_name, timestamp1])

    ble_data.sort(key=lambda a: a[1]) #, reverse=True)

    split_ble_data = split_by_hash_and_uuid(ble_data)

    ble_twin_candidates = find_in_split_data(split_ble_data)

    return ble_twin_candidates

    mac1 = ble_data[0][0]
    hash1 = ble_data[0][1]
    uuid1 = ble_data[0][4]
    mac2 = mac1
    hash2 = hash1
    uuid2 = uuid1

    n1 = 0
    n2 = 1
    ble_twin_candidates = []
    while n2 < len(ble_data):
        while hash2 == hash1 and uuid1 == uuid2 and n2 < len(ble_data):
            mac2 = ble_data[n2][0]
            hash2 = ble_data[n2][1]
            uuid2 = ble_data[n2][4]
            if hash2 == hash1 and mac2 == mac1 and uuid1 == uuid2:
                n2 += 1
                continue
            if hash2 == hash1 and uuid1 == uuid2 and mac2 != mac1:
                #ble_twins.append(ble_data[n1])
                #ble_twins.append(ble_data[n2])
                ble_twin_candidates.append([ble_data[n1], ble_data[n2]])
            n2 += 1

        if n2 < len(ble_data):
            mac1 = mac2
            hash1 = hash2
            uuid2 = uuid1
            n1 = n2

    return ble_twin_candidates

## find the duplicate beacons
#
# @param[in] ble_twin_candidates - list of beacons that can be twins
# @param[out] ble_twins_changed - real twins
#
def cross_twins(ble_twin_candidates):
    ble_twins_changed = []
    for i in range(len(ble_twin_candidates)):
        #print(ble_twin_candidates[i])
        tmin1 = ble_twin_candidates[i][0][5]
        tmax1 = ble_twin_candidates[i][0][6]
        tmin2 = ble_twin_candidates[i][1][5]
        tmax2 = ble_twin_candidates[i][1][6]
        if ((tmin1 >= tmin2 and tmin1 <= tmax2) or (tmax1 >= tmin2 and tmax1 <= tmax2) or
            (tmin2 >= tmin1 and tmin2 <= tmax1) or (tmax1 >= tmin2 and tmax1 <= tmax2)):
            #print('twin')
            ble_twins_changed.append([ble_twin_candidates[i], 'twin', [tmin1, tmax1, tmin2, tmax2]])
        else:
            #print('changed')
            ble_twins_changed.append([ble_twin_candidates[i], 'changed', [tmin1, tmax1, tmin2, tmax2]])

    return ble_twins_changed

# The function sorts and grous beacon scans by beacon hash
def split_by_hash_and_uuid(ble_data):
    #fid = open('yourfile.txt', 'w')
    n1 = 0
    n2 = 0
    split_ble_data = []
    length = len(ble_data)
    while n1 < length:
        hash1 = ble_data[n1][1]
        hash2 = ble_data[n2][1]
        while hash1 == hash2:
            n2 += 1
            if n2 >= length:
                break
            hash2 = ble_data[n2][1]
        ble_data_part = ble_data[n1:n2]
        #print(ble_data_part)
        ble_data_part.sort(key=lambda a: a[4]) #, reverse=True)
        #print(ble_data_part)
        m1 = 0
        m2 = 0
        length_p = len(ble_data_part)
        while m1 < length_p:
            uuid1 = ble_data_part[m1][4]
            uuid2 = ble_data_part[m2][4]
            while uuid1 == uuid2:
                m2 += 1
                if m2 >= length_p:
                    break
                uuid2 = ble_data_part[m2][4]
            ble_data_part_1 = ble_data_part[m1:m2]
            split_ble_data.append(ble_data_part_1)

            #for ble in ble_data_part_1:
            #    fid.write(str(ble) +'\n')
            #fid.write('============\n')
            #fid.flush()
            m1 = m2
        n1 = n2

    return split_ble_data

# The function search for twin candiates in splited beacon list
def find_in_split_data(split_ble_data):
    ble_twin_candidates = []
    for ble_data in split_ble_data:
        length = len(ble_data)
        for i in range(length):
            for j in range(i + 1, length):
                if ble_data[i][0] != ble_data[j][0]:
                    ble_twin_candidates.append([ble_data[i], ble_data[j]])

    return ble_twin_candidates
