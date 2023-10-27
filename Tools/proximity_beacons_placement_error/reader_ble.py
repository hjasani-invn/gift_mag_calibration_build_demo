## @package reader_blp
#  This module contains functions for reading of BLP data
#
import io
import csv
import re
import convert_ble4_to_ble3

#remove empty strings from strings list
def remove_empty_str(list):
    new_list = []
    for str in list:
        if str != '':
            new_list.append(str)

    return new_list

# reading the blp4 file
def read_4format(ble4_file, RSSI_cutoff_threshold):
    ble3_file = convert_ble4_to_ble3.convert(ble4_file)
    ble_map = read(ble3_file, RSSI_cutoff_threshold)
    return ble_map

# reading the blp3 file
def read(ble3_file, RSSI_cutoff_threshold):
    position_mask = "POINT"
    ble_mask = "AP_MAC"
    count = 0
    #timestamp = 0
    ble_map = []
    cell = {}
    list_ble = []
    #time_count = 0
    file = open(ble3_file)
    for line in file:
        reader = csv.reader(io.StringIO(line), delimiter=' ')
        for beacon in reader:
            if len(beacon) > 0:
                beacon = remove_empty_str(beacon)
                if re.match(position_mask, beacon[0]) != None:
                    if len(cell) > 0:
                        if len(list_ble) > 0:
                            cell['list_ble'] = list_ble
                            ble_map.append(cell.copy())

                    list_ble = []
                    cell['x'] = float(beacon[1])
                    cell['y'] = float(beacon[2])
                    cell['floor'] = float(beacon[3])
                    #print (beacon)
                if re.match(ble_mask, beacon[0]) != None:
                    mac = int(beacon[1])
                    w1 = float(beacon[2])
                    mu1 = float(beacon[3])
                    sig1 = float(beacon[4])
                    w2 = float(beacon[5])
                    mu2 = float(beacon[6])
                    sig2 = float(beacon[7])

                    ble = []
                    ble_tmp = {}

                    ble_tmp['mac'] = mac
                    ble_tmp['w'] = w1
                    ble_tmp['mu'] = mu1
                    ble_tmp['sig'] = sig1
                    ble.append(ble_tmp.copy())

                    ble_tmp['mac'] = mac
                    ble_tmp['w'] = w2
                    ble_tmp['mu'] = mu2
                    ble_tmp['sig'] = sig2
                    ble.append(ble_tmp.copy())

                    list_ble.append(ble.copy())

    if len(cell) > 0:
        if len(list_ble) > 0:
            cell['list_ble'] = list_ble
            ble_map.append(cell.copy())

    #for cell in ble_map:
    #    print(cell)

    return ble_map

# removes non proximity beacons from scanning list using BLE proximity DB
def remove_not_prox(map_ble, prox_beacons, RSSI_cutoff_threshold):

    ble_prox_only = map_ble.copy()
    #for ble in ble_prox_only:
    #    print(ble)
    #print('1 ===========')

    for cell in ble_prox_only:
        list_ble = cell['list_ble']
        length = len(list_ble)
        for j in range(length):
            i = length - j - 1
            ble = list_ble[i]
            mac = ble[0]['mac']

            # find  mac in prox_beacons
            find_success = False
            for prox_beacon in prox_beacons:
                prox_hash = prox_beacon[1]
                #print('hash   ', prox_hash)
                if mac == prox_hash:
                    find_success = True
                    txPower = prox_beacon[5]
                    #print('hash   ', prox_hash)
                    #print('txPower   ', txPower)

                    if not (ble[1]['w'] > 0 and ble[1]['mu'] > (txPower + RSSI_cutoff_threshold) and ble[1]['mu'] < 0):
                        ble.pop(1)
                    if not (ble[0]['w'] > 0 and ble[0]['mu'] > (txPower + RSSI_cutoff_threshold) and ble[0]['mu'] < 0):
                        ble.pop(0)
                    break

            if not find_success or (len(ble) == 0):
                list_ble.pop(i)

    #for ble in ble_prox_only:
    #    print(ble)
    #print('2 ===========')

    length = len(ble_prox_only)
    for j in range(length):
        i = length - j - 1
        cell = ble_prox_only[i]
        list_ble = cell['list_ble']
        if len(list_ble) == 0:
            ble_prox_only.pop(i)

    #for ble in ble_prox_only:
    #    print(ble)
    #print('3 ===========')

    return ble_prox_only
