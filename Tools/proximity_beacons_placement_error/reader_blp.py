## @package reader_blp
#  This module contains functions for reading of BLP data
#
import io
import os
import csv
import re
import Geo2LocalCoordinateConverter
import ble_hash
import numpy as np

# reading the blp4 file
def read_4format(blp4_file):
    prox_beacons_map = []
    header_size = 168
    fid = open(blp4_file, 'rb')
    mfp_header = fid.read(header_size)
    #print(len(mfp_header))
    data = fid.read()

    output_blp3 = blp4_file[:-5] + 'blp3'
    with open(output_blp3, 'wb') as binary_file:
        binary_file.write(data)

    prox_beacons_map = read(output_blp3)

    return prox_beacons_map

# reading the blp3 file
def read(blp3_file):
    uuid_mask = "[0-9,a-f,A-F]{8}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{12}"
    count = 0
    timestamp = 0
    beacons = []
    geopositions = []
    time_count = 0
    file = open(blp3_file)
    for line in file:
        reader = csv.reader(io.StringIO(line), delimiter=',')
        for beacon in reader:
            if re.match(uuid_mask, beacon[0]) != None:
                hash = ble_hash.get_ble_hash(beacon[1],  beacon[2],  beacon[0])
                #beacon.append(hash)
                x, y = Geo2LocalCoordinateConverter.Geo2Local(float(beacon[3]), float(beacon[4]))
                #beacon.append(x)
                #beacon.append(y)
                #beacon.append(int(beacon[5]))
                #beacon.append(int(beacon[6]))
                #beacons.append(beacon)
                beacons.append([beacon, hash, x, y, int(beacon[5]), int(beacon[6])])
    #print (beacons)

    return beacons
