## @package reader_blp
#  This module contains functions for reading of BLP data
#
import io
import os
import csv
import re
import reader_blp3
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

    prox_beacons_map = reader_blp3.read(output_blp3)

    return prox_beacons_map

