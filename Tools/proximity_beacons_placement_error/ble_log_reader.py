## @package ble_log_reader
#  This module provides ble logs reading
import re
import os
import sys


## the function reads ble log file
#
# @param[in] file_name - ble log file with BLE scanning information
# @param[out] log_data - BLE scanning information list
#
def read(file_name):
    logArray = [line.rstrip('\n') for line in open(file_name)]
    log_data = []
    for i in range(len(logArray)):
        line = logArray[i]
        line1 = line.strip()
        lineSplitted = re.split(r',\s*', line1)
        grid_line_floor = float(lineSplitted[2])
        time = float(lineSplitted[5]) / 1000
        log_data.append(lineSplitted)

    return log_data