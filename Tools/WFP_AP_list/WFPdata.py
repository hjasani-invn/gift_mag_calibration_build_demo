import matplotlib.pyplot as plt
import os
import numpy as np
import re
import json

def unique(myList):
    unique_list = []

    # traverse for all elements
    for x in myList:
        y = mac2str(x) # mac address as int int string
        # check if exists in unique_list or not
        if y not in unique_list:
            unique_list.append(y)

    return unique_list


def mac2uint(mac_str):
    mac = mac_str.replace(":", "")
    return int(mac, 16)

def mac2str(mac):
    str_ = '{:>02X}'.format((mac >> 48) & 0xFF) + ":"
    str_ = str_ + '{:>02X}'.format((mac >> 40) & 0xFF) + ":"
    str_ = str_ + '{:>02X}'.format((mac >> 32) & 0xFF) + ":"
    str_ = str_ + '{:>02X}'.format((mac >> 24) & 0xFF) + ":"
    str_ = str_ + '{:>02X}'.format((mac >> 16) & 0xFF) + ":"
    str_ = str_ + '{:>02X}'.format((mac >> 8) & 0xFF) + ":"
    str_ = str_ + '{:>02X}'.format((mac >> 0) & 0xFF)

    #print(mac, '  ', str_)
    return str_

def get(has_wifi_db3, db3_file, has_wifi_db4, db4_file, floor):

    #wifi3 format
    if has_wifi_db3:
        has_wifi_db4 = False # one WFP list only        

        db3 = [line.rstrip('\n') for line in open(db3_file)]
        APs = []

        #curfloor = -1
        FlagPOINT = False
        for i in range(len(db3)):
            if re.match(r'POINT.*', db3[i]):
                #curfloor = int(float((db3[i].split()[3])))
                if FlagPOINT:
                    break
                else:
                    FlagPOINT = True
            #if curfloor == floor:
            if re.match(r'AP_MAC.*', db3[i]):
                AP = int(db3[i].split()[1])
                APs.append(AP)

        AP3List = unique(APs)
    else:
        AP3List = []

    #wifi4 format
    if has_wifi_db4:
        fid = open(db4_file, 'rb')
        wifi_header = fid.read(186)
        data_array = np.fromfile(fid, dtype=np.byte)

        str_ = ''
        db4 = []
        FlagPOINT = False
        for by in data_array:
            str_ = str_ + '{:>1c}'.format(by)
            if '{:>1c}'.format(by) == '\n':
                #str_.rstrip('\r')
                if re.match(r'POINT.*', str_):
                    if FlagPOINT:
                        break
                    else:
                        FlagPOINT = True
                db4.append(str_)
                str_ = ''

        APs = []

        curfloor = -1
        for i in range(len(db4)):
            if re.match(r'POINT.*', db4[i]):
                curfloor = int(float((db4[i].split()[3])))

            if curfloor == floor:
                if re.match(r'AP_MAC.*', db4[i]):
                    AP = int(db4[i].split()[1])
                    APs.append(AP)

        AP4List = unique(APs)
    else:
        AP4List = []

    return AP3List, AP4List


