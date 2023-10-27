import matplotlib.pyplot as plt
import os
import numpy as np
import re
import json
import coordinate_converter as cc


def unique(myList):
    unique_list = []

    # traverse for all elements
    for x in myList:
        # check if exists in unique_list or not
        if x not in unique_list:
            unique_list.append(x)

    return unique_list


def mac2uint(mac_str):
    mac = mac_str.replace(":", "")
    return int(mac, 16)


def getBFPcoverage(gridFile, db3_file, settings_json, floor):

    if not os.path.isfile(gridFile):
        return False, np.zeros([1, 1])
    elif os.path.getsize(gridFile) < 5:
        return False, np.zeros([1, 1])
    if not os.path.isfile(db3_file):
        return False, np.zeros([1, 1])
    elif os.path.getsize(db3_file) < 5:
        return False, np.zeros([1, 1])

    settings = json.load(open(settings_json))
    cellSize = settings['ble_cellsize']

    gridArray = [line.rstrip('\n') for line in open(gridFile)]

    # parsing database file
    db3 = [line.rstrip('\n') for line in open(db3_file)]
    APs = []

    curfloor = -1
    for i in range(len(db3)):
        if re.match(r'POINT.*', db3[i]):
            curfloor = int(float((db3[i].split()[3])))

        if curfloor == floor:
            if re.match(r'AP_MAC.*', db3[i]):
                AP = int(db3[i].split()[1])
                APs.append(AP)

    if len(APs) == 0:
        return False, np.zeros([1, 1])

    APList = unique(APs)

    # parsing grid file
    xt = []
    yt = []
    ft = []
    ct = []
    rssi = []

    for i in range(len(gridArray)):
        line = gridArray[i]
        lineSplitted = re.split(r'\t+', line)
        grid_line_floor = float(lineSplitted[2])
        if grid_line_floor == floor:
            xt.append(float(lineSplitted[0]))
            yt.append(float(lineSplitted[1]))
            ft.append(float(lineSplitted[2]))
            ct.append(int(lineSplitted[3]))
            rssi.append(float(lineSplitted[4]))

    if len(rssi) == 0:
        return False, np.zeros([1, 1])

    maxX = int(settings['venue']['size_x'])
    maxY = int(settings['venue']['size_y'])

    x1 = xt[0]
    y1 = yt[0]
    bssid = []
    numX = round((maxX - cellSize / 2) / cellSize) + 1
    numY = round((maxY - cellSize / 2) / cellSize) + 1
    numMeas = np.zeros([numX, numY])
    NumAP = np.zeros([numX, numY])
    numMeasPerAP = np.zeros([numX, numY])
    AverLevel = np.zeros([numX, numY])
    MaxLevel = -100 * np.ones([numX, numY])

    for i in range(len(xt)):
        mX = round((xt[i] - cellSize / 2) / cellSize)
        mY = round((yt[i] - cellSize / 2) / cellSize)
        macStr = (ct[i])

        try:
            ind = APList.index(macStr)
        except ValueError:
            ind = -1

        if ind != -1 and ct[i] != '00:00:00:00:00:00:00' and ct[i] != '0':
            numMeas[mX, mY] = numMeas[mX, mY] + 1
            AverLevel[mX, mY] = AverLevel[mX, mY] + rssi[i]
            MaxLevel[mX, mY] = max(MaxLevel[mX, mY], rssi[i])

            if xt[i] == x1 and yt[i] == y1:
                bssid.append(ct[i])
            else:
                q = sorted(bssid)
                LenBssID = len(q)
                if LenBssID > 1:
                    b1 = q[0]
                    NumAP[mX, mY] = 1
                    for j in range(1, LenBssID):
                        if q[j] != b1:
                            NumAP[mX, mY] = NumAP[mX, mY] + 1
                            b1 = q[j]

                x1 = xt[i]
                y1 = yt[i]
                bssid = []

    for m in range(numX):
        for n in range(numY):
            if numMeas[m, n] > 0:
                if AverLevel[m, n] != 0:
                    AverLevel[m, n] = AverLevel[m, n] / numMeas[m, n]
                else:
                    AverLevel[m, n] = -100

            else:
                AverLevel[m, n] = -100

            if NumAP[m, n] > 0:
                numMeasPerAP[m, n] = numMeas[m, n]/NumAP[m, n]

    return True, numMeasPerAP


def plot_BFP_coverage(numMeasPerAP, settings_json, output_directory, output_plot_file_name, output_lat_lon_file, floor):
    settings = json.load(open(settings_json))
    maxX = numMeasPerAP.shape[0]
    maxY = numMeasPerAP.shape[1]
    cellSize = settings['ble_cellsize']

    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(floor+1) + output_plot_file_name[-4:]
    output_lat_lon_file = output_lat_lon_file[:-5] + '_' + str(floor+1) + output_lat_lon_file[-5:]

    plt.pcolor(numMeasPerAP.transpose(), cmap='jet', vmin=0, vmax=50, edgecolors='k', linewidths=0.1)
    plt.title('Amount of BLE measurements per beacon')
    plt.colorbar()
    outpath = os.path.join(output_directory, output_plot_file_name)
    plt.savefig(outpath, pad_inches=0, dpi=200)
    plt.close()

    # coordinates of corners for overlay

    lat0 = settings['venue']['origin_lattitude']
    lon0 = settings['venue']['origin_longitude']
    x1 = maxX * cellSize
    y1 = 0
    x2 = maxX * cellSize
    y2 = maxY * cellSize
    x3 = 0
    y3 = maxY * cellSize

    H = settings['venue']['origin_azimuth']

    lat1, lon1 = cc.local2geo(lat0, lon0, H, x1, y1)
    lat2, lon2 = cc.local2geo(lat0, lon0, H, x2, y2)
    lat3, lon3 = cc.local2geo(lat0, lon0, H, x3, y3)
    
	# Output coordinates in the JSON format and write to a file
    output_lat_lon_file_name = os.path.join(output_directory, output_lat_lon_file)
    out_lat_lon = open(output_lat_lon_file_name, 'w')
    coordinates_json = {"coordinates":{"p1":{"lat": lat0, "lng": lon0 },"p2":{ "lat": lat1, "lng": lon1 },"p3": { "lat": lat2, "lng": lon2 }, "p4":{"lat": lat3, "lng": lon3}}}
    with open(output_lat_lon_file_name, 'w') as f:
        json.dump(coordinates_json, f)


