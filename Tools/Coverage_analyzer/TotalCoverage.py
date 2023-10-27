import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
import os
import numpy as np
import json
import coordinate_converter as cc
import MFPcoverage as MFP
import BFPcoverageEx as BFP
import WFPcoverageEx as WFP
import color_map as colormap
import sys


def getTotalCoverage(BFP_gridFile, BFP_db3_file, WFP_gridFile, WFP_db3_file, MFP_gridFile, settings_json, floor):
    haveBFP, BFPnumMeasPerAP = BFP.getBFPcoverage(BFP_gridFile, BFP_db3_file, settings_json, floor)
    haveWFP, WFPnumMeasPerAP = WFP.getWFPcoverage(WFP_gridFile, WFP_db3_file, settings_json, floor)
    haveMFP, mfp_coverage, mag_mean_num = MFP.getMFPcoverage(MFP_gridFile, settings_json, floor)

    settings = json.load(open(settings_json))
    mag_cellsize = settings['magnetic_cellsize']
    wifi_cellsize = settings['wifi_cellsize']
    ble_cellsize = settings['ble_cellsize']

    if not haveBFP and not haveMFP and not haveWFP:
        return False, np.zeros([1, 1]), np.zeros([1, 1]), np.zeros([1, 1]), 0, 0

    numXmag = mfp_coverage.shape[0]
    numYmag = mfp_coverage.shape[1]
    numXwifi = WFPnumMeasPerAP.shape[0]
    numYwifi = WFPnumMeasPerAP.shape[1]
    numXble = BFPnumMeasPerAP.shape[0]
    numYble = BFPnumMeasPerAP.shape[1]

    if numXmag > 1 and numYmag > 1:
        numX = numXmag
        numY = numYmag
    elif numXwifi > 1 and numYwifi > 1:
        numX = numXwifi
        numY = numYwifi
    elif numXble > 1 and numYble > 1:
        numX = numXble
        numY = numYble

    coverage = np.zeros([numX, numY])
    for i in range(0, numX):
        for j in range(0, numY):
            if i < numXmag and j < numYmag:
                mag = mfp_coverage[i, j]
                if mag > 0 and mag <= 200:
                    mag = 1
                elif mag > 200 and mag <=400:
                    mag = 2
                elif mag > 400:
                    mag = 3
                else:
                    mag = 0
            else:
                mag = 0

            x = mag_cellsize/2 + i * mag_cellsize
            y = mag_cellsize/2 + j * mag_cellsize

            float_iwfp = (x-wifi_cellsize/2)/wifi_cellsize
            float_jwfp = (y-wifi_cellsize/2)/wifi_cellsize
            check_wifi_horizontally = False
            check_wifi_vertically = False
            if float_iwfp%1 == 0.5 and numXwifi > 1:
                check_wifi_horizontally = True
            if float_jwfp%1 == 0.5 and numYwifi > 1:
                check_wifi_vertically = True

            iwfp = round((x-wifi_cellsize/2)/wifi_cellsize)
            jwfp = round((y-wifi_cellsize/2)/wifi_cellsize)

            iwfp_left = iwfp
            iwfp_right = iwfp
            jwfp_top = jwfp
            jwfp_bottom = jwfp

            if check_wifi_horizontally:
                iwfp_left = round((x-0.0001-wifi_cellsize/2)/wifi_cellsize)
                iwfp_right = round((x+0.0001-wifi_cellsize/2)/wifi_cellsize)
            if check_wifi_vertically:
                jwfp_top = round((y+0.0001-wifi_cellsize/2)/wifi_cellsize)
                jwfp_bottom = round((y-0.0001-wifi_cellsize/2)/wifi_cellsize)

            if iwfp_right < numXwifi and jwfp_top < numYwifi:
                # selecting the best Wi-Fi cell
                max_wifi = max(WFPnumMeasPerAP[iwfp_left, jwfp_top], WFPnumMeasPerAP[iwfp_left, jwfp_bottom],
                               WFPnumMeasPerAP[iwfp_right, jwfp_top], WFPnumMeasPerAP[iwfp_right, jwfp_bottom])
                wifi = max_wifi
                if wifi > 0 and wifi <= 4:
                    wifi = 1
                elif wifi > 4 and wifi <= 25:
                    wifi = 2
                elif wifi > 25:
                    wifi = 3
                else:
                    wifi = 0
            else:
                wifi = 0

            float_ibfp = (x - ble_cellsize / 2) / ble_cellsize
            float_jbfp = (y - ble_cellsize / 2) / ble_cellsize
            check_ble_horizontally = False
            check_ble_vertically = False
            if float_ibfp % 1 == 0.5 and numXble > 1:
                check_ble_horizontally = True
            if float_jbfp % 1 == 0.5 and numYble > 1:
                check_ble_vertically = True

            ibfp = round((x - ble_cellsize / 2) / ble_cellsize)
            jbfp = round((y - ble_cellsize / 2) / ble_cellsize)

            ibfp_left = ibfp
            ibfp_right = ibfp
            jbfp_top = jbfp
            jbfp_bottom = jbfp

            if check_ble_horizontally:
                ibfp_left = round((x - 0.0001 - ble_cellsize / 2) / ble_cellsize)
                ibfp_right = round((x + 0.0001 - ble_cellsize / 2) / ble_cellsize)
            if check_ble_vertically:
                jbfp_top = round((y + 0.0001 - ble_cellsize / 2) / ble_cellsize)
                jbfp_bottom = round((y - 0.0001 - ble_cellsize / 2) / ble_cellsize)

            if ibfp_right < numXble and jbfp_top < numYble:
                # selecting the best BLE cell
                max_ble = max(BFPnumMeasPerAP[ibfp_left, jbfp_top], BFPnumMeasPerAP[ibfp_left, jbfp_bottom],
                              BFPnumMeasPerAP[ibfp_right, jbfp_top], BFPnumMeasPerAP[ibfp_right, jbfp_bottom])
                ble = max_ble
                if ble > 0 and ble <= 4:
                    ble = 1
                elif ble > 4 and ble <= 25:
                    ble = 2
                elif ble > 25:
                    ble = 3
                else:
                    ble = 0
            else:
                ble = 0

            num = []
            if haveMFP:
                num.append(mag)
            if haveWFP:
                num.append(wifi)
            if haveBFP:
                num.append(ble)

            min_num = min(num)
            coverage[i, j] = min_num
            # adjusting the colors for more pleasant look
            if coverage[i, j] == 1:
                coverage[i, j] = 0.8
            if coverage[i, j] == 0:
                coverage[i, j] = 0.2

    return True, coverage, BFPnumMeasPerAP, WFPnumMeasPerAP, mfp_coverage, mag_mean_num


def plot_Total_coverage(coverage, settings_json, output_directory, output_plot_file_name, output_lat_lon_file, floor):
    settings = json.load(open(settings_json))
    cellSize = settings['magnetic_cellsize']
    maxX = coverage.shape[0]
    maxY = coverage.shape[1]
    # adding floor number to output file names
    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(floor+1) + output_plot_file_name[-4:]
    output_lat_lon_file = output_lat_lon_file[:-5] + '_' + str(floor+1) + output_lat_lon_file[-5:]

    A = colormap.get_colormap()
    the_colormap = mpl.colors.ListedColormap(A / 255.0)

    outpath = os.path.join(output_directory, output_plot_file_name)
    plt.pcolor(coverage.transpose(), cmap=the_colormap, vmin=0, vmax=3, edgecolors='k', linewidths=0.3)
    plt.axis("off")
    plt.subplots_adjust(bottom=0)
    plt.subplots_adjust(top=1)
    plt.subplots_adjust(right=1)
    plt.subplots_adjust(left=0)
    plt.savefig(outpath, pad_inches=0, dpi=200)
    plt.close()

    # calculate lat-lon for overlays

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
