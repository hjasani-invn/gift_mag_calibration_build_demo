import matplotlib.pyplot as plt
import os
import numpy as np
import json
import coordinate_converter as cc


def getMFPcoverage(gridFile, settings_json, floor):
    settings = json.load(open(settings_json))
    cellSize = settings['magnetic_cellsize']

    if not os.path.isfile(gridFile):
        return False, np.zeros([1, 1]), 0
    elif os.path.getsize(gridFile) < 5:
        return False, np.zeros([1, 1]), 0

    GridArray = np.loadtxt(gridFile)

    dimX = int(settings['venue']['size_x'])
    dimY = int(settings['venue']['size_y'])

    maxY = round(dimY / cellSize + 0.5)
    maxX = round(dimX / cellSize + 0.5)

    cover = (-50) * np.ones([maxX, maxY])
    xyh = GridArray[:, 0:3]
    ih = np.where(GridArray[:, [2]] == floor)
    xy = xyh[ih[0], 0:2]
    lenXY = len(xy)

    if lenXY == 0:
        return False, np.zeros([1, 1]), 0

    k = 0
    m = 0
    xc = xy[0, 0]
    yc = xy[0, 1]

    while k < lenXY:
        if xc == xy[k, 0] and yc == xy[k, 1]:
            m = m + 1
            k = k + 1
        else:
            i = int(round(xc / cellSize + 0.5)) - 1
            j = int(round(yc / cellSize + 0.5)) - 1

            cover[i, j] = m

            m = 0
            xc = xy[k, 0]
            yc = xy[k, 1]

    mean_num = 0
    k = 0
    for i in range(0, maxX):
        for j in range(0, maxY):
            if cover[i, j] > 0:
                mean_num = mean_num + cover[i, j]
                k = k + 1

    if k > 0:
        mean_num = mean_num / k

    return True, cover, mean_num


def plot_MFP_coverage(cover, settings_json, output_directory, output_plot_file_name, output_lat_lon_file, mean_num, floor):
    settings = json.load(open(settings_json))
    cellSize = settings['magnetic_cellsize']
    maxX = cover.shape[0]
    maxY = cover.shape[1]

    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(floor+1) + output_plot_file_name[-4:]
    output_lat_lon_file = output_lat_lon_file[:-5] + '_' + str(floor+1) + output_lat_lon_file[-5:]

    outpath = os.path.join(output_directory, output_plot_file_name)
    # Start plotting
    plt.pcolor(cover.transpose(), cmap='jet', vmin=0, vmax=400, edgecolors='k', linewidths=0.1)
    plt.title('Mean mag measurements per cell: ' + str(int(round(mean_num))))
    plt.colorbar()
    plt.savefig(outpath, format='png', pad_inches=0, dpi=400)
    plt.close()

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

