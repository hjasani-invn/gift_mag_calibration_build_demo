import os
import numpy as np
import re
import json
import Geo2LocalCoordinateConverter
import color_map as colormap
import math
import plot_images


def get_bfp_coverage_colormap():
    # blue, red, yellow, green
    a = np.array([[140, 212, 252],
                  [255, 0, 0],
                  [255, 255, 0],
                  [0, 255, 0]])

    return a


def mac2uint(mac_str):
    mac = mac_str.replace(":", "")
    return int(mac, 16)


def one_cell_convert_bfp_coverage(num_meas):
    threshold_red = 15
    threshold_yellow = 25

    if num_meas <= 0:
        res = 0

    elif num_meas <= threshold_red:
        res = 1
    elif num_meas <= threshold_yellow:
        res = 2
    else:
        res = 3

    return res


def convert_bfp_coverage(m):
    m_conv = np.zeros([m.shape[0], m.shape[1]])
    for i in range(0, m.shape[0]):
        for j in range(0, m.shape[1]):
            m_conv[i, j] = one_cell_convert_bfp_coverage(m[i, j])

    return m_conv


def get_bfp_coverage(db4_file, settings_json, floor):
    settings = json.load(open(settings_json))

    cell_size = settings['ble_cellsize']
    max_x = settings['venue']['size_x']
    max_y = settings['venue']['size_y']
    num_x = int(math.ceil(max_x / cell_size))
    num_y = int(math.ceil(max_y / cell_size))

    if not os.path.isfile(db4_file):
        return False, np.zeros([num_x, num_y])
    elif os.path.getsize(db4_file) < 5:
        return False, np.zeros([num_x, num_y])

    # parsing database file

    bfp_lines = []

    with open(db4_file, 'rb') as bfp:
        data = bfp.read()
        point_location = data.find(b'POINT')

        if point_location < 0:
            return False, np.zeros([num_x, num_y])

        cut_data = data[point_location:]
        ble4_header = data[:point_location]

        text_ble_fp = cut_data.decode('utf-8')
        bfp_lines = text_ble_fp.splitlines(False)

    access_points = []

    curfloor = -5000
    cur_i = 0
    cur_j = 0
    num_meas = np.zeros([num_x, num_y])
    num_ap = np.zeros([num_x, num_y])
    num_meas_per_ap = np.zeros([num_x, num_y])

    for i in range(len(bfp_lines)):
        if re.match(r'POINT.*', bfp_lines[i]):
            curfloor = int(float((bfp_lines[i].split()[3])))
            cur_x = float((bfp_lines[i].split()[1]))
            cur_y = float((bfp_lines[i].split()[2]))

            cur_i = int(round((cur_x - cell_size / 2) / cell_size))
            cur_j = int(round((cur_y - cell_size / 2) / cell_size))

        if curfloor == floor:
            if re.match(r'AP_MAC.*', bfp_lines[i]):
                ap = int(bfp_lines[i].split()[1])
                access_points.append(ap)

                if len(bfp_lines[i].split()) < 9:
                    print('old BFP format detected, this version of ASCA does not work with it')

                meas_count = int(float(bfp_lines[i].split()[8]))
                if meas_count > 0:
                    num_ap[cur_i][cur_j] += 1

                num_meas[cur_i][cur_j] += meas_count

    if len(access_points) == 0:
        return False, np.zeros([1, 1])

    for i in range(0, num_x):
        for j in range(0, num_y):
            if num_ap[i][j] > 0:
                num_meas_per_ap[i][j] = num_meas[i][j] / num_ap[i][j]

    return True, num_meas_per_ap


def plot_bfp_coverage(num_meas_per_ap, availability, settings_json, output_directory,
                      output_plot_file_name, output_lat_lon_file, real_floor):
    settings = json.load(open(settings_json))
    num_x_ble = num_meas_per_ap.shape[0]
    num_y_ble = num_meas_per_ap.shape[1]
    ble_cellsize = settings['ble_cellsize']
    mag_cellsize = settings['magnetic_cellsize']
    num_x_avail = availability.shape[0]
    num_y_avail = availability.shape[1]

    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]
    output_lat_lon_file = output_lat_lon_file[:-5] + '_' + str(real_floor) + output_lat_lon_file[-5:]

    bfp_coverage_converted = convert_bfp_coverage(num_meas_per_ap.transpose())
    outpath = os.path.join(output_directory, output_plot_file_name)
    bfp_coverage_cmap = get_bfp_coverage_colormap()

    plot_images.draw_picture(bfp_coverage_converted, bfp_coverage_cmap, outpath)

    n = 0
    m_black = 0
    m_gray = 0  # Gray
    m_red = 0  # Red
    m_yellow = 0  # Yellow
    m_green = 0  # Green
    for i in range(0, num_x_avail):
        for j in range(0, num_y_avail):
            value = availability[i, j]
            # if value != None:
            if value == colormap.grey_color:
                n += 1
                x = mag_cellsize / 2 + i * mag_cellsize
                y = mag_cellsize / 2 + j * mag_cellsize
                float_iwfp = (x - ble_cellsize / 2) / ble_cellsize
                float_jwfp = (y - ble_cellsize / 2) / ble_cellsize
                check_wifi_horizontally = False
                check_wifi_vertically = False
                if float_iwfp % 1 == 0.5 and num_x_ble > 1:
                    check_wifi_horizontally = True
                if float_jwfp % 1 == 0.5 and num_y_ble > 1:
                    check_wifi_vertically = True

                iwfp = int(round((x - ble_cellsize / 2) / ble_cellsize))
                jwfp = int(round((y - ble_cellsize / 2) / ble_cellsize))

                iwfp_left = iwfp
                iwfp_right = iwfp
                jwfp_top = jwfp
                jwfp_bottom = jwfp

                if check_wifi_horizontally:
                    iwfp_left = int(round((x - 0.0001 - ble_cellsize / 2) / ble_cellsize))
                    iwfp_right = int(round((x + 0.0001 - ble_cellsize / 2) / ble_cellsize))
                if check_wifi_vertically:
                    jwfp_top = int(round((y + 0.0001 - ble_cellsize / 2) / ble_cellsize))
                    jwfp_bottom = int(round((y - 0.0001 - ble_cellsize / 2) / ble_cellsize))

                if iwfp_right < num_x_ble and jwfp_top < num_y_ble:
                    # selecting the best Wi-Fi cell
                    max_wifi = max(num_meas_per_ap[iwfp_left, jwfp_top], num_meas_per_ap[iwfp_left, jwfp_bottom],
                                   num_meas_per_ap[iwfp_right, jwfp_top], num_meas_per_ap[iwfp_right, jwfp_bottom])
                    wifi = max_wifi
                    # print("wifi = ", wifi)
                    if 0 < wifi <= 4:
                        m_gray += 1
                    elif 4 < wifi <= 15:
                        m_red += 1
                    elif 15 < wifi <= 25:
                        m_yellow += 1
                    elif wifi > 25:
                        m_green += 1
                    else:
                        m_black += 1

    black_percent = 0
    gray_percent = 0
    red_percent = 0
    yellow_percent = 0
    green_percent = 0

    if n > 0:
        black_percent = m_black / n
        gray_percent = m_gray / n
        red_percent = m_red / n
        yellow_percent = m_yellow / n
        green_percent = m_green / n

    # coordinates of corners for overlay

    lat0 = settings['venue']['origin_lattitude']
    lon0 = settings['venue']['origin_longitude']
    x1 = num_x_ble * ble_cellsize
    y1 = 0
    x2 = num_x_ble * ble_cellsize
    y2 = num_y_ble * ble_cellsize
    x3 = 0
    y3 = num_y_ble * ble_cellsize

    lat1, lon1 = Geo2LocalCoordinateConverter.local2geo(x1, y1)
    lat2, lon2 = Geo2LocalCoordinateConverter.local2geo(x2, y2)
    lat3, lon3 = Geo2LocalCoordinateConverter.local2geo(x3, y3)

    # Output coordinates in the JSON format and write to a file
    output_lat_lon_file_name = os.path.join(output_directory, output_lat_lon_file)
    coordinates_json = {"coordinates": {"p1": {"lat": lat0, "lng": lon0}, "p2": {"lat": lat1, "lng": lon1},
                                        "p3": {"lat": lat2, "lng": lon2}, "p4": {"lat": lat3, "lng": lon3}}}
    with open(output_lat_lon_file_name, 'w') as f:
        json.dump(coordinates_json, f)
    return black_percent, gray_percent, red_percent, yellow_percent, green_percent
