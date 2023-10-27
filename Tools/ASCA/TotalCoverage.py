import matplotlib as mpl
import os
import numpy as np
import json
from tkinter import *
from PIL import Image
import MFPcoverage as MFP
import BFPcoverageEx as BFP
import WFPcoverageEx as WFP
import color_map as colormap
import Geo2LocalCoordinateConverter
import plot_images
mpl.use('Agg')


def modification_color_squares(
        img_blue,
        img_gray,
        img_red,
        img_yellow,
        img_green,
        cell_size):  # cell size in pixels

    width = int(cell_size / 10)  # width of light/dark line in pixels
    if width <= 2:
        width = 3

    a = np.asarray(img_blue)
    b = a.copy()
    for i in range(0, len(a)):
        for j in range(0, len(a[i])):
            n = (i % width)
            if n <= int(width / 2):
                b[i, j] = [0, 0, 210, 255]
            # print(b[i, j])

    img_blue1 = Image.fromarray(b)

    a = np.asarray(img_gray)
    b = a.copy()
    for i in range(0, len(a)):
        for j in range(0, len(a[i])):
            n = (i % width)
            if n <= int(width / 2):
                b[i, j] = [60, 60, 0, 255]
            # print(b[i, j])

    img_gray1 = Image.fromarray(b)

    a = np.asarray(img_red)
    b = a.copy()
    for i in range(0, len(a)):
        for j in range(0, len(a[i])):
            n = (i % width)
            if n <= int(width / 2):
                b[i, j] = [210, 0, 0, 255]
            # print(b[i, j])

    img_red1 = Image.fromarray(b)

    a = np.asarray(img_yellow)
    b = a.copy()
    for i in range(0, len(a)):
        for j in range(0, len(a[i])):
            n = (i % width)
            if n <= int(width / 2):
                b[i, j] = [210, 210, 0, 255]
            # print(b[i, j])

    img_yellow1 = Image.fromarray(b)

    a = np.asarray(img_green)
    b = a.copy()
    for i in range(0, len(a)):
        for j in range(0, len(a[i])):
            # if (i > 0) and (j > 0):
            # n=(abs(i-j) % 4)
            n = (i % width)
            if n <= int(width / 2):
                # if i == j:
                b[i, j] = [0, 210, 0, 255]
            # print(b[i, j])

    img_green1 = Image.fromarray(b)

    return img_blue1, img_gray1, img_red1, img_yellow1, img_green1


def get_total_coverage(bfp_db4_file, wfp_db4_file, mfp_db4_file, settings_json, floor, availability):
    settings = json.load(open(settings_json))

    if 'BLE_disabled_floors' in settings:
        ble_disabled_floors = settings['BLE_disabled_floors']

        if floor in ble_disabled_floors:
            bfp_db4_file = ''

    have_mfp, mfp_coverage, mag_quality_converted, survey_type_matrix = \
        MFP.get_mfp_coverage(mfp_db4_file, settings_json, floor, availability)
    have_wfp, wf_pnum_meas_per_ap, wf_pnum_ap = WFP.get_wfp_coverage(wfp_db4_file, settings_json, floor)
    have_bfp, bf_pnum_meas_per_ap = BFP.get_bfp_coverage(bfp_db4_file, settings_json, floor)

    mag_cellsize = settings['magnetic_cellsize']
    wifi_cellsize = settings['wifi_cellsize']
    ble_cellsize = settings['ble_cellsize']

    num_x_mag = int(mfp_coverage.shape[0])
    num_y_mag = int(mfp_coverage.shape[1])
    num_x_wifi = int(wf_pnum_meas_per_ap.shape[0])
    num_y_wifi = int(wf_pnum_meas_per_ap.shape[1])
    num_x_ble = int(bf_pnum_meas_per_ap.shape[0])
    num_y_ble = int(bf_pnum_meas_per_ap.shape[1])

    num_x = max(num_x_mag, num_x_wifi, num_x_ble)
    num_y = max(num_y_mag, num_y_wifi, num_y_ble)
    coverage = np.zeros([num_x, num_y])

    if not have_bfp and not have_mfp and not have_wfp:
        return False, coverage, bf_pnum_meas_per_ap, wf_pnum_meas_per_ap, mfp_coverage, \
               mag_quality_converted, have_mfp, have_wfp, have_bfp

    if not have_mfp:
        mag_cellsize = wifi_cellsize
        if not have_wfp:
            mag_cellsize = ble_cellsize

    for i in range(0, num_x):
        for j in range(0, num_y):
            if i < num_x_mag and j < num_y_mag:
                mag = mfp_coverage[i, j]
                if 30 >= mag > 0:
                    mag = 1.5
                elif 200 >= mag > 30:
                    mag = 2.5
                elif 400 >= mag > 200:
                    mag = 3.5
                elif mag > 400:
                    mag = 4.5
                else:
                    mag = 0.5
            else:
                mag = 0

            x = mag_cellsize / 2 + i * mag_cellsize
            y = mag_cellsize / 2 + j * mag_cellsize

            float_iwfp = (x - wifi_cellsize / 2) / wifi_cellsize
            float_jwfp = (y - wifi_cellsize / 2) / wifi_cellsize
            check_wifi_horizontally = False
            check_wifi_vertically = False
            if float_iwfp % 1 == 0.5 and num_x_wifi > 1:
                check_wifi_horizontally = True
            if float_jwfp % 1 == 0.5 and num_y_wifi > 1:
                check_wifi_vertically = True

            iwfp = int(round((x - wifi_cellsize / 2) / wifi_cellsize))
            jwfp = int(round((y - wifi_cellsize / 2) / wifi_cellsize))

            iwfp_left = iwfp
            iwfp_right = iwfp
            jwfp_top = jwfp
            jwfp_bottom = jwfp

            if check_wifi_horizontally:
                iwfp_left = int(round((x - 0.0001 - wifi_cellsize / 2) / wifi_cellsize))
                iwfp_right = int(round((x + 0.0001 - wifi_cellsize / 2) / wifi_cellsize))
            if check_wifi_vertically:
                jwfp_top = int(round((y + 0.0001 - wifi_cellsize / 2) / wifi_cellsize))
                jwfp_bottom = int(round((y - 0.0001 - wifi_cellsize / 2) / wifi_cellsize))

            if iwfp_right < num_x_wifi and jwfp_top < num_y_wifi:
                # selecting the best Wi-Fi cell
                max_wifi = max(wf_pnum_meas_per_ap[iwfp_left, jwfp_top], wf_pnum_meas_per_ap[iwfp_left, jwfp_bottom],
                               wf_pnum_meas_per_ap[iwfp_right, jwfp_top], wf_pnum_meas_per_ap[iwfp_right, jwfp_bottom])
                wifi = max_wifi
                if 4 >= wifi > 0:
                    wifi = 1.5
                elif 15 >= wifi > 4:
                    wifi = 2.5
                elif 25 >= wifi > 15:
                    wifi = 3.5
                elif wifi > 25:
                    wifi = 4.5
                else:
                    wifi = 0
            else:
                wifi = 0

            float_ibfp = (x - ble_cellsize / 2) / ble_cellsize
            float_jbfp = (y - ble_cellsize / 2) / ble_cellsize
            check_ble_horizontally = False
            check_ble_vertically = False
            if float_ibfp % 1 == 0.5 and num_x_ble > 1:
                check_ble_horizontally = True
            if float_jbfp % 1 == 0.5 and num_y_ble > 1:
                check_ble_vertically = True

            ibfp = int(round((x - ble_cellsize / 2) / ble_cellsize))
            jbfp = int(round((y - ble_cellsize / 2) / ble_cellsize))

            ibfp_left = ibfp
            ibfp_right = ibfp
            jbfp_top = jbfp
            jbfp_bottom = jbfp

            if check_ble_horizontally:
                ibfp_left = int(round((x - 0.0001 - ble_cellsize / 2) / ble_cellsize))
                ibfp_right = int(round((x + 0.0001 - ble_cellsize / 2) / ble_cellsize))
            if check_ble_vertically:
                jbfp_top = int(round((y + 0.0001 - ble_cellsize / 2) / ble_cellsize))
                jbfp_bottom = int(round((y - 0.0001 - ble_cellsize / 2) / ble_cellsize))

            if ibfp_right < num_x_ble and jbfp_top < num_y_ble:
                # selecting the best BLE cell
                max_ble = max(bf_pnum_meas_per_ap[ibfp_left, jbfp_top], bf_pnum_meas_per_ap[ibfp_left, jbfp_bottom],
                              bf_pnum_meas_per_ap[ibfp_right, jbfp_top], bf_pnum_meas_per_ap[ibfp_right, jbfp_bottom])
                ble = max_ble
                if 4 >= ble > 0:
                    ble = 1.5
                elif 6 >= ble > 4:
                    ble = 2.5
                elif 10 >= ble > 6:
                    ble = 3.5
                elif ble > 10:
                    ble = 4.5
                else:
                    ble = 0
            else:
                ble = 0

            num = []
            if have_mfp:
                num.append(mag)
            if have_wfp:
                num.append(wifi)
            if have_bfp:
                num.append(ble)

            min_num = min(num)
            coverage[i, j] = min_num

    return True, coverage, bf_pnum_meas_per_ap, \
        wf_pnum_meas_per_ap, mfp_coverage, mag_quality_converted, survey_type_matrix, have_mfp, have_wfp, have_bfp


def plot_total_coverage(coverage, settings_json, output_directory, output_plot_file_name, availability_plot_file,
                        output_lat_lon_file, real_floor, availability_table, availability_sv_table,
                        availability_cs_table, availability):
    settings = json.load(open(settings_json))
    cell_size = settings['magnetic_cellsize']
    max_cov_x = int(coverage.shape[0])
    max_cov_y = int(coverage.shape[1])
    max_x = int(availability.shape[0])
    max_y = int(availability.shape[1])
    print("max_x = ", max_x, "  max_y = ", max_y, "  max_cov_x = ", max_cov_x, "  max_cov_y = ", max_cov_y)
    # adding floor number to output file names
    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]
    output_lat_lon_file = output_lat_lon_file[:-5] + '_' + str(real_floor) + output_lat_lon_file[-5:]

    availability_plot_file = availability_plot_file[:-4] + '_' + str(real_floor) + availability_plot_file[-4:]

    a = colormap.get_colormap()
    outpath = os.path.join(output_directory, output_plot_file_name)
    plot_images.draw_picture(coverage.transpose(), a, outpath, availability_cs_table)

    # availability = np.zeros([max_x, max_y])
    n = 0
    m_black = 0
    m_gray = 0  # Gray
    m_red = 0  # Red
    m_yellow = 0  # Yellow
    m_green = 0  # Green

    for i in range(0, max_x):
        # print("i =  ",i)
        for j in range(0, max_y):
            # print("j =  ",j)
            # print(i, "   ",j, "   ",availability[i,j])
            # index_sum = j * max_x + i
            # value = availability_table.get(index_sum)
            value = availability[i, j]
            # if value != None:
            if value != 0:
                # print(i, "   ",j, "   ",value)
                # value = availability[i, j]
                # availability[i,j] = 4.5
                n += 1
                if coverage[i, j] == 0.5:
                    m_black += 1
                elif coverage[i, j] == 1.5:
                    m_gray += 1
                elif coverage[i, j] == 2.5:
                    m_red += 1
                elif coverage[i, j] == 3.5:
                    m_yellow += 1
                elif coverage[i, j] == 4.5:
                    m_green += 1
            # else:
            # availability[i,j] = 0
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

    outpath = os.path.join(output_directory, availability_plot_file)
    conv_availability = convert_availability(availability.transpose())
    plot_images.draw_picture(conv_availability, a, outpath)

    # calculate lat-lon for overlays

    lat0 = settings['venue']['origin_lattitude']
    lon0 = settings['venue']['origin_longitude']
    x1 = max_x * cell_size
    y1 = 0
    x2 = max_x * cell_size
    y2 = max_y * cell_size
    x3 = 0
    y3 = max_y * cell_size

    lat1, lon1 = Geo2LocalCoordinateConverter.local2geo(x1, y1)
    lat2, lon2 = Geo2LocalCoordinateConverter.local2geo(x2, y2)
    lat3, lon3 = Geo2LocalCoordinateConverter.local2geo(x3, y3)

    # Output coordinates in the JSON format and write to a file
    output_lat_lon_file_name = os.path.join(output_directory, output_lat_lon_file)
    coordinates_json = {"coordinates": {"p1": {"lat": lat0, "lng": lon0}, "p2": {"lat": lat1, "lng": lon1},
                                        "p3": {"lat": lat2, "lng": lon2}, "p4": {"lat": lat3, "lng": lon3}}}
    with open(output_lat_lon_file_name, 'w') as f:
        json.dump(coordinates_json, f)
    return availability, black_percent, gray_percent, red_percent, yellow_percent, green_percent


def plot_total_quality(mag_quality, have_wifi, WFPnumMeasPerAP, have_ble, BFPnumMeasPerAP, settings_json,
                       output_directory, output_plot_file_name, real_floor):
    settings = json.load(open(settings_json))
    mag_cell_size = settings['magnetic_cellsize']
    wifi_cell_size = settings['wifi_cellsize']
    ble_cell_size = settings['ble_cellsize']
    koef_wifi = wifi_cell_size / mag_cell_size
    koef_ble = ble_cell_size / mag_cell_size
    max_x = int(mag_quality.shape[0])
    max_y = int(mag_quality.shape[1])

    wf_pnum_meas_per_ap_min = 20
    bf_pnum_meas_per_ap_min = 20

    # print(koef_wifi, '  ', koef_ble )
    if have_wifi:
        for i in range(0, max_x):
            for j in range(0, max_y):
                n = int(i / koef_wifi)
                m = int(j / koef_wifi)
                # print (i, ' ', j, ' ', n, ' ', m )
                if WFPnumMeasPerAP[n, m] < wf_pnum_meas_per_ap_min and mag_quality[i, j] > 0:
                    mag_quality[i, j] = colormap.red_color

    if have_ble:
        for i in range(0, max_x):
            for j in range(0, max_y):
                n = int(i / koef_wifi)
                m = int(j / koef_wifi)
                if BFPnumMeasPerAP[n, m] < bf_pnum_meas_per_ap_min and mag_quality[i, j] > 0:
                    mag_quality[i, j] = colormap.red_color

    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]

    outpath = os.path.join(output_directory, output_plot_file_name)
    # Start plotting
    a = get_mag_quality_colormap()

    conv_mag_quality = convert_total_quality(mag_quality.transpose())

    plot_images.draw_picture(conv_mag_quality, a, outpath)


def get_mag_quality_colormap():
    # blue, grey, green, yellow, red
    a = np.array([[140, 212, 252],
                  [100, 100, 100],
                  [0, 255, 0],
                  [255, 255, 0],
                  [255, 0, 0]])

    return a


def one_cell_convert_total_quality(element):
    threshold_yellow = 15
    threshold_grey = -3
    threshold_green = 5

    if element < threshold_grey:
        res = 0

    elif element < 0:
        res = 1
    elif element <= threshold_green:
        res = 2
    elif element <= threshold_yellow:
        res = 3
    else:
        res = 4

    return res


def convert_total_quality(m):
    m_conv = np.zeros([m.shape[0], m.shape[1]])
    for i in range(0, m.shape[0]):
        for j in range(0, m.shape[1]):
            m_conv[i, j] = one_cell_convert_total_quality(m[i, j])

    return m_conv


def one_cell_convert_availability(element):
    threshold_green = -3

    if element < threshold_green:
        res = 0

    else:
        res = 1

    return res


def convert_availability(m):
    m_conv = np.zeros([m.shape[0], m.shape[1]])
    for i in range(0, m.shape[0]):
        for j in range(0, m.shape[1]):
            m_conv[i, j] = one_cell_convert_availability(m[i, j])

    return m_conv


def convert_coverage_value(value):
    result = 0.0
    if value == 1.5:
        result = 0.0
    elif value == 2.5:
        result = 0.25
    elif value == 3.5:
        result = 0.5
    elif value == 4.5:
        result = 1.0

    return result


def create_total_coverage_json(coverage, out_folder, real_floor):
    output_json_file_name = 'cell_information_' + str(real_floor) + '.json'
    out_path = os.path.join(out_folder, output_json_file_name)

    max_coverage_x = int(coverage.shape[0])
    max_coverage_y = int(coverage.shape[1])

    out_list = []

    for i in range(0, max_coverage_x):
        for j in range(0, max_coverage_y):
            coverage_value = convert_coverage_value(coverage[i][j])
            if coverage_value > 0.0:
                d = {"ix": i, "iy": j, "coverage": coverage_value}
                out_list.append(d)

    json_object = {"cell_list": out_list}
    json.dump(json_object, open(out_path, 'w'), sort_keys=True, indent=4)