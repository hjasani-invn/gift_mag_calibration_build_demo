import matplotlib as mpl
import os
import numpy as np
import json
import Geo2LocalCoordinateConverter
import color_map as colormap
import plot_images
import math
import LoadMFP4
mpl.use('Agg')


def get_mfp_coverage_colormap():
    # blue, red, yellow, green
    a = np.array([[140, 212, 252],
                  [255, 0, 0],
                  [255, 255, 0],
                  [0, 255, 0]])

    return a


def one_cell_convert_mfp_coverage(num_meas):
    threshold_red = 200
    threshold_yellow = 400

    if num_meas < 0:
        res = 0

    elif num_meas <= threshold_red:
        res = 1
    elif num_meas <= threshold_yellow:
        res = 2
    else:
        res = 3

    return res


def convert_mfp_coverage(M):
    m_conv = np.zeros([M.shape[0], M.shape[1]])
    for i in range(0, M.shape[0]):
        for j in range(0, M.shape[1]):
            m_conv[i, j] = one_cell_convert_mfp_coverage(M[i, j])

    return m_conv


def get_mfp_coverage(mfp4_file, settings_json, floor, availability):

    settings = json.load(open(settings_json))

    cell_size = settings['magnetic_cellsize']
    dim_x = settings['venue']['size_x']
    dim_y = settings['venue']['size_y']

    max_x = int(math.ceil(dim_x / cell_size))
    max_y = int(math.ceil(dim_y / cell_size))

    if not os.path.isfile(mfp4_file):
        return False, np.zeros([max_x, max_y]), 0, colormap.blue_color * np.ones([max_x, max_y])

    mag_quality_converted = availability.copy()

    mfp, sig, mag_quality, mag_coverage, survey_type_matrix = LoadMFP4.load_mfp4(mfp4_file, dim_x, dim_y,
                                                                                 cell_size, floor)

    for i in range(0, mag_quality_converted.shape[0]):
        for j in range(0, mag_quality_converted.shape[1]):
            if mag_quality[i][j][0] > 0.0 or mag_quality[i][j][1] > 0.0 or mag_quality[i][j][2] > 0.0:
                worst_quality = max(mag_quality[i][j][0], mag_quality[i][j][1], mag_quality[i][j][2])
                mag_quality_converted[i][j] = min(worst_quality, 20.0)
                if mag_coverage[i][j] < 200:
                    mag_quality_converted[i][j] = -3
                if mag_coverage[i][j] > 5000:
                    mag_quality_converted[i][j] = 5

    return True, mag_coverage, mag_quality_converted, survey_type_matrix


def plot_mfp_coverage(mfp_coverage, settings_json, output_directory, output_plot_file_name, output_lat_lon_file, 
                      real_floor, availability_table):
    settings = json.load(open(settings_json))
    cell_size = settings['magnetic_cellsize']
    max_x = int(mfp_coverage.shape[0])
    max_y = int(mfp_coverage.shape[1])

    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]
    output_lat_lon_file = output_lat_lon_file[:-5] + '_' + str(real_floor) + output_lat_lon_file[-5:]

    mfp_coverage_converted = convert_mfp_coverage(mfp_coverage.transpose())
    outpath = os.path.join(output_directory, output_plot_file_name)
    mfp_coverage_cmap = get_mfp_coverage_colormap()

    plot_images.draw_picture(mfp_coverage_converted, mfp_coverage_cmap, outpath)
    # adding text output for mag coverage (percentage of gray, red, yellow and green cells)
    text_mag_coverage_out = outpath[:-4] + '.txt'
    plot_images.write_stats_for_picture(mfp_coverage_converted, mfp_coverage_cmap, text_mag_coverage_out)

    n = 0
    m_black = 0
    m_gray = 0  # Gray
    m_red = 0   # Red
    m_yellow = 0   # Yellow
    m_green = 0   # Green
    for i in range(0, max_x):
        for j in range(0, max_y):
            index_sum = j * max_x + i
            value = availability_table.get(index_sum)
            if value is not None:
                n += 1
                mag = mfp_coverage[i, j]
                if mag == 0:
                    m_black += 1
                elif 0 < mag <= 30:
                    m_gray += 1
                elif 30 < mag <= 200:
                    m_red += 1
                elif 200 < mag <= 400:
                    m_yellow += 1
                elif mag > 400:
                    m_green += 1
    
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
    coordinates_json = {"coordinates": {"p1": {"lat": lat0, "lng": lon0},
                                        "p2": {"lat": lat1, "lng": lon1},
                                        "p3": {"lat": lat2, "lng": lon2},
                                        "p4": {"lat": lat3, "lng": lon3}}}

    with open(output_lat_lon_file_name, 'w') as f:
        json.dump(coordinates_json, f)
    return black_percent, gray_percent, red_percent, yellow_percent, green_percent


def get_mag_quality_colormap():
    # blue, grey, green, yellow, red
    a = np.array([[140, 212, 252],
                  [100, 100, 100],
                  [0, 255, 0],
                  [255, 255, 0],
                  [255, 0, 0]])

    return a


def one_cell_convert_mag_quality(element):
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


def convert_mag_quality(m):
    m_conv = np.zeros([m.shape[0], m.shape[1]])
    for i in range(0, m.shape[0]):
        for j in range(0, m.shape[1]):
            m_conv[i, j] = one_cell_convert_mag_quality(m[i, j])

    return m_conv


def plot_mag_quality(mag_quality, output_directory, output_plot_file_name, real_floor):
    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]

    outpath = os.path.join(output_directory, output_plot_file_name)
    # Start plotting

    mag_quality_cmap = get_mag_quality_colormap()
    mag_quality_converted = convert_mag_quality(mag_quality.transpose())

    plot_images.draw_picture(mag_quality_converted, mag_quality_cmap, outpath)
    # adding text output for mag quality (percentage of gray, red, yellow and green cells)
    text_mag_quality_out = outpath[:-4] + '.txt'
    plot_images.write_stats_for_picture(mag_quality_converted, mag_quality_cmap, text_mag_quality_out)
