import Geo2LocalCoordinateConverter
import os
import json
import numpy as np
import plot_images
import math


def create_empty_grid(settings_json, output_directory, output_plot_file_name, output_lat_lon_file):
    settings = json.load(open(settings_json))

    cell_size = settings['magnetic_cellsize']
    dim_x = settings['venue']['size_x']
    dim_y = settings['venue']['size_y']
    # max_x = int(round((dim_x - cell_size / 2) / cell_size) + 1)
    # max_y = int(round((dim_y - cell_size / 2) / cell_size) + 1)

    max_x = int(math.ceil(dim_x/cell_size))
    max_y = int(math.ceil(dim_y/cell_size))

    x1 = max_x * cell_size
    y1 = 0
    x2 = max_x * cell_size
    y2 = max_y * cell_size
    x3 = 0
    y3 = max_y * cell_size

    origin_latitude = settings['venue']['origin_lattitude']
    origin_longitude = settings['venue']['origin_longitude']
    origin_azimuth = settings['venue']['origin_azimuth']
    alfa = settings['venue']['alfa']
    beta = settings['venue']['beta']

    # set venue parametres

    Geo2LocalCoordinateConverter.set_geo_param(origin_latitude, origin_longitude, alfa, beta, origin_azimuth)

    lat0 = settings['venue']['origin_lattitude']
    lon0 = settings['venue']['origin_longitude']
    lat1, lon1 = Geo2LocalCoordinateConverter.local2geo(x1, y1)
    lat2, lon2 = Geo2LocalCoordinateConverter.local2geo(x2, y2)
    lat3, lon3 = Geo2LocalCoordinateConverter.local2geo(x3, y3)

    output_lat_lon_file_name = os.path.join(output_directory, output_lat_lon_file)
    coordinates_json = {"coordinates": {"p1": {"lat": lat0, "lng": lon0},
                                        "p2": {"lat": lat1, "lng": lon1},
                                        "p3": {"lat": lat2, "lng": lon2},
                                        "p4": {"lat": lat3, "lng": lon3}}}

    with open(output_lat_lon_file_name, 'w') as f:
        json.dump(coordinates_json, f)

    # creating empty grid image file
    outpath = os.path.join(output_directory, output_plot_file_name)

    grid = 1 * np.ones([max_y, max_x])

    plot_images.draw_empty_grid(grid, outpath)


