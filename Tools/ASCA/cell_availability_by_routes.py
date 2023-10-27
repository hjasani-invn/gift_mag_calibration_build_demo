import geojson
import json
import Geo2LocalCoordinateConverter
import line2points
import get_file_name
import get_linenumber


def cell_availability_by_routes(venue_json_file, geo_json_file_path):
    # parse venue json file 
    f = open(venue_json_file, 'r')
    text = f.read()
    data = json.loads(text)
    venue = data['venue']
    # print (venue)

    origin_latitude = venue['origin_lattitude']
    origin_longitude = venue['origin_longitude']
    origin_azimuth = venue['origin_azimuth']
    alfa = venue['alfa']
    beta = venue['beta']

    magnetic_cellsize = data['magnetic_cellsize']
    mag_max_x = venue['size_x']
    mag_max_y = venue['size_y']

    # set venue parametres
    Geo2LocalCoordinateConverter.set_geo_param(origin_latitude, origin_longitude, alfa, beta, origin_azimuth)

    # parse irl data from geojson file and conver them to local  coordinate
    #print(geo_json_file)
    try:
        with open(geo_json_file_path, 'r', encoding='utf-8') as geojson_file:
            map_json_data = geojson.load(geojson_file)
    except Exception as ex:
        print("geojson file is corrupted or has unknown format:")
        print(geo_json_file_path)
        print("file name is ", get_file_name.get_file_name())
        print("line is  ", get_linenumber.get_linenumber())
        print("exception code:")
        print(ex.args)
        map_json_data = {}

    features = map_json_data['features']

    all_points_list = []
    all_sv_points_list = []
    all_cs_points_list = []
    for feature in features:
        # print (feature.geometry.type)
        # print (len(feature.geometry.coordinates))
        # print (feature.geometry.coordinates)
        x_prev = 0
        y_prev = 0

        old_format = True

        if 'properties' in feature.geometry:
            if 'routeType' in feature.geometry.properties:
                old_format = False
                # if feature.geometry._data['properties']['routeType'] == 'single-cell':
                #     continue

        if not old_format:
            if feature.geometry.properties['routeType'] == 'crowd-source':
                for idx, coordinate in enumerate(feature.geometry.coordinates):
                    (x, y) = Geo2LocalCoordinateConverter.geo2local(coordinate[1], coordinate[0])
                    if idx > 0:
                        points_list = line2points.line2points(x_prev, y_prev, x, y, magnetic_cellsize / 100)
                        all_cs_points_list.extend(points_list)
                        all_points_list.extend(points_list)
                    x_prev = x
                    y_prev = y
            elif feature.geometry.properties['routeType'] == 'single-cell':
                if feature.geometry['type'] == 'Point':
                    lon_lat = feature.geometry.coordinates
                else:
                    lon_lat = feature.geometry.coordinates[0]  # Point and MultiPoint work differently
                [x, y] = Geo2LocalCoordinateConverter.geo2local(lon_lat[1], lon_lat[0])
                all_sv_points_list.append([x, y])
                all_points_list.append([x, y])
            else:
                for idx, coordinate in enumerate(feature.geometry.coordinates):
                    (x, y) = Geo2LocalCoordinateConverter.geo2local(coordinate[1], coordinate[0])
                    if idx > 0:
                        points_list = line2points.line2points(x_prev, y_prev, x, y, magnetic_cellsize / 100)
                        all_sv_points_list.extend(points_list)
                        all_points_list.extend(points_list)
                    x_prev = x
                    y_prev = y

        else:
            for idx, coordinate in enumerate(feature.geometry.coordinates):
                (x, y) = Geo2LocalCoordinateConverter.geo2local(coordinate[1], coordinate[0])
                if idx > 0:
                    points_list = line2points.line2points(x_prev, y_prev, x, y, magnetic_cellsize / 100)
                    all_sv_points_list.extend(points_list)
                    all_points_list.extend(points_list)
                x_prev = x
                y_prev = y

    return all_points_list, all_sv_points_list, all_cs_points_list
