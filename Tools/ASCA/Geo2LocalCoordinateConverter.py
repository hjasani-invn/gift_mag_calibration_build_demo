import math
import venueparameters

M_PI = 3.1415926535897932384626433832795

CC_MIN_LAT0 = (-M_PI / 2.)
CC_MAX_LAT0 = (M_PI / 2.)
CC_MIN_LON0 = -M_PI
CC_MAX_LON0 = M_PI
CC_MAX_ALFA_LAT = 1e10
CC_MAX_ALFA_LON = 1e10
CC_MIN_HEADING = (-M_PI)
CC_MAX_HEADING = (M_PI)
CC_MIN_LAT_DEGREES = -89.9
CC_MAX_LAT_DEGREES = 89.9
CC_MIN_LON_DEGREES = -180.0
CC_MAX_LON_DEGREES = 180.0
R_EARTH_A = 6378137
R_EARTH_B = 6356752


def set_geo_param(new_lat0_deg, new_lon0_deg, new_alfa_lat, new_alfa_lon, new_heading):
    # conversion to degrees
    new_lat0 = new_lat0_deg * M_PI / 180.
    new_lon0 = new_lon0_deg * M_PI / 180.
    new_heading *= M_PI / 180.

    # checking of input parameters
    if new_lat0 < CC_MIN_LAT0:
        return 0

    if new_lat0 > CC_MAX_LAT0:
        return 0

    if new_lon0 < CC_MIN_LON0:
        return 0

    if new_lon0 > CC_MAX_LON0:
        return 0

    if new_alfa_lat < -CC_MAX_ALFA_LAT:
        return 0

    if new_alfa_lat > CC_MAX_ALFA_LAT:
        return 0

    if new_alfa_lon < -CC_MAX_ALFA_LON:
        return 0

    if new_alfa_lon > CC_MAX_ALFA_LON:
        return 0

    venueparameters.lat0 = new_lat0
    venueparameters.lon0 = new_lon0

    venueparameters.heading = new_heading
    # f_init = 1

    venueparameters.alfa_lat = new_alfa_lat
    venueparameters.alfa_lon = new_alfa_lon

    # print ( "alfa_lat,  alfa_lon   before = ", venueparameters.alfa_lat,  venueparameters.alfa_lon)
    if ((new_alfa_lat > -1e-15) and (new_alfa_lat < 1e-15)) or ((new_alfa_lon > -1e-15) and (new_alfa_lon < 1e-15)):
        (venueparameters.alfa_lat, venueparameters.alfa_lon) = calc_def_scale_llf2geo_rad(new_lat0, new_lon0)
        venueparameters.alfa_lon = -venueparameters.alfa_lon  # to NWU local frame

        # print ( "alfa_lat,  alfa_lon  after = ", venueparameters.alfa_lat,  venueparameters.alfa_lon)
        # print ("new alfa latitude = ", venueparameters.alfa_lat)
        # print ("new alfa longitude = ", venueparameters.alfa_lon)
    else:
        venueparameters.alfa_lat = new_alfa_lat
        venueparameters.alfa_lon = new_alfa_lon


def calc_def_scale_llf2geo_rad(lat0, lon0):
    if (lat0 >= CC_MIN_LAT_DEGREES) and (lat0 <= CC_MAX_LAT_DEGREES) and (lon0 >= CC_MIN_LON_DEGREES) and (
            lon0 <= CC_MAX_LON_DEGREES):
        scale_lat = 2. * M_PI / (2. * M_PI * R_EARTH_B)
        scale_lon = 2. * M_PI / (2. * M_PI * R_EARTH_A * math.cos(lat0))
    else:
        scale_lat = 0
        scale_lon = 0

    return scale_lat, scale_lon


def geo2local(lat, lon):
    sin_head = math.sin(venueparameters.heading)
    cos_head = math.cos(venueparameters.heading)

    # conversion from degrees
    lat *= M_PI / 180
    lon *= M_PI / 180

    # print ( "alfa_lat,  alfa_lon  = ", venueparameters.alfa_lat,  venueparameters.alfa_lon)
    x = 0
    y = 0
    # if f_init :
    x = (lat - venueparameters.lat0) * cos_head / venueparameters.alfa_lat + (
                lon - venueparameters.lon0) * sin_head / venueparameters.alfa_lon
    y = -(lat - venueparameters.lat0) * sin_head / venueparameters.alfa_lat + (
                lon - venueparameters.lon0) * cos_head / venueparameters.alfa_lon
    return x, y


def local2geo(x, y):
    heading = venueparameters.heading

    lat = venueparameters.lat0 + venueparameters.alfa_lat * (x * math.cos(heading) - y * math.sin(heading))
    lon = venueparameters.lon0 + venueparameters.alfa_lon * (x * math.sin(heading) + y * math.cos(heading))

    lat = math.degrees(lat)
    lon = math.degrees(lon)

    return lat, lon
