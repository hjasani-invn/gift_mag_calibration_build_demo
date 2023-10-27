import math


def local2geo(lat0, lon0, heading, x, y):
    heading = -math.radians(heading)
    y = -y

    R_b = 6356752
    R_a = 6378137
    alpha = 1/R_b
    beta = 1/(R_a * math.cos(math.radians(lat0)))

    lat = math.radians(lat0) + alpha * (x*math.cos(heading) - y*math.sin(heading))
    lon = math.radians(lon0) + beta * (x*math.sin(heading) + y*math.cos(heading))

    lat = math.degrees(lat)
    lon = math.degrees(lon)

    return lat, lon