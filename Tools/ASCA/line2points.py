import math


def line2points(x0, y0, x1, y1, distance_between_points):
    d_x = (x1 - x0)
    d_y = (y1 - y0)
    line_length = math.sqrt(d_x * d_x + d_y * d_y)

    if line_length == 0:
        return []

    points_list = []
    cur_point_x = x0
    cur_point_y = y0
    # points_list.append((cur_point_x,cur_point_y))
    flag = True
    while flag:
        points_list.append((cur_point_x, cur_point_y))
        cur_point_x += distance_between_points * d_x / line_length
        cur_point_y += distance_between_points * d_y / line_length
        if d_x > 0:
            flag_x = (cur_point_x < x1)
        else:
            flag_x = (cur_point_x > x1)
        if d_y > 0:
            flag_y = (cur_point_y < y1)
        else:
            flag_y = (cur_point_y > y1)
        flag = flag_x and flag_y

    # print ("points_list   ", points_list)
    return points_list
