import math
import numpy as np
import color_map as colormap


def formgrid(points_list, points_sv_list, points_cs_list, max_x, max_y, cellsize):
    # max_index_x = int(round((max_x - cellsize/2) / cellsize) + 1)
    # max_index_y = int(round((max_y - cellsize/2) / cellsize) + 1)

    max_index_x = int( math.ceil( max_x / cellsize) )
    max_index_y = int( math.ceil( max_y / cellsize) )

    max_index_sum = max_index_x * max_index_y
    # print("number of cells for x = ", max_index_x, "number of ceils for y = ", max_index_y, "common number of ceils = ", max_index_sum)
    points_table = {}
    points_sv_table = {}
    points_cs_table = {}
    for point in points_list:
        # get index of cell
        index_x = int(math.trunc(point[0] / cellsize))
        index_y = int(math.trunc(point[1] / cellsize))
        index_sum = index_y * max_index_x + index_x
        # print(point[0], "     ", point[1], "     ", index_x, "     ", index_y, "     ", index_sum )
        points_table[index_sum] = 1

    for point in points_sv_list:
        # get index of cell
        index_x = int(math.trunc(point[0] / cellsize))
        index_y = int(math.trunc(point[1] / cellsize))
        index_sum = index_y * max_index_x + index_x
        # print(point[0], "     ", point[1], "     ", index_x, "     ", index_y, "     ", index_sum )
        points_sv_table[index_sum] = 1

    for point in points_cs_list:
        # get index of cell
        index_x = int(math.trunc(point[0] / cellsize))
        index_y = int(math.trunc(point[1] / cellsize))
        index_sum = index_y * max_index_x + index_x
        # print(point[0], "     ", point[1], "     ", index_x, "     ", index_y, "     ", index_sum )
        points_cs_table[index_sum] = 1

    availability = colormap.blue_color * np.ones([max_index_x, max_index_y])
    # while index_sum < max_index_sum :
    for i in range(0, max_index_x):
        for j in range(0, max_index_y):
            index_sum = j * max_index_x + i
            # index_sum = i * max_index_y + j

            value = points_table.get(index_sum)
            if value is not None:
                availability[i, j] = colormap.grey_color

    return points_table, points_sv_table, points_cs_table, availability
