from PIL import Image
import json
import numpy as np
import matplotlib.pyplot as plt
import math
import coordinate_converter as cc
from fastkml import kml
from shapely.geometry import Point


class Map:
    def __init__(self, image_file_full_path, settings_file_full_path):
        im = Image.open(image_file_full_path)

        settings = json.load(open(settings_file_full_path))

        self.X_m = float(settings['venue']['size_x'])
        self.Y_m = float(settings['venue']['size_y'])

        self.lat0 = float(settings['venue']['origin_lattitude'])
        self.lon0 = float(settings['venue']['origin_longitude'])
        self.azimuth = float(settings['venue']['origin_azimuth'])

        self.m_in_pix = self.X_m / im.size[0]
        pix_objects = im.load()

        self.pix = [[0] * im.size[1] for i in range(im.size[0])]

        self.pix_size_x = im.size[0]
        self.pix_size_y = im.size[1]

        for i in range(0, len(self.pix)):
            for j in range(0, len(self.pix[i])):
                self.pix[i][j] = pix_objects[i, j]

        self.cell_size = 0.5 # in meters
        self.h_size = round(self.X_m / self.cell_size) # obstacle grid horizontal cells count
        self.v_size = round(self.Y_m / self.cell_size) # obstacle grid vertical cells count

        self.obstacle_grid = np.zeros([self.h_size, self.v_size])
        self.priority_grid = np.zeros([self.h_size, self.v_size])
        self.fill_grids()

        del self.pix  # deleting object, since it is bad for multiprocessing

        self.debug_grid = self.obstacle_grid.copy()

        self.point_found = False

    def delete_grids(self):
        del self.obstacle_grid
        del self.priority_grid

    def save_output_picture(self):
        plt.pcolor(self.debug_grid.transpose()[::-1, ], cmap='viridis', vmin=0, vmax=3, edgecolors='k', linewidths=0.1)
        plt.savefig('beacons.png', format='png', pad_inches=0, dpi=400)
        plt.close()

    def fill_grids(self):
        for i in range(0, self.h_size):
            for j in range(0, self.v_size):
                self.obstacle_grid[i, j] = self.check_area_for_obstacles(i, j)
                self.priority_grid[i, j] = self.check_area_for_high_priority_areas(i, j)

    def check_area_for_obstacles(self, cell_i, cell_j):
        x_min_m = self.cell_size/2 + self.cell_size * cell_i - self.cell_size/2
        y_min_m = self.cell_size/2 + self.cell_size * cell_j - self.cell_size/2
        x_max_m = x_min_m + self.cell_size
        y_max_m = y_min_m + self.cell_size

        x_min_pix, y_min_pix = self.meters_to_pix(x_min_m, y_min_m)
        x_max_pix, y_max_pix = self.meters_to_pix(x_max_m, y_max_m)

        obstacle_pix_count = 0
        free_pix_count = 0

        for i in range(x_min_pix, x_max_pix+1):
            for j in range(y_min_pix, y_max_pix+1):
                if self.check_pixel_for_obstacle(self.pix[i][j]) == 1:
                    free_pix_count += 1
                else:
                    obstacle_pix_count += 1

        if free_pix_count >= obstacle_pix_count:
            return 1
        else:
            return 0

    def check_pixel_for_obstacle(self, pixel):
        # color red used for available spots and green for high priority
        abs = (pixel[0]**2 + pixel[1]**2 + pixel[2]**2)**0.5
        if abs > 80 and (pixel[0] > pixel[1] + pixel[2]):
            return 1   # red pixel
        elif abs > 70 and (pixel[1] > pixel[0] + pixel[2]):
            return 1   # green pixel
        else:
            return 0

    def check_area_for_high_priority_areas(self, cell_i, cell_j):
        x_min_m = self.cell_size/2 + self.cell_size * cell_i - self.cell_size/2
        y_min_m = self.cell_size/2 + self.cell_size * cell_j - self.cell_size/2
        x_max_m = x_min_m + self.cell_size
        y_max_m = y_min_m + self.cell_size

        x_min_pix, y_min_pix = self.meters_to_pix(x_min_m, y_min_m)
        x_max_pix, y_max_pix = self.meters_to_pix(x_max_m, y_max_m)

        normal_pix_count = 0
        high_priority_pix_count = 0

        for i in range(x_min_pix, x_max_pix+1):
            for j in range(y_min_pix, y_max_pix+1):
                if self.check_pixel_for_high_priority_areas(self.pix[i][j]) == 1:
                    high_priority_pix_count += 1
                else:
                    normal_pix_count += 1

        if high_priority_pix_count >= normal_pix_count:
            return 1
        else:
            return 0

    def check_pixel_for_high_priority_areas(self, pixel):
        # color green used for high priority
        abs = (pixel[0]**2 + pixel[1]**2 + pixel[2]**2)**0.5
        if abs > 70 and (pixel[1] > pixel[0] + pixel[2]):
            return 1    # green pixel
        else:
            return 0

    def pix_to_meters(self, pix_x, pix_y):
        X_m = self.m_in_pix * pix_x
        Y_m = self.m_in_pix * pix_y
        return X_m, Y_m

    def meters_to_pix(self, x_m, y_m):
        pix_X = round(x_m / self.m_in_pix)
        pix_Y = round(y_m / self.m_in_pix)

        out_X = min(int(pix_X), self.pix_size_x-1)
        out_Y = min(int(pix_Y), self.pix_size_y-1)

        return out_X, out_Y

    def get_segment_between_two_points(self, start, end, debug_enabled):
        # return a list of "i, j, distance, obstacle_factor"
        # Bresenham's line algorithm being used
        path = []

        saved_start = start.copy()

        steep = abs(end[1] - start[1]) > abs(end[0] - start[0])
        if steep:
            # if line is steep we iterate "y" instead of "x"
            start = swap(start[0], start[1])
            end = swap(end[0], end[1])

        reverse = False

        if start[0] > end[0]:
            x0 = start[0]
            x1 = end[0]
            start[0] = x1
            end[0] = x0

            y0 = start[1]
            y1 = end[1]
            start[1] = y1
            end[1] = y0

            reverse = True

        dx = end[0] - start[0]
        dy = abs(end[1] - start[1])
        error = 0
        delta_error = dy/float(dx)

        ystep = 0
        y = start[1]

        if start[1] < end[1]:
            ystep = 1
        else:
            ystep = -1

        for x in range(start[0], end[0]+1):
            if steep:
                d = self.meter_distance_between_cells(y, x, saved_start[0], saved_start[1])
                passable = self.is_passable(y, x)
                priority = self.is_high_priority(y, x)
                path.append((y, x, d, passable, priority))

                if debug_enabled:
                    if passable:
                        self.debug_grid[y, x] = 3

            else:
                d = self.meter_distance_between_cells(x, y, saved_start[0], saved_start[1])
                passable = self.is_passable(x, y)
                priority = self.is_high_priority(x, y)
                path.append((x, y, d, passable, priority))

                if debug_enabled:
                    if passable:
                        self.debug_grid[x, y] = 3

            error += delta_error

            if error >= 0.5:
                y += ystep
                error -= 1.0

        if reverse:
            path = path[::-1]

        for item in list(path):
            if not self.point_is_within_boundaries(item[0], item[1]):
                path.remove(item)

        return path

    def meter_distance_between_cells(self, x1, y1, x2, y2):
        x1_m = self.cell_size/2 + x1 * self.cell_size
        y1_m = self.cell_size/2 + y1 * self.cell_size
        x2_m = self.cell_size/2 + x2 * self.cell_size
        y2_m = self.cell_size/2 + y2 * self.cell_size

        return ((x2_m - x1_m) ** 2 + (y2_m - y1_m) ** 2) ** 0.5

    def coverage_index_to_meters(self, index):
        return self.cell_size/2 + index * self.cell_size

    def meters_to_coverage_index(self, l):
        return int(l/self.cell_size)

    def point_is_within_boundaries(self, x, y):
        if x >= self.h_size or x < 0:
            return False
        if y >= self.v_size or y < 0:
            return False
        return True

    def is_passable(self, x, y):
        if not self.point_is_within_boundaries(x, y):
            return False
        else:
            if self.obstacle_grid[x, y] == 0:
                return False
            if self.obstacle_grid[x, y] == 1:
                return True

    def is_high_priority(self, x, y):
        if not self.point_is_within_boundaries(x, y):
            return False
        else:
            if self.priority_grid[x, y] == 0:
                return False
            if self.priority_grid[x, y] == 1:
                return True

    def get_points_within_arc(self, x, y, r, heading, max_angle):
        hor_min_index = self.meters_to_coverage_index(self.coverage_index_to_meters(x) - (r + 1))
        hor_max_index = self.meters_to_coverage_index(self.coverage_index_to_meters(x) + (r + 1))
        vert_min_index = self.meters_to_coverage_index(self.coverage_index_to_meters(y) - (r + 1))
        vert_max_index = self.meters_to_coverage_index(self.coverage_index_to_meters(y) + (r + 1))

        point_list = []

        for i in range(hor_min_index, hor_max_index + 1):
            for j in range(vert_min_index, vert_max_index + 1):
                d = self.meter_distance_between_cells(i, j, x, y)
                if abs(r - d) < 0.5 * self.cell_size * (2 ** 0.5):
                    # NOTE: beacon heading angle is from -pi to pi, counted from X axis
                    # pi/2 is vertical direction, pi is left, 0 is right

                    point_heading = math.atan2(j - y, i - x)
                    tmp_dh = point_heading - heading
                    delta_heading = math.atan2(math.sin(tmp_dh), math.cos(tmp_dh))

                    if abs(delta_heading) <= max_angle:
                        point_list.append([i, j])

        return point_list

    def get_closest_possible_location(self, x, y):
        # for (x, y) getting the closest cell where beacon can be placed

        max_d = max(self.v_size, self.h_size)

        is_obstacle = not self.is_passable(x, y)
        if not is_obstacle:
            # 1) if given cell is free of obstacle, then search for closest obstacle
            #    when found obstacle, then use line get_segment_between_two_points to get closest free cell
            #    then calculate heading and distance
            d = 1
            min_distance = max_d
            self.point_found = False
            best_x = -1
            best_y = -1

            while (d < max_d) and not self.point_found:
                search_square = self.get_square_around_XY(x, y, d)
                found = False
                for cell in search_square:
                    if not self.is_passable(cell[0], cell[1]):
                        current_distance = self.meter_distance_between_cells(x, y, cell[0], cell[1])
                        if current_distance < min_distance:
                            best_x = cell[0]
                            best_y = cell[1]
                            min_distance = current_distance
                            found = True

                if found:
                    start = [x, y]
                    end = [best_x, best_y]
                    path_to_closest_obstacle = self.get_segment_between_two_points(start, end, False)
                    available_location = path_to_closest_obstacle[-2]
                    self.point_found = True
                    heading = math.atan2(y - best_y, x - best_x)

                    return available_location[0], available_location[1], min_distance, heading

                d += 1

            return -1, -1, -1, -1 # couldn't find obstacle on a map, this should never happen

        else:
            # 2) if given cell is inside an obstacle, then search for closest free cell
            #   then calculate heading and distance
            #  return: x_closest, y_closest, distance, heading
            d = 1
            min_distance = max_d
            self.point_found = False
            best_x = -1
            best_y = -1

            while (d < max_d) and not self.point_found:
                search_square = self.get_square_around_XY(x, y, d)
                found = False
                for cell in search_square:
                    if self.is_passable(cell[0], cell[1]):
                        current_distance = self.meter_distance_between_cells(x, y, cell[0], cell[1])
                        if current_distance < min_distance:
                            best_x = cell[0]
                            best_y = cell[1]
                            min_distance = current_distance
                            found = True

                if found:
                    available_location = [best_x, best_y]
                    self.point_found = True
                    heading = math.atan2(available_location[1] - y, available_location[0] - x)

                    return available_location[0], available_location[1], min_distance, heading

                d += 1

            return -1, -1, -1, -1 # couldn't find obstacle on a map, this should never happen

    def get_square_around_XY(self, x, y, d):
        # returns a list of points around x, y located on a square of size (1+d*2)
        # (points outside of borders are skipped)
        output_list = []
        for i in range(x-d, x+d+1):
            point = [i, y+d]
            if self.point_is_within_boundaries(point[0], point[1]):
                output_list.append(point)
            point = [i, y-d]
            if self.point_is_within_boundaries(point[0], point[1]):
                output_list.append(point)

        for j in range(y-d+1, y+d):
            point = [x-d, j]
            if self.point_is_within_boundaries(point[0], point[1]):
                output_list.append(point)
            point = [x+d, j]
            if self.point_is_within_boundaries(point[0], point[1]):
                output_list.append(point)

        return output_list

    def get_sum_of_minimal_distances(self, points_list):
        sum_distance = 0
        for p1 in points_list:
            min_d = self.h_size * self.cell_size
            for p2 in points_list:
                same = False
                if p2[0] == p1[0] and p2[1] == p1[1]:
                    same = True
                if not same:
                    d = self.meter_distance_between_cells(p1[0], p1[1], p2[0], p2[1])
                    if d < min_d:
                        min_d = d

            sum_distance += min_d

        return sum_distance

    def generate_kml(self, beacons_list, possible_beacons_positions, total_beacons_positions_number):
        beacon_pos_count = total_beacons_positions_number
        k = kml.KML()
        ns = '{http://www.opengis.net/kml/2.2}'
        d = kml.Document(ns, 'docid', 'Beacons', 'doc description')
        k.append(d)
        f = kml.Folder(ns, 'fid', 'Beacons list', 'f description')
        d.append(f)

        beacon_index = 0
        with open('coordinates_out.txt', 'w+') as outfile:
            for idx in range(0, beacon_pos_count):
                if beacons_list[idx] == 1:
                    for b in possible_beacons_positions:
                        if b[0] == idx:
                            bx = b[1] * self.cell_size
                            by = self.Y_m - b[2] * self.cell_size
                            lat, lon = cc.local2geo(self.lat0, self.lon0, self.azimuth, bx, by)
                            out_line = str(idx) + ' : ' + str(bx) + ' ' + str(by) + ' ' + str(lat) + ' ' + str('lon') + '\n'
                            outfile.write(out_line)

                            p = kml.Placemark(ns, 'id', str(beacon_index), 'description')
                            p.geometry = Point(lon, lat)

                            f.append(p)

                            beacon_index += 1

        with open('beacons.kml', 'w+') as outkml:
            outkml.write(k.to_string())


def swap(n1, n2):
        return [n2, n1]

