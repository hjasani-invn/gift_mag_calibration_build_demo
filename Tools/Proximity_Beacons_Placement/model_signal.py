import geometry
import beacon
import math


class SignalModel:
    def __init__(self, model_angle, model_rssi_1m, model_type, priority_factor, venue, min_intersection_RSSI):
        self.angle = model_angle
        self.rssi_1m = model_rssi_1m
        self.model_type = model_type
        self.priority_factor = priority_factor

        self.min_RSSI = -75
        self.min_intersection_RSSI = min_intersection_RSSI

        if self.model_type == 'proximity':
            self.min_RSSI = -75
            self.obstacle_factor = 5
            self.max_distance = 10

        self.venue = venue

    def add_one_beacon_signal(self, M_coverage, beacon):
        # for one beacon adds its signal to coverage matrix, updated matrix and quality of beacon are returned
        beacon_quality = 0
        if self.model_type == 'proximity':
            max_angle = self.angle

            size_x = self.venue.h_size
            size_y = self.venue.v_size

            arc_point_list = self.venue.get_points_within_arc(beacon.x, beacon.y, self.max_distance, beacon.heading, max_angle)
            for arc_point in arc_point_list:
                start = [beacon.x, beacon.y]
                end = [arc_point[0], arc_point[1]]
                path = self.venue.get_segment_between_two_points(start, end, False)
                count_obstacle_factor = 0

                for path_point in path:
                    if not path_point[3]:
                        count_obstacle_factor += 1

                    if path_point[2] > 0:
                        point_RSSI = self.rssi_1m - 20 * (math.log(path_point[2]) / math.log(10)) - count_obstacle_factor * self.obstacle_factor
                        signal_already_added = False

                        for element in M_coverage[path_point[0]][path_point[1]]:
                            if element[0] == beacon.number:
                                signal_already_added = True

                        if not signal_already_added:
                            M_coverage[path_point[0]][path_point[1]].append([beacon.number, point_RSSI])
                            if point_RSSI > self.min_RSSI:
                                additional_quality = 1
                                if path_point[4] == 1.0:
                                    additional_quality *= self.priority_factor
                                beacon_quality += additional_quality

        return M_coverage, beacon_quality

