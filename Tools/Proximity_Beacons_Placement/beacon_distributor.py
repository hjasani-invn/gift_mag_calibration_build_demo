import json
import numpy as np
import model_signal
import geometry
import beacon
import math
import random
import multiprocessing as mp
import tqdm
import itertools
import statistics


class BeaconDistributor:
    def __init__(self, settings_file_full_path, image_file_full_path):
        self.settings = settings_file_full_path
        settings = json.load(open(settings_file_full_path))
        model_angle = math.radians(settings['beacon_angle'])
        model_rssi_1m = settings['beacon_RSSI_1m']
        model_type = 'proximity'
        if 'signal_model' in settings:
            model_type = settings['signal_model']
        self.beacon_grid_cellsize = settings['beacon_grid_cellsize']
        self.beacon_limit = settings['maximum_beacon_count']

        self.priority_factor = 1.0  # default value for high priority areas
        if 'priority_area_factor' in settings:
            self.priority_factor = settings['priority_area_factor']

        self.rejection_coeff = 1.0 # default value for rejecting beacon locations (1.0 rejects all beacons with quality < 1.0 * median)
        if 'rejection_coefficient' in settings:
            self.rejection_coeff = settings['rejection_coefficient']

        min_intersection_RSSI = -75
        if 'min_intersection_RSSI' in settings:
            min_intersection_RSSI = settings['min_intersection_RSSI']

        self.iterations_number = 5000
        if 'number_of_iterations' in settings:
            self.iterations_number = settings['number_of_iterations']

        self.image_file_path = image_file_full_path
        self.venue = geometry.Map(image_file_full_path, settings_file_full_path)
        self.sig_model = model_signal.SignalModel(model_angle, model_rssi_1m, model_type, self.priority_factor, self.venue, min_intersection_RSSI)

        self.possible_beacons_positions = []    # each element has: [beacon_number, b_x, b_y, b_heading]
        self.total_beacons_positions_number = 0
        self.overlapping_beacons = []
        self.beacon_quality_list = []
        self.out_file_name = 'test_out.txt'
        out_f = open(self.out_file_name, 'w+')
        out_f.close()

        # creating RSSI coverage matrix M_coverage
        # M_coverage is actually a list of lists of lists of pairs
        # each M_coverage[i, j] element is a list of elements, each element is: [beacon_number, beacon_RSSI]
        self.M_coverage = [[0] * self.venue.v_size for i in range(self.venue.h_size)]
        for i in range(0, len(self.M_coverage)):
            for j in range(0, len(self.M_coverage[i])):
                self.M_coverage[i][j] = []

    def generate_possible_beacon_positions(self):
        available_pos_list = []
        corrected_pos_list = []
        x = round(self.beacon_grid_cellsize/self.venue.cell_size/2)
        increment = round(self.beacon_grid_cellsize/self.venue.cell_size)

        while x < self.venue.h_size:
            y = round(self.beacon_grid_cellsize / self.venue.cell_size / 2)
            while y < self.venue.v_size:
                # x,y here is a center of current beacon grid cell
                # find closest beacon location, check that it is within the cell, if yes - add it to list
                bX, bY, min_distance, heading = self.venue.get_closest_possible_location(x, y)
                if abs((bX - x)*self.venue.cell_size) <= self.beacon_grid_cellsize/2:
                    if abs((bY - y)*self.venue.cell_size) <= self.beacon_grid_cellsize/2:
                        available_pos_list.append([bX, bY, heading])
                y += increment
            x += increment

        # removing all duplicates from available_pos_list
        filtered_available_pos_list = []
        for b_pos in available_pos_list:
            b_x = b_pos[0]
            b_y = b_pos[1]

            exists = False
            for f_pos in filtered_available_pos_list:
                if f_pos[0] == b_x and f_pos[1] == b_y:
                    exists = True
            if not exists:
                filtered_available_pos_list.append(b_pos)

        self.possible_beacons_positions = filtered_available_pos_list

        i = 0
        for i, b_pos in enumerate(self.possible_beacons_positions):
            # adding beacon numbers
            b_pos.insert(0, i)

        # now self.possible_beacons_positions list has possible positions numbered

        # adding a matrix of overlapping beacons to each element of possible_beacons_positions list

        for b_pos in self.possible_beacons_positions:
            b_num = b_pos[0]
            bx = b_pos[1]
            by = b_pos[2]
            bh = b_pos[3]
            b = beacon.Beacon(bx, by, bh, b_num)

            self.M_coverage, quality = self.sig_model.add_one_beacon_signal(self.M_coverage, b)
            self.beacon_quality_list.append(quality)

        # removing the worst beacon positions, so that they are never chosen
        beacon_pos_count = len(self.possible_beacons_positions)

        print('possible positions count before rejection:', len(self.possible_beacons_positions))
        self.total_beacons_positions_number = len(self.possible_beacons_positions)
        self.reject_bad_beacon_positions()
        print('possible positions count after rejection:', len(self.possible_beacons_positions))

        overlapping_Matrix = np.zeros([beacon_pos_count, beacon_pos_count])
        # filling overlapping matrix
        for i in range(0, self.venue.h_size, 1):
            for j in range(0, self.venue.v_size, 1):
                if len(self.M_coverage[i][j]) > 1:
                    visible_beacons = []
                    for M_cov_element in self.M_coverage[i][j]:
                        if M_cov_element[1] > self.sig_model.min_intersection_RSSI:
                            visible_beacons.append(M_cov_element[0])

                    # we have a list of visible beacons for a cell, for example [0, 1, 3, 6]
                    # list(itertools.combinations(visible_beacons, 2)) has the following output:
                    # [(0, 1), (0, 3), (0, 6), (1, 3), (1, 6), (3, 6)]
                    for pair in list(itertools.combinations(visible_beacons, 2)):
                        overlapping_Matrix[pair[0], pair[1]] = 1
                        overlapping_Matrix[pair[1], pair[0]] = 1

        # creating a list of overlapping beacons from the overlapping matrix (it works faster)
        for i in range(0, overlapping_Matrix.shape[0]):
            over_list = []
            for j in range(0, overlapping_Matrix.shape[1]):
                if overlapping_Matrix[i, j] == 1:
                    over_list.append(j)
            self.overlapping_beacons.append(over_list)

        return

    def reject_bad_beacon_positions(self):
        # this method removes worst beacon positions from the list, to make main algorithm run faster
        median_quality = statistics.median(self.beacon_quality_list)
        bad_beacon_list = []
        copied_possible_positions = self.possible_beacons_positions[::-1]

        for b in self.possible_beacons_positions:
            q = self.beacon_quality_list[b[0]]
            threshold = self.rejection_coeff * median_quality
            if q < threshold:
                copied_possible_positions.remove(b)
                bad_beacon_list.append(b[0])

        self.possible_beacons_positions = copied_possible_positions[::-1]

        # removing deleted beacons from M_coverage matrix
        # explanation: M_coverage is rebuilt to contain only elements with "good" beacon position numbers
        for bad_beacon in bad_beacon_list:
            self.M_coverage = [[[sub_subelement
                                 for sub_subelement in subelement
                                 if (len(sub_subelement) > 0 and sub_subelement[0] != bad_beacon)
                                 or (len(sub_subelement) == 0)]
                                for subelement in element] for element in self.M_coverage]

    def improved_generate_random_beacons_set(self):
        # this method creates a set of random beacons
        # with the condition that none of their signals are overlapping
        # it stops adding beacons to the set when either beacon limit is reached, or when zero positions are available
        beacon_pos_count = len(self.possible_beacons_positions)
        random_beacons_set = np.zeros([self.total_beacons_positions_number])
        free_beacon_places = []
        for i in range(0, beacon_pos_count):
            k = self.possible_beacons_positions[i][0]
            free_beacon_places.append(k)

        for idx_beacon in range(0, self.beacon_limit):
            length = len(free_beacon_places)
            if length == 0:
                break

            r_i = random.randint(0, length - 1)
            rnd_idx = free_beacon_places[r_i]
            random_beacons_set[rnd_idx] = 1
            for overlapping_beacons_index in self.overlapping_beacons[rnd_idx]:
                if free_beacon_places.__contains__(overlapping_beacons_index):
                    free_beacon_places.remove(overlapping_beacons_index)
            free_beacon_places.remove(rnd_idx)

        return random_beacons_set

    def variate_beacons_set(self, beacons_set, unsuccessful_attempts_count):
        # this method takes a beacons set and randomly replaces 1 beacon with a random one from available positions
        original_set = beacons_set.copy()
        beacon_pos_count = len(self.possible_beacons_positions)
        beacons_numbers_list = []
        i = 0

        for b in beacons_set:
            if b == 1:
                beacons_numbers_list.append(i)
            i += 1

        count = len(beacons_numbers_list)
        r_i = random.randint(0, count - 1)
        removed_idx = beacons_numbers_list[r_i]
        beacons_set[removed_idx] = 0
        beacons_numbers_list.remove(beacons_numbers_list[r_i])

        free_beacon_places = []
        for i in range(0, beacon_pos_count):
            k = self.possible_beacons_positions[i][0]
            free_beacon_places.append(k)

        for num in beacons_numbers_list:
            for overlapping_beacons_index in self.overlapping_beacons[num]:
                if free_beacon_places.__contains__(overlapping_beacons_index):
                    free_beacon_places.remove(overlapping_beacons_index)
            free_beacon_places.remove(num)
        # now we should have only free places left

        if free_beacon_places.__contains__(removed_idx):
            free_beacon_places.remove(removed_idx)

        length = len(free_beacon_places)
        if length == 0:
            if unsuccessful_attempts_count < 5:
                beacons_set = self.variate_beacons_set(original_set.copy(), unsuccessful_attempts_count + 1)
            else:
                beacons_set = original_set.copy()  # couldn't variate set, return the same one

            return beacons_set

        # one beacon removed, now replace it with another one
        else:
            # trying to fill all available spots
            # sometimes we can remove one beacon and add more than one
            # (it only happens when venue has very high beacon density, so the beacon limit is not reached)
            current_beacons_count = get_beacons_number_in_a_set(beacons_set)
            for idx_beacon in range(current_beacons_count, self.beacon_limit):
                free_length = len(free_beacon_places)
                if free_length == 0:
                    break

                r_i = random.randint(0, free_length - 1)
                rnd_idx = free_beacon_places[r_i]
                beacons_set[rnd_idx] = 1
                for overlapping_beacons_index in self.overlapping_beacons[rnd_idx]:
                    if free_beacon_places.__contains__(overlapping_beacons_index):
                        free_beacon_places.remove(overlapping_beacons_index)
                free_beacon_places.remove(rnd_idx)

            return beacons_set

    def run_simulated_annealing(self, dummy):
        # this method is suitable for multiprocessing
        # it runs simulated annealing algorithm for one random starting beacon set and saves result to file
        T = 1000
        k = 0.0005
        alpha_T = 0.95
        beacons_set = self.improved_generate_random_beacons_set()
        E_prev = (-1) * self.estimate_beacons_set_quality(beacons_set)
        while T > 0.0005:
            # we either move to another set or remain at previous set of beacons
            saved_set = beacons_set.copy()
            candidate_set = self.variate_beacons_set(beacons_set, 0)
            beacons_set = saved_set
            E_new = (-1) * self.estimate_beacons_set_quality(candidate_set)
            power = -(E_new - E_prev)/(k * T)
            if power > 0:
                P = 1 # calculating high P values makes no sense
            else:
                P = math.exp(power)
            r = random.random()
            if P > r:
                beacons_set = candidate_set.copy()
                E_prev = E_new

            T = T * alpha_T

        q = self.estimate_beacons_set_quality(beacons_set)

        lock.acquire()

        with open(self.out_file_name, 'a') as out_f:
            out_f.write(str(q) + ' ')

            out_string_beacons = ''
            for b in beacons_set:
                out_string_beacons += str(int(b)) + ' '

            out_f.write(out_string_beacons)
            out_f.write('\n')

        lock.release()

        return beacons_set

    def run_monte_carlo(self, dummy):
        # method is suitable for multiprocessing
        # this method creates one random beacons position and saves result to file

        beacons_set = self.improved_generate_random_beacons_set()

        q = self.estimate_beacons_set_quality(beacons_set)
        lock.acquire()
        out_f = open(self.out_file_name, 'a')
        out_f.write(str(q) + ' ')

        out_string_beacons = ''
        for b in beacons_set:
            out_string_beacons += str(int(b)) + ' '

        out_f.write(out_string_beacons)
        out_f.write('\n')

        out_f.close()
        lock.release()

        return beacons_set

    def estimate_beacons_set_quality(self, beacons_set):
        # individual beacon quality is higher if its signal covers more ground
        # covering priority areas adds more value to the beacon
        # beacon set quality is higher when beacons are spread further away from each other
        q = 0

        beacon_pos_list = []
        beacon_pos_count = self.total_beacons_positions_number
        for idx in range(0, beacon_pos_count):
            if beacons_set[idx] == 1:
                for b in self.possible_beacons_positions:
                    if b[0] == idx:
                        q += self.beacon_quality_list[idx]
                        bx = b[1]
                        by = b[2]
                        beacon_pos_list.append([bx, by])

        min_distances_sum = self.venue.get_sum_of_minimal_distances(beacon_pos_list)
        dist_coeff = 1.0
        q += min_distances_sum * dist_coeff
        q /= self.beacon_limit

        return q

    def visualize_beacons(self, beacons_list):
        # this method fills debug_grid matrix with values, so that output colored image can be produced
        # 1) beacon positions are visualized
        beacon_pos_count = self.total_beacons_positions_number
        for idx in range(0, beacon_pos_count):
            if beacons_list[idx] == 1:
                for b in self.possible_beacons_positions:
                    if b[0] == idx:
                        bx = b[1]
                        by = b[2]
                        self.venue.debug_grid[bx][by] = 3

        # 2) beacon signal is visualized (if it's above the intersection threshold (example: -78dBm))
        beacon_numbers = create_list_of_beacon_numbers(beacons_list)
        for i in range(0, len(self.M_coverage)):
            for j in range(0, len(self.M_coverage[i])):
                elements = [x for x in self.M_coverage[i][j]]
                for one_element in elements:
                    if one_element[1] > self.sig_model.min_intersection_RSSI:
                        for beacon_number in beacon_numbers:
                            if beacon_number == one_element[0]:
                                if self.venue.debug_grid[i][j] == 1:
                                    self.venue.debug_grid[i][j] = 2

    def get_best_beacons_positions(self):
        # searches for the best beacon set in file
        best_quality = 0
        results = open(self.out_file_name, 'r')
        beacons_list = []
        lines = results.readlines()

        for l in lines:
            line_elements = l.split()
            q = float(line_elements[0])
            if q > best_quality:
                best_quality = q
                beacons_list = []
                # print('l:', l)
                for element in line_elements[1:]:
                    beacons_list.append(int(element))
                    # print('appending: ', int(element))

        results.close()
        print('best quality:', best_quality)
        return beacons_list

    def place_beacons(self):
        self.generate_possible_beacon_positions()
        self.venue.delete_grids()

        beacon_pos_count = len(self.possible_beacons_positions)

        full_set = []
        for i in range(0, self.total_beacons_positions_number):
            full_set.append(0)

        for b_pos in self.possible_beacons_positions:
            idx = b_pos[0]
            full_set[idx] = 1

        # multipool
        total_iterations = self.iterations_number

        params = []
        for i in range(0, total_iterations):
            params.append(1)

        l = mp.Lock()

        pcount = 2 * mp.cpu_count() - 1
        print('pool threads: ' + str(pcount))
        pool = mp.Pool(processes=pcount, initializer=init, initargs=(l,))

        try:
            for _ in tqdm.tqdm(pool.imap_unordered(self.run_simulated_annealing, params), total=len(params)):
                pass

        except Exception:
            print("a worker failed, aborting...")
            pool.close()
            pool.terminate()
        else:
            pool.close()
            pool.join()

        best_set = self.get_best_beacons_positions()

        self.visualize_beacons(best_set)
        self.venue.save_output_picture()
        self.save_debug_information(best_set)

        self.venue.generate_kml(best_set, self.possible_beacons_positions, self.total_beacons_positions_number)

    def save_debug_information(self, beacons_list):
        beacon_pos_count = self.total_beacons_positions_number
        with open('debug_out.txt', 'a+') as dbgout:
            dbgout.write('\n')
            for idx in range(0, beacon_pos_count):
                if beacons_list[idx] == 1:
                    for b in self.possible_beacons_positions:
                        if b[0] == idx:
                            bx = b[1]
                            by = b[2]
                            out_line = str(idx) + ' : ' + str(bx) + ' ' + str(by) + '\n'
                            dbgout.write(out_line)


def get_beacons_number_in_a_set(beacons_set):
    dbg_sum = 0
    for c in beacons_set:
        if c == 1:
            dbg_sum += 1
    return dbg_sum


def create_list_of_beacon_numbers(beacons_set):
    output = []
    i = 0
    for element in beacons_set:
        if element == 1:
            output.append(i)
        i += 1
    return output


def init(l):
    global lock
    lock = l