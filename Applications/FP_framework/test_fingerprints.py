__author__ = 'kotik'

# CLass FingerprintsTester.
# FingerprintsTester.process_tests() method runs integration tests for fingerprints.
# Input settings file contains path to test tracks folders. Script runs navigator app from current folder.
# Then it compares output position at each checkpoint with its real position using 'pos_20***.log' and '*checkpoints.txt'
# The resulting navigation accuracy and availability is calculated. Output file is "test_results.txt"

import subprocess
import os
import re
import math
import shutil
import statistics
import contextlib

class Checkpoint(object):
    def __init__(self, x, y, floor, time):
        self.x = x
        self.y = y
        self.floor = floor
        self.time = time
        self.available = 1


class NavigationGrade(object):
    def __init__(self, pos_error, floor_error, availability):
        self.pos_error = pos_error
        self.floor_error = floor_error
        self.availability = availability


class FingerprintsTester(object):

    def __init__(self, settings):
        self.settings = settings
        self.ref_checkpoints = []
        self.timeframe = 500
        self.NavGradeMatrix = []
        self.mean_pos_error = 0
        self.mean_floor_error = 0
        self.mean_availability = 0
        self.median_pos_error = 0
        self.median_floor_error = 0
        self.median_availability = 0

    def process_tests(self):
        os.chdir(self.settings['tests_path'])
        dirs = [f for f in os.listdir(self.settings['tests_path']) if (os.path.isdir(f) and re.match(r'track.*', f))]
        for dir in dirs:
            full_path_to_track = self.settings['tests_path'] + dir
            self.call_navigator(self.settings['navigator_name'], full_path_to_track)

        self.parse_reference_checkpoints(self.settings['venue_checkpoints'])
        self.get_track_ref_positions()
        self.calculate_accuracy()

    def call_navigator(self, nav_name, full_path_to_track):
        with contextlib.suppress(FileNotFoundError):
            os.remove('IndoorDemo.cfg')
        nav = nav_name
        nav += ' ' + full_path_to_track + '/' + ' ' + full_path_to_track + '/'
        print('calling navigator: ', nav)
        return_code = subprocess.call(nav, shell=True)
        shutil.copy('pf_log_init.log', full_path_to_track)

    def parse_reference_checkpoints(self, ref_checkpoints_file):
        for l in open(ref_checkpoints_file).readlines():
            current_line = l.split(',')
            x = float(current_line[2])
            y = float(current_line[3])
            floor = float(current_line[4])
            self.ref_checkpoints.append(Checkpoint(x, y, floor, 0))

    def parse_track_checkpoints(self, track_checkpoints_file):
        track_checkpoints = []
        for l in open(track_checkpoints_file).readlines():
            current_line = l.split(',')
            x = 0
            y = 0
            floor = 0
            time = 1000 * float(current_line[1])
            track_checkpoints.append(Checkpoint(x, y, floor, time))
        return track_checkpoints

    def parse_pflog(self, pflog_file, track_checkpoints):
        pf1_lines = [l for l in open(pflog_file).readlines() if re.match(r'.*pf1:.*', l)]

        for chp in track_checkpoints:
            target_time = chp.time
            dt = self.timeframe
            for l in pf1_lines:
                if abs(float(l.split()[0]) - target_time) < dt:
                    dt = abs(float(l.split()[0]) - target_time)
                    chp.x = float(l.split()[2])
                    chp.y = float(l.split()[3])
                    chp.floor = float(l.split()[4])

            if dt >= self.timeframe:
                chp.available = 0
                # if filter couldn't initialize in time

        return track_checkpoints

    def get_track_ref_positions(self):
        dirs = [f for f in os.listdir() if (os.path.isdir(f) and re.match(r'track.*', f))]
        # example:  /track02/, /track03/, etc
        for d in dirs:
            os.chdir(d)
            pos_files = [f for f in os.listdir() if re.match(r'pos.*\.log', f)]
            print(os.getcwd()) # should be the folder /trackXX/
            track_checkpoints = self.parse_track_checkpoints(pos_files[0])
            # track_checkpoints contains only times in ms at this the moment
            # now we parse pf_log_init.log located in /track01/ (for example)
            track_checkpoints = self.parse_pflog('pf_log_init.log', track_checkpoints)
            # track_checkpoints now has positions taken from navigator track output
            track_cp_out = open('track_cp_out.txt', 'w')

            i = 0
            for cp in track_checkpoints:
                pos_error = math.sqrt((self.ref_checkpoints[i].x - cp.x)**2 + (self.ref_checkpoints[i].y - cp.y)**2)
                floor_error = abs(cp.floor - self.ref_checkpoints[i].floor)

                line = ''
                line += str(cp.available) + ' '
                line += str(cp.time/1000) + ' '
                line += str(cp.x) + ' '
                line += str(cp.y) + ' '
                line += str(cp.floor) + ' '
                line += str(pos_error) + ' '
                line += str(floor_error) + '\n'

                track_cp_out.write(line)
                i += 1

            os.chdir('..')

    def calculate_accuracy(self):
        # parsing through each trackXX/track_cp_out.txt file
        dirs = [f for f in os.listdir() if (os.path.isdir(f) and re.match(r'track.*', f))]

        track_num = 0

        for d in dirs:
            os.chdir(d)
            tmp = []
            for l in open('track_cp_out.txt').readlines():
                NG = NavigationGrade(float(l.split()[5]), float(l.split()[6]), float(l.split()[0]))
                tmp.append(NG)
            self.NavGradeMatrix.append(tmp)

            track_num += 1
            os.chdir('..')

        # now we have NavGradeMatrix (a list of lists) shaped like : (rows = NumTracks , columns = NumCheckpoints)

        pos_data = []
        floor_data = []
        availability_data = []

        tmp = []

        for i in range(0, len(self.NavGradeMatrix)):
            for j in range(0, len(self.NavGradeMatrix[0])):
                availability_data.append(self.NavGradeMatrix[i][j].availability)
                if self.NavGradeMatrix[i][j].availability > 0:

                    tmp.append(self.NavGradeMatrix[i][j].pos_error)

                    pos_data.append(self.NavGradeMatrix[i][j].pos_error)
                    floor_data.append(self.NavGradeMatrix[i][j].floor_error)
            print(tmp)
            tmp = []

        self.mean_pos_error = statistics.mean(pos_data)
        self.mean_floor_error = statistics.mean(floor_data)
        self.mean_availability = statistics.mean(availability_data)

        self.median_pos_error = statistics.median(pos_data)
        self.median_floor_error = statistics.median(floor_data)
        self.median_availability = statistics.median(availability_data)

        str_out = str(self.mean_pos_error) + ' ' + str(self.mean_floor_error) + ' ' + str(self.mean_availability) + '\n'
        str_out += str(self.median_pos_error) + ' ' + str(self.median_floor_error) + ' ' + str(self.median_availability) + '\n'

        accuracy_out = open('test_results.txt', 'w')
        accuracy_out.write(str_out)






