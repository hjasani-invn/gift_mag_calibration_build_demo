import re
import os
import zipfile
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.use('TkAgg')
#mpl.use('Agg')
from scipy import stats
import statistics

def delete_files(folder, pattern, Flag):

    patterns = []
    patterns.append(pattern)

    deleted_file_list = []

    for pattern in patterns:
        if Flag == True:
            list_ = [f for f in os.listdir(folder) if re.match(pattern, f) is not None ]
        else:
            list_ = [f for f in os.listdir(folder) if re.match(pattern, f) is None]
        if len(list_) > 0 :
            for l in list_:
                f = os.path.join(folder, l)
                if os.path.isdir(f):
                    del_list = delete_files(f, pattern, Flag)
                    deleted_file_list.extend(del_list)
                else:
                    deleted_file_list.append(f)
                    print(f)
                    os.remove(f)

    return deleted_file_list

def unpack_archive(folder, pattern):

    patterns = []
    patterns.append(pattern)

    unpacked_file_list = []

    for pattern in patterns:
        list = [f for f in os.listdir(folder) if re.match(pattern, f) is not None]
        if len(list) > 0 :
            for l in list:
                one_zip = os.path.join(os.path.join(folder, l))
                print(one_zip)
                unpacked_file_list.append(one_zip)
                zf = zipfile.ZipFile(one_zip, 'r')
                zf.extractall(path=folder)

    return unpacked_file_list

def rename_files(folder, pattern1, pattern2):

    patterns = []
    patterns.append(pattern1)

    input_file_list = []

    for pattern in patterns:
        #print('pattern: ', pattern)
        list = [f for f in os.listdir(folder) if re.match(pattern1, f) is not None ]
        if len(list) > 0 :
            for l in list:
                input_file_list.append(os.path.join(folder, l))
                os.rename(os.path.join(folder, l), os.path.join(folder, pattern2))
    #print(input_file_list)

    return input_file_list

def create_json(folder, pattern):

    patterns = []
    patterns.append(pattern)

    input_file_list = []

    for pattern in patterns:
        #print('pattern: ', pattern)
        list = [f for f in os.listdir(folder) if re.match(pattern, f) is not None ]
        if len(list) > 0 :
            for l in list:
                print("\t\t[\"", end='')
                print(folder, end='')
                print("\"],")
                input_file_list.append(folder)
    #print(input_file_list)

    return input_file_list

def plot_floor(folder, pattern):

    patterns = []
    patterns.append(pattern)

    path, folder1 = os.path.split(folder)

    input_file_list = []

    for pattern in patterns:
        #print('pattern: ', pattern)
        list = [f for f in os.listdir(folder) if re.match(pattern, f) is not None ]
        if len(list) > 0:
            for l in list:
                print(folder)
                input_file_list.append(folder)
                #print(os.path.join(folder, l))
                logArray = [line.rstrip('\n') for line in open(os.path.join(folder, l))]
                floors = []
                x = []
                for i in range(len(logArray)):
                    line = logArray[i]
                    line1 = line.strip()
                    lineSplitted = re.split(r'\s+', line1)
                    grid_line_floor = float(lineSplitted[2])
                    floors.append(grid_line_floor)
                    time = float(lineSplitted[5])/1000
                    x.append(time)
                #print(x)
                #print(floors)
                plt.plot(x, floors)
                plt.axis([0, max(x), min(floors)-0.5, max(floors)+0.5])
                plt.legend([folder1], loc='lower center')
                plt.grid()
                plt.xlabel('time (s)')
                plt.ylabel('floor')
                plt.show()
                #plt.close()

    #print(input_file_list)

    return input_file_list

def plot_floor_float(folder, pattern):

    patterns = []
    patterns.append(pattern)

    path, folder1 = os.path.split(folder)

    input_file_list = []

    for pattern in patterns:
        #print('pattern: ', pattern)
        list = [f for f in os.listdir(folder) if re.match(pattern, f) is not None ]
        if len(list) > 0:
            for l in list:
                print(folder)
                input_file_list.append(folder)
                #print(os.path.join(folder, l))
                logArray = [line.rstrip('\n') for line in open(os.path.join(folder, l))]
                floors = []
                floors1 = []
                x = []
                for i in range(len(logArray)):
                    line = logArray[i]
                    if line.find("pf_mix") == -1:
                        continue
                    line1 = line.strip()
                    lineSplitted = re.split(r'\s+', line1)
                    grid_line_floor = float(lineSplitted[4])
                    grid_line_floor1 = float(lineSplitted[5])
                    floors.append(grid_line_floor)
                    floors1.append(grid_line_floor1)
                    time = float(lineSplitted[0])/1000
                    x.append(time)
                #print(x)
                #print(floors)
                plt.plot(x, floors)
                plt.plot(x, floors1)
                plt.axis([0, max(x), 0, max(floors)+0.5])
                #plt.legend([folder1], loc='lower center')
                #plt.grid()
                plt.xlabel('time (s)')
                plt.ylabel('floor')
                #plt.show()
                #plt.close()

    #print(input_file_list)

    return input_file_list

def plot_converter_height(folder, pattern):

    patterns = []
    patterns.append(pattern)

    path, folder1 = os.path.split(folder)

    input_file_list = []

    for pattern in patterns:
        #print('pattern: ', pattern)
        list = [f for f in os.listdir(folder) if re.match(pattern, f) is not None ]
        if len(list) > 0:
            for l in list:
                print(folder)
                input_file_list.append(folder)
                #print(os.path.join(folder, l))
                logArray = [line.rstrip('\n') for line in open(os.path.join(folder, l))]
                floors = []
                x = []
                for i in range(len(logArray)):
                    line = logArray[i]
                    line1 = line.strip()
                    lineSplitted = re.split(r'\s+,', line1)
                    if len(lineSplitted) < 10:
                        continue
                    grid_line_floor = float(lineSplitted[9])
                    floors.append(grid_line_floor)
                    time = float(lineSplitted[0])/1000
                    x.append(time)
                f_0 = floors[0]
                for i in range (len (floors)):
                    floors[i] -= f_0
                    floors[i] /= 3.8
                    floors[i] += 4
                #print(x)
                #print(floors)
                plt.plot(x, floors)
                plt.axis([0, max(x), 0, max(floors)+0.5])
                #plt.axis([0, max(x), 0, max(floors)+0.5])
                #plt.legend([folder1], loc='lower center')
                #plt.grid()
                plt.xlabel('time (s)')
                plt.ylabel('floor')
                #plt.show()
                #plt.close()

    #print(input_file_list)

    return input_file_list

def check_flickering(folder, pattern):

    patterns = []
    patterns.append(pattern)

    path, folder1 = os.path.split(folder)

    input_file_list = []

    for pattern in patterns:
        list = [f for f in os.listdir(folder) if re.match(pattern, f) is not None ]
        if len(list) > 0:
            for l in list:
                #print(folder, end=' ')
                #input_file_list.append(folder)
                #print(os.path.join(folder, l), end=' ')
                logArray = [line.rstrip('\n') for line in open(os.path.join(folder, l))]
                floors = []
                x = []
                for i in range(len(logArray)):
                    line = logArray[i]
                    line1 = line.strip()
                    lineSplitted = re.split(r'\s+', line1)
                    grid_line_floor = float(lineSplitted[2])
                    floors.append(grid_line_floor)
                    time = float(lineSplitted[5])/1000
                    x.append(time)
                #print(x)
                df_change = 0
                dft_change = 0
                dfc = []
                dftc = []
                for i in range(len(floors)-1):
                    df = floors[i+1] - floors[i]
                    if df != 0:
                        df_change = df
                        dft_change = x[i+1]
                        dfc.append(df_change)
                        dftc.append(dft_change)
                        #print(x[i+1], "   ",  df)
                count = 0
                for i in range (len(dfc)-2):
                    if dfc[i] < dfc[i+1] and dfc[i+1] > dfc[i+2]:
                        if dftc[i+2] - dftc[i] < 10:  # seconds
                            #print('--    ', dftc[i+1], '    ',dfc[i+1])
                            count += 1
                #print(os.path.join(folder, l), count)
                input_file_list.append([os.path.join(folder, l), count])

    #print(input_file_list)

    return input_file_list

def get_files_names(folder, pattern):

    patterns = []
    patterns.append(pattern)

    input_file_list = []

    for pattern in patterns:
        list1 = [f for f in os.listdir(folder) if re.match(pattern, f) is not None]
        if len(list1) > 0:
            for l in list1:
                input_file_list.append(os.path.join(folder, l))

    return input_file_list


def check_delays(route_name, phone_name,file_list):

    tracks = []
    if len(file_list) > 0:
        # all tracks
        len_time = []
        for file in file_list:
            logArray = [line.rstrip('\n') for line in open(file)]
            floors = []
            time = []
            for i in range(len(logArray)):
                line = logArray[i]
                line1 = line.strip()
                lineSplitted = re.split(r'\s+', line1)
                grid_line_floor = float(lineSplitted[2])
                floors.append(grid_line_floor)
                t = float(lineSplitted[5])/1000
                time.append(t)
            len_time.append(len(time))
            tracks.append([file, time, floors])

        # reference track
        min_len = min(len_time)
        ref_track = []
        ref_track_time = []
        ref_track_floor = []
        ref_track_floor_set = set()
        for i in range(min_len):
            time_i = []
            floor_i = []
            for track in tracks:
                time_i.append(track[1][i])
                floor_i.append(track[2][i])
            t_mean = statistics.mean(time_i)
            f_med = round(statistics.median(floor_i))
            ref_track.append([t_mean, f_med])
            ref_track_time.append(t_mean)
            ref_track_floor.append(f_med)
            ref_track_floor_set.add(f_med)
            ##print("f_med", i, "  ", t_mean, "  ", f_med)

        plt.plot(ref_track_time, ref_track_floor)
        plt.axis([0, max(ref_track_time), min(ref_track_floor) - 0.5, max(ref_track_floor) + 0.5])
        plt.legend(['reference'], loc='lower center')
        plt.grid()
        plt.xlabel('time (s)')
        plt.ylabel('floor')
        #plt.show()
        ##print(os.getcwd()) # working folder
        if not os.path.isdir('./references/'):
            os.mkdir('./references/')
        plt.savefig('./references/' + route_name + '-' + phone_name + '.png')
        plt.close()

        # difference between some track and reference track
        for track in tracks:
            print(track[0])
            # show image of track
            #folder, file = os.path.split(track[0])
            #plot_floor(folder, file)
            time_for_diff_floor_negative = []
            time_for_diff_floor_positive = []
            max_time_for_diff_floor_negative = 0
            max_time_for_diff_floor_positive = 0

            track_floor_set = set()
            i = 0
            while i < min_len:
                f = track[2][i]
                track_floor_set.add(f)
                i += 1

            i = 1
            while i < min_len:
                #print("tracks  ", i, "  ", ref_track[i][1], "   ", track[2][i])
                if ref_track[i][1] == track[2][i]:
                    i += 1
                    continue
                #print(i, "   ", ref_track[i][1], "   ", track[2][i])
                ref_track_first = False
                ref_track_second = False
                if ref_track[i][1] != ref_track[i-1][1]:
                    ref_track_first = True
                if track[2][i] != track[2][i-1]:
                    ref_track_second = True

                m1 = 0
                m2 = 0
                if ref_track_first:
                    if not (ref_track[i][1] in track_floor_set):
                        while (i+m1) < min_len-1 and not (ref_track[i + m1][1] in track_floor_set):
                            #print("tracks_12 ", m1, "  ", i + m1, "   ", ref_track[i + m1][1])
                            m1 += 1
                        #print("tracks_13 ", m1, "  ", i + m1, "   ", ref_track[i+m1][1])

                if ref_track_second:
                    if not (track[2][i] in ref_track_floor_set):
                        while (i+m2) < min_len-1 and not (track[2][i + m2] in ref_track_floor_set):
                            #print("tracks_10 ", m2, "  ", i + m2, "   ", track[2][i + m2])
                            m2 += 1
                        #print("tracks_11 ", m2, "  ", i + m2, "   ", track[2][i + m2])

                i1 = i + m1
                i2 = i + m2

                if True:
                    while True:
                        n1 = 0
                        while (i2 + n1) < (min_len-1) and ref_track[i1][1] != track[2][i2 + n1]:
                            #print("tracks_0  ", n1, "  ", i2 + n1, "   ", ref_track[i1][1],  "   ",  track[2][i2 + n1])
                            n1 += 1
                        #print("tracks_00  ", n1, "  ", i1 + n1, "   ", ref_track[i1][1],  "   ", track[2][i1 + n1])
                        if (i2 + n1) < (min_len-1) or i1 == (min_len-1):
                            break
                        else:
                            i1 += 1
                            if i1 == (min_len-1):
                                break
                    while True:
                        n2 = 0
                        while (i1 + n2) < (min_len-1) and ref_track[i1 + n2][1] != track[2][i2]:
                            #print("tracks_1  ", n2, "  ", i1 + n2, "   ", ref_track[i1 + n2][1], "   ", track[2][i2])
                            n2 += 1
                        #print("tracks_11  ", n2, "  ", i1 + n2, "   ", ref_track[i1 + n2][1], "   ", track[2][i2])
                        if (i1 + n2) < (min_len - 1) or i2 == (min_len-1):
                            break
                        else:
                            i2 += 1
                            if i2 == (min_len-1):
                                break
                    if n1 <= n2:
                        print("time  ", track[1][i2 + n1], "    ",  ref_track[i][0])
                        dt = abs(track[1][i2 + n1] - ref_track[i][0])
                    else:
                        print("time  ", ref_track[i1 + n2][0], "    ", track[1][i])
                        dt = abs(ref_track[i1 + n2][0] - track[1][i])
                    n = min(n1, n2)

                print("dt = ", dt)
                if ref_track[i][1] > ref_track[i-1][1] or track[2][i] > track[2][i-1]:
                    time_for_diff_floor_positive.append(dt)
                if ref_track[i][1] < ref_track[i-1][1] or track[2][i] < track[2][i-1]:
                    time_for_diff_floor_negative.append(dt)
                i = max(i1, i2)
                i += n
                #print("tracks_14  ", i,  "   ", ref_track[i][1], track[2][i])

            if len(time_for_diff_floor_positive) > 0:
                max_time_for_diff_floor_positive = max(time_for_diff_floor_positive)
            if len(time_for_diff_floor_negative) > 0:
                max_time_for_diff_floor_negative = max(time_for_diff_floor_negative)
            track.append(max_time_for_diff_floor_positive)
            track.append(max_time_for_diff_floor_negative)

            # flickering
            max_flickering_time = 10  # seconds
            dfc = [] # floor change
            dftc = [] # time of floor change
            for i in range(len(track[2]) - 1):
                #print(track[2][i])
                df = track[2][i+1] - track[2][i]
                if df != 0:
                    dfc.append(df)
                    dftc.append(track[1][i])
                    #print(i, "    ", track[1][i], "   ",  df)
            count = 0
            i = 0
            while i < (len(dfc) - 1):
                if ((dfc[i] < 0 and dfc[i + 1] > 0) or
                    (dfc[i] > 0 and dfc[i + 1] < 0)):
                    if (dftc[i + 1] - dftc[i]) < max_flickering_time:
                        #print('--    ', dftc[i], '    ',dfc[i])
                        count += 1
                        i += 1
                i += 1
            track.append(count)

    return tracks

