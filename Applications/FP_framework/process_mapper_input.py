__author__ = 'kotik'

# Calls archive processing and data conversion tools.
#
import os
import glob
import re
import shutil
import subprocess
import process_archives
import statistics
import collections


def sort_tracks(main_settings):
    data_path = main_settings['mapper_data_path']
    os.chdir(data_path) # we have data0, data1... folders  at this location, OR data_from_archive_0, data_from_archive_1
    data_folders = [x for x in glob.glob('data*') if os.path.isdir(x)]
    all_tracks_paths = []

    for dat in data_folders:
        os.chdir(data_path+'/'+dat)
        trk_files = [f for f in os.listdir() if re.match(r'(Track_.*\.trk)', f)]
        all_files = [f for f in os.listdir()]
        # sort tracks here, creating a separate folder for each of them

        iterator = 0
        for trk in trk_files:
            iterator += 1
            mask = trk[6:-4]   # selecting date&time from track file name
            mask_regex = r'.*' + re.escape(mask) + r'\..*'
            files_from_one_track = [f for f in all_files if re.match(mask_regex, f)]
            iter_d = 'track' + str(iterator) + '/'
            iterated_path = os.path.join('.', iter_d)
            tst_path = os.path.join(os.getcwd(), iter_d)

            all_tracks_paths.append(tst_path)
            os.makedirs(iterated_path, exist_ok=True)  # create subdirectories track0/ , track1/ etc...
            for f in files_from_one_track:
                full_file_path = os.path.join('.', f)
                shutil.move(full_file_path, iterated_path)

    return all_tracks_paths


def convert_data(main_settings, all_tracks_paths):
    os.chdir(main_settings['main_path'])

    for track_path in all_tracks_paths:

        # dbg_out = open('//cayyc-share01/users/ykotik/Bad_data/debug.log', 'a+')
        # dbg_out.write(track_path)
        # dbg_out.write('\n')

        MapperDataConverter = 'MapperDataConverter.exe' + ' --input_path ' + track_path + ' --output_path ' + track_path + ' --settings ' + main_settings['json_FP_builder']

        print(MapperDataConverter)
        return_code = subprocess.call(MapperDataConverter, shell=True)

        wifi_log_file = [f for f in os.listdir(track_path) if re.match('wifi_.*', f)]
        ble_log_file = [f for f in os.listdir(track_path) if re.match('ble_.*', f)]

        ModelDataConverter = 'ModelDataToTPNConverter.exe'
        path_to_txt = track_path + '/irl_data_mt_converter.txt '
        path_to_dat = track_path + '/tpn.dat'
        ModelDataConverter += ' --input_model '
        ModelDataConverter += path_to_txt
        if len(wifi_log_file):
            ModelDataConverter += ' --input_wifi '
            path_to_wifi = track_path + '/' + wifi_log_file[0]
            ModelDataConverter += path_to_wifi
        if len(ble_log_file):
            ModelDataConverter += ' --input_ble '
            path_to_ble = track_path + '/' + ble_log_file[0]
            ModelDataConverter += path_to_ble
        ModelDataConverter += ' --output_tpn '
        ModelDataConverter += path_to_dat
        ModelDataConverter += ' > 1.txt'

        print(ModelDataConverter)

        return_code = subprocess.call(ModelDataConverter, shell=True)

        # dbg_out.close()


def process_input(main_settings):

    os.chdir(main_settings['mapper_data_path'])

     # usually Mapper data comes in archives, if no archives were found - no problem, but the unpacked data should be in /data*/ folders
    if main_settings['mapper_data_is_archived'] == 1:
        process_archives.process()
        # tracks are sorted into /track1/, /track2/ etc (if *.trk file is not present - files with that timestamp don't belong to a track)
        all_tracks_paths_1 = sort_tracks(main_settings)

    all_tracks_paths = []
    for root, dirs, files in os.walk(main_settings['mapper_data_path']):
        for dir in dirs:
            d = os.path.join(root, dir)
            if re.search(r'track', d):
                all_tracks_paths.append(d)

    convert_data(main_settings, all_tracks_paths)
    calculate_uuid(main_settings, all_tracks_paths)



def remove_bad_data(main_settings):
    all_tracks_paths = []
    for root, dirs, files in os.walk(main_settings['mapper_data_path']):
        for dir in dirs:
            d = os.path.join(root, dir)
            if re.search(r'track', d):
                all_tracks_paths.append(d)

    for track_path in all_tracks_paths:
        os.chdir(track_path)
        mg_files = [f for f in os.listdir() if re.match(r'mg_enu_.*\.log', f)]
        mg_enu = mg_files[0]
        result = check_mg_enu(mg_enu)

        os.chdir(main_settings['mapper_data_path'])

        if result == 0:
            start = track_path.find('data')
            substr = track_path[start:]

            path_to_move =  main_settings['path_to_bad_mapper_data'] + '/' + substr + '/'
            print('source: ', track_path)
            print('dest: ', path_to_move)

            shutil.move(track_path, path_to_move)


def check_mg_enu(path_to_mg):
    mg_hor = []
    mg_vert = []
    file = open(path_to_mg)
    for l in file.readlines():
            current_line = l.split(',')
            mg_x = float(current_line[2])
            mg_y = float(current_line[3])
            mg_z = float(current_line[4])
            mg_hor.append( ((mg_x**2) + (mg_y**2))**0.5 )
            mg_vert.append(mg_z)

    file.close()

    mean_mg_hor = 0
    mean_mg_vert = 0

    if mg_hor:
        mean_mg_hor = statistics.mean(mg_hor)
    if mg_vert:
        mean_mg_vert = statistics.mean(mg_vert)

    Mg_hor_real = 24.41
    Mg_z_real = -42.13

    threshold = 20

    if abs(mean_mg_hor - Mg_hor_real) > threshold:
        return 0

    if abs(mean_mg_vert - Mg_z_real) > threshold:
        return 0

    return 1


def calculate_uuid(main_settings, all_tracks_paths):
    os.chdir(main_settings['mapper_data_path'])

    ble_file_list = []

    for track_path in all_tracks_paths:
        files = [f for f in os.listdir(track_path) if re.search('ble.*\.log', f) is not None]
        for file in files:
            ble_file_list.append(os.path.join(track_path, file))

    uuid_list = []

    for ble_file in ble_file_list:
        ble = open(ble_file, 'r')
        for line in ble.readlines():
            line_elements = line.split(',')
            uuid_element = line_elements[8] # WARNING: for old format change it to line_elements[7]
            if len(uuid_element) > 35:
                uuid_list.append(uuid_element)
        ble.close()

    counter = collections.Counter(uuid_list)
    counter_list = list(counter.items())

    out = os.path.join(main_settings['mapper_data_path'], 'uuid.txt')
    outfile = open(out, 'w')

    for element in counter_list:
        out_str = str(element) + '\n'
        outfile.write(out_str)

    outfile.close()