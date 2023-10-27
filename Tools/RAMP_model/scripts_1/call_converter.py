__author__ = 'kotik'

import os
import re
import subprocess
import shutil
import contextlib

def run(converter_path, model_out_path):
    os.chdir(model_out_path)

    subdirs = [x[0] for x in os.walk(os.getcwd())]
    subdirs.pop(0)

    input_data_folders = []
    for subdir in subdirs:
        # selecting subfolders which have the data files in them, appending to list "input_data_folders"
        src_files = [f for f in os.listdir(subdir) if re.match(r'(RAMP_out.*\.log)', f)]
        if src_files:
            input_data_folders.append(subdir)

    call_converter(input_data_folders, converter_path)
    # fix_files(input_data_folders)


def call_converter(input_data_folders, converter_path):
    os.chdir(converter_path)

    for track_folder in input_data_folders:
        os.chdir(track_folder)
        IRL_out_file = [f for f in os.listdir() if re.match(r'(RAMP_out.*\.log)', f)]
        path_to_IRL = track_folder + '/' + IRL_out_file[0]
        path_to_dat = track_folder + '/' + 'tpn.dat'

        os.chdir(converter_path)
        command = 'ModelDataToTPNConverter.exe '
        command += '--input_model '
        command += path_to_IRL
        command += ' --output_tpn '
        command += path_to_dat
        print(command)
        return_code = subprocess.call(command, shell=True)


def fix_files(input_data_folders):
    # this function just fixes the output of Matlab model and is not used anymore
    # but maybe some editing will be needed again, so it is left here
    for track_folder in input_data_folders:
        os.chdir(track_folder)
        IRL_out_file = [f for f in os.listdir() if re.match(r'(RAMP_out.*\.log)', f)]
        path_to_file = track_folder + '/' + IRL_out_file[0]
        # print(path_to_file)
        RAMP_file = open(path_to_file)
        new_RAMP_file = open('RAMP_out_fixed.log', 'w+')
        lines = RAMP_file.readlines()
        for l in lines:
            edited_l = re.sub(r',  0$', ', 1, 0', l)
            new_RAMP_file.write(edited_l)

        RAMP_file.close()
        new_RAMP_file.close()

        with contextlib.suppress(FileNotFoundError):
            os.remove('RAMP_out.log')

        shutil.move('RAMP_out_fixed.log', 'RAMP_out.log')


converter_path = 'C:/FP_Generation/Model_Test/'      # converter executable should be there
model_out_path = 'C:/FP_Generation/Model_Test/ideal_grid_noise/data/'
# model_out_path = 'C:/FP_Generation/Model_Test/test_0/data/'

run(converter_path, model_out_path)
