# -*- coding: utf-8 -*-

# fpbl_runner - fp_builder console command line wrapper
# python 3.6 is required
# Python script for running of FpBuilder console from command line.
# The script set folder settings of venue.json from command line parameters,
# run FpBuilder console and provide saving of all fingerprints, grids, logs
# and console output in specified folder.
# dchurikov@invensense.com
# Copyright (C) TDK-Invensense 2020

import argparse
import os
import sys
import json

import re

import subprocess
import multiprocessing
import tqdm
        

def search_tpn_dat(template, root):
    list = []
    for cur, _dirs, files in os.walk(root):
        for f in files:
            if re.match(template, f) is not None:
                path = os.path.join(cur, f)
                list.append(path)
    return list


def run_fp_builder(fp_builder_exe, params):

    params.settingsJson['folder_in'] = params.input_path
    params.settingsJson['folder_fp'] = params.output_path
    params.settingsJson['folder_grid'] = params.output_path
    params.settingsJson['folder_out'] = params.output_path

    os.makedirs(params.output_path, mode=0o777, exist_ok=True)

    # command line compiling
    command_line = fp_builder_exe
    print('fp_builder: ' + fp_builder_exe)

    #settins json
    json_name = os.path.join(params.output_path, 'venue.json')
    with open(os.path.join(params.output_path, json_name), 'w') as outfile:
        json.dump(params.settingsJson, outfile, sort_keys=True, indent=4, separators=(',', ': '))
    print('settings/venue.json: ' + json_name)
    command_line = command_line + ' --settings ' + json_name

    #ignore list
    if params.ignore_listJson != None:
        json_name = os.path.join(params.output_path, 'ignore_list.json')
        with open(os.path.join(params.output_path, json_name), 'w') as outfile:
            json.dump(params.ignore_listJson, outfile, sort_keys=True, indent=4, separators=(',', ': '))
        print('ignore list: ' + json_name)
        command_line = command_line + ' --ignore_list ' + json_name

    # ignore list
    if params.input_listJson!= None:
        json_name = os.path.join(params.output_path, 'input_list.json')
        with open(os.path.join(params.output_path, json_name), 'w') as outfile:
            json.dump(params.input_listJson, outfile, sort_keys=True, indent=4, separators=(',', ': '))
        print('dataset list: ' + json_name)
        command_line = command_line + ' --input_list ' + json_name

    #output stream
    f = open(os.path.join(params.output_path, "fp_builder.log"), "w")
    #print (command_line)
    #subprocess.call(command_line, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.call(command_line, stdout=f)
    f.close()


def set_critical_parameter(param, param_name):
    if param is None:
        #print colored('No ' + param_name + ' specified', 'yellow')
        print('No ' + param_name + ' specified')
        sys.exit()
    else:
        print (param_name + ': ' + param)
        return param


def set_extended_parameter(param, param_name, param_default):
    if param is None:
        ext_param = param_default
    else:
         ext_param = param
    print (param_name + ': ' + ext_param)
    return ext_param


class Params:
    input_path = ''
    output_path = ''
    settingsJson = None
    ignore_listJson = None
    input_listJson = None


if __name__ == '__main__':

    # command line keys
    parser = argparse.ArgumentParser(description="fp_builder console command line wrapper")
    parser.add_argument("--input_data_path", "--i", help="path of input data (survey datasets)")
    parser.add_argument("--output_path", "--o", help="output folder (for fingerprints and grids)")
    parser.add_argument("--settings_json", help="fpbl settings json file (venue.json)")
    parser.add_argument("--exe_path", help="path of fp_builder console exe-file")
    parser.add_argument("--exe_name", help="fp_builder console exe-file name")
    parser.add_argument("--ignore_list", help="ignore list json file (ignore_list.json)")
    parser.add_argument("--input_list", help="input dataset list json file (dataset_list.json)")
    # parse command line
    args = parser.parse_args()

    # set parameters for data processing
    p = Params()

    p.input_path = set_critical_parameter(args.input_data_path, "input_data_path")
    p.output_path = set_critical_parameter(args.output_path, "output_path")
    json_name = set_critical_parameter(args.settings_json, "settings_json")
    exe_path = set_critical_parameter(args.exe_path, "exe_path")

    exe_name = set_extended_parameter(args.exe_name, "exe_name", 'FP_builder.exe')
    ignore_list_name = set_extended_parameter(args.ignore_list, "ignore_list", '')
    input_list_name = set_extended_parameter(args.input_list, "input_list", '')

    #set working dir
    os.chdir(exe_path)
    fp_builder_exe = os.path.join(exe_path, exe_name)

    # ToDo: add pathes and parameters checking

    # load json files
    p.settingsJson = json.load(open(json_name, 'r'))
    if ignore_list_name != '':
        print("ignore list: " + ignore_list_name)
        p.ignore_listJson= json.load(open(ignore_list_name, 'r'))
    if input_list_name != '':
        p.input_listJson = json.load(open(input_list_name, 'r'))

    # run fp_builder
    run_fp_builder(fp_builder_exe, p)

