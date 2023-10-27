import json
import os
import sys
import csv
import io
import shutil
import subprocess
import numpy as np
import math
import merge
import read_json_file

def processing_test(application, input_path, test_json_data, output_folder=None):
	print(input_path)
	print(output_folder)
	print(test_json_data)

	test_name = test_json_data['test_name']
	test_folder = os.path.join(output_folder, test_name)
	if not os.path.isdir(test_folder):
		os.mkdir(test_folder)

	working_folder = test_folder
	specific_file_name = test_json_data['json_file_for_test']
	specific_file_name = os.path.join(input_path, specific_file_name)
	#print(specific_file_name)

	# reading json_with_parameters file
	json_with_parameters_data = read_json_file.read_json_file(specific_file_name)
	#print(json_with_parameters_data)

	#print(json_with_parameters_data)
	folder_fp = json_with_parameters_data['folder_fp']
	folder_grid = json_with_parameters_data['folder_grid']
	folder_out = json_with_parameters_data['folder_out']

	folder_fp = os.path.join(working_folder, folder_fp)
	folder_grid = os.path.join(working_folder, folder_grid)
	folder_out = os.path.join(working_folder, folder_out)

	folder_fp = os.path.abspath(folder_fp)
	folder_grid = os.path.abspath(folder_grid)
	folder_out = os.path.abspath(folder_out)
	#print(folder_fp)
	#print(folder_grid)
	#print(folder_out)
	if not os.path.isdir(folder_fp):
		os.mkdir(folder_fp)
	if not os.path.isdir(folder_grid):
		os.mkdir(folder_grid)
	if not os.path.isdir(folder_out):
		os.mkdir(folder_out)

	json_with_parameters_data['folder_fp'] = folder_fp
	json_with_parameters_data['folder_grid'] = folder_grid
	json_with_parameters_data['folder_out'] = folder_out

	folder_in = json_with_parameters_data['folder_in']
	folder_in = os.path.join(input_path, folder_in)
	json_with_parameters_data['folder_in'] = folder_in

	if 'in_ble_fprox_file' in json_with_parameters_data:
		in_ble_fprox_file = json_with_parameters_data['in_ble_fprox_file']
		in_ble_fprox_file = os.path.join(input_path, in_ble_fprox_file)
		json_with_parameters_data['in_ble_fprox_file'] = in_ble_fprox_file
		shutil.copy2(in_ble_fprox_file, working_folder)

	fixed_venue_file_name = json_with_parameters_data['venue_file']
	fixed_venue_data = read_json_file.read_json_file( os.path.join(input_path, fixed_venue_file_name))
	#print(fixed_venue_data)

	output_data = merge.merge(fixed_venue_data, json_with_parameters_data)
	del output_data['venue_file']
	#print(output_data)

	json_out_data = json.dumps(output_data, sort_keys=False, indent=4, separators=(',', ': '))
	fw = open(os.path.join(working_folder, "venue.json"), 'wt')
	fw.write(json_out_data)
	fw.close()

	command = application
	command += '  --settings  '
	command += os.path.join(working_folder, "venue.json")
	#print(command)
	return_code = subprocess.call(command, shell=True)

