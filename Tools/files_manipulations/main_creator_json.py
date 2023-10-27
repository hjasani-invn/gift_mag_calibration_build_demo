import argparse
#from termcolor import colored
import os, shutil
#import run_batch_rtw
#import Stats_Processor_III.tppmain
import subprocess
import platform
import glob
import re
import os
import sys
sys.path.insert(0, '..')
import pygeoj
import json
#import ASCA
import folder_parser
#import maps_list_to_dict
import process_logs

if __name__ == "__main__":

	print("")
	scipt_path = os.path.dirname(sys.argv[0])
	full_scipt_path = os.path.abspath(scipt_path)

	root_path = sys.argv[1].replace("\r", "")
	os.chdir(root_path)
	dir_pattern = '.*'
	print("{")
	print("\"datasets_list:\"")
	print("\t[")
  
	for cur, dirs, files in os.walk(root_path):
		for d in dirs:
			if re.match(dir_pattern, d) is not None:
				path, out_venue_folder = os.path.split(cur)

				input_file_list = folder_parser.create_json(os.path.join(cur, d), 'irl_out.dat')

				os.chdir(full_scipt_path)
				os.chdir(root_path)
				#print("======================")

	print("\t]")
	print("}")
	print("")
