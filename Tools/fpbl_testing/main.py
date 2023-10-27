import json
import os
import sys
import csv
import io
import argparse
import numpy as np
import math
import read_json_file
import merge
import processing_test

if  __name__ == "__main__":
    version = "1.0.0"
    print("fpbl testting tool, version ", version)
    #print(sys.platform)
    if sys.platform == "linux" or sys.platform == "linux2":
        #linux
        print('Linux')
    elif sys.platform == "win32":
        # Windows...
        print('Windows')

    parser = argparse.ArgumentParser(description="combined script to process trajectories using IRPL and to run KPI tool")
    #parser.add_argument("INPUT_DIR", help="path of root directory of trajectories to be processed")
    #parser.add_argument("OUTPUT_DIR", help="path of root directory of processing output results")
    #parser.add_argument("VENUE_DIR", help="path of venue directory containing .json settings file and fingerprinting maps")

    parser.add_argument("-app", "--application", help="FP builder name")
    parser.add_argument("-in", "--input", help="input data path")
    parser.add_argument("-t", "--tests", help="json file with schedule of tests")
    parser.add_argument("-out", "--output", help="output folder")

    args = parser.parse_args()

    #input_dir = args.INPUT_DIR
    #output_dir = args.OUTPUT_DIR
    #venue_dir = args.VENUE_DIR
    application = args.application
    input_path = args.input
    tests_schedule = args.tests
    output_folder = args.output

    if not os.path.isdir(output_folder):
        os.mkdir(output_folder)

    # reading tests schedule json file
    tests_schedule_data = read_json_file.read_json_file(tests_schedule)

    for test in tests_schedule_data:
        #print("test data is ", test)
        processing_test.processing_test(application, input_path, test, output_folder)
