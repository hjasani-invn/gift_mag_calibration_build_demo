#!/bin/sh
#
# "-app", "--application", help="FP builder name"
# "-in", "--input", help="input data path"
# "-t", "--tests", help="json file with list of tests"
# "-out", "--output", help="output folder"

python main.py  -app fpbilder.out -in /projects/compute02/FPL_DATA/test_data/FPBL_release_testing  -t tests_schedule.json -out release_1.2.0_linux