__author__ = 'kotik'

# The main script: loads input path and settings, calls FP builder tools.
# Optionally calls the tests script.

import process_mapper_input

import generate_fingerprints as gen_fp
import test_fingerprints as tst_fp
import json
import os


def call_generator(main_settings):
    FPBL_json = main_settings['json_FP_builder']
    fp_settings = json.load(open(FPBL_json))
    FP_gen = gen_fp.FingerprintsGenerator(fp_settings)
    FP_gen.call_generator(main_settings)


main_path, filename = os.path.split(os.path.realpath(__file__))
# main_settings = json.load(open('northland_settings.json'))
# main_settings = json.load(open('core_settings_new.json'))
# main_settings = json.load(open('verve_settings.json'))
# main_settings = json.load(open('ijp_settings.json'))
# main_settings = json.load(open('isj_settings.json'))
main_settings = json.load(open('inven_ca_settings.json'))

main_settings['main_path'] = main_path

# main_settings['tests_path'] = "C:/FP_tests/09_03_r10/"
# main_settings['input_data_path'] = "C:/FP_Generation/09_03_Regus/3_set/10/"

if main_settings['mapper_data'] == 1:
    process_mapper_input.process_input(main_settings)

# if main_settings['remove_bad_mapper_data'] == 1:
#     process_mapper_input.remove_bad_data(main_settings)

os.chdir(main_path)

# call_generator(main_settings)

os.chdir(main_path)

# if main_settings['integration_testing'] == 'ON':
#     FP_tester = tst_fp.FingerprintsTester(main_settings)
#     FP_tester.process_tests()

