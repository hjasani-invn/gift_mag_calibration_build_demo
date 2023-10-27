# -*- coding: utf-8 -*-
import os
import re
import json
import subprocess
import multiprocessing
import tqdm
import argparse

def search_paths_with_file(template, root):
    list = []
    for cur, _dirs, files in os.walk(root):
        for f in files:
            if re.match(template, f) is not None:
                path = os.path.join(cur, f)
                list.append(path)
    return list

def run_rtfppl_console (params):

    out_dir = params.dir_out
    os.makedirs(out_dir, mode=0o777, exist_ok=True)

    params.jO['folder_fp'] = params.fp
    params.jO['folder_in'] = params.dir_in
    params.jO['folder_out'] = out_dir

    settings_json = os.path.join(out_dir, 'rtfppl_venue.json')
    with open(os.path.join(out_dir, settings_json), 'w') as outfile:
        json.dump(params.jO, outfile, sort_keys=True, indent=4, separators=(',', ': '))
     
    print('run for: ' + params.dir_in)
    cmd_line = params.exe + ' --settings ' + settings_json
    f = open(os.path.join(out_dir, "rtfppl_console.log"), "w")
    #print(cmd_line)
    os.chdir(out_dir)
    subprocess.call(cmd_line, stdout=f, stderr=f, shell=True)

class Params:
    dir_in = ''
    dir_out = ''
    exe = ''
    fp = ''
    jO = None

def run_rtfppl_batch(exe_folder, root_path, fp_path, case_name, output_path, \
                     exe_filename, venue_json, dat_file_template):

    js_path = os.path.join(fp_path, venue_json)
    jO = json.load(open(js_path, 'r'))

    dir_list = search_paths_with_file(dat_file_template, root_path)
    print('Datasets found: ' + str(len(dir_list)))
    #print(dir_list)

    params = []
    for dir in dir_list:
        p = Params()
        p.dir_in = os.path.dirname(dir)
        tmp = os.path.join(output_path, case_name)
        tmp1 = os.path.relpath(p.dir_in,start=root_path)
        p.dir_out = os.path.join(tmp, tmp1)
        p.fp = fp_path
        p.jO = jO
        p.exe = os.path.join(exe_folder, exe_filename)
        params.append(p)
        #run_rtfppl_console(p)
    
    pcount = 2 * multiprocessing.cpu_count()
    print('pool threads: '+str(pcount))
    pool = multiprocessing.Pool(processes=pcount)
    
    try:
        for _ in tqdm.tqdm(pool.imap_unordered(run_rtfppl_console, params, chunksize=1), total=len(params)):
            pass
    except Exception as ex:
        print("a worker failed, aborting...")
        if hasattr(ex, 'message'):
            print(ex.message)
        else:
            print(ex)
        pool.close()
        pool.terminate()
    else:
        pool.close()
        pool.join()

    pool.close()
    pool.join()


if __name__ == '__main__':

    # input_path = '//cayyc-proj01/compute02/dchurikov/Tasks_of_deployment/2020_02-BS-Hikone-aftercamp_WFP_investigation/wifi_tes.rep9/'
    # fp_collection = '//cayyc-proj01/compute02/dchurikov/Tasks_of_deployment/2020_02-BS-Hikone-aftercamp_WFP_investigation/venues.adjustm/cases#005/'
    # rtfppl_collection = '//cayyc-proj01/compute02/dchurikov/Polygon/rtfppl-console/alfa-versions/v1102.12/'

    #input_path = '//cayyc-proj01/compute02/FPL_DATA/test_data/Vectors_sources.rel/Sato/'
    #fp_collection = '//cayyc-proj01/compute02/FPL_DATA/test_data/Vectors_sources.rel/Sato/2019_11_13/venues/venue.mwp4/'
    #rtfppl_collection = '//cayyc-proj01/compute02/dchurikov/Polygon/rtfppl-console/alfa_group_mm-lkh/'
    #output_path = '//cayyc-proj01/compute02/dchurikov/vectors.proc/v1.14.0/Sato/'  # this is the path to datasets folder

    # vdr-fpl
    # input_path = '//cayyc-proj01/compute02/dchurikov/vectors/Cambrian-p2/VDR.dotdat_run/'
    # fp_collection = '//cayyc-proj01/compute02/dchurikov/Tasks_of_dev/vdr-fpl/venues/cambrian-p2/fp-20_03_12/'
    # rtfppl_collection = '//cayyc-proj01/compute02/dchurikov/Polygon/rtfppl-console/alfa_group_vdr#1/vdr-fpl.delta.01noise.mag_start/'

    # crowdsourcing
    # input_path = '//cayyc-proj01/compute02/dchurikov/vectors/ICA_crowdsourcing/Salah_Test'
    # fp_collection = '//cayyc-proj01/compute02/dchurikov/vectors/ICA_crowdsourcing/Salah_Test/venues.fp/Original/'
    # rtfppl_collection = '//cayyc-proj01/compute02/dchurikov/Polygon/rtfppl-console/alfa_group_css#2/'

    # ica-calgary
    #input_path = '//cayyc-proj01/compute02/dchurikov/vectors.special/Release_III/3.1/'  # this is the path to datasets folder
    #fp_collection = '//cayyc-proj01/compute02/dchurikov/vectors.special/Release_III/Venue/fp_ica_copy.mwp4/'  # this is the path to fingerprint folder
    #rtfppl_collection = '//cayyc-proj01/compute02/dchurikov/Polygon/rtfppl-console/alfa-versions/mm+new_lkh'  # this is the path to RTFPPL console
    #output_path = 'C://Users//vpentyukhov//ivl_result_to_share_May2020_output'  # this is the path to datasets folder

    #exe_filename = 'fp_positioning.console.vc17.exe'
    #venue_json = 'venue.json'
    #dat_file_template = r'(.*)TppOutput(.*)dat'
    #dat_file_template = r'ivl_out.dat'

    ##################
    parser = argparse.ArgumentParser(description="json automation tool parameters")

    parser.add_argument("-i", "--input_path", help="the path to datasets folder")
    parser.add_argument("-f", "--fp_collection", help="path to fingerprint folder")
    parser.add_argument("-c", "--rtfppl_collection", help="path to RTFPPL console")
    parser.add_argument("-o", "--output_path", help="path to output folder")
    parser.add_argument("-e", "--exe_filename", default='fp_positioning.console.exe', \
                        help="execute file name (default=fp_positioning.console.exe)")
    parser.add_argument("-v", "--venue_json", default='venue.json', \
                        help="venue settings file name (default=venue.json)")
    parser.add_argument("-d", "--dat_file_template", default='(.*)TppOutput(.*)dat', \
                        help="template for data files (default=(.*)TppOutput(.*)dat)")

    args = parser.parse_args()

    input_path = args.input_path
    fp_collection = args.fp_collection
    rtfppl_collection = args.rtfppl_collection
    output_path = args.output_path
    exe_filename = args.exe_filename
    venue_json = args.venue_json
    dat_file_template = args.dat_file_template
    ##################

    rtfppl_dir_list = search_paths_with_file(r'fppe.dll', rtfppl_collection)
    print('RTFPPL found: ' + str(len(rtfppl_dir_list)))

    venue_file_template = r'venue.json'
    fp_dir_list = search_paths_with_file(venue_file_template, fp_collection)
    print('Fingerprints found: ' + str(len(fp_dir_list)))
    # print(dir_list)

    for exe_dir in rtfppl_dir_list:
        exe_folder = os.path.dirname(exe_dir)
        for dir in fp_dir_list:
            fp_path = os.path.dirname(dir)
            case_name = os.path.basename(exe_folder) + '.' + os.path.basename(fp_path)
            #case_name = 'test_run'
            print('Processing for fingerprint:' + case_name)
            run_rtfppl_batch(exe_folder, input_path, fp_path, case_name, \
                             output_path, exe_filename, venue_json, dat_file_template)