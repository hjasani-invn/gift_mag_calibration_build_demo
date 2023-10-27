#import matplotlib as mpl
#mpl.use('Agg')
#import matplotlib.pyplot as plt
import os
import sys
import numpy as np
import pygeoj
import json
import time
import csv
import io
import re
import ctypes
import mfp_loader


if  __name__ == "__main__":
    settings_json_file = sys.argv[1]
    major_mfp_file = sys.argv[2]
    minor_mfp_file = sys.argv[3]

    print(settings_json_file)
    print(major_mfp_file)
    print(minor_mfp_file)
    print("")

    mfp_format_patterns = ['(.*\.mfp3)', '(.*\.mfp4)']

    # reading venue parameters
    f = open(settings_json_file, 'r')
    json_text = f.read()
    json_data = json.loads(json_text)
    venue = json_data['venue']
    #print (venue)

    # venue sizes
    size_x = venue['size_x']
    size_y = venue['size_y']
    magnetic_cellsize = json_data['magnetic_cellsize']
    floors_count = venue['floors_count']

    for mfp_pattern in range(len( mfp_format_patterns)):
        if re.match(mfp_format_patterns[mfp_pattern], major_mfp_file) and re.match(mfp_format_patterns[mfp_pattern], minor_mfp_file):

            result_mfp = []
            result_sig = []
            for floor in range(floors_count):
                if mfp_pattern == 0:
                    major_mfp, numX, numY = mfp_loader.loadMFP3(major_mfp_file, size_x, size_y, magnetic_cellsize, floor)
                    minor_mfp, numX, numY = mfp_loader.loadMFP3(minor_mfp_file, size_x, size_y, magnetic_cellsize, floor)
                else:
                    major_header, major_mfp, numX, numY = mfp_loader.loadMFP4(major_mfp_file, size_x, size_y, magnetic_cellsize, floor)
                    minor_header, minor_mfp, numX, numY = mfp_loader.loadMFP4(minor_mfp_file, size_x, size_y, magnetic_cellsize, floor)

                print('floor , ', floor)

                # magnetic value
                #print('x , y , mag_x , mag_y , mag_z')
                print('x , y , mag_x1, mag_x2, mag_x1 - mag_x2, mag_y1, mag_y2, mag_y1 - mag_y2, mag_z1, mag_z2, mag_z1 - mag_z2')
                print('')
                coord = ['coord_x', 'coord_y', 'coord_z']
                for i in range(numX):
                    for j in range(numY):
                        non_zero_content = False
                        for n in range(3):
                            if(minor_mfp[n]['av1'][i, j] != 0 and  major_mfp[n]['av1'][i, j] != 0 ):
                                non_zero_content = True
                        if non_zero_content:
                            print(i * magnetic_cellsize + magnetic_cellsize/2,' , ', j * magnetic_cellsize + magnetic_cellsize/2, end=' , ')
                            for n in range(3):
                                #print(major_mfp[n]['av1'][i, j], end=' , ')
                                #print(minor_mfp[n]['av1'][i, j], end=' , ')
                                print(major_mfp[n]['av1'][i, j] - minor_mfp[n]['av1'][i, j], end=' , ')
                            print('')

                print("")
                print("==========================================================")
                print("")

                # magnetic sigma
                print('x , y , sig_x1, sig_x2, sig_x1 - sig_x2, sig_y1, sig_y2, sig_y1 - sig_y2, sig_z1, sig_z2, sig_z1 - sig_z2')
                print('')
                coord = ['coord_x', 'coord_y', 'coord_z']
                for i in range(numX):
                    for j in range(numY):
                        non_zero_content = False
                        for n in range(3):
                            if(minor_mfp[n]['sig1'][i, j] != 60 and  major_mfp[n]['sig1'][i, j] != 60 ):
                                non_zero_content = True
                        if non_zero_content == True:
                            print(i * magnetic_cellsize + magnetic_cellsize/2,' , ', j * magnetic_cellsize + magnetic_cellsize/2, end=' , ')
                            for n in range(3):
                                #print(major_mfp[n]['sig1'][i, j], end=' , ')
                                #print(minor_mfp[n]['sig1'][i, j], end=' , ')
                                print(major_mfp[n]['sig1'][i, j] - minor_mfp[n]['sig1'][i, j], end=' , ')
                            print('')

                print("")
                print("==========================================================")
                print("")
