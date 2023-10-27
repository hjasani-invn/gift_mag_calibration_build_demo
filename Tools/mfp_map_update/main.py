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
    result_mfp_file = sys.argv[4]

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
        if re.match(mfp_format_patterns[mfp_pattern], major_mfp_file) and re.match(mfp_format_patterns[mfp_pattern], minor_mfp_file)  and re.match(mfp_format_patterns[mfp_pattern], result_mfp_file) :

            result_mfp = []
            result_sig = []
            fout = open(result_mfp_file + '_1', 'wb')
            for floor in range(floors_count):
                if mfp_pattern == 0:
                    major_mfp, numX, numY = mfp_loader.loadMFP3(major_mfp_file, size_x, size_y, magnetic_cellsize, floor)
                    minor_mfp, numX, numY = mfp_loader.loadMFP3(minor_mfp_file, size_x, size_y, magnetic_cellsize, floor)
                else:
                    major_header, major_mfp, numX, numY = mfp_loader.loadMFP4(major_mfp_file, size_x, size_y, magnetic_cellsize, floor)
                    minor_header, minor_mfp, numX, numY = mfp_loader.loadMFP4(minor_mfp_file, size_x, size_y, magnetic_cellsize, floor)

                if mfp_pattern == 1 and floor == 0:
                    fout.write(major_header)
                for j in range(numY):
                    for i in range(numX):
                        for n in range(3):
                            fout.write(ctypes.c_float(major_mfp[n]['av1'][i, j]))
                            fout.flush()
                            fout.write(ctypes.c_float(major_mfp[n]['sig1'][i, j]))
                            fout.flush()
                            fout.write(ctypes.c_float(major_mfp[n]['w1'][i, j]))
                            fout.flush()
                            fout.write(ctypes.c_float(major_mfp[n]['av2'][i, j]))
                            fout.flush()
                            fout.write(ctypes.c_float(major_mfp[n]['sig2'][i, j]))
                            fout.flush()
                            fout.write(ctypes.c_float(major_mfp[n]['w2'][i, j]))
                            fout.flush()
                            fout.write(ctypes.c_float(major_mfp[n]['reserved1'][i, j]))
                            fout.flush()
                            fout.write(ctypes.c_float(major_mfp[n]['reserved2'][i, j]))
                            fout.flush()

                print('floor = ', floor)
                coord = ['coord_x', 'coord_y', 'coord_z']
                for i in range(numX):
                    for j in range(numY):
                        non_zero_content = False
                        for n in range(3):
                            if(minor_mfp[n]['av1'][i, j].any() != 0 or minor_mfp[n]['av2'][i, j].any() != 0 ):
                                non_zero_content = True
                        if non_zero_content == True:
                            for n in range(3):
                                major_mfp[n]['av1'][i, j] = minor_mfp[n]['av1'][i, j]
                                major_mfp[n]['sig1'][i, j] = minor_mfp[n]['sig1'][i, j]
                                major_mfp[n]['w1'][i, j] = minor_mfp[n]['w1'][i, j]
                                major_mfp[n]['av2'][i, j] = minor_mfp[n]['av2'][i, j]
                                major_mfp[n]['sig2'][i, j] = minor_mfp[n]['sig2'][i, j]
                                major_mfp[n]['w2'][i, j] = minor_mfp[n]['w2'][i, j]
                                major_mfp[n]['reserved1'][i, j] = minor_mfp[n]['reserved1'][i, j]
                                major_mfp[n]['reserved2'][i, j] = minor_mfp[n]['reserved2'][i, j]
                                #print(coord[n], '  pos_x = ', i * magnetic_cellsize,'   pos_y = ', j * magnetic_cellsize, '  major_mfp aver = ', major_mfp[n]['av1'][i, j], '   ', major_mfp[n]['av2'][i, j])
                                #print(coord[n], '  pos_x = ', i * magnetic_cellsize,'   pos_y = ', j * magnetic_cellsize, '  major_mfp sig = ', major_mfp[n]['sig1'][i, j], '   ', major_mfp[n]['sig2'][i, j])
                                #print(coord[n], '  pos_x = ', i * magnetic_cellsize,'   pos_y = ', j * magnetic_cellsize, '  major_mfp w = ', major_mfp[n]['w1'][i, j], '   ', major_mfp[n]['w2'][i, j])

                result_mfp.append(major_mfp)

            fout.close()
            fout = open(result_mfp_file, 'wb')
            if mfp_pattern == 1:
                fout.write(major_header)
            for floor in range(floors_count):
                    for j in range(numY):
                        for i in range(numX):
                            for n in range(3):
                                fout.write(ctypes.c_float(result_mfp[floor][n]['av1'][i, j]))
                                fout.flush()
                                fout.write(ctypes.c_float(result_mfp[floor][n]['sig1'][i, j]))
                                fout.flush()
                                fout.write(ctypes.c_float(result_mfp[floor][n]['w1'][i, j]))
                                fout.flush()
                                fout.write(ctypes.c_float(result_mfp[floor][n]['av2'][i, j]))
                                fout.flush()
                                fout.write(ctypes.c_float(result_mfp[floor][n]['sig2'][i, j]))
                                fout.flush()
                                fout.write(ctypes.c_float(result_mfp[floor][n]['w2'][i, j]))
                                fout.flush()
                                fout.write(ctypes.c_float(result_mfp[floor][n]['reserved1'][i, j]))
                                fout.flush()
                                fout.write(ctypes.c_float(result_mfp[floor][n]['reserved2'][i, j]))
                                fout.flush()
            fout.close()

