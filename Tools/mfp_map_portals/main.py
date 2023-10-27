#import matplotlib as mpl
#mpl.use('Agg')
#import matplotlib.pyplot as plt
import os
import sys
import math
import numpy as np
import pygeoj
import json
import time
import csv
import io
import re
import ctypes
import mfp_loader
import color_map as colormap
import plot_images

if  __name__ == "__main__":
    settings_json_file = sys.argv[1]
    major_mfp_file = sys.argv[2]
    result_portals_file = sys.argv[3]

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
        if re.match(mfp_format_patterns[mfp_pattern], major_mfp_file):

            result_mfp = []
            result_sig = []
            for floor in range(floors_count):
                if mfp_pattern == 0:
                    major_mfp, numX, numY = mfp_loader.loadMFP3(major_mfp_file, size_x, size_y, magnetic_cellsize, floor)
                else:
                    major_header, major_mfp, numX, numY = mfp_loader.loadMFP4(major_mfp_file, size_x, size_y, magnetic_cellsize, floor)

                print('floor = ', floor)
                portals = colormap.blue_color * np.ones([numX, numY])
                for j in range(numY):
                    for i in range(numX):
                        if major_mfp[0]['reserved1'][i, j] != 0:
                            print(major_mfp[0]['reserved1'][i, j], '   ',i, j)
                            portals[i, j] = 1# colormap.green_color

                result_portals_file_1 = result_portals_file[:-4] + '_' + str(floor) + result_portals_file[-4:]
                A = colormap.get_colormap()
                outpath = result_portals_file_1
                plot_images.draw_picture(portals.transpose(), A, outpath)

