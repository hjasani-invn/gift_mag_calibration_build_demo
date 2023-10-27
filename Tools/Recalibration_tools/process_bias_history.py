import os
import re

base_path = os.getcwd()
input_path = '//cayyc-proj01/compute02/ykotik/SurveyTool/datasets/prod/TDK_HQ_Nihonbashi/SurveyDatasets_Added_50_50_50_bias/'

dat_file_mask = '.*irl_out\.dat'
input_folder_list = []

x_outfile = 'x_bias_history.csv'
y_outfile = 'y_bias_history.csv'
z_outfile = 'z_bias_history.csv'

iterations_number = 15

for cur, dirs, files in os.walk(input_path):
    for f in files:
        if re.match(dat_file_mask, f) is not None:
            input_folder_list.append(cur)

x_line_list = []
y_line_list = []
z_line_list = []

for folder in input_folder_list:
    os.chdir(folder)
    line_x = folder
    line_y = folder
    line_z = folder

    if not os.path.isfile('0_mag_out_by_recalib_history.txt'):
        continue

    for i in range(0, iterations_number):
        bias_file = str(i) + '_' + 'mag_out_by_recalib_history.txt'
        with open(bias_file, 'r') as bf:
            lines = bf.readlines()
            line = lines[0]
            elements = line.split(',')
            x_bias = elements[1]
            y_bias = elements[2]
            z_bias = elements[3]

            line_x += ', ' + str(x_bias)
            line_y += ', ' + str(y_bias)
            line_z += ', ' + str(z_bias)

    line_x += '\n'
    line_y += '\n'
    line_z += '\n'

    x_line_list.append(line_x)
    y_line_list.append(line_x)
    z_line_list.append(line_x)

os.chdir(base_path)

with open(x_outfile, 'w') as out:
    for lx in x_line_list:
        out.write(lx)

with open(y_outfile, 'w') as out:
    for ly in y_line_list:
        out.write(ly)

with open(z_outfile, 'w') as out:
    for lz in z_line_list:
        out.write(lz)


