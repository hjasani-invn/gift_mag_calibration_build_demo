import sys
import os
import re



if __name__ == "__main__":
    # root_path = sys.argv[1].replace("\r", "")
    root_path = '//cayyc-proj01/compute02/ykotik/SurveyTool/datasets/prod/TDK_HQ_Nihonbashi/SurveyDatasets/'

    out_file_name = '50_50_50_biases.csv'

    bias_mask = '.*mag_out_by_recalib_we.*\.txt'

    bias_list = []

    for cur, dirs, files in os.walk(root_path):
        for f in files:
            if re.match(bias_mask, f) is not None:
                bias_file_full_path = os.path.join(cur, f)
                with open(bias_file_full_path, 'r') as bf:
                    print(bias_file_full_path)
                    lines = bf.readlines()
                    elements = lines[0].split(',')
                    bias_x = float(elements[1])
                    bias_y = float(elements[2])
                    bias_z = float(elements[3])

                    # if abs(bias_x) < 5.0 and abs(bias_y) < 5.0 and abs(bias_z) < 5.0:
                    bias_list.append([bias_file_full_path, bias_x, bias_y, bias_z])

    with open(out_file_name, 'w') as out_f:
        for b in bias_list:
            l = str(b[0]) + ' , ' + str(b[1]) + ' , ' + str(b[2]) + ' , ' + str(b[3]) + '\n'
            out_f.write(l)

