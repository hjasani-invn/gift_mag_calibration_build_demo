import re
import os
import sys

sys.path.insert(0, '..')
import folder_parser
import matplotlib as mpl
import matplotlib.pyplot as plt
#mpl.use('TkAgg')
#mpl.use('Agg')
import write_excel_table


if __name__ == "__main__":
    print("")
    scipt_path = os.path.dirname(sys.argv[0])
    full_scipt_path = os.path.abspath(scipt_path)

    root_path = sys.argv[1].replace("\r", "")
    os.chdir(root_path)

    fprint_names = [
        #"mp4$",
        "mp4m$",
    ]

    phone_names = [
        #"01_iPhone_6",
        #"02_iPhone_6",
        #"03_iPhone_6",
        #"06_iPhone_8",
        #"07_iPhone_8",
        "08_iPhone_8",
    ]

    for fprint_name in fprint_names:
        for phone_name in phone_names:
            input_file_list = []
            for cur, dirs, files in os.walk(root_path):
                for rn in dirs:
                    cur_path = os.path.join(cur, rn)
                    for cur1, dirs1, files1 in os.walk(cur_path):
                        for pn in dirs1:
                            if re.search(fprint_name, pn) is not None:
                                cur_path2 = os.path.join(cur1, pn)
                                for cur2, dirs2, files2 in os.walk(cur_path2):
                                    for pn2 in dirs2:
                                        if re.match(phone_name, pn2):
                                            file_list = folder_parser.get_files_names(os.path.join(cur2, pn2), 'mixed_out.kml')
                                            for file in file_list:
                                                input_file_list.append(file)
                                            os.chdir(full_scipt_path)
                                            os.chdir(root_path)

    os.chdir(scipt_path)
    bat_file_name = 'mixed_out.bat'
    bat_file = open(bat_file_name, 'w')
    for file in input_file_list:
        print(file)
        bat_file.write(file + '\n')
    bat_file.close()
    os.system(bat_file_name)
    print("=============================")
