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

    route_names = [
        "1.1",
        "1.2",
        "1.3",
        "2.1",
        "3.1",
        "3.2",
        "3.3",
    ]

    phone_names = [
        "Pixel_3a_daf1976e_da39_4ce3_a4fd_38bac8454d35_",
        "SM_G960W_a39ce74e_f619_4401_8ca7_a1f3620ea010",
        "Unknown_CFDE6F2C_977A_45B3_A7CF_D23119F35D06_",
        "Unknown_E13F2989_51FA_4CA4_B047_CE9C0CE7337E_",
    ]

    all_tracks =[]
    for route_name in route_names:
        for phone_name in phone_names:
            input_file_list = []
            for cur, dirs, files in os.walk(root_path):
                for rn in dirs:
                    if re.match(route_name, rn) is not None:
                        cur_path = os.path.join(cur, rn)
                        for cur1, dirs1, files1 in os.walk(cur_path):
                            for pn in dirs1:
                                if re.match(phone_name, pn) is not None:
                                    #file_list = folder_parser.check_flickering(os.path.join(cur1, pn), 'mixed_out_dbg.log')
                                    file_list = folder_parser.get_files_names(os.path.join(cur1, pn), 'mixed_out_dbg.log')
                                    for file in file_list:
                                        input_file_list.append(file)
                                    os.chdir(full_scipt_path)
                                    os.chdir(root_path)
            for file in input_file_list:
                print(file)
            print("=============================")
            tracks = folder_parser.check_delays(route_name, phone_name,input_file_list)
            for track in tracks:
                all_tracks.append(track)
            #    print(track[0])
            #    print(track[3], "   ", track[4], "   ", track[5])

    filename = os.path.join(root_path,"multi_floor_results.xlsx")
    write_excel_table.write_to_excel(all_tracks, filename)
