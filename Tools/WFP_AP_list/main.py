import os
import sys
import re
import process_WFP_list

if __name__ == "__main__":
    # print ('sys.argv[0] =', sys.argv[0])
    script_path = os.path.dirname(sys.argv[0])
    # print ('path =', scipt_path)
    full_script_path = os.path.abspath(script_path)
    # print ('full path =', full_scipt_path)

    folder = sys.argv[1]
    output_folder =  sys.argv[2].replace("\r", "")
    os.chdir(full_script_path)
    #print(full_script_path)

    #settings_pattern = '(.*\.json)'
    wifibase3_pattern = '(.*\.wifi3)'
    wifibase4_pattern = '(.*\.wifi4)'

    patterns = []
    #patterns.append(settings_pattern)
    patterns.append(wifibase3_pattern)
    patterns.append(wifibase4_pattern)

    input_file_list = []

    for pattern in patterns:
        #print('pattern: ', pattern)
        list = [f for f in os.listdir(folder) if re.match(pattern, f) is not None]
        if len(list) > 0 :
            input_file_list.append(os.path.join(folder, list[0]))

    print(input_file_list)
    print("output folder = ", output_folder)
    process_WFP_list.process(output_folder, input_file_list)
    print("Finish")


