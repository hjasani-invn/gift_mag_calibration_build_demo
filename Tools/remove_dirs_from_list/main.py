import os
import sys
import json
import shutil

if __name__ == "__main__":  # for tests only

    full_in_folder = sys.argv[1]                  # root folder
    print ('in folder: ', full_in_folder)
    os.chdir(full_in_folder)

    dirs_list = sys.argv[2].replace("\r", "")     # file of subfolber list to will be removed
    print ('file list: ', dirs_list)
    if os.path.isfile(dirs_list):
        handle = open(dirs_list, "r")
        list = handle.readlines()  # read ALL the lines!
        handle.close()
        for dir_name in list:
            dir_name = dir_name.replace("\n", "")
            print(dir_name)
            try:
                shutil.rmtree(dir_name)
            except OSError:
                print ("The directory %s is not exist" % dir_name)
            else:
                print ("The directory %s has been successfully deleted" % dir_name)


