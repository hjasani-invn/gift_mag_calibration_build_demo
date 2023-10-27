__author__ = 'kotik'

import os
import re
import shutil
import glob


# data_path = 'C:/FP_Generation/IRL/TPN/'


#data_path = 'C:/Share/Vectors/December_9_2016_Target/Target_Front_Pocket/'
data_path = 'C:/Users/vpentyukhov/Development/InvenSenseInc/Gift\Applications/fp_builder.console/FP_builder/Collection2/'
os.chdir(data_path)
paths = glob.glob('*/')
# print(paths)


# for one_path in paths:
#     os.chdir(one_path)
#     track_folders = glob.glob('*/')
#
#     for one_track_folder in track_folders:
#         os.chdir(one_track_folder)
#
#         files = [f for f in os.listdir() if os.path.isfile(f)]
#         # print(files)
#
#         unused_files = [f for f in files if not re.match(r'(irl_out\.dat|default\.mbias)', f)]
#         print(unused_files)
#
#         for file in unused_files:
#             os.remove(file)
#
#         # subfolders = [sub for sub in glob.glob('*/') if not re.match(r'(tpp_offline_output_sd)', sub)]
#         # subfolders = [sub for sub in glob.glob('*/')]
#         # print(subfolders)
#
#         # for sub in subfolders:
#         #     shutil.rmtree(sub)
#
#         # for sub in subfolders:
#         #     os.chdir(sub)
#         #     files = [f for f in os.listdir() if os.path.isfile(f)]
#         #     unused_files = [f for f in files if not re.match(r'(output_lib\.dat|default\.mbias)', f)]
#         #     # print(unused_files)
#         #
#         #     # for file in unused_files:
#         #     #     os.remove(file)
#         #
#         #     os.chdir('..')
#
#         os.chdir('..')
#
#     os.chdir('..')


all_subdirs = []

for root, dirs, files in os.walk(data_path):
    for dir in dirs:
        d = os.path.join(root, dir)
        all_subdirs.append(d)


# # for root, dirs, files in os.walk(data_path):
# #     for dir in dirs:
# #         d = os.path.join(root, dir)
# #         if re.search(r'(\\M2)', d):
# #             print(d)
# #             # shutil.rmtree(d)


for one_subdir in all_subdirs:
    os.chdir(one_subdir)

    src_files = [f for f in os.listdir(one_subdir) if re.match(r'(irl_out\.dat|mag_out\.txt)', f)]
    #src_files = [f for f in os.listdir(one_subdir) if re.match(r'(irl_out\.dat|mag_out\.txt|irl\.kml)', f)]
    #src_files = [f for f in os.listdir(one_subdir) if re.match(r'(nav\.dat)', f)]
    #src_files = [f for f in os.listdir(one_subdir) if re.match(r'(velref_updated_V2\.0\.txt)', f)]

    files = [f for f in os.listdir() if os.path.isfile(f)]

    unused_files = [f for f in files if not re.match(r'(irl_out\.dat|mag_out\.txt)', f)]
    #unused_files = [f for f in files if not re.match(r'(irl_out\.dat|mag_out\.txt|irl\.kml)', f)]
    #unused_files = [f for f in files if not re.match(r'(nav\.dat)', f)]
    #unused_files = [f for f in files if not re.match(r'(velref_updated_V2\.0\.txt)', f)]
    # src_files = [f for f in os.listdir(one_subdir) if re.match(r'(default\.mbias)', f)]

    print(unused_files)
    for file in unused_files:
             os.remove(file)

    for file_name in src_files:
            full_file_name = os.path.join(one_subdir, file_name)
            os.chdir(one_subdir)
            os.chdir('..')
            dest = os.getcwd()
            shutil.move(full_file_name, dest)
            print(full_file_name, ' : ', dest)
