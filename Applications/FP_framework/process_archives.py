__author__ = 'kotik'

# Extracts data from zip archives in current folder.
# Filters data, so that it is only from mapper and only the necessary file formats are left.

import zipfile
import glob
import os
import shutil
import re


# function checks that data in archive was collected with Mapper (MapCreator)
# returns
def from_mapper(directory):
    subdirs = [x[0] for x in os.walk(directory)]
    for subdir in subdirs:
        current_dir = os.path.basename(os.path.normpath(subdir))
        if current_dir == 'MapCreator':
            return True, subdir
    return False, []


def get_mapper_data(directory, destination):
    subdirs = [x[0] for x in os.walk(directory)]
    for subdir in subdirs:

        # src_files = [f for f in os.listdir(subdir) if
        #              re.match(r'(Track_.*\.trk|wifi_.*\.log|ble_.*\.log|mg_enu.*\.log)', f)]
        # NOTE: now we need all mapper files (for conversion to new format)
        # WARNING: this means that old utils can't be used anymore without choosing only correct mfp format files
        # use old python project to generate FP with old utils

        src_files = [f for f in os.listdir(subdir)]

        for file_name in src_files:
            full_file_name = os.path.join(subdir, file_name)
            if os.path.isfile(full_file_name):
                shutil.copy(full_file_name, destination)


def process():
    # deleting previous data_from_archive_0/, data_from_archive_1/ ...
    dat = [f for f in os.listdir() if re.match(r'(data_from_archive_.*)', f)]
    for f in dat:
        shutil.rmtree(f)
    iterator = 0
    all_zips = [x for x in glob.glob('*.zip') if os.path.isfile(x)]
    for one_zip in all_zips:
        zf = zipfile.ZipFile(one_zip, 'r')
        archive_name = os.path.splitext(one_zip)[0]
        os.makedirs(archive_name, exist_ok=True)
        zf.extractall(path=archive_name)

        # fetching MapCreator data only
        [is_Mapper_data, path] = from_mapper(archive_name)
        if is_Mapper_data:
            iter_d = 'data_from_archive_' + str(iterator)
            iterated_path = os.path.join('.', iter_d)
            os.makedirs(iterated_path, exist_ok=True)  # create directories data0/ data1/ etc...
            get_mapper_data(path, iterated_path)
            iterator += 1

        shutil.rmtree(archive_name)  # deleting temp extracted data

        # Now we have a number of directories with only needed file types (wifi, ble, mg_enu, Track)
