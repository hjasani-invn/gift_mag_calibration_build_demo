import numpy as np
import math
import os
import FPBL_version


def read_mfp4_header(f_name):
    with open(f_name, 'rb') as fid:
        fid.seek(0, os.SEEK_SET)
        data_array = np.fromfile(fid, dtype='uint8')

    survey_type_added = False

    header_4_format_size = int.from_bytes(data_array[8:12], byteorder='little')
    # reading FPBL version
    FPBL_version.major = int(data_array[24])
    FPBL_version.minor = int(data_array[25])
    FPBL_version.build = int.from_bytes(data_array[26:30], byteorder='little')

    # # TODO: WARNING - this is for debugging only!
    # FPBL_version.major = 1
    # FPBL_version.minor = 6

    if FPBL_version.major >= 1 and FPBL_version.minor >= 6:
        survey_type_added = True

    return header_4_format_size, survey_type_added


def load_mfp4(f_name, size_x, size_y, cell_size, floor):

    num_x = int(math.ceil(size_x/cell_size))
    num_y = int(math.ceil(size_y/cell_size))

    header_4_format_size, survey_type_added = read_mfp4_header(f_name)
    floor_cells = num_x * num_y
    skip_bytes = floor * num_x * num_y * 8 * 3 * 4 + header_4_format_size

    av_x = np.zeros([num_x, num_y])
    av_y = np.zeros([num_x, num_y])
    av_z = np.zeros([num_x, num_y])
    sig_x = 100*np.ones([num_x, num_y])
    sig_y = 100*np.ones([num_x, num_y])
    sig_z = 100*np.ones([num_x, num_y])
    qual_x = np.zeros([num_x, num_y])
    qual_y = np.zeros([num_x, num_y])
    qual_z = np.zeros([num_x, num_y])
    mag_coverage = np.ones([num_x, num_y]) * (-5)
    survey_type_matrix = np.ones([num_x, num_y]) * (-1)
    robotic_type = 5
    regular_survey_type = 2
    robotic_coverage_scale = 4

    if os.path.isfile(f_name):
        with open(f_name, 'rb') as fid:
            fid.seek(skip_bytes, os.SEEK_SET)
            data_array = np.fromfile(fid, np.float32)

        i_x = 0
        i_y = 0
        i = 0
        idx = 0

        while i < len(data_array) and idx < floor_cells:
            av_x[i_x, i_y] = data_array[i]
            sig_x[i_x, i_y] = data_array[i + 1]
            av_y[i_x, i_y] = data_array[i + 8]
            sig_y[i_x, i_y] = data_array[i + 9]
            av_z[i_x, i_y] = data_array[i + 16]
            sig_z[i_x, i_y] = data_array[i + 17]
            qual_x[i_x, i_y] = data_array[i + 4]
            qual_y[i_x, i_y] = data_array[i + 12]
            qual_z[i_x, i_y] = data_array[i + 20]
            if data_array[i + 7] > 0:
                mag_coverage[i_x, i_y] = data_array[i + 7]
                survey_type_matrix[i_x, i_y] = regular_survey_type

            if survey_type_added:
                if mag_coverage[i_x, i_y] > 0:
                    survey_type_matrix[i_x, i_y] = data_array[i + 14]
                if survey_type_matrix[i_x, i_y] == robotic_type:
                    mag_coverage[i_x, i_y] *= robotic_coverage_scale
                    # NOTE: human survey threshold is 0-200-400+, while robot threshold is 0-50-100+

            i_x += 1
            if i_x > num_x - 1:
                i_x = 0
                i_y += 1

            idx += 1
            i += 24

    mfp = np.zeros([num_y, num_x, 3])
    sig = np.zeros([num_y, num_x, 3])
    mag_quality = np.zeros([num_x, num_y, 3])

    mfp[:, :, 0] = av_x.transpose()
    mfp[:, :, 1] = av_y.transpose()
    mfp[:, :, 2] = av_z.transpose()

    sig[:, :, 0] = sig_x.transpose()
    sig[:, :, 1] = sig_y.transpose()
    sig[:, :, 2] = sig_z.transpose()

    mag_quality[:, :, 0] = qual_x
    mag_quality[:, :, 1] = qual_y
    mag_quality[:, :, 2] = qual_z

    return mfp, sig, mag_quality, mag_coverage, survey_type_matrix
