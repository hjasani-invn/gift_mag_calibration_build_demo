import os
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import math


mag_results = []  # list contains elements: [MFP_unc_converted_to_high_res, floor]
wifi_results = []  # list contains elements: [WF_unc_converted_to_high_res, floor]


def internal_floor_to_real_floor(i_floor, floor_shift, floor_zero_enable):
    real_floor = i_floor + floor_shift
    if not floor_zero_enable:
        if floor_shift < 0:
            if real_floor >= 0:
                real_floor += 1

    return real_floor


def create_bp_area(mfp_unc, floor, magnetic_cellsize):
    # this function creates image from MFP_Unc matrix, potentially can be changed to use other inputs
    # we need to convert MFP_Unc to other colors according to some rules
    # for example, black is a wall (or unavailable), green - available zone, red - high priority zone

    x_size = mfp_unc.shape[0]
    y_size = mfp_unc.shape[1]
    bp_area_matrix = np.zeros([x_size, y_size])

    threshold = 5

    for i in range(0, x_size):
        for j in range(0, y_size):
            i1 = x_size - (i + 1)
            j1 = j
            bp_area_matrix[i, j] = bp_convert_mfp_uncertainty(mfp_unc[i1, j1], threshold)

    # saving matrix with 0.5 meters cell size
    mfp_unc_converted_to_high_res = convert_matrix(bp_area_matrix, magnetic_cellsize)
    global mag_results
    mag_results.append([mfp_unc_converted_to_high_res, floor])

    return


def bp_convert_mfp_uncertainty(unc, threshold):
    res = 0
    if unc > threshold:
        res = 5
    elif unc > 0:
        res = 20

    # < 0 = wall( or unmapped)
    # x > 20 = red  area(high  priority)
    # 20 > x >= 0 = green
    # area(passable, low priority)

    return res


def get_bp_colormap():
    a = np.array([[0, 0, 255],
                  [255, 0, 0],
                  [255, 255, 0],
                  [0, 255, 0]])

    return a


def create_wfp_quality_image(wfp_unc, floor, wifi_cellsize, output_folder, output_plot_file_name):

    x_size = wfp_unc.shape[0]
    y_size = wfp_unc.shape[1]
    wfp_quality_matrix = np.zeros([x_size, y_size])

    threshold = 10  # TODO: update threshold value after discussing it

    for i in range(0, x_size):
        for j in range(0, y_size):
            i1 = x_size - (i + 1)
            j1 = j
            wfp_quality_matrix[i, j] = convert_wfp_quality(wfp_unc[i1, j1], threshold)

    wf_unc_converted_to_high_res = convert_matrix(wfp_quality_matrix, wifi_cellsize)

    global wifi_results
    wifi_results.append([wf_unc_converted_to_high_res, floor])

    return


def convert_wfp_quality(value, threshold):
    res = 0
    if value > threshold:
        res = 5
    elif value > 0:
        res = 20

    return res


def convert_matrix(m, cell_size):
    coeff = round(cell_size / 0.5)
    m_size_x = m.shape[0]
    m_size_y = m.shape[1]
    x_max_new = m_size_x * coeff
    y_max_new = m_size_y * coeff

    new_matrix = np.zeros([x_max_new, y_max_new])

    for i in range(0, x_max_new):
        for j in range(0, y_max_new):
            i_old = math.floor(i/coeff)
            j_old = math.floor(j/coeff)

            new_matrix[i, j] = m[i_old, j_old]

    return new_matrix


def combine_results(mode, output_folder, floor_shift, floor_zero_enable):
    global mag_results
    global wifi_results

    try:
        # iterate over all floors (mag_results and wifi_results length is both equal to total floors number)
        floor_iterator = 0
        for one_floor_mag in mag_results:
            # processing for one floor
            # one_floor_mag[0] - matrix
            output_matrix = one_floor_mag[0]
            if mode == 'mag':
                output_matrix = one_floor_mag[0]

            elif mode == 'wifi':
                # wifi is more complex - we need to use unmapped cells from mag matrix

                dim_x = min(wifi_results[floor_iterator][0].shape[0], one_floor_mag[0].shape[0])
                dim_y = min(wifi_results[floor_iterator][0].shape[1], one_floor_mag[0].shape[1])

                output_matrix = np.zeros([dim_x, dim_y])

                for i in range(0, dim_x):
                    for j in range(0, dim_y):
                        if one_floor_mag[0][i, j] == 0:
                            output_matrix[i, j] = 0
                        else:
                            output_matrix[i, j] = wifi_results[floor_iterator][0][i, j]

            elif mode == 'mixed':
                # mixed is the worst between wifi and mag coverage
                dim_x = min(wifi_results[floor_iterator][0].shape[0], one_floor_mag[0].shape[0])
                dim_y = min(wifi_results[floor_iterator][0].shape[1], one_floor_mag[0].shape[1])

                output_matrix = np.zeros([dim_x, dim_y])

                for i in range(0, dim_x):
                    for j in range(0, dim_y):
                        output_matrix[i, j] = min(wifi_results[floor_iterator][0][i, j], one_floor_mag[0][i, j])
                        # area must be considered unmapped only if it is unmapped in MFP coverage
                        if output_matrix[i, j] == 0 and one_floor_mag[0][i, j] > 0:
                            output_matrix[i, j] = one_floor_mag[0][i, j]


            # saving the picture
            out_floor = internal_floor_to_real_floor(one_floor_mag[1], floor_shift, floor_zero_enable)

            output_plot_file_name = 'beacons_placement_' + str(out_floor) + '.png'
            outpath = os.path.join(output_folder, output_plot_file_name)

            fig, ax = plt.subplots(1)
            bp_cmap = get_bp_colormap()
            the_colormap = matplotlib.colors.ListedColormap(bp_cmap / 255.0)
            plt.imshow(output_matrix, cmap=the_colormap, interpolation='nearest')
            ax.axis('scaled')
            plt.tick_params(
                axis='both',
                which='both',
                bottom=False,
                top=False,
                labelbottom=False,
                right=False,
                left=False,
                labelleft=False)

            plt.savefig(outpath, pad_inches=0, dpi=800, bbox_inches='tight')
            plt.close()

            floor_iterator += 1

        mag_results = []
        wifi_results = []

    except Exception as ex:
        print('failed to produce beacons placement')

