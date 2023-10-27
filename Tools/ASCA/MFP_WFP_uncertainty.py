import os
import matplotlib as mpl
import numpy as np
import plot_images
import MFP_uncertainty
mpl.use('Agg')


def get_mfp_wfp_unc_colormap():
    # blue, gray, green, yellow, red
    a = np.array([[140, 212, 252],
                  [100, 100, 100],
                  [0, 255, 0],
                  [255, 255, 0],
                  [255, 0, 0]])

    return a


def one_cell_convert_mfp_wfp_unc(mfp_unc, mfp_cov, wifi_unc):
    wifi_threshold_red = 15
    wifi_threshold_yellow = 7.5
    mfp_threshold_red = 7.5
    mfp_threshold_yellow = 4

    if mfp_unc < 0:
        res = 0
    elif mfp_cov < 200:
        res = 1
    elif mfp_unc <= mfp_threshold_yellow:
        res = 2
    elif mfp_unc <= mfp_threshold_red:
        res = 3
    elif (wifi_unc > 0) and (wifi_unc <= wifi_threshold_yellow):
        res = 3
    else:
        res = 4

    return res


def convert_mfp_wfp_unc(mfp_unc, mfp_coverage, wifi_unc):
    m_conv = np.zeros([mfp_unc.shape[0], mfp_unc.shape[1]])
    mfp_coverage_trans = mfp_coverage.transpose()
    n0 = mfp_coverage_trans.shape[0]
    n1 = mfp_coverage_trans.shape[1]
    m_mfp = mfp_unc.shape[0]
    n_mfp = mfp_unc.shape[1]
    m_wifi = wifi_unc.shape[0]
    n_wifi = wifi_unc.shape[1]

    ratio_m = m_wifi / m_mfp
    ratio_n = n_wifi / n_mfp

    for i in range(0, m_mfp):
        for j in range(0, n_mfp):
            k = int(i * ratio_m)
            l = int(j * ratio_n)
            if i < n0 and j < n1:
                m_conv[i, j] = one_cell_convert_mfp_wfp_unc(mfp_unc[i, j],
                                                            mfp_coverage_trans[i, j],
                                                            wifi_unc[k, l])
    return m_conv


def plot_mfp_wfp_uncertainty(mfp_unc_matrix, mfp_coverage, wfp_unc, output_directory,
                             output_plot_file_name, real_floor, wifi_unc_result):

    if wifi_unc_result == 1:
        output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]
        outpath = os.path.join(output_directory, output_plot_file_name)

        # Start plotting
        wfp_unc_cmap = get_mfp_wfp_unc_colormap()
        mfp_wfp_unc_converted = convert_mfp_wfp_unc(mfp_unc_matrix, mfp_coverage, wfp_unc)
        plot_images.draw_picture(mfp_wfp_unc_converted, wfp_unc_cmap, outpath)

    else:
        # total uncertainty is equal to MFP uncertainty if no Wi-Fi is available
        conv_mfp_unc = MFP_uncertainty.convert_mfp_unc(mfp_unc_matrix, mfp_coverage)

        mfp_unc_cmap = MFP_uncertainty.get_mfp_unc_colormap()

        output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]
        outpath = os.path.join(output_directory, output_plot_file_name)

        plot_images.draw_picture(conv_mfp_unc, mfp_unc_cmap, outpath)