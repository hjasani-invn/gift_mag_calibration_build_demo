import numpy as np
import math
import LoadMFP4
import statistics
import json
import os
import plot_images


def mfp_uncertainty(mfp4_fname, settings_json, floor, mag_quality_converted):
    ini_sig_thr = 60
    sig_mag_noise = 0

    init_unc = -5

    # 2d filter (gaussian)
    hsize = 1
    hsig = 0.5
    hsum = 0

    settings = json.load(open(settings_json))
    cellsize = settings['magnetic_cellsize']
    dim_x = math.ceil(settings['venue']['size_x'])
    dim_y = math.ceil(settings['venue']['size_y'])

    h = np.zeros([hsize * 2 + 1, hsize * 2 + 1])
    for k in range(-hsize, hsize + 1):
        for m in range(-hsize, hsize + 1):
            b = math.exp(-(k ** 2 + m ** 2) * cellsize ** 2 / 2 / hsig ** 2)
            hsum += b
            h[k + hsize, m + hsize] = b

    h /= hsum

    mag_fp, sig, mag_q_unused, mag_cov_unused, survey_type_m_unused = LoadMFP4.load_mfp4(mfp4_fname, dim_x, dim_y,
                                                                                      cellsize, floor)

    dim_x = int(math.ceil(dim_x / cellsize))
    dim_y = int(math.ceil(dim_y / cellsize))
    # print(dim_x, dim_y)

    # 2d filtering of mag_fp
    mag = np.zeros([mag_fp.shape[0], mag_fp.shape[1], mag_fp.shape[2]])

    for i in range(0, dim_y):
        for j in range(0, dim_x):
            fres = np.zeros([3, 1])
            hsum = 0
            for k in range(0, hsize * 2 + 1):
                for m in range(0, hsize * 2 + 1):
                    ih = i - k + 1
                    jh = j - m + 1
                    if dim_y > ih >= 0 and dim_x > jh >= 0:
                        if sig[ih, jh, 0] < 60 and sig[ih, jh, 1] < 60 and sig[ih, jh, 2] < 60:
                            vec = np.zeros([3, 1])
                            vec[0] = mag_fp[ih, jh, 0] * h[k, m]
                            vec[1] = mag_fp[ih, jh, 1] * h[k, m]
                            vec[2] = mag_fp[ih, jh, 2] * h[k, m]
                            fres = np.add(fres, vec)
                            hsum += h[k, m]

            if np.all(fres * hsum == 0):
                mag[i, j, :] = mag_fp[i, j, :]
            else:
                tmp = fres / hsum
                mag[i, j, 0] = tmp[0, 0]
                mag[i, j, 1] = tmp[1, 0]
                mag[i, j, 2] = tmp[2, 0]

    macc = np.ones([dim_y, dim_x]) * init_unc
    # MFP uncertainty
    # FP  uncertainty
    k = 0
    m = 0
    mean_grad = np.zeros([3, 1])
    accdata = []
    accdata.append(0)

    mag_quality_bad = 15
    macc_quality_bad = 20

    for i in range(0, dim_y):
        for j in range(0, dim_x):
            cxx = np.zeros([2, 2])

            im1 = i - 1
            ip1 = i + 1
            jm1 = j - 1
            jp1 = j + 1

            # print(i,' ', j,' ', mag_quality[j,i])
            if im1 >= 0 and jm1 >= 0 and ip1 < dim_y and jp1 < dim_x and max(sig[i, j, :]) < ini_sig_thr and \
                    mag_quality_bad > mag_quality_converted[j, i] > 0:
                mag_sig2 = np.matrix([0.0, 0.0, 0.0])
                mag_sig2[0, 0] = sig[i, j, 0] ** 2 + sig_mag_noise ** 2
                mag_sig2[0, 1] = sig[i, j, 1] ** 2 + sig_mag_noise ** 2
                mag_sig2[0, 2] = sig[i, j, 2] ** 2 + sig_mag_noise ** 2

                cmeas = np.diag(mag_sig2.A1)
                hor_dev = np.zeros([3, 1])
                count_hor = 0
                mag_curr = np.zeros([3, 1])
                mag_im1 = np.zeros([3, 1])
                mag_ip1 = np.zeros([3, 1])
                mag_jm1 = np.zeros([3, 1])
                mag_jp1 = np.zeros([3, 1])

                mag_curr[0, 0] = mag[i, j, 0]
                mag_curr[1, 0] = mag[i, j, 1]
                mag_curr[2, 0] = mag[i, j, 2]

                mag_im1[0, 0] = mag[im1, j, 0]
                mag_im1[1, 0] = mag[im1, j, 1]
                mag_im1[2, 0] = mag[im1, j, 2]

                mag_ip1[0, 0] = mag[ip1, j, 0]
                mag_ip1[1, 0] = mag[ip1, j, 1]
                mag_ip1[2, 0] = mag[ip1, j, 2]

                mag_jm1[0, 0] = mag[i, jm1, 0]
                mag_jm1[1, 0] = mag[i, jm1, 1]
                mag_jm1[2, 0] = mag[i, jm1, 2]

                mag_jp1[0, 0] = mag[i, jp1, 0]
                mag_jp1[1, 0] = mag[i, jp1, 1]
                mag_jp1[2, 0] = mag[i, jp1, 2]

                if max(sig[im1, j, :]) < ini_sig_thr and mag_quality_converted[j, im1] > 0 and mag_quality_converted[
                    j, im1] < mag_quality_bad:
                    hor_dev += (mag_curr - mag_im1) / cellsize
                    count_hor += 1

                if max(sig[ip1, j, :]) < ini_sig_thr and mag_quality_converted[j, ip1] > 0 and mag_quality_converted[
                    j, ip1] < mag_quality_bad:
                    hor_dev += (mag_ip1 - mag_curr) / cellsize
                    count_hor += 1

                if count_hor > 0:
                    hor_dev /= count_hor

                ver_dev = np.zeros([3, 1])
                count_ver = 0

                if max(sig[i, jm1, :]) < ini_sig_thr and mag_quality_bad > mag_quality_converted[jm1, i] > 0:
                    ver_dev += (mag_curr - mag_jm1) / cellsize
                    count_ver += 1

                if max(sig[i, jp1, :]) < ini_sig_thr and mag_quality_bad > mag_quality_converted[jp1, i] > 0:
                    ver_dev += (mag_jp1 - mag_curr) / cellsize
                    count_ver += 1

                if count_ver > 0:
                    ver_dev /= count_ver
                acc = init_unc

                if count_hor > 0 and count_ver > 0:
                    df = np.matrix('0.0 0.0; 0.0 0.0; 0.0 0.0')
                    df[0, 0] = hor_dev[0, 0]
                    df[1, 0] = hor_dev[1, 0]
                    df[2, 0] = hor_dev[2, 0]
                    df[0, 1] = ver_dev[0, 0]
                    df[1, 1] = ver_dev[1, 0]
                    df[2, 1] = ver_dev[2, 0]

                    if np.linalg.det(cmeas) > 0.00000001:
                        m_matrix = df.transpose() * np.linalg.inv(cmeas) * df
                        if np.linalg.det(m_matrix) > 0.00000001:
                            cxx = np.linalg.inv(m_matrix)
                            t1, t2 = np.linalg.eig(cxx)
                            acc = max(t1) ** 0.5  # here acc matches Matlab

                elif count_hor > 0 and count_ver == 0:
                    acc = 1 / (hor_dev[0, 0] ** 2 / cmeas[0, 0] + hor_dev[1, 0] ** 2 / cmeas[1, 1] +
                               hor_dev[2, 0] ** 2 / cmeas[2, 2]) ** 0.5
                elif count_hor == 0 and count_ver > 0:
                    acc = 1 / (ver_dev[0, 0] ** 2 / cmeas[0, 0] + ver_dev[1, 0] ** 2 / cmeas[1, 1] +
                               ver_dev[2, 0] ** 2 / cmeas[2, 2]) ** 0.5
                elif count_hor == 0 and count_ver == 0:
                    acc = macc_quality_bad

                # acc is the same here
                # mean gradient
                m += 1
                if count_hor > 0 and count_ver > 0:
                    mean_grad += (abs(hor_dev) + abs(ver_dev)) / 2
                elif count_hor > 0 and count_ver == 0:
                    mean_grad += abs(hor_dev)
                elif count_hor == 0 and count_ver > 0:
                    mean_grad += abs(ver_dev)

                if acc > init_unc:
                    k += 1
                    accdata.append(acc)

                macc[i, j] = acc
                # print(macc[i, j])
            else:
                if im1 >= 0 and jm1 >= 0 and ip1 < dim_y and jp1 < dim_x and max(sig[i, j, :]) < ini_sig_thr:
                    macc[i, j] = macc_quality_bad
        # print("================")
    if m > 0:
        mean_grad /= m

    mean_unc = statistics.mean(accdata)
    # print('mean unc', mean_unc)

    mean_std = np.zeros([3, 1])
    count = 0
    for i in range(0, dim_y):
        for j in range(0, dim_x):
            curstd = np.zeros([3, 1])
            curstd[0] = sig[i, j, 0]
            curstd[1] = sig[i, j, 1]
            curstd[2] = sig[i, j, 2]

            if max(curstd) < 60:
                count += 1
                mean_std += curstd

    if count > 0:
        mean_std /= count

    # print(mean_std)

    mfp_unc = macc

    # for i in range(0, dim_y):
    #    for j in range(0, dim_x-1):
    #        print(mfp_unc[i, j])
    #    print("================")

    return mean_unc, mfp_unc


def get_mfp_unc_colormap():
    # blue, gray, green, yellow, red
    a = np.array([[140, 212, 252],
                  [100, 100, 100],
                  [0, 255, 0],
                  [255, 255, 0],
                  [255, 0, 0]])

    return a


def one_cell_convert_mfp_unc(unc, mfp_cov):
    threshold_red = 7.5
    threshold_yellow = 4

    if unc < 0:
        res = 0
    elif mfp_cov < 200:
        res = 1
    elif unc <= threshold_yellow:
        res = 2
    elif unc <= threshold_red:
        res = 3
    else:
        res = 4

    return res


def convert_mfp_unc(M, mfp_coverage):
    m_conv = np.zeros([M.shape[0], M.shape[1]])
    mfp_coverage_trans = mfp_coverage.transpose()
    n0 = mfp_coverage_trans.shape[0]
    n1 = mfp_coverage_trans.shape[1]
    for i in range(0, M.shape[0]):
        for j in range(0, M.shape[1]):
            if i < n0 and j < n1:
                m_conv[i, j] = one_cell_convert_mfp_unc(M[i, j], mfp_coverage_trans[i, j])

    return m_conv


def plot_mfp_uncertainty(mfp_unc_matrix, mfp_coverage, real_floor, output_directory, output_plot_file_name):
    conv_mfp_unc = convert_mfp_unc(mfp_unc_matrix, mfp_coverage)

    mfp_unc_cmap = get_mfp_unc_colormap()

    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]
    outpath = os.path.join(output_directory, output_plot_file_name)

    plot_images.draw_picture(conv_mfp_unc, mfp_unc_cmap, outpath)
