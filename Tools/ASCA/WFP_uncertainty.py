# function WFP_uncertainty estimates predicted or expected uncertainy of
# WFP-based positioning based on properties of WFP map
# Parameters:
# file_wfp4 is WFP map in wifi4 format
# dimX and dimY are WFP horizontal and vertical dimensions in meters
# floor is a number of floor starting from 0
# cellsize is  WFP cell size in meters
# Return values:
# WFP_Unc is 2D array of WFP uncertainties per cell
# mean_unc is an average uncertainty
# reslt=1 if OK and 0 otherwise (usually if wifi4 file has not been found)

import os
import sys
import re
import matplotlib as mpl
# mpl.use('TkAgg')
mpl.use('Agg')
import math
import numpy as np
import plot_images


def get_wfp_uncertainty(file_wfp4, dim_x, dim_y, floor, cell_size):
    if not os.path.isfile(file_wfp4):
        return 0, 0, 0, 0

    with open(file_wfp4, 'rb') as wfp:
        data = wfp.read()
        point_location = data.find(b'POINT')

        if point_location < 0:
            return 0, 0, 0, 0

        cut_data = data[point_location:]
        wifi4_header = data[:point_location]

        text_wifi_fp = cut_data.decode('utf-8')
        wfp_lines = text_wifi_fp.splitlines(False)

    b = wfp_lines

    wfp = []
    ap_index = []
    xy_index = []
    k = 0
    curfloor = -5000
    # APs = []
    init_unc = -5
    wifi_point = np.zeros(6)

    for i in range(len(b)):
        # c=b(i)
        new_str = b[i].split()
        if re.match(r'POINT.*', b[i]):
            cur_x = float(new_str[1])
            cur_y = float(new_str[2])
            curfloor = int(float(new_str[3]))
        if curfloor == floor:
            if re.match(r'AP_MAC', b[i]):
                alpha1 = float(new_str[2])
                mean1 = float(new_str[3])
                std1 = float(new_str[4])
                alpha2 = float(new_str[5])
                mean2 = float(new_str[6])
                std2 = float(new_str[7])

                if mean1 == -100 and alpha2 > 0.5 and 0 > mean2 > -80:
                    av_mean = mean2
                else:
                    if mean2 == -100 and alpha1 > 0.5 and 0 > mean1 > -80:
                        av_mean = mean1
                    else:
                        if 0 > mean1 > -80 and 0 > mean2 > -80:
                            av_mean = alpha1 * mean1 + alpha2 * mean2
                        else:
                            av_mean = -100
                av_std = 7.5  # sqrt(av_std^2+2*25)
                if av_mean > -90:
                    k = k + 1
                    # wfp[k,1]=cur_x
                    # wfp[k,2]=cur_y
                    # wfp[k,3]=curfloor
                    ##wfp[k,4]=str2double(new_str(1)) #BSSID
                    # wfp[k,5]=av_mean
                    # wfp[k,6]=av_std
                    wifi_point[0] = cur_x
                    wifi_point[1] = cur_y
                    wifi_point[2] = curfloor
                    wifi_point[3] = float(new_str[1])  # BSSID
                    wifi_point[4] = av_mean
                    wifi_point[5] = av_std
                    wfp.append(wifi_point.copy())
                    # ap_index=[ap_index,AP_search_key(cur_x, cur_y, new_str[1])]
                    # xy_index=[xy_index,XY_search_key(cur_x, cur_y)]
                    ap_index.append(ap_search_key(cur_x, cur_y, new_str[1]))
                    xy_index.append(xy_search_key(cur_x, cur_y))

    if len(wfp) == 0:
        return 0, 0, 0, 0

    max_x = 0
    max_y = 0
    for i in range(len(wfp)):
        wifi_point = wfp[i]
        max_x = max(max_x, wifi_point[0])
        max_y = max(max_y, wifi_point[1])
    # dim_x = int((dim_X + cellsize / 2) / cellsize + 0.5)
    # dim_y = int((dim_Y + cellsize / 2) / cellsize + 0.5)

    dim_x = int(math.ceil(dim_x / cell_size))
    dim_y = int(math.ceil(dim_y / cell_size))

    macc = init_unc * np.ones([dim_y, dim_x])
    macc_smooth = init_unc * np.ones([dim_y, dim_x])

    # wfp uncertainty
    for i in range(dim_y):
        for j in range(dim_x):
            cxx = np.zeros([2, 2])
            x = float(j) * cell_size + cell_size / 2
            y = float(i) * cell_size + cell_size / 2
            xy_key = xy_search_key(x, y)
            curr_cell = []
            try:
                curr_index = xy_index.index(xy_key)
                curr_cell.append(curr_index)
                while curr_index < (len(xy_index) - 2):
                    curr_index = xy_index.index(xy_key, curr_index + 1, len(xy_index) - 1)
                    curr_cell.append(curr_index)
            except Exception as ex:
                if len(curr_cell) == 0:
                    continue
            # print(curr_cell)

            xm1 = x - cell_size
            xp1 = x + cell_size
            ym1 = y - cell_size
            yp1 = y + cell_size
            # gradient for every ap
            hor_dev = []
            ver_dev = []
            sig2 = []
            sorted_wfp = []
            for k in curr_cell:
                # wifi_point = wfp[i]
                sorted_wfp.append(wfp[k])
                # print(wfp[k])
            sorted_wfp.sort(key=sort_method, reverse=True)
            # print(a)
            len_sorted_wfp = len(sorted_wfp)

            for k in range(len_sorted_wfp):
                wifi_point = sorted_wfp[k]
                rssi_curr, sig_curr = get_rssi_wfp(wifi_point)
                cur_ap = str(int(wifi_point[3]))

                count_hor = 0
                hor_dev_ap = 0
                count_ver = 0
                ver_dev_ap = 0
                # cell_xm1=find(strcmp(ap_index(:),xm1_key))
                # if length(cell_xm1)==1:
                #    [rssi, sig]=get_rssi_WFP(wfp(cell_xm1,:))
                #    hor_dev_ap=hor_dev_ap+(rssi_curr-rssi)
                #    count_hor=count_hor+1
                xm1_key = ap_search_key(xm1, y, cur_ap)
                try:
                    cell_xm1 = ap_index.index(xm1_key)
                    wifi_point = wfp[cell_xm1]
                    rssi, sig = get_rssi_wfp(wifi_point)
                    hor_dev_ap = hor_dev_ap + (rssi_curr - rssi)
                    count_hor = count_hor + 1
                except Exception as ex:
                    cell_xm1 = -1

                # cell_xp1=find(strcmp(ap_index(:),xp1_key))
                # if length(cell_xp1)==1:
                #    [rssi, sig]=get_rssi_WFP(wfp(cell_xp1,:))
                #    hor_dev_ap=hor_dev_ap+(rssi-rssi_curr)
                #    count_hor=count_hor+1
                xp1_key = ap_search_key(xp1, y, cur_ap)
                try:
                    cell_xp1 = ap_index.index(xp1_key)
                    wifi_point = wfp[cell_xp1]
                    rssi, sig = get_rssi_wfp(wifi_point)
                    hor_dev_ap = hor_dev_ap + (rssi - rssi_curr)
                    count_hor = count_hor + 1
                except Exception as ex:
                    cell_xp1 = -1
                # cell_ym1=find(strcmp(ap_index(:),ym1_key))
                # if length(cell_ym1)==1:
                #    [rssi, sig]=get_rssi_WFP(wfp(cell_ym1,:))
                #    ver_dev_ap=ver_dev_ap+(rssi_curr-rssi)
                #    count_ver=count_ver+1
                ym1_key = ap_search_key(x, ym1, cur_ap)
                try:
                    cell_ym1 = ap_index.index(ym1_key)
                    wifi_point = wfp[cell_ym1]
                    rssi, sig = get_rssi_wfp(wifi_point)
                    ver_dev_ap = ver_dev_ap + (rssi_curr - rssi)
                    count_ver = count_ver + 1
                except Exception as ex:
                    cell_ym1 = -1
                # cell_yp1=find(strcmp(ap_index(:),yp1_key))
                # if length(cell_yp1)==1:
                #    [rssi, sig]=get_rssi_WFP(wfp(cell_yp1,:))
                #    ver_dev_ap=ver_dev_ap+(rssi-rssi_curr)
                #    count_ver=count_ver+1
                yp1_key = ap_search_key(x, yp1, cur_ap)
                try:
                    cell_yp1 = ap_index.index(yp1_key)
                    wifi_point = wfp[cell_yp1]
                    rssi, sig = get_rssi_wfp(wifi_point)
                    ver_dev_ap = ver_dev_ap + (rssi - rssi_curr)
                    count_ver = count_ver + 1
                except Exception as ex:
                    cell_yp1 = -1

                if (count_hor > 0) and (count_ver > 0):
                    # hor_dev=[hor_dev;hor_dev_ap/count_hor]
                    hor_dev.append(hor_dev_ap / count_hor)
                    # ver_dev=[ver_dev;ver_dev_ap/count_ver]
                    ver_dev.append(ver_dev_ap / count_ver)
                elif (count_hor > 0) and (count_ver == 0):
                    # hor_dev=[hor_dev;hor_dev_ap/count_hor]
                    hor_dev.append(hor_dev_ap / count_hor)
                    # ver_dev=[ver_dev;0]
                    ver_dev.append(0)
                elif (count_hor == 0) and (count_ver > 0):
                    # hor_dev=[hor_dev;0];
                    hor_dev.append(0)
                    # ver_dev=[ver_dev;ver_dev_ap/count_ver]
                    ver_dev.append(ver_dev_ap / count_ver)
                if (count_hor > 0) or (count_ver > 0):
                    # sig2=[sig2;sig_curr*sig_curr];
                    sig2.append(sig_curr * sig_curr)

            # end for k in range(len_sorted_wfp)

            hor_0 = hor_dev.count(0)
            ver_0 = ver_dev.count(0)
            len_dev = len(hor_dev)
            if len_dev >= 2:
                if (len_dev > hor_0) and (len_dev > ver_0):
                    df = [hor_dev, ver_dev]
                    dp = np.zeros([len_dev, 2])
                    for k in range(len_dev):
                        dp[k][0] = df[0][k] / cell_size
                        dp[k][1] = df[1][k] / cell_size
                    cmeas = np.diag(sig2)
                    if np.linalg.det(cmeas) > 0.00000001:
                        # dp_trans = np.matrix.transpose(dp)
                        dp_trans = dp.transpose()
                        cmeas_inv = np.linalg.inv(cmeas)
                        m1 = np.dot(dp_trans, cmeas_inv)
                        m = np.dot(m1, dp)
                        if np.linalg.det(m) > 0.00000001:
                            cxx = np.linalg.inv(m)
                            eig_ = np.linalg.eig(cxx)
                            acc = math.sqrt(np.ndarray.max(eig_[0]))
                            macc[i, j] = acc
                elif (len_dev > hor_0) and (len_dev == ver_0):
                    # r=((hor_dev/cellsize).^2)./sig2
                    r = []
                    for k in range(len_dev):
                        x = math.pow(hor_dev[k] / cell_size, 2) / sig2[k]
                        r.append(x)
                    acc = 1 / math.sqrt(sum(r))
                    macc[i, j] = acc
                elif (len_dev == hor_0) and (len_dev > ver_0):
                    # r=((ver_dev/cellsize).^2)./sig2
                    r = []
                    for k in range(len_dev):
                        x = math.pow(ver_dev[k] / cell_size, 2) / sig2[k]
                        r.append(x)
                    acc = 1 / math.sqrt(sum(r))
                    macc[i, j] = acc

    # end for i in range(dim_y):
    #    end for j in range(dim_x)
    WFP_Unc = macc

    # print(WFP_Unc)

    # 2D filter (gaussian)
    hsize = 1
    # hsig=2.5
    hsig = cell_size
    hsum = 0
    n = 2 * hsize + 1
    h = np.zeros([n, n])
    for k1 in range(n):
        k = k1 - hsize
        for m1 in range(n):
            m = m1 - hsize
            b = math.exp(-(math.pow(k, 2) + math.pow(m, 2)) * pow(cell_size, 2) / 2 / pow(hsig, 2))
            hsum = hsum + b
            h[k + hsize, m + hsize] = b
    h = h / hsum

    for i in range(dim_y):
        for j in range(dim_x):
            fres = 0
            hsum = 0
            for k in range(hsize * 2 + 1):
                for m in range(hsize * 2 + 1):
                    ih = i - k + 1
                    jh = j - m + 1
                    if dim_y > ih >= 0 and dim_x > jh >= 0:
                        if macc[ih, jh] > 0:
                            fres = fres + macc[ih, jh] * h[k, m]
                            hsum = hsum + h[k, m]
            if macc[i, j] > 0:
                macc_smooth[i, j] = fres / hsum

    # print(macc_smooth)
    # mean_unc=mean(macc(find(macc>0)))
    x = []
    for i in range(len(macc)):
        for j in range(len(macc[i])):
            if macc[i, j] > 0:
                x.append(macc[i, j])

    mean_unc = -1
    if len(x) > 0:
        mean_unc = sum(x) / len(x)  # mean

    return macc, macc_smooth, mean_unc, 1


def sort_method(a):
    return a[4]


def ap_search_key(x, y, ap):
    s_x = str(int(x * 10))
    s_y = str(int(y * 10))
    key = s_x + s_y + ap
    return key


def xy_search_key(x, y):
    s_x = str(int(x * 10))
    s_y = str(int(y * 10))
    key = s_x + s_y
    return key


def get_rssi_wfp(wfp):
    rssi = wfp[4]
    sig = wfp[5]
    return rssi, sig


def get_wfp_unc_colormap():
    # blue, green, yellow, red
    a = np.array([[140, 212, 252],
                  [0, 255, 0],
                  [255, 255, 0],
                  [255, 0, 0]])

    return a


def one_cell_convert_wfp_unc(unc):
    threshold_red = 15
    threshold_yellow = 7.5

    if unc < 0:
        res = 0

    elif unc <= threshold_yellow:
        res = 1
    elif unc <= threshold_red:
        res = 2
    else:
        res = 3

    return res


def convert_wfp_unc(M):
    m_conv = np.zeros([M.shape[0], M.shape[1]])
    for i in range(0, M.shape[0]):
        for j in range(0, M.shape[1]):
            m_conv[i, j] = one_cell_convert_wfp_unc(M[i, j])

    return m_conv


def plot_wfp_uncertainty(wfp_unc, output_directory, output_plot_file_name, real_floor):

    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]

    outpath = os.path.join(output_directory, output_plot_file_name)
    # Start plotting

    wfp_unc_cmap = get_wfp_unc_colormap()

    wfp_unc_converted = convert_wfp_unc(wfp_unc)

    plot_images.draw_picture(wfp_unc_converted, wfp_unc_cmap, outpath)


if __name__ == "__main__":  # for tests only

    if len(sys.argv) < 7:
        print('small number of args')
        raise SystemExit
    print('sys.argv[1] =', sys.argv[1])
    print('sys.argv[2] =', sys.argv[2])
    print('sys.argv[3] =', sys.argv[3])
    print('sys.argv[4] =', sys.argv[4])
    print('sys.argv[5] =', sys.argv[5])
    print('sys.argv[6] =', sys.argv[6])

    file_wfp4 = sys.argv[1]
    dim_x = float(sys.argv[2])
    dim_y = float(sys.argv[3])
    floor = float(sys.argv[4])
    cell_size = float(sys.argv[5])
    out_folder = sys.argv[6].replace("\r", "")

    print("")
    wfp_unc, wfp_unc_smooth, mean_unc, result = get_wfp_uncertainty(file_wfp4, dim_x, dim_y, floor, cell_size)

    output_plot_file_name = "WFP_uncertainty.png"
    if result == 1:
        plot_wfp_uncertainty(wfp_unc_smooth, out_folder, output_plot_file_name, int(floor))
