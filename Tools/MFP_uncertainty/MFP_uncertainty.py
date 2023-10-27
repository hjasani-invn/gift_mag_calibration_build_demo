import numpy as np
import math
import os
import sys
import LoadMFP3
import statistics
import matplotlib.pyplot as plt


def MFP_uncertainty(MFP3_fName, dimX, dimY, floor, cellsize):
    ini_sig_thr = 60
    sig_mag_noise = 0

    init_unc = -5

    # 2d filter (gaussian)
    hsize = 1
    hsig = 0.5
    hsum = 0

    h = np.zeros([hsize*2+1, hsize*2+1])
    for k in range(-hsize, hsize+1):
        for m in range(-hsize, hsize+1):
            b = math.exp(-(k**2 + m**2)*cellsize**2/2/hsig**2)
            hsum += b
            h[k+hsize, m+hsize] = b

    h /= hsum

    magFP, sig = LoadMFP3.LoadMFP3(MFP3_fName, dimX, dimY, cellsize, floor)

    dimX = int(math.ceil(dimX/cellsize))
    dimY = int(math.ceil(dimY/cellsize))
    print(dimX, dimY)

    # 2d filtering of magFP
    mag = np.zeros([magFP.shape[0], magFP.shape[1], magFP.shape[2]])

    for i in range(0, dimY):
        for j in range(0, dimX):
            fres = np.zeros([3, 1])
            hsum = 0
            for k in range(0, hsize*2 + 1):
                for m in range(0, hsize*2 + 1):
                    ih = i - k + 1
                    jh = j - m + 1
                    if ih >= 0 and jh >= 0 and ih < dimY and jh < dimX:
                        if sig[ih, jh, 0] < 60 and sig[ih, jh, 1] < 60 and sig[ih, jh, 2] < 60:
                            vec = np.zeros([3, 1])
                            vec[0] = magFP[ih, jh, 0] * h[k, m]
                            vec[1] = magFP[ih, jh, 1] * h[k, m]
                            vec[2] = magFP[ih, jh, 2] * h[k, m]
                            fres = np.add(fres, vec)
                            hsum += h[k, m]

            if np.all(fres*hsum == 0):
                mag[i, j, :] = magFP[i, j, :]
            else:
                tmp = fres/hsum
                mag[i, j, 0] = tmp[0, 0]
                mag[i, j, 1] = tmp[1, 0]
                mag[i, j, 2] = tmp[2, 0]

    macc = np.ones([dimY, dimX]) * init_unc
    # MFP uncertainty
    # FP  uncertainty
    k = 0
    m = 0
    mean_grad = np.zeros([3, 1])
    accdata = []
    accdata.append(0)


    for i in range(0, dimY):
        for j in range(0, dimX):
            Cxx = np.zeros([2, 2])

            im1 = i - 1
            ip1 = i + 1
            jm1 = j - 1
            jp1 = j + 1

            if im1 >= 0 and jm1 >= 0 and ip1 < dimY and jp1 < dimX and max(sig[i, j, :]) < ini_sig_thr:
                mag_sig2 = np.matrix([0.0, 0.0, 0.0])
                # mag_sig2 = np.matrix([0, 0, 0])
                mag_sig2[0, 0] = sig[i, j, 0]**2 + sig_mag_noise**2
                mag_sig2[0, 1] = sig[i, j, 1]**2 + sig_mag_noise**2
                mag_sig2[0, 2] = sig[i, j, 2]**2 + sig_mag_noise**2

                Cmeas = np.diag(mag_sig2.A1)
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

                if max(sig[im1, j, :]) < ini_sig_thr:
                    hor_dev += (mag_curr - mag_im1)/cellsize
                    count_hor += 1

                if max(sig[ip1, j, :]) < ini_sig_thr:
                    hor_dev += (mag_ip1 - mag_curr)/cellsize
                    count_hor += 1

                if count_hor > 0:
                    hor_dev /= count_hor

                ver_dev = np.zeros([3, 1])
                count_ver = 0

                if max(sig[i, jm1, :]) < ini_sig_thr:
                    ver_dev += (mag_curr - mag_jm1)/cellsize
                    count_ver += 1

                if max(sig[i, jp1, :]) < ini_sig_thr:
                    ver_dev += (mag_jp1 - mag_curr)/cellsize
                    count_ver += 1


                if count_ver > 0:
                    ver_dev /= count_ver
                acc = init_unc

                if count_hor > 0 and count_ver > 0:
                    DF = np.matrix('0.0 0.0; 0.0 0.0; 0.0 0.0')
                    DF[0, 0] = hor_dev[0, 0]
                    DF[1, 0] = hor_dev[1, 0]
                    DF[2, 0] = hor_dev[2, 0]
                    DF[0, 1] = ver_dev[0, 0]
                    DF[1, 1] = ver_dev[1, 0]
                    DF[2, 1] = ver_dev[2, 0]

                    if np.linalg.det(Cmeas) > 0.00000001:
                        M = DF.transpose() * np.linalg.inv(Cmeas) * DF
                        if np.linalg.det(M) > 0.00000001:
                            Cxx = np.linalg.inv(M)
                            t1, t2 = np.linalg.eig(Cxx)
                            acc = max(t1)**0.5  # here acc matches Matlab

                elif count_hor > 0 and count_ver == 0:
                    acc = 1/( hor_dev[0, 0]**2/Cmeas[0, 0] + hor_dev[1, 0]**2/Cmeas[1, 1] + hor_dev[2, 0]**2/Cmeas[2, 2])**0.5
                elif count_hor == 0 and count_ver > 0:
                    acc = 1/( ver_dev[0, 0]**2/Cmeas[0, 0] + ver_dev[1, 0]**2/Cmeas[1, 1] + ver_dev[2, 0]**2/Cmeas[2, 2])**0.5

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

    if m > 0:
        mean_grad /= m

    mean_unc = statistics.mean(accdata)
    print('mean unc', mean_unc)

    mean_std = np.zeros([3, 1])
    count = 0
    for i in range(0, dimY):
        for j in range(0, dimX):
            curstd = np.zeros([3, 1])
            curstd[0] = sig[i, j, 0]
            curstd[1] = sig[i, j, 1]
            curstd[2] = sig[i, j, 2]

            if max(curstd) < 60:
                count += 1
                mean_std += curstd

    if count > 0:
        mean_std /= count

    print(mean_std)

    MFP_Unc = macc

    # plt.pcolor(macc, cmap='jet', vmin=0, vmax=20, edgecolors='k', linewidths=0.1)
    # plt.title('Amount of BLE measurements per beacon')
    # plt.colorbar()
    # plt.show()
    # plt.close()
    return MFP_Unc


if __name__ == "__main__":
    fName = sys.argv[1]
    floor = int(sys.argv[2])
    sizeX = float(sys.argv[3])
    sizeY = float(sys.argv[4])
    cellsize = float(sys.argv[5])

    # fName = 'core.mfp3'
    # sizeX = 385
    # sizeY = 149
    # cellsize = 2
    # floor = 0

    MFP_Unc = MFP_uncertainty(fName, sizeX, sizeY, floor, cellsize)