import numpy as np
import math
import os


def LoadMFP3(fName, sizeX, sizeY, cell_size, floor):
    # floorCells = sizeX*sizeY/(cell_size**2)
    # skipBytes = int(floor*sizeX*sizeY/(cell_size**2)*8*3*4) #TODO: check if works (needed when floor is > 0)

    numX = int(math.ceil(sizeX/cell_size))
    numY = int(math.ceil(sizeY/cell_size))

    floorCells = numX * numY
    skipBytes = floor * numX * numY * 8 * 3 *4

    avX = np.zeros([numX, numY])
    avY = np.zeros([numX, numY])
    avZ = np.zeros([numX, numY])
    sigX = np.zeros([numX, numY])
    sigY = np.zeros([numX, numY])
    sigZ = np.zeros([numX, numY])

    with open(fName, 'rb') as fid:
        fid.seek(skipBytes, os.SEEK_SET)
        data_array = np.fromfile(fid, np.float32)

    iX = 0
    iY = 0
    i = 0
    idx = 0

    while i < len(data_array) and idx < floorCells:
        # print('ix iy ', iX, iY, ' size x ', numX, ' size y ', numY)

        avX[iX, iY] = data_array[i]
        sigX[iX, iY] = data_array[i + 1]
        avY[iX, iY] = data_array[i + 8]
        sigY[iX, iY] = data_array[i + 9]
        avZ[iX, iY] = data_array[i + 16]
        sigZ[iX, iY] = data_array[i + 17]

        iX += 1
        if iX > numX - 1:
            iX = 0
            iY += 1

        idx += 1
        i += 24

    mfp = np.zeros([numY, numX, 3])
    sig = np.zeros([numY, numX, 3])

    mfp[:, :, 0] = avX.transpose()
    mfp[:, :, 1] = avY.transpose()
    mfp[:, :, 2] = avZ.transpose()

    sig[:, :, 0] = sigX.transpose()
    sig[:, :, 1] = sigY.transpose()
    sig[:, :, 2] = sigZ.transpose()

    return mfp, sig


# fName = 'core.mfp3'
# sizeX = 385
# sizeY = 149
# cell_size = 2
# floor = 0

# fName = 'test_10x5.mfp3'
# sizeX = 10
# sizeY = 5
# cell_size = 1
# floor = 0

# LoadMFP3(fName, sizeX, sizeY, cell_size, floor)