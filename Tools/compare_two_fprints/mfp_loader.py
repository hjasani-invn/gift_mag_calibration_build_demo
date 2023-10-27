import numpy as np
import math
import os


def loadMFP3(fName, sizeX, sizeY, cell_size, floor):
    # floorCells = sizeX*sizeY/(cell_size**2)
    # skipBytes = int(floor*sizeX*sizeY/(cell_size**2)*8*3*4) #TODO: check if works (needed when floor is > 0)

    numX = int(math.ceil(sizeX/cell_size))
    numY = int(math.ceil(sizeY/cell_size))

    floorCells = numX * numY
    skipBytes = floor * numX * numY * 8 * 3 *4

    av1X = np.zeros([numX, numY], np.float32)
    sig1X = np.zeros([numX, numY], np.float32)
    w1X = np.zeros([numX, numY], np.float32)
    av2X = np.zeros([numX, numY], np.float32)
    sig2X = np.zeros([numX, numY], np.float32)
    w2X = np.zeros([numX, numY], np.float32)
    reserved1X = np.zeros([numX, numY], np.float32)
    reserved2X = np.zeros([numX, numY], np.float32)

    av1Y = np.zeros([numX, numY], np.float32)
    sig1Y = np.zeros([numX, numY], np.float32)
    w1Y = np.zeros([numX, numY], np.float32)
    av2Y = np.zeros([numX, numY], np.float32)
    sig2Y = np.zeros([numX, numY], np.float32)
    w2Y = np.zeros([numX, numY], np.float32)
    reserved1Y = np.zeros([numX, numY], np.float32)
    reserved2Y = np.zeros([numX, numY], np.float32)

    av1Z = np.zeros([numX, numY], np.float32)
    sig1Z = np.zeros([numX, numY], np.float32)
    w1Z = np.zeros([numX, numY], np.float32)
    av2Z = np.zeros([numX, numY], np.float32)
    sig2Z = np.zeros([numX, numY], np.float32)
    w2Z = np.zeros([numX, numY], np.float32)
    reserved1Z = np.zeros([numX, numY], np.float32)
    reserved2Z = np.zeros([numX, numY], np.float32)


    with open(fName, 'rb') as fid:
        fid.seek(skipBytes, os.SEEK_SET)
        data_array = np.fromfile(fid, np.float32)

    iX = 0
    iY = 0
    i = 0
    idx = 0

    mfp = []

    while i < len(data_array) and idx < floorCells:
        # print('ix iy ', iX, iY, ' size x ', numX, ' size y ', numY)

        av1X[iX, iY] = data_array[i]
        sig1X[iX, iY] = data_array[i + 1]
        w1X[iX, iY] = data_array[i + 2]
        av2X[iX, iY] = data_array[i + 3]
        sig2X[iX, iY] = data_array[i + 4]
        w2X[iX, iY] = data_array[i + 5]
        reserved1X[iX, iY] = data_array[i + 6]
        reserved2X[iX, iY] = data_array[i + 7]

        av1Y[iX, iY] = data_array[i + 8]
        sig1Y[iX, iY] = data_array[i + 9]
        w1Y[iX, iY] = data_array[i + 10]
        av2Y[iX, iY] = data_array[i + 11]
        sig2Y[iX, iY] = data_array[i + 12]
        w2Y[iX, iY] = data_array[i + 13]
        reserved1Y[iX, iY] = data_array[i + 14]
        reserved2Y[iX, iY] = data_array[i + 15]

        av1Z[iX, iY] = data_array[i + 16]
        sig1Z[iX, iY] = data_array[i + 17]
        w1Z[iX, iY] = data_array[i + 18]
        av2Z[iX, iY] = data_array[i + 19]
        sig2Z[iX, iY] = data_array[i + 20]
        w2Z[iX, iY] = data_array[i + 21]
        reserved1Z[iX, iY] = data_array[i + 22]
        reserved2Z[iX, iY] = data_array[i + 23]

        #mfpX.append(data_array)

        iX += 1
        if iX > numX - 1:
            iX = 0
            #mfpY.append(mfpX)
            iY += 1

        idx += 1
        i += 24



    #mfp = {'av1X': av1X, 'sig1X': sig1X, 'w1X': w1X, 'av2X': av2X, 'sig2X': sig2X, 'w2X': w2X, 'reserved1X': reserved1X, 'reserved2X': reserved2X, 'av1Y': av1Y, 'sig1Y': sig1Y, 'w1Y': w1Y, 'av2Y': av2Y, 'sig2Y': sig2Y, 'w2Y': w2Y, 'reserved1Y': reserved1Y, 'reserved2Y': reserved2Y, 'av1Z': av1Z, 'sig1Z': sig1Z, 'w1Z': w1Z, 'av2Z': av2Z, 'sig2Z': sig2Z, 'w2Z': w2Z, 'reserved1Z': reserved1Z, 'reserved2Z': reserved2Z}
    mfpX = {'av1': av1X, 'sig1': sig1X, 'w1': w1X, 'av2': av2X, 'sig2': sig2X, 'w2': w2X, 'reserved1': reserved1X, 'reserved2': reserved2X}
    mfpY = {'av1': av1Y, 'sig1': sig1Y, 'w1': w1Y, 'av2': av2Y, 'sig2': sig2Y, 'w2': w2Y, 'reserved1': reserved1Y, 'reserved2': reserved2Y}
    mfpZ = {'av1': av1Z, 'sig1': sig1Z, 'w1': w1Z, 'av2': av2Z, 'sig2': sig2Z, 'w2': w2Z, 'reserved1': reserved1Z, 'reserved2': reserved2Z}

    mfp.append(mfpX)
    mfp.append(mfpY)
    mfp.append(mfpZ)

    return mfp, numX, numY

def loadMFP4(fName, sizeX, sizeY, cell_size, floor):
    # floorCells = sizeX*sizeY/(cell_size**2)
    # skipBytes = int(floor*sizeX*sizeY/(cell_size**2)*8*3*4) #TODO: check if works (needed when floor is > 0)

    numX = int(math.ceil(sizeX/cell_size))
    numY = int(math.ceil(sizeY/cell_size))

    floorCells = numX * numY
    skipBytes = floor * numX * numY * 8 * 3 *4

    av1X = np.zeros([numX, numY], np.float32)
    sig1X = np.zeros([numX, numY], np.float32)
    w1X = np.zeros([numX, numY], np.float32)
    av2X = np.zeros([numX, numY], np.float32)
    sig2X = np.zeros([numX, numY], np.float32)
    w2X = np.zeros([numX, numY], np.float32)
    reserved1X = np.zeros([numX, numY], np.float32)
    reserved2X = np.zeros([numX, numY], np.float32)

    av1Y = np.zeros([numX, numY], np.float32)
    sig1Y = np.zeros([numX, numY], np.float32)
    w1Y = np.zeros([numX, numY], np.float32)
    av2Y = np.zeros([numX, numY], np.float32)
    sig2Y = np.zeros([numX, numY], np.float32)
    w2Y = np.zeros([numX, numY], np.float32)
    reserved1Y = np.zeros([numX, numY], np.float32)
    reserved2Y = np.zeros([numX, numY], np.float32)

    av1Z = np.zeros([numX, numY], np.float32)
    sig1Z = np.zeros([numX, numY], np.float32)
    w1Z = np.zeros([numX, numY], np.float32)
    av2Z = np.zeros([numX, numY], np.float32)
    sig2Z = np.zeros([numX, numY], np.float32)
    w2Z = np.zeros([numX, numY], np.float32)
    reserved1Z = np.zeros([numX, numY], np.float32)
    reserved2Z = np.zeros([numX, numY], np.float32)


    #with open(fName, 'rb') as fid:
    #    header = np.fromfile(fid, np.int8)
    #    fid.seek(skipBytes, os.SEEK_SET)
    #    data_array = np.fromfile(fid, np.float32)

    fid = open(fName, 'rb')
    mfp_header = fid.read(195)
    print(len(mfp_header))
    fid.seek(skipBytes, os.SEEK_CUR)
    data_array = np.fromfile(fid, np.float32)


    iX = 0
    iY = 0
    i = 0
    idx = 0

    mfp = []

    while i < len(data_array) and idx < floorCells:
        # print('ix iy ', iX, iY, ' size x ', numX, ' size y ', numY)

        av1X[iX, iY] = data_array[i]
        sig1X[iX, iY] = data_array[i + 1]
        w1X[iX, iY] = data_array[i + 2]
        av2X[iX, iY] = data_array[i + 3]
        sig2X[iX, iY] = data_array[i + 4]
        w2X[iX, iY] = data_array[i + 5]
        reserved1X[iX, iY] = data_array[i + 6]
        reserved2X[iX, iY] = data_array[i + 7]

        av1Y[iX, iY] = data_array[i + 8]
        sig1Y[iX, iY] = data_array[i + 9]
        w1Y[iX, iY] = data_array[i + 10]
        av2Y[iX, iY] = data_array[i + 11]
        sig2Y[iX, iY] = data_array[i + 12]
        w2Y[iX, iY] = data_array[i + 13]
        reserved1Y[iX, iY] = data_array[i + 14]
        reserved2Y[iX, iY] = data_array[i + 15]

        av1Z[iX, iY] = data_array[i + 16]
        sig1Z[iX, iY] = data_array[i + 17]
        w1Z[iX, iY] = data_array[i + 18]
        av2Z[iX, iY] = data_array[i + 19]
        sig2Z[iX, iY] = data_array[i + 20]
        w2Z[iX, iY] = data_array[i + 21]
        reserved1Z[iX, iY] = data_array[i + 22]
        reserved2Z[iX, iY] = data_array[i + 23]

        #mfpX.append(data_array)

        iX += 1
        if iX > numX - 1:
            iX = 0
            #mfpY.append(mfpX)
            iY += 1

        idx += 1
        i += 24



    #mfp = {'av1X': av1X, 'sig1X': sig1X, 'w1X': w1X, 'av2X': av2X, 'sig2X': sig2X, 'w2X': w2X, 'reserved1X': reserved1X, 'reserved2X': reserved2X, 'av1Y': av1Y, 'sig1Y': sig1Y, 'w1Y': w1Y, 'av2Y': av2Y, 'sig2Y': sig2Y, 'w2Y': w2Y, 'reserved1Y': reserved1Y, 'reserved2Y': reserved2Y, 'av1Z': av1Z, 'sig1Z': sig1Z, 'w1Z': w1Z, 'av2Z': av2Z, 'sig2Z': sig2Z, 'w2Z': w2Z, 'reserved1Z': reserved1Z, 'reserved2Z': reserved2Z}
    mfpX = {'av1': av1X, 'sig1': sig1X, 'w1': w1X, 'av2': av2X, 'sig2': sig2X, 'w2': w2X, 'reserved1': reserved1X, 'reserved2': reserved2X}
    mfpY = {'av1': av1Y, 'sig1': sig1Y, 'w1': w1Y, 'av2': av2Y, 'sig2': sig2Y, 'w2': w2Y, 'reserved1': reserved1Y, 'reserved2': reserved2Y}
    mfpZ = {'av1': av1Z, 'sig1': sig1Z, 'w1': w1Z, 'av2': av2Z, 'sig2': sig2Z, 'w2': w2Z, 'reserved1': reserved1Z, 'reserved2': reserved2Z}

    mfp.append(mfpX)
    mfp.append(mfpY)
    mfp.append(mfpZ)

    return mfp_header, mfp, numX, numY

