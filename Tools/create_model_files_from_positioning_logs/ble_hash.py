import numpy as np

def get_ble_hash(major, minor, uuid):
    hash = np.dtype('u4') 
    hash = ((int(major)) << 16) + int(minor)
    #print(hash)
    return hash

