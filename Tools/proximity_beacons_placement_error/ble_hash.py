## @package get_ble_hash
#  This module provides ble hash conversions
import numpy as np

## calculates hash from ble major, minor, uuid
#
# @param[in] major - ble major
# @param[in] minor - ble minor
# @param[in] uuid - ble uuid
# @param[out] nle hash
#
def get_ble_hash(major, minor, uuid):
    hash = np.dtype('u4') 
    hash = ((int(major)) << 16) + int(minor)
    #print(hash)
    return hash

