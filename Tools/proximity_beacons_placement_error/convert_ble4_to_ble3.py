## @package convert_ble4_to_ble3
#  This module provides ble4 to ble3 conversions

def convert(input_ble4):

    output_ble3 = input_ble4[:-5] + 'ble3'

    with open(input_ble4, 'rb') as wfp:
        data = wfp.read()
        t = data.find(b'POINT')
        cut_data = data[t:]
        header = data[:t]

    with open(output_ble3, 'wb') as binary_file:
        binary_file.write(cut_data)

    return output_ble3