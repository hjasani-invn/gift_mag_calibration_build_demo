import re


def convert(input_wifi4):

    output_wifi3 = input_wifi4[:-5] + 'wifi3'

    with open(input_wifi4, 'rb') as wfp:
        data = wfp.read()
        t = data.find(b'POINT')
        cut_data = data[t:]
        header = data[:t]

    with open(output_wifi3, 'wb') as binary_file:
        binary_file.write(cut_data)

    return output_wifi3, header