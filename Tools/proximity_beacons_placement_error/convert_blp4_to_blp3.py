## @package convert_blp4_to_blp3
#  This module provides blp4 to blp3 conversions

def convert(input_blp4):

    output_bl[3 = input_blp4[:-5] + 'blp3'

    with open(input_blp4, 'rb') as wfp:
        data = wfp.read()
        t = data.find(b'POINT')
        cut_data = data[t:]
        header = data[:t]

    with open(output_blp3, 'wb') as binary_file:
        binary_file.write(cut_data)

    return output_blp3