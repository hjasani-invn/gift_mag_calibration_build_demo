import json
import os
import sys
import csv
import io
import numpy as np
import math
import merge

import coordinates_venue_angles_parser
import TransformParametersCalculation

if  __name__ == "__main__":
    version = "1.0.5"
    print("json automation tool, version ", version)

    template_json = sys.argv[1]
    venue_spec_json = sys.argv[2]
    coordinates_angles_file = sys.argv[3]
    floors_json = sys.argv[4]

    # reading template file
    f1 = open(template_json, 'r')
    template_text = f1.read()
    template_data = json.loads(template_text)
    f1.close()

    # reading venue file
    f3 = open(venue_spec_json, 'r')
    venue_spec_text = f3.read()
    venue_spec_data = json.loads(venue_spec_text)
    f3.close()

    # coordinates of venue angles parsing from *_CoordinatesFile.txt
    P1, P2, P3, P4, size_x, size_y = coordinates_venue_angles_parser.coordinates_venue_angles_parsing(coordinates_angles_file, "P1", "P2", "P3", "P4")
    fp_size_x = size_x #math.ceil(size_x) # ceiling to match manual rules for fingerprint size
    fp_size_y = size_y #math.ceil(size_y) # ceiling to match manual rules for fingerprint size
    #print("P1 = ", P1, "  P2 = ", P2, "  P3 = ", P3, "  P4 = ", P4, "size_x = ", size_x , "size_y = ", size_y)
    # geo is array with geo coordinates of venue angles
    geo = np.array([P1, P2, P3, P4])
    #print("geo", geo)
    #local = np.array([ [ 0, size_y], [size_x, size_y], [size_x, 0], [0, 0] ], float)
    local = np.array([  [0, 0],  [size_x, 0], [size_x, size_y], [ 0, size_y]], float)
    #print("local", local)
    AttParams = TransformParametersCalculation.TransformParametersCalculation(geo, local)

    if(template_data.get('magnetic_cellsize') != None):
        magnetic_cellsize = template_data['magnetic_cellsize']
    else:
        magnetic_cellsize = 0
    if (template_data.get('wifi_cellsize') != None):
        wifi_cellsize = template_data['wifi_cellsize']
    else:
        wifi_cellsize = 0
    if (template_data.get('ble_cellsize') != None):
        ble_cellsize = template_data['ble_cellsize']
    else:
        ble_cellsize = 0

    venue = template_data['venue']
    venue['origin_lattitude'] = AttParams[0]
    venue['origin_longitude'] = AttParams[1]
    venue['origin_azimuth'] = AttParams[4]
    venue['alfa'] = AttParams[2]
    venue['beta'] = AttParams[3]

    venue['size_x'] = fp_size_x
    venue['size_y'] = fp_size_y

    # reading floors file
    floors_json =  floors_json.replace("\r", "")
    f2 = open(floors_json, 'r')
    floors_text = f2.read()
    floors_data = json.loads(floors_text)
    f2.close()

    floors_numbers = []
    floor_zero_enable = False
    for flr in floors_data:
        floor_number = flr['floor']
        floors_numbers.append(floor_number)
        if(floor_number == 0):
            floor_zero_enable = True

    index_min = floors_numbers.index(min(floors_numbers))
    index_max = floors_numbers.index(max(floors_numbers))

    floor_data_min = floors_data[index_min]
    floor_data_max = floors_data[index_max]
    origin_altitude = floor_data_min['altitude']
    venue['origin_altitude'] = origin_altitude

    floor_number_min = floor_data_min['floor']
    floor_number_max = floor_data_max['floor']
    floor_shift = floor_number_min

    if floor_number_min >= 0:
        floors_count = floor_number_max - floor_number_min + 1
    if floor_number_min < 0:
        floors_count = floor_number_max - floor_number_min
        if floor_zero_enable == True:
            floors_count = floors_count + 1
    venue['floors_count'] = floors_count
    venue['floor_shift'] = floor_shift
    venue['floor_zero_enable'] = floor_zero_enable
    
    floors_numbers.sort()
    venue['floors'] = floors_numbers

    if(venue_spec_data.get('magnetic_cellsize') != None):
        magnetic_cellsize = venue_spec_data['magnetic_cellsize']
    if (venue_spec_data.get('wifi_cellsize') != None):
        wifi_cellsize = venue_spec_data['wifi_cellsize']
    if (venue_spec_data.get('ble_cellsize') != None):
        ble_cellsize = venue_spec_data['ble_cellsize']

    if(magnetic_cellsize > 0):
        magnetic_grid = dict( celltype = 0, cellsize = magnetic_cellsize, min = [0,0,0], max = [ fp_size_x, fp_size_y, floors_count-1 ])
        template_data['magnetic_grid'] = magnetic_grid
    if(wifi_cellsize > 0):
        wifi_grid = dict( celltype = 0, cellsize = wifi_cellsize, min = [0,0,0], max = [fp_size_x, fp_size_y, floors_count-1 ])
        template_data['wifi_grid'] = wifi_grid
    if(ble_cellsize > 0):
        ble_grid = dict( celltype = 0, cellsize = ble_cellsize, min = [0,0,0], max = [fp_size_x, fp_size_y, floors_count-1 ])
        template_data['ble_grid'] = ble_grid

    output_data = merge.merge(template_data, venue_spec_data)

    if (output_data.get('name') != None):
        name = output_data['name']
    else:
        name = 'default'

    json_t_data = json.dumps(output_data, sort_keys=False, indent=4, separators=(',', ': '))
    fw = open(name + ".json", 'wt')
    fw.write(json_t_data)
    fw.close()
