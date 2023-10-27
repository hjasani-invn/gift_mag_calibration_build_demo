import numpy as np
import matplotlib.pyplot as plt
import math
import convert_wifi4_to_wifi3
import json
import sys
import statistics


def XYF_to_cellid(x, y, floor_num, max_x, max_y, cellsize):
    i_x = math.floor(x / cellsize)
    i_y = math.floor(y / cellsize)

    max_i_x = math.floor(max_x / cellsize)
    max_i_y = math.floor(max_y / cellsize)

    cellid = i_x + math.floor(max_i_x * i_y) + math.floor(floor_num * max_i_x * max_i_y)

    return cellid


def cellid_to_XYF(cellid, max_x, max_y, cellsize):
    max_i_x = math.floor(max_x / cellsize)
    max_i_y = math.floor(max_y / cellsize)

    floor_num = math.floor(cellid/(max_i_x * max_i_y))
    tmp = cellid % (max_i_x * max_i_y)
    y = cellsize * math.floor((tmp / max_i_x)) + cellsize/2
    x = cellsize * (tmp % max_i_x) + cellsize/2

    return x, y, floor_num


def select_by_cellid(database, cell_id, floor):
    # format of rows is: [file_id, cellid, x, y, mac, w1, rssi1, w2, rssi2, floor_num]
    result = []
    for row in database:

        tst = row[9]
        if row[1] == cell_id and row[9] == floor:
            result.append(row)

    return result


def select_by_mac(database, mac, floor):
    # format of rows is: [file_id, cellid, x, y, mac, w1, rssi1, w2, rssi2, floor_num]
    result = []
    for row in database:
        if row[4] == mac and row[9] == floor:
            result.append(row)

    return result


def parse_wifi3(input_file, file_id, database, cellsize, max_x, max_y):
    cellid = 0
    x = 0.0
    y = 0.0
    floor_num = 0

    mac_list = []
    cell_list = []


    with open(input_file) as wfp:
        lines = wfp.readlines()
        for line in lines:
            l = line[:-1].split()

            if len(l) > 1:

                if l[0] == 'POINT':
                    x = float(l[1])
                    y = float(l[2])
                    floor_num = int(float(l[3]))
                    cellid = XYF_to_cellid(x, y, floor_num, max_x, max_y, cellsize)
                    cell_list.append(cellid)

                elif l[0] == 'AP_MAC':
                    mac = int(l[1])
                    w1 = float(l[2])
                    rssi1 = float(l[3])
                    w2 = float(l[5])
                    rssi2 = float(l[6])

                    mac_list.append(int(mac))

                    # adding row to database

                    row = [file_id, cellid, x, y, mac, w1, rssi1, w2, rssi2, floor_num]
                    database.append(row)


    return list(set(mac_list)), list(set(cell_list))


def plot_cell_difference(database, mac_list, cell_list, floor, max_x, max_y, cellsize):
    max_i_x = int(math.floor(max_x / cellsize))
    max_i_y = int(math.floor(max_y / cellsize))
    full_result = []
    all_differences = []
    out_img = np.zeros([max_i_x, max_i_y])

    print('cell difference calculation')
    cell_list.sort()
    #  iterate through cell_list and calculate cell_difference for each cell
    for cell_id in cell_list:
        result, mean = calculate_cell_difference(cell_id, floor, mac_list, database)
        x, y, f = cellid_to_XYF(cell_id, max_x, max_y, cellsize)
        if f == floor:
            ix = int(math.floor(x / cellsize))
            iy = int(math.floor(y / cellsize))
            for res in result:
                full_result.append([x, y, f, res[0], res[1], res[2], res[3], res[4], res[5], res[6], res[7], res[8], res[9]])
                all_differences.append(res[1])
            out_img[ix][iy] = mean


    output_sum = sum(all_differences)
    full_result.sort(key=lambda a: a[0])
    for res in full_result:
         print(res[0], ' , ', res[1], ' , ', res[2], ' , ', res[3], ' , ,', res[4], ', , ', res[5], ' , ', res[6], ' , ', res[7], ' , ', res[8], ' , ', res[9], ' , ', res[10], ' , ', res[11], ' , ', res[12])
    #     sum += res[4]
    #     count += 1

    count = len(all_differences)
    # print('all:', all_differences)

    if count > 0:
        #print('sum: , ', output_sum, ' ,  count: , ', count)
        #median:
        print(' , ')
        print('median , , , , ,', statistics.median(all_differences))
    #max = 0
    #min = 10000
    #for row in out_img:
    #    for element in row:
    #        if element > max:
    #            max = element
    #        if element < min and element > 0:
    #            min = element

    #print(min, max)

    #outpath = 'image.png'
    #plt.pcolor(out_img.transpose(), cmap='jet', vmin=min, vmax=max, edgecolors='k', linewidths=0.1)
    #plt.title('Wi-Fi difference')
    #plt.colorbar()
    #plt.savefig(outpath, format='png', pad_inches=0, dpi=400)
    #plt.close()

def calculate_cell_difference(cell_id, floor, mac_list, database):
    s = 0
    count = 0

    r = select_by_cellid(database, cell_id, floor)

    result = []
    for mac in mac_list:

        r_mac = []

        for row in r:
            if row[4] == mac:
                r_mac.append(row)

        if len(r_mac) > 1:
		
            d1 = 0
            d2 = 0
            d = 0
            v1 = -100
            w1s = r_mac[0][5]
            m1s = r_mac[0][6]
            w1c = r_mac[1][5]
            m1c = r_mac[1][6]
            w2s = r_mac[0][7]
            m2s = r_mac[0][8]
            w2c =  r_mac[1][7]
            m2c = r_mac[1][8]
		
            #t = (r_mac[0][5] * r_mac[0][6] - r_mac[1][5] * r_mac[1][6] + r_mac[0][7] * r_mac[0][8] - r_mac[1][7] * r_mac[1][8])**2
            #t1 = (r_mac[0][5] * r_mac[0][6] - r_mac[1][5] * r_mac[1][6]) + (r_mac[0][7] * r_mac[0][8] - r_mac[1][7] * r_mac[1][8])

            #if w1s>0.1 and m1s>-100 and w1c>0.1 and m1c>-100 and (w2s<0.1 or m2s<=-100):
            #    d=m1s-m1c
            #    v1=m1s
            #else:
            #    if w2s>0.1 and m2s>-100 and w2c>0.1 and m2c>-100 and (w1s<0.1 or m1s<=-100):
            #        d=m2s-m2c
            #        v1=m2s
            #    else:
            #        if w1s>0.1 and m1s>-100 and w2c>0.1 and m2c>-100 and (w1c<0.1 or m1c<=-100):
            #            d=m1s-m2c
            #            v1=m1s
            #        else:
            #            if w2s>0.1 and m2s>-100 and w1c>0.1 and m1c>-100 and (w2c<0.1 or m2c<=-100):
            #                d=m2s-m1c
            #                v1=m2s
            #            else:
            #                if w1s>0.1 and m1s>-100 and w1c>0.1 and m1c>-100 and w2s>0.1 and m2s>-100 and w2c>0.1 and m2c>-100:
            #                    v1=w1s*m1s+w2s*m2s
            #                    v2=w1c*m1c+w2c*m2c
            #                    d=v1-v2
            #                else:
            #                    #d=0
            #                    #v1=-100
            #                    continue

            # 1. Select maximal nonzero mode for both phones:  max_m_p1 and max_m_p2                    
            # 2. Check if (-45 < max_m_p1 < -85) and  weight of this mode > 0.2; skip this cell if no   
            # 3. Check if (-45 < max_m_p2 < -85) and  weight of this mode > 0.2; skip this cell if no   
            # 4. Calc difference max_m_p1 and max_m_p2                                                  

            m1 = -1000
            m2 = -1000
            if (w1s > 0.2) and (m1s < -45) and (m1s > -85):
                m1 = m1s
            if (w2s > 0.2) and (m2s < -45) and (m2s > -85):
                m2 = m2s
            d1 = max(m1, m2)
            if d1 < -85:
                continue

            m1 = -1000
            m2 = -1000
            if (w1c > 0.2) and (m1c < -45) and (m1c > -85):
                m1 = m1c
            if (w2c > 0.2) and (m2c < -45) and (m2c > -85):
                m2 = m2c
            d2 = max(m1, m2)
            if d2 < -85:
                continue

            d = d1 - d2

            t1 = d
            t = t1**2
            s += t
            result.append([mac, t1, r_mac[0][5], r_mac[0][6], r_mac[1][5], r_mac[1][6], r_mac[0][7], r_mac[0][8], r_mac[1][7], r_mac[1][8]])
            count += 1

    result.sort(key=lambda a: a[0])
    if count > 0:
        mean = (s/count)**0.5
    else:
        mean = 0

    return result, mean


def calculate_AP_difference(database, mac_list, cell_list, floor, max_x, max_y, cellsize):
    #print('mac difference calculation')

    mac_results_list = []

    #  iterate through cell_list and calculate cell_difference for each cell
    for mac in mac_list:
        res = calculate_mac_difference(mac, cell_list, database, floor)
        out = [mac, res]
        mac_results_list.append(out)

    #with open('AP_metric.txt', 'w') as out_file:
    #    for mac_res in mac_results_list:
    #        output_line = str(mac_res[0]) + ' ' + str(mac_res[1]) + '\n'
    #        out_file.write(output_line)


def calculate_mac_difference(mac, cell_list, database, floor):
    r = select_by_mac(database, mac, floor)
    r_mac = []

    result = 0

    s = 0
    count = 0

    for cell_id in cell_list:

        for row in r:
            if row[1] == cell_id:
                r_mac.append(row)

        if len(r_mac) > 1: # we have AP with this mac in this cell_id in both files
            t = (r_mac[0][5] * r_mac[0][6] - r_mac[1][5] * r_mac[1][6] + r_mac[0][7] * r_mac[0][8] - r_mac[1][7] * r_mac[1][8]) ** 2
            s += t
            count += 1

        r_mac = []

    if count > 0:
        result = (s / count) ** 0.5

    # result - standard deviation for current mac, across the whole WFP (all cells)

    return result


def compare_wifi_fingerprints(settings_json, wfp_1, wfp_2, floor_number):
    file1 = convert_wifi4_to_wifi3.convert(wfp_1)
    file2 = convert_wifi4_to_wifi3.convert(wfp_2)

    settings = json.load(open(settings_json))
    cellsize = settings['wifi_cellsize']
    max_x = settings['venue']['size_x']
    max_y = settings['venue']['size_y']

    database = []

    file_id = 1
    mac_list_1, cell_list_1 = parse_wifi3(file1, file_id, database, cellsize, max_x, max_y)

    file_id = 2
    mac_list_2, cell_list_2 = parse_wifi3(file2, file_id, database, cellsize, max_x, max_y)

    mac_list = list(set(mac_list_1 + mac_list_2))
    cell_list = list(set(cell_list_1 + cell_list_2))

    plot_cell_difference(database, mac_list, cell_list, floor_number, max_x, max_y, cellsize)
    calculate_AP_difference(database, mac_list, cell_list, floor_number, max_x, max_y, cellsize)

    return


if __name__ == "__main__":  # for test only
    #settings_json = sys.argv[1]
    #wfp_1 = sys.argv[2]
    #wfp_2 = sys.argv[3]
    #floor_number = int(sys.argv[4])

    settings_json = 'c:/Users/vpentyukhov/Development/InvenSenseInc/Gift/Tools/ASCA/venues_/ICA_crowdsource/venue.json'
    wfp_1 = '//cayyc-proj01/compute02/vpentyukhov/fingerprints/ICA_crowdsource/ICA_CrowdSource.wifi4'
    wfp_2 = 'c:/Users/vpentyukhov/Development/InvenSenseInc/Gift/Applications/fp_builder.console/venues/ICA_crowdsource/fp/ICA_CrowdSource.wifi4'
    floor_number = 5

    compare_wifi_fingerprints(settings_json, wfp_1, wfp_2, floor_number)
