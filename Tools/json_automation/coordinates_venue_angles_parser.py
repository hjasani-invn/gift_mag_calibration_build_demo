import csv
import io
import six

def coordinates_venue_angles_parsing(coordinates_angles_file, P1_name, P2_name, P3_name, P4_name):

    # coordinates of venue angles
    P1 = [0,0] # x = 0, y = y_max
    P2 = [0,0] # x = x_max, y = 0
    P3 = [0,0] # x = x_max, y = y_max
    P4 = [0,0] # x = 0, y = y_max
    p1 = 0
    p2 = 0
    p3 = 0
    p4 = 0

    P1_no = True
    P2_no = True
    P3_no = True
    P4_no = True

    #reading venue coordinates file
    flag_parse_finish = False
    f1 = open(coordinates_angles_file)
    for line in f1:
        if(flag_parse_finish == True):
            break
        reader = csv.reader(six.StringIO(line), delimiter=':')
        for n in reader:
            #print(n[0])
            #if(n[0].find(Stop_name) != -1):
            #    flag_parse_finish = True
            #    break
            # P1 parsing
            if(n[0].find(P1_name) != -1 and P1_no):
                P1_no = False
                r = csv.reader(six.StringIO(n[1]), delimiter=' ')
                for m in r:
                    #print(m)
                    for i in m:
                        if(len(i) > 1):
                            P1[p1] = float(i)
                            p1 += 1
            # P2 parsing
            if(n[0].find(P2_name) != -1 and P2_no):
                P2_no = False
                r = csv.reader(six.StringIO(n[1]), delimiter=' ')
                for m in r:
                    #print(m)
                    for i in m:
                        if(len(i) > 1):
                            P2[p2] = float(i)
                            p2 += 1
            # P3 parsing
            if(n[0].find(P3_name) != -1 and P3_no):
                P3_no = False
                r = csv.reader(six.StringIO(n[1]), delimiter=' ')
                for m in r:
                    #print(m)
                    for i in m:
                        if(len(i) > 1):
                            P3[p3] = float(i)
                            p3 += 1
            # P4 parsing
            if(n[0].find(P4_name) != -1 and P4_no):
                P4_no = False
                r = csv.reader(six.StringIO(n[1]), delimiter=' ')
                for m in r:
                    #print(m)
                    for i in m:
                        if(len(i) > 1):
                            P4[p4] = float(i)
                            p4 += 1
            # P1 & P2 parsing
            if (n[0].find(P1_name) != -1 and n[0].find(P2_name) != -1 and n[0].find("&") != -1):
                r = csv.reader(six.StringIO(n[1]), delimiter=' ')
                for m in r:
                    # print(m)
                    for i in m:
                        if (len(i) > 1):
                            size_x = float(i)
            # P1 & P4 parsing
            if (n[0].find(P1_name) != -1 and n[0].find(P4_name) != -1 and n[0].find("&") != -1):
                r = csv.reader(six.StringIO(n[1]), delimiter=' ')
                for m in r:
                    # print(m)
                    for i in m:
                        if (len(i) > 1):
                            size_y = float(i)
    return P1, P2, P3, P4, size_x, size_y
