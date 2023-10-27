import io
import csv
import re
import ble_hash

# reading the blp3 file
def read(blp3_file):
    uuid_mask = "[0-9,a-f,A-F]{8}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{4}-[0-9,a-f,A-F]{12}"
    count = 0
    timestamp = 0
    beacons = []
    geopositions = []
    time_count = 0
    file = open(blp3_file)
    for line in file:
        reader = csv.reader(io.StringIO(line), delimiter=',')
        for beacon in reader:
            if re.match(uuid_mask, beacon[0]) != None:
                hash = ble_hash.get_ble_hash(beacon[1],  beacon[2],  beacon[0])
                beacon.append(hash)
                beacon.append(float(beacon[3]))
                beacon.append(float(beacon[4]))
                beacon.append(int(float(beacon[5])))
                beacon.append(int(beacon[6]))
                #print (beacon)
                beacons.append(beacon)
    #print (beacons)

    return beacons
