import sys
import pygeoj
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

import cell_availability_by_routes

def main():

    #print ("Number of arguments:", len(sys.argv), "arguments.")
    #print ("Argument List:  ", str(sys.argv))
    if len(sys.argv) == 4 :
        #print (sys.argv[1])
        #print (sys.argv[2])
        #print (sys.argv[3])
        print ("")
        cell_availability_by_routes.cell_availability_by_routes(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print("command line must include: json file with description venue (for example venue.json), geojson file witn irl data, output file")

if __name__ == "__main__":
    main()
