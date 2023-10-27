import sys
import pygeoj
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

import Geo2LocalCoordinateConverter
import line2points
import formgrid

def cell_availability_by_routes(venue_json_file, geo_json_file, out_file) :
    # parse venue json file 
    f = open(venue_json_file, 'r')
    text = f.read()
    data = json.loads(text)
    venue = data['venue']
    #print (venue)

    origin_lattitude = venue['origin_lattitude']
    origin_longitude = venue['origin_longitude']
    origin_azimuth = venue['origin_azimuth']
    alfa = venue['alfa']
    beta = venue['beta']

    magnetic_cellsize =  data['magnetic_cellsize']
    mag_max_x = venue['size_x']
    mag_max_y = venue['size_y']

    print ("origin latitude = ", origin_lattitude)
    print ("origin longitude = ", origin_longitude)
    print ("origin azimuth = ", origin_azimuth)
    print ("alfa latitude = ", alfa)
    print ("alfa longitude = ", beta)
    print ("magnetic cellsize = ", magnetic_cellsize)
    print ("max x = ", mag_max_x)
    print ("max y = ", mag_max_y)


    # set venue parametres

    Geo2LocalCoordinateConverter.SetGeoParam(    origin_lattitude,    origin_longitude,    alfa,     beta,    origin_azimuth )

    #plt.pcolor(coverage.transpose(), cmap=the_colormap, vmin=0, vmax=5, edgecolors='k', linewidths=0.3)
    #plt.axis("off")
    #plt.colorbar()
    #plt.subplots_adjust(bottom=0)
    #plt.subplots_adjust(top=1)
    #plt.subplots_adjust(right=1)
    #plt.subplots_adjust(left=0)
    #plt.savefig(outpath, pad_inches=0, dpi=200)
    #plt.close()

    #w = 120
    #h = 115
    d = 70
    plt.figure(figsize=(int(mag_max_x), int(mag_max_y)), dpi=d)


    # parse irl data from geojson file and conver them to local  coordinate 
    map_json_data = pygeoj.load(geo_json_file)
    #print  (testfile.bbox)
    #print  (testfile.crs)
    all_points_list = []
    for feature in map_json_data:
        #print (feature.geometry.type)
        #print (len(feature.geometry.coordinates))
        #print (feature.geometry.coordinates)
        x_prev = 0
        y_prev = 0
        count = 0
        points_list = []


        for coordinate in feature.geometry.coordinates:
            (x, y) = Geo2LocalCoordinateConverter.Geo2Local(coordinate[1], coordinate[0])
        	#print ("geo = ", coordinate[1], "   ", coordinate[0], "     local = ", x, "   ",  y)
            if count > 0 :
                points_list = line2points.line2points(x_prev, y_prev, x, y, magnetic_cellsize / 100)
        		#print ("points_list   ", points_list)
                all_points_list.extend(points_list)
                x1 = [x_prev, x]
                y1 = [y_prev, y]
                plt.plot(x1, y1)
            count += 1
            x_prev = x
            y_prev = y
    #print (len(all_points_list))
    #print (all_points_list)
    #for point in all_points_list:
    #		print (point[0], "     ", point[1])
    
    plt.show()
    #plt.savefig("routes.png", pad_inches=0, dpi=200)
    plt.close
    
    formgrid.formgrid(all_points_list, mag_max_x, mag_max_y, magnetic_cellsize, out_file)

    print('Finish')

