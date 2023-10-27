#import matplotlib as mpl
#mpl.use('Agg')
#import matplotlib.pyplot as plt
import os
import sys
import numpy as np
import pygeoj
import json
import tkinter as tk
from tkinter import *
from PIL import ImageTk, Image # Pillow package name
from PIL import Image, ImageDraw, ImageFont
#import random
import time
import threading
from threading import Thread
from threading import Lock
from threading import RLock
import csv
import io
import json
import Geo2LocalCoordinateConverter
import coordinates_venue_angles_parser
import reading_particles_file
import TransformParametersCalculation
import patricles_player
#import thread_for_draw

SCREEN_HEIGHT_MINUS = 100

BG_COLOR = 'white'

M_PI = (3.1415926535897932384626433832795)

class MyThread(Thread):

    def __init__(self, name, callback, geopositions):
        """Thread initialization"""
        Thread.__init__(self)
        self.name = name
        self.callback = callback
        self.geopositions = geopositions

    def start(self):
        """Start the thread."""
        self.stop = False
        Thread.start(self)

    def stop(self):
        """Stop the thread."""
        self.stop = True

    def run(self):
        """Run the thread."""
  
        for one_time_geopositions in self.geopositions:
            self.callback(one_time_geopositions)


if  __name__ == "__main__":
    settings_json = sys.argv[1]
    coordinates_angles_file = sys.argv[2]
    venue_map = sys.argv[3]
    visualization_type = sys.argv[4] # 1 - mixed mode, 2 - magnetic only
    particles_file = sys.argv[5]

    # reading venue parameters
    f = open(settings_json, 'r')
    text = f.read()
    data = json.loads(text)
    venue = data['venue']
    #print (venue)

    # venue sizes
    size_x = venue['size_x']
    size_y = venue['size_y']

    #print(size_x, size_y)

    # geographical binding
    origin_lattitude = venue['origin_lattitude']
    origin_longitude = venue['origin_longitude']
    origin_azimuth = venue['origin_azimuth']
    alfa = venue['alfa']
    beta = venue['beta']

    # set venue parametres
    Geo2LocalCoordinateConverter.SetGeoParam(origin_lattitude, origin_longitude, alfa, beta, origin_azimuth)
    
    geopositions = reading_particles_file.reading_particles_file(particles_file)

    myDrawClass = patricles_player.DrawClass(venue_map, geopositions)

    # convertion geo coordinate to screen pixels

    # local is array with image sizes in pixels    
    image_size = myDrawClass.getImageSize()
    #print("image_size", image_size)
    local = np.array([ [ 0  ,  image_size[1] ], [image_size[0] , image_size[1] ] , [image_size[0] , 0] , [0  ,  0] ], float)
    #local = np.array([ [ 0  ,  image_size[1] ] , [0  ,  0] , [image_size[0] , 0], [image_size[0] , image_size[1] ] ], float)
    #local = np.array([[0,  0], [image_size[0], 0], [image_size[0], image_size[1]], [0,  image_size[1]]], float)
    #print("local",local)

    # coordinates of venue angles parsing
    P1, P2, P3, P4 = coordinates_venue_angles_parser.coordinates_venue_angles_parsing(coordinates_angles_file, "P1", "P2", "P3", "P4", "----")
    #print("P1 = ", P1, "  P2 = ", P2, "  P3 = ", P3, "  P4 = ", P4)
    # geo is array with geo coordinates of venue angles
    geo = np.array([P1, P2, P3, P4])
    #print("geo",geo)

    AttParams = TransformParametersCalculation.TransformParametersCalculation(geo, local)

    #result
    print("lat = ", AttParams[0])
    print("lon = ",AttParams[1])
    print("alfa_lat = ",AttParams[2])
    print("alfa_lon = ",AttParams[3])
    print("heading = ",AttParams[4])

    lat0 = AttParams[0]
    lon0 = AttParams[1]
    alfa_lat = AttParams[2]
    alfa_lon = AttParams[3]
    heading = AttParams[4]

    Geo2LocalCoordinateConverter.SetGeoParam(lat0, lon0, alfa_lat, alfa_lon, heading)

    myDrawClass.start()
    myDrawClass.root.mainloop()
    #for one_time_geopositions in geopositions:
    #    for geoparticle in one_time_geopositions:
    #        pixel_x, pixel_y = Geo2LocalCoordinateConverter.Geo2Local(  geoparticle[0],  geoparticle[1] )
    #        print( geoparticle[0], "    ",  geoparticle[1], "    ",  pixel_x, "    ", pixel_y)
    #    print("=============")
