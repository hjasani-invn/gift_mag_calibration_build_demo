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
        self.current = 0
        self.direction_forward = True
    
    def reset(self):
        self.current = 0
        self.direction_forward = True

    def start(self):
        """Start the thread."""
        self.stop_ = False
        Thread.start(self)

    def stop(self):
        """Stop the thread."""
        self.stop_ = True

    def set_current(self, n):
        self.current = n

    def get_current(self):
        return self.current

    def set_direction(self, direction):
        self.direction_forward = direction

    def run(self):
        """Run the thread."""
        N = len(self.geopositions)
        if self.direction_forward:
            flag_continue = (self.current < N)
        else:
            flag_continue = (self.current >= 0)
        while (not self.stop_) and flag_continue:
            #print("self.current: ", self.current)
            self.callback(self.geopositions[self.current])
            if self.direction_forward:
                self.current += 1
                flag_continue = (self.current < N)
            else:
                self.current -= 1
                flag_continue = (self.current >= 0)
        print("Finish the thread")