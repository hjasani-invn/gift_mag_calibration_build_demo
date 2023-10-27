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
import thread_for_draw

# constants
SCREEN_HEIGHT_MINUS = 100

BG_COLOR = 'white'

M_PI = (3.1415926535897932384626433832795)

class DrawClass:
    """
    Class for drawing using tkinter
    """
    def __init__(self, venue_map, geopositions):
        #draw the venue_map
        # using tkinter
        self.geopositions = geopositions
        N = len(geopositions)
        self.tmax = int(geopositions[N-1][0][0])
        self.len_geopositions = N
        self.times = []
        for geo in geopositions:
            t = geo[0][0]
            self.times.append(float(t))

        # create a window, the canvas and controls
        self.root = tk.Tk()
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        print("screen sizes = ", screen_width, "    ", screen_height)
        self.root.geometry('{}x{}'.format(screen_width, screen_height))
        self.root.title("Particles")

        ##############################
        self.controls = tk.Frame(self.root)

        label_time1 = tk.Label(self.controls, text="Time  ")
        label_time1.pack(side=tk.LEFT)
        self.label_time2 = tk.Label(self.controls, text="0        ")
        self.label_time2.pack(side=tk.LEFT)

        label_N1 = tk.Label(self.controls, text="Number of particles  ")
        label_N1.pack(side=tk.LEFT)
        self.label_N2 = tk.Label(self.controls, text="0        ")
        self.label_N2.pack(side=tk.LEFT)
        label_empty1 = tk.Label(self.controls, text="     ")
        label_empty1.pack(side=tk.LEFT)
        self.scale_progress = Scale(self.controls, from_=0, to=self.tmax, tickinterval=int(self.tmax/10),  
                                    orient=HORIZONTAL, length=350, command=self.get_scale_progress)
        self.scale_progress.pack(side=tk.LEFT)
        label_empty2 = tk.Label(self.controls, text="     ")
        label_empty2.pack(side=tk.LEFT)

        label_rate1 = tk.Label(self.controls, text="  Rate  ")
        label_rate1.pack(side=tk.LEFT)
        self.entry_rate = Entry(self.controls, width=5)
        self.entry_rate.insert(0, "1")
        self.entry_rate.pack(side=tk.LEFT)

        button_start_restart = tk.Button(self.controls, text="  Restart  ", command=self.restart)
        button_start_restart.pack(side=tk.LEFT)
        self.button_play_pause = tk.Button(self.controls, text="  Play    ", command=self.play_pause)
        self.button_play_pause.config(command=self.play_pause)
        self.button_play_pause.pack(side=tk.LEFT)
        
        self.forward_back = tk.Frame(self.controls)
        self.forward_back_var = IntVar()
        self.r_forward = Radiobutton(self.forward_back, text='Forward', 
                                variable=self.forward_back_var, padx = 20, value=0, 
                                command=self.get_direction_forward)
        self.r_back = Radiobutton(self.forward_back, text='Back      ',
                             variable=self.forward_back_var, padx = 20, value=1, 
                             command=self.get_direction_back)
        self.forward_back_var.set(0)
        self.r_forward.pack( anchor = tk.W )
        self.r_back.pack( anchor = tk.W )
        self.forward_back.pack(side=tk.LEFT)

        #button_back = tk.Button(self.controls, text="Back")
        #button_back.pack(side=tk.LEFT)
        #button_forward = tk.Button(self.controls, text="Forward")
        #button_forward.pack(side=tk.LEFT)
        label_steps = tk.Label(self.controls, text="  Steps  ")
        label_steps.pack(side=tk.LEFT)
        self.entry_steps = Entry(self.controls, width=5)
        self.entry_steps.insert(0,"1")
        self.entry_steps.pack(side=tk.LEFT)
        self.button_prev = tk.Button(self.controls, text="  Previus  ", command=self.previus)
        self.button_prev.pack(side=tk.LEFT)
        self.button_next = tk.Button(self.controls, text="  Next  ", command=self.next)
        self.button_next.pack(side=tk.LEFT)

        self.controls.pack()

        self.play_flag = False
        self.steps = 1
        self.onestep = False

        ##############################

        self.canvas_width = screen_width - 2
        self.canvas_height = screen_height - SCREEN_HEIGHT_MINUS
        self.canvas = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height, bg=BG_COLOR)
        self.canvas.pack()

        mapImage = Image.open(venue_map)
        mapImage.save("temp.bmp", "BMP")
        self.mapImage = Image.open("temp.bmp")
        w, h = mapImage.size
        k = w / h
        print("w = ", w, "   h = ",h, "   k = ", k)
        self.image_scale = 1
        if( (self.canvas_width > self.canvas_height) and (w > h) ):
            self.w1 = self.canvas_width
            self.h1 = self.w1 / k
            print("w1 = ", self.w1, "   h1 = ", self.h1)
            if(self.h1 > self.canvas_height):
                self.h1 = self.canvas_height
                self.w1 = self.h1 * k
        self.mapImage_resize = self.mapImage.resize((int(self.w1*self.image_scale),
                                                     int(self.h1*self.image_scale)),
                                                     Image.ANTIALIAS) # new size of image

        self.mapPhotoImage1 = ImageTk.PhotoImage(self.mapImage_resize)

        self.imageindex1 = self.canvas.create_image(self.mapImage_resize.size[0] / 2, 
                                                    self.mapImage_resize.size[1] / 2, 
                                                    image=self.mapPhotoImage1) # shift of image center in canvas
        self.imageindex2 = 0

        self.ovals_array = []
        self.imageindex = 0
        self.locked = False

        #self.canvas.bind('<Button-1>', self.get_image_scale)
        #self.canvas.bind('<Button-3>', self.continue_)
        self.canvas.bind('<MouseWheel>', self.get_image_scale)
        # Hack to make zoom work on Windows
        self.root.bind_all("<MouseWheel>",self.get_image_scale)

    def start(self):                                      
        name = "Thread #%s" % (1)
        self.my_thread = thread_for_draw.MyThread(name, self.draw, self.geopositions)
        self.my_thread.daemon = True
        self.locked = True
        self.my_thread.start()

    def restart(self):  
        self.my_thread.stop()
        self.start()
        self.locked = False
        self.my_thread.reset()
        self.play_flag = False
        self.forward_back_var.set(0)
        self.play_pause()

    def play_pause(self):
        if self.play_flag == True:
            self.onestep = True
            self.play_flag = False
            self.button_play_pause['text'] = "  Play    "
            self.r_forward['state'] = 'active'
            self.r_back['state'] = 'active'
            self.button_prev['state'] = 'active'
            self.button_next['state'] = 'active'
            self.pause()
        else:
            self.onestep = False
            self.play_flag = True
            self.button_play_pause['text'] = "  Pause "
            self.r_forward['state'] = 'disabled'
            self.r_back['state'] = 'disabled'
            self.button_prev['state'] = 'disabled'
            self.button_next['state'] = 'disabled'
            self.play()

    def pause(self):
        self.locked = True

    def play(self):
        self.locked = False

    def previus(self):
        if self.play_flag == False:
            #self.steps = int(self.entry_steps.get())
            #n = self.my_thread.get_current()
            #n -= self.steps
            #if n < 0:
            #    n = 0
            #self.my_thread.set_current(n)
            self.locked = False

    def next(self):
        if self.play_flag == False:
            #self.steps = int(self.entry_steps.get())
            #n = self.my_thread.get_current()
            #n += self.steps
            #if n >= self.len_geopositions:
            #    n = self.len_geopositions - 1
            #self.my_thread.set_current(n)
            self.locked = False

    def get_nearest_value(self, x_value, x_list):
        left = 0
        right = len(x_list)
        while (right - left > 1):
            i = left + (right - left) // 2
            if x_value < x_list[i]:
                right = i
            else:
                left = i
        a = min([(abs(x_value - x_list[j]), x_list[j], j) for j in (i - 1, i, i + 1)])
        return a[1:3]

    def get_scale_progress(self, n):
        if self.play_flag == False:
            t = float(n)
            a = self.get_nearest_value(t, self.times)
            self.my_thread.set_current(int(a[1]))
            ts = ("%.2f" % a[0])
            self.label_time2['text'] = ts

    def get_direction_forward(self):
        if self.play_flag == False:
            self.my_thread.set_direction(True)

    def get_direction_back(self):
        if self.play_flag == False:
            self.my_thread.set_direction(False)

    def get_image_scale(self, event):
        #print(event.delta)
        if event.delta > 0:
            self.image_scale *= 1.1
        else:
            self.image_scale /= 1.1
        if self.image_scale > 10:
            self.image_scale = 10
        if self.image_scale < 1:
            self.image_scale = 1
        print(self.image_scale)

    def getImageSize(self):
        return self.mapImage_resize.size

    def draw(self, one_time_geopositions):

        t = one_time_geopositions[0][0]
        N = len(one_time_geopositions)
        ts = ("%.2f" % t)
        self.label_time2['text'] = ts
        Ns = ("%d" % N)
        self.label_N2['text'] = Ns
        self.scale_progress.set(int(t))

        self.ovals_array = []
        pixel_array = []
        count = 0
        for geoparticle in one_time_geopositions:
            pixel_x, pixel_y = Geo2LocalCoordinateConverter.Geo2Local(geoparticle[1], geoparticle[2])
            floor = round(float(geoparticle[3]))
            pixel_array.append([pixel_x, pixel_y, geoparticle[4], geoparticle[5], floor])

        rates = self.entry_rate.get()
        rate = float(rates)
        if rate > 10:
            rate = 10
        if rate < 0.1:
            rate = 0.1
        time.sleep(0.05 / rate)

        self.mapImage_resize = self.mapImage.resize((int(self.w1*self.image_scale),
                                                     int(self.h1*self.image_scale)),
                                                     Image.ANTIALIAS) # new size of image
        img = self.mapImage_resize.copy()
        pixels = img.load()  # Create the pixel map
        for pixel in pixel_array:
            if(int(pixel[0]) >= 0 and int(pixel[1]) >= 0 and
                    int(pixel[0]) < self.mapImage_resize.size[0] and
                    int(pixel[1]) < self.mapImage_resize.size[1]):
                if pixel[2] > 0:
                    if pixel[3] == 0:
                        if pixel[4] == 17: # 5th floor
                            pixels[int(pixel[0]), int(pixel[1])] = (0, 0, 255)  # blue
                        elif pixel[4] == 3: # 3th floor
                            pixels[int(pixel[0]), int(pixel[1])] = (255, 0, 0)  # red
                        elif pixel[4] == 2: # 2th floor
                            pixels[int(pixel[0]), int(pixel[1])] = (255, 255, 0)  # yellow
                        else:
                            pixels[int(pixel[0]), int(pixel[1])] = (0, 255, 0)  # green
                    else: # type == 1 -- lkh particle
                        pixels[int(pixel[0]), int(pixel[1])] = (255, 0, 0) #red
                        pixels[int(pixel[0])+1, int(pixel[1])+1] = (255, 0, 0) #red
                        pixels[int(pixel[0])-1, int(pixel[1])-1] = (255, 0, 0) #red
                        pixels[int(pixel[0])+1, int(pixel[1])-1] = (255, 0, 0) #red
                        pixels[int(pixel[0])-1, int(pixel[1])+1] = (255, 0, 0) #red
                        pixels[int(pixel[0]), int(pixel[1])+1] = (255, 0, 0) #red
                        pixels[int(pixel[0]), int(pixel[1])-1] = (255, 0, 0) #red
                        pixels[int(pixel[0])+1, int(pixel[1])] = (255, 0, 0) #red
                        pixels[int(pixel[0])-1, int(pixel[1])] = (255, 0, 0) #red

        if(self.imageindex2 != 0):
            self.mapPhotoImage1 = ImageTk.PhotoImage(img)
        else:    
            self.mapPhotoImage2= ImageTk.PhotoImage(img)

        #if self.image_scale > 1:
        x_s = -(self.mapImage_resize.size[0] - self.canvas_width) / 2
        y_s = -(self.mapImage_resize.size[1] - self.canvas_height) / 2


        if(self.imageindex2 != 0):
            # shift of image center in canvas
            self.imageindex1 = self.canvas.create_image(self.mapImage_resize.size[0] / 2 + x_s,
                                                        self.mapImage_resize.size[1] / 2 + y_s,
                                                        image=self.mapPhotoImage1)
            self.canvas.delete(self.imageindex2)
            self.imageindex2 = 0
        else:
            # shift of image center in canvas
            self.imageindex2 = self.canvas.create_image(self.mapImage_resize.size[0] / 2 + x_s,
                                                        self.mapImage_resize.size[1] / 2 + y_s,
                                                        image=self.mapPhotoImage2)
            self.canvas.delete(self.imageindex1)
            self.imageindex1 = 0

        while(self.locked == True):
            time.sleep(0.2)
        if self.onestep == True:
            self.locked = True
