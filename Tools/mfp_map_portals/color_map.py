import numpy as np
import matplotlib as mpl


def get_colormap():

    A = np.array([[140, 212, 252],
                  [100, 100, 100],
                  [255, 0, 0],
                  [255, 255, 0],
                  [0, 255, 0]])

    return A


def get_colormap_another():

    A = np.array([[140, 212, 252],
                  [100, 100, 100],
                  [0, 255, 0],
                  [0, 255, 0],
                  [255, 255, 0],
                  [255, 255, 0],
                  [255, 0, 0],
                  [255, 0, 0]])

    return A

blue_color = -10
grey_color = -3
green_color = 5
yellow_color = 10
red_color = 20
min_scale_value = -10
max_scale_value = 22
