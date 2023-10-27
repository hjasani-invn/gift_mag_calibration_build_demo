import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
import os
import sys
import re
import numpy as np
import pygeoj
import json

def process_logs(input_log1, input_log2):

    #try:
    with open(input_log1) as f:
        list = [line.split() for line in f]
    #print(list)

    data1 =[]
    for l in list:
        data1.append(float(l[5]))
    #print(data1)

    with open(input_log2) as f:
        list = [line.split() for line in f]
    #print(list)

    data2 =[]
    for l in list:
        data2.append(float(l[5]))
   #print(data2)

    len1 = len(data1)
    len2 = len(data2)
    len_min = min(len1 , len2)

    diff_max = -100000
    diff_min =  100000
    for idx in range(0, len_min):
        diff = data1[idx] - data2[idx]
        if diff > diff_max:
            diff_max = diff
        if diff < diff_min:
            diff_min = diff

    #print(diff_min, "     ", diff_max)

    return diff_min, diff_max
