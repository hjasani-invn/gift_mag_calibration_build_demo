import numpy as np
import matplotlib as mpl


def get_colormap():

    A = np.matrix([[0,0,255],[
    4,0,252],[
    8,0,248],[
    12,0,244],[
    16,0,240],[
    20,0,236],[
    24,0,232],[
    28,0,228],[
    32,0,224],[
    36,0,220],[
    40,0,216],[
    44,0,212],[
    48,0,208],[
    52,0,204],[
    56,0,200],[
    60,0,196],[
    64,0,192],[
    68,0,188],[
    72,0,184],[
    76,0,180],[
    80,0,176],[
    84,0,172],[
    88,0,168],[
    92,0,163],[
    96,0,159],[
    100,0,155],[
    104,0,151],[
    108,0,147],[
    112,0,143],[
    116,0,139],[
    120,0,135],[
    124,0,131],[
    128,0,127],[
    132,0,123],[
    136,0,119],[
    140,0,115],[
    144,0,111],[
    148,0,107],[
    152,0,103],[
    156,0,99],[
    160,0,95],[
    164,0,91],[
    169,0,87],[
    173,0,83],[
    177,0,79],[
    181,0,75],[
    185,0,71],[
    189,0,67],[
    193,0,63],[
    197,0,59],[
    201,0,55],[
    205,0,51],[
    209,0,47],[
    213,0,43],[
    217,0,39],[
    221,0,35],[
    225,0,31],[
    229,0,27],[
    233,0,23],[
    237,0,19],[
    241,0,15],[
    245,0,11],[
    249,0,7],[
    253,0,3],[
    255,1,0],[
    255,5,0],[
    255,9,0],[
    255,13,0],[
    255,17,0],[
    255,21,0],[
    255,25,0],[
    255,29,0],[
    255,33,0],[
    255,37,0],[
    255,41,0],[
    255,45,0],[
    255,49,0],[
    255,53,0],[
    255,57,0],[
    255,61,0],[
    255,65,0],[
    255,69,0],[
    255,73,0],[
    255,77,0],[
    255,81,0],[
    255,85,0],[
    255,89,0],[
    255,93,0],[
    255,97,0],[
    255,101,0],[
    255,105,0],[
    255,109,0],[
    255,113,0],[
    255,117,0],[
    255,121,0],[
    255,125,0],[
    255,129,0],[
    255,133,0],[
    255,137,0],[
    255,141,0],[
    255,145,0],[
    255,149,0],[
    255,153,0],[
    255,157,0],[
    255,161,0],[
    255,165,0],[
    255,170,0],[
    255,174,0],[
    255,178,0],[
    255,182,0],[
    255,186,0],[
    255,190,0],[
    255,194,0],[
    255,198,0],[
    255,202,0],[
    255,206,0],[
    255,210,0],[
    255,214,0],[
    255,218,0],[
    255,222,0],[
    255,226,0],[
    255,230,0],[
    255,234,0],[
    255,238,0],[
    255,242,0],[
    255,246,0],[
    255,250,0],[
    255,254,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    255,255,0],[
    253,255,0],[
    249,255,0],[
    245,255,0],[
    241,255,0],[
    237,255,0],[
    233,255,0],[
    229,255,0],[
    225,255,0],[
    221,255,0],[
    217,255,0],[
    213,255,0],[
    209,255,0],[
    205,255,0],[
    201,255,0],[
    197,255,0],[
    193,255,0],[
    189,255,0],[
    185,255,0],[
    181,255,0],[
    177,255,0],[
    173,255,0],[
    169,255,0],[
    164,255,0],[
    160,255,0],[
    156,255,0],[
    152,255,0],[
    148,255,0],[
    144,255,0],[
    140,255,0],[
    136,255,0],[
    132,255,0],[
    128,255,0],[
    124,255,0],[
    120,255,0],[
    116,255,0],[
    112,255,0],[
    108,255,0],[
    104,255,0],[
    100,255,0],[
    96,255,0],[
    92,255,0],[
    88,255,0],[
    84,255,0],[
    80,255,0],[
    76,255,0],[
    72,255,0],[
    68,255,0],[
    64,255,0],[
    60,255,0],[
    56,255,0],[
    52,255,0],[
    48,255,0],[
    44,255,0],[
    40,255,0],[
    36,255,0],[
    32,255,0],[
    28,255,0],[
    24,255,0],[
    20,255,0],[
    16,255,0],[
    12,255,0],[
    8,255,0],[
    4,255,0],[
    0,255,0]])

    return A
