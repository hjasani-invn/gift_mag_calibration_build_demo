## @package pos_average
#  This module provides BLE position estimation
#
import math
import numpy as np

## Estimates position of beacon using position grid and collected Rx-levels of the beacons in grig cells
#
def calculate_using_level(position_grid, measured_level, tx_power_nominal, x0, y0, tx_power_error0, max_iter_number, min_delta):

    len_grid = len(position_grid)

    sigma_measured_level = np.zeros([len_grid, 1])
    weight = np.zeros([len_grid, 1])
    delta_position = np.zeros([3, 1])

    # weight average position - alternative method
    sum_weight = 0
    for i in range(len_grid):
        s1 = (-60 - min(-60, int(measured_level[i])));
        sigma_measured_level[i] = 1 + 0.2 * s1
        weight[i] = 1 / (sigma_measured_level[i] * sigma_measured_level[i])
        #d1 = -s1
        #exp_ = math.exp(d1 / (2 * sigma_measured_level[i] * sigma_measured_level[i]))
        #weight[i] = (1 / sigma_measured_level[i]) * exp_
        sum_weight += weight[i]

    for i in range(len_grid):
        weight[i] /= sum_weight

    xb1 = 0
    yb1 = 0
    for i in range(len_grid):
        xi = position_grid[i][0]
        yi = position_grid[i][1]
        xb1 += xi * weight[i]
        yb1 += yi * weight[i]

    xb = xb1[0]
    yb = yb1[0]

    # calculate LMS estimate accuracy
    xd1 = 0
    yd1 = 0
    for i in range(len_grid):
        xi = position_grid[i][0]
        yi = position_grid[i][1]
        x_2 = (xi - xb) * weight[i]
        xd1 += x_2 * x_2
        y_2 = (yi - yb) * weight[i]
        yd1 += y_2 * y_2

    rms = math.sqrt(xd1 + yd1)

    delta_power_err = 0
    iter_number = 0
    flag_det_0 = False
    # tx_power_err = 0
    # rectification of tx power error
    expected_distance = np.zeros([len_grid, 1])
    expected_attenuation = np.zeros([len_grid, 1])
    expected_level = np.zeros([len_grid, 1])
    sum = 0
    for i in range(len_grid):
        xi = position_grid[i][0]
        yi = position_grid[i][1]
        # to except lms matrix degradation
        if abs(xb - xi) < 0.001:
            xi = xi + 0.1
        if abs(yb - yi) < 0.001:
            yi = yi + 0.1
        while abs(xb - xi) < 0.1:
            xi = xi + (xi - xb)
        while abs(yb - yi) < 0.1:
            yi = yi + (yi - yb)
        # print(xb, '  ', xi, '  ', yb, '  ', yi)
        expected_distance[i] = math.sqrt((xb - xi) ** 2 + (yb - yi) ** 2)
        expected_attenuation[i] = 20 * math.log(expected_distance[i], 10)
        expected_level[i] = tx_power_nominal - expected_attenuation[i]
        delta1 = expected_level[i] - measured_level[i]
        sum += delta1
    tx_power_err = - sum[0] / len_grid

    return xb, yb, delta_position[0][0], delta_position[1][0], tx_power_err, delta_power_err, rms, iter_number + 1, not flag_det_0
