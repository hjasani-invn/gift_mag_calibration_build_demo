## @package lms_for_level
#  This module calculate lms result
#
import math
import numpy as np


# expected level = tx_power - 20 * math.log(distance, 10) + tx_bias

## The function calculates derevatives for attenuation equation
#
#from sympy import *
#symbolic deriative evaluation
#hi, xb, yb, xi, yi = symbols('hi xb yb xi yi')
#hi = 20 * log(sqrt((xb - xi)**2 + (yb - yi)**2), 10)
#diff(hi, xb) = 20*(xb - xi)/(((xb - xi)**2 + (yb - yi)**2)*log(10))
#diff(hi, yb) = 20*(yb - yi)/(((xb - xi)**2 + (yb - yi)**2)*log(10))

def get_attenuation_derivative(xb, xi, yb, yi):
    # hi = 20 * log(sqrt((xb - xi)**2 + (yb - yi)**2), 10)
    dhi_xb = 20*(xb - xi)/(((xb - xi)**2 + (yb - yi)**2) * math.log(10))
    dhi_yb = 20*(yb - yi)/(((xb - xi)**2 + (yb - yi)**2) * math.log(10))
    return dhi_xb, dhi_yb

## calculate lms solution using power level
#
# @param[in]  position_grid - grid of positions
# @param[in] measured_level - ble beacon level
# @param[in] tx_power_nominal - nominal tx power
# @param[in] x0 - init coordinate x
# @param[in] y0 - init coordinate y
# @param[in] tx_power_error0 - init value of tx power error
# @param[in] max_iter_number - max iteration number of lms process
# @param[in] min_delta - men delta for neighbor iterations
#
def calculate_using_level(position_grid, measured_level, tx_power_nominal, x0, y0, tx_power_error0, max_iter_number, min_delta):

    len_grid = len(position_grid)
    #print(len_grid)

    koef = 0.5

    measured_distance = np.zeros([len_grid, 1])
    expected_distance = np.zeros([len_grid, 1])
    expected_attenuation = np.zeros([len_grid, 1])
    expected_level = np.zeros([len_grid, 1])
    delta = np.zeros([len_grid, 1])
    delta_level = np.zeros([len_grid, 1])
    derivative = np.zeros([len_grid, 3])
    sigma_measured_level = np.zeros([len_grid, 1])
    weight = np.zeros([len_grid, 1])
    delta_position = np.zeros([3, 1])

    # LMS
    while koef > 0.01:
        xb = x0
        yb = y0
        tx_power_err = tx_power_error0
        flag_det_0 = False

        for iter_number in range(max_iter_number):
            #print(iter_number, '    ', xb, '    ', yb)
            sum_weight = 0
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
                expected_level[i] = tx_power_nominal - expected_attenuation[i] + tx_power_err
                # print(expected_distance[i])
                xd, yd = get_attenuation_derivative(xb, xi, yb, yi)
                derivative[i, 0] = xd
                derivative[i, 1] = yd
                derivative[i, 2] = 1

                delta[i] = measured_level[i] - expected_level[i]

                s1 = (-60 - min(-60, int(measured_level[i])));
                sigma_measured_level[i] = 1 + 0.2 * s1
                #weight[i] = 1 / (sigma_measured_level[i] * sigma_measured_level[i])
                d1 = -s1
                exp_ = math.exp(d1 / (2 * sigma_measured_level[i] * sigma_measured_level[i]))
                weight[i] = (1 / sigma_measured_level[i]) * exp_
                sum_weight += weight[i]

            for i in range(len_grid):
                weight[i] /= sum_weight
            #    print(weight[i][0])
            #print('============')

            #delta_position, residuals, rank, singular_values = np.linalg.lstsq(derivative, delta, rcond=None)

            #d1 = np.matmul(derivative.transpose(), derivative)
            #d2 = np.linalg.inv(d1)
            #d3 = np.dot(derivative.transpose(), delta)
            #delta_position = np.dot(d2, d3)

            weighted_matrix = np.zeros([len_grid, len_grid])
            for n in range(len_grid):
                #weighted_matrix[n, n] = 1
                weighted_matrix[n, n] = weight[n]
            d4 = np.dot(derivative.transpose(), weighted_matrix)
            d1 = np.dot(d4, derivative)
            det1 = np.linalg.det(d1)
            #print(det1)
            if(det1 < 1e-9):
                flag_det_0 = True
                break
            d2 = np.linalg.inv(d1)
            d5 = np.dot(derivative.transpose(), weighted_matrix)
            d3 = np.dot(d5, delta)
            delta_position = np.dot(d2, d3)

            #print(iter_number, '  ,  ', xb, '    ', yb, end='    ')
            xb = xb - delta_position[0][0] * koef
            yb = yb - delta_position[1][0] * koef
            tx_power_err = tx_power_err + delta_position[2][0]/3

            #print(delta_position[0][0], '    ', delta_position[1][0], '    ',  xb, '    ', yb)
            if (abs(delta_position[0]) < min_delta and abs(delta_position[1]) < min_delta):
                #tx_power_b = 5
                break

        if iter_number == max_iter_number - 1 or flag_det_0 == True:
            koef = koef / 2
            continue
        if iter_number < max_iter_number - 1:
            break

    # calculate LMS estimate accuracy
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

        expected_distance[i] = math.sqrt((xb - xi) ** 2 + (yb - yi) ** 2)
        expected_attenuation[i] = 20 * math.log(expected_distance[i], 10)
        expected_level[i] = tx_power_nominal - expected_attenuation[i] + tx_power_err
        delta_level[i] = measured_level[i] - expected_level[i]
        measured_distance[i] = 10 ** ((tx_power_nominal - measured_level[i]) / 20)
        delta[i] = measured_distance[i] - expected_distance[i]

    rms1 = np.dot(delta.transpose(), delta)
    #rms = math.sqrt(rms1[0][0] / len_grid)
    rms = math.sqrt(rms1[0][0]) / len_grid
    #print(rms)
    #iter_number = 0
    delta_power_err = 0
    flag_det_0 = False
    return xb, yb, delta_position[0][0], delta_position[1][0], tx_power_err, delta_power_err, rms, iter_number + 1, not flag_det_0

