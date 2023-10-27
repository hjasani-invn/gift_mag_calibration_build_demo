__author__ = 'kotik'

import os
import re
import numpy as np
import csv
import sys

class validator(object):
    def __init__(self, name, number):
        self.name = name
        self.number = number

        self.all_total = np.array([0, 0])
        self.all_front_pocket = np.array([0, 0])
        self.all_back_pocket = np.array([0, 0])
        self.all_purse = np.array([0, 0])
        self.all_other = np.array([0, 0])

        self.android_total = np.array([0, 0])
        self.android_front_pocket = np.array([0, 0])
        self.android_back_pocket = np.array([0, 0])
        self.android_purse = np.array([0, 0])
        self.android_other = np.array([0, 0])

        self.ios_total = np.array([0, 0])
        self.ios_front_pocket = np.array([0, 0])
        self.ios_back_pocket = np.array([0, 0])
        self.ios_purse = np.array([0, 0])
        self.ios_other = np.array([0, 0])


def update_validator_results(val, passed, is_android, is_ios, is_front_pocket, is_back_pocket, is_purse, is_other):

    res = np.array([passed, 1])
    val.all_total += res
    val.all_front_pocket += is_front_pocket * res
    val.all_back_pocket += is_back_pocket * res
    val.all_purse += is_purse * res
    val.all_other += is_other * res

    if is_ios == 1:
        val.ios_total += res
        val.ios_front_pocket += is_front_pocket * res
        val.ios_back_pocket += is_back_pocket * res
        val.ios_purse += is_purse * res
        val.ios_other += is_other * res

    if is_android == 1:
        val.android_total += res
        val.android_front_pocket += is_front_pocket * res
        val.android_back_pocket += is_back_pocket * res
        val.android_purse += is_purse * res
        val.android_other += is_other * res

    return val


def main(argv):
    input_file = os.path.abspath(argv[1])
    output_file = os.path.abspath(argv[2])
    csv_in = open(input_file, 'r', newline='')

    reader = csv.reader(csv_in)

    rownum = 0
    for row in reader:
        if rownum == 0:
            header_string = row
            header = header_string[0].split(';')
            # header now is a list of strings
            header = header[2:]
            validator_names = header[::2]

            validator_list = []

            i = 2
            for name in validator_names:
                v = validator(name, i)
                validator_list.append(v)
                # print(v.number)
                i += 2

            # print(validator_list[0].name, validator_list[0].number)
            # print(validator_list[len(validator_list)-1].name, validator_list[len(validator_list)-1].number)
            # last_idx = validator_list[len(validator_list)-1].number

            #TODO: now validator list contains names and numbers of validators
            #TODO: we can iterate through this list and get values for each track
        else:
            data_string = row
            data = data_string[0].split(';')

            is_android = 1
            is_ios = 0
            is_front_pocket = 0
            is_back_pocket = 0
            is_purse = 0
            is_other = 0

            #if re.search(r'-iP', data[0]):
            if re.search(r'(IOS|-iP|_iP)', data[0]):
                is_ios = 1
                is_android = 0
            else:
                is_ios = 0
                is_android = 1

            if re.search(r'(Back|back)', data[0]):
                is_back_pocket = 1

            if re.search(r'(Front|front)', data[0]):
                is_front_pocket = 1

            if re.search(r'(Purse|purse)', data[0]):
                is_purse = 1

            if not (is_back_pocket or is_front_pocket or is_purse):
                is_other = 1

            for val in validator_list:
                if val.number <= (len(data) - 2):

                    #TODO: here we need to update validator by adding numbers where necessary
                    passed = 0
                    if re.search(r'1', data[val.number+1]):
                        passed = 1

                    val = update_validator_results(val, passed, is_android, is_ios, is_front_pocket, is_back_pocket, is_purse, is_other)

        rownum += 1

    csv_in.close()


    csv.register_dialect('test', delimiter=';', quoting=csv.QUOTE_NONE)
    csv_out = open(output_file, 'w', newline='')
    writer = csv.writer(csv_out, dialect = 'test')

    first_row = []
    first_row.append(' ')
    first_row.append('Use case')
    first_row.append('Total dataset')
    first_row.append('% Total dataset')

    for val in validator_list:
        first_row.append(val.name)
        first_row.append('% ' + val.name)

    writer.writerow(first_row)

    modes = ['All datasets', 'iOS', 'android']
    use_cases = ['All use cases', 'Front pocket', 'Back pocket', 'General', 'Purse']

    for mode in modes:
        for case in use_cases:
            row = []
            row.append(mode)
            row.append(case)

#TODO: --------------------------------------------------------------------

            if mode == 'All datasets':
                if case == 'All use cases':
                    total_datasets = max( validator_list[0].all_total[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.all_total[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Front pocket':
                    total_datasets = max(validator_list[0].all_front_pocket[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.all_front_pocket[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Back pocket':
                    total_datasets = max( validator_list[0].all_back_pocket[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.all_back_pocket[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'General':
                    total_datasets = max( validator_list[0].all_other[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.all_other[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Purse':
                    total_datasets = max( validator_list[0].all_purse[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.all_purse[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

#TODO: --------------------------------------------------------------------

            elif mode == 'iOS':
                if case == 'All use cases':
                    total_datasets = max( validator_list[0].ios_total[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.ios_total[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Front pocket':
                    total_datasets = max( validator_list[0].ios_front_pocket[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.ios_front_pocket[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Back pocket':
                    total_datasets = max( validator_list[0].ios_back_pocket[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.ios_back_pocket[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'General':
                    total_datasets = max( validator_list[0].ios_other[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.ios_other[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Purse':
                    total_datasets = max( validator_list[0].ios_purse[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.ios_purse[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

#TODO: --------------------------------------------------------------------

            elif mode == 'android':
                if case == 'All use cases':
                    total_datasets = max( validator_list[0].android_total[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.android_total[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Front pocket':
                    total_datasets = max(validator_list[0].android_front_pocket[1] , 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.android_front_pocket[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Back pocket':
                    total_datasets = max( validator_list[0].android_back_pocket[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.android_back_pocket[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'General':
                    total_datasets = max( validator_list[0].android_other[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.android_other[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

                elif case == 'Purse':
                    total_datasets = max( validator_list[0].android_purse[1], 0.1)
                    row.append(round(total_datasets))
                    row.append('100%')
                    for val in validator_list:
                        passed_datasets = val.android_purse[0]
                        row.append(passed_datasets)
                        percentage_left = str(100 * passed_datasets/total_datasets) + '%'
                        row.append(percentage_left)

            writer.writerow(row)

    csv_out.close()


if __name__ == '__main__':
    main(sys.argv)

    # arg_test = []
    # arg_test.append([])
    # arg_test.append('validation_results.csv')
    # arg_test.append('statistical_report.csv')
    # main(arg_test)