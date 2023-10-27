import os
import re
import subprocess
import logging
import sys


def int_status_to_string(int_status):
    string_status = ''

    if int_status == 0:
        string_status = 'SUCCESS'
    if int_status == -1:
        string_status = 'UNKNOWN_ERROR'
    if int_status == -2:
        string_status = 'UNREALISTIC_SCALE_FACTOR'
    if int_status == -3:
        string_status = 'HIGH_DOP'
    if int_status == -4:
        string_status = 'LOW_CALIBRATION_ACCURACY'
    if int_status == -5:
        string_status = 'UNSTABLE_TEMPERATURE'
    if int_status == -6:
        string_status = 'TOO_LONG_CALIBRATION_TIME'
    if int_status == -7:
        string_status = 'INSUFFICIENT_DATA'
    if int_status == -8:
        string_status = 'STATUS_ERRORS_IN_INPUT_DATA'

    return string_status


if __name__ == "__main__":

    try:
        exe_full_path = sys.argv[1]
        target_dir = sys.argv[2]
        # target_dir = '//cayyc-proj01/compute02/dchurikov/temp/Robotic_survey.gimbal_test/stage7/'
        # exe_full_path = 'C:/Github/Gift/Applications/Calibration_console/x64/Release/Calibration_console.exe'

        acc_mask = r'.*_imu\.csv'
        mag_mask = r'.*_mag\.csv'
        result_file_name = 'calib_out.txt'
        result_mask = r'calib_out\.txt'

        input_file_list = []

        for cur, dirs, files in os.walk(target_dir):
            for f in files:
                # if re.match(acc_mask, f) or re.match(mag_mask, f):
                if re.match(mag_mask, f):
                    input_file_list.append(os.path.join(cur, f))

        for input_file_path in input_file_list:
            out_path = os.path.join(os.path.dirname(input_file_path), result_file_name)
            command = exe_full_path + ' ' + input_file_path + ' > ' + out_path

            return_code = subprocess.call(command, shell=True)

        test_results = []

        with open('calib_test_results.csv', 'w') as res_file:

            out_line = 'path, status, calib_level, calib_accuracy, DOP, bx, by, bz, CM \n'
            res_file.write(out_line)

            for cur, dirs, files in os.walk(target_dir):
                for f in files:
                    if re.match(result_mask, f):
                        f_path = os.path.join(cur, f)
                        with open(f_path, 'r') as one_result_f:
                            lines = one_result_f.readlines()
                            status = int_status_to_string(int(lines[0].split()[1]))
                            calib_level = lines[1].split()[1]
                            accuracy = lines[2].split()[1]
                            DOP = lines[3].split()[1]
                            bx = lines[4].split()[1]
                            by = lines[4].split()[2]
                            bz = lines[4].split()[3]
                            CM_00 = lines[5].split()[1]
                            CM_01 = lines[5].split()[2]
                            CM_02 = lines[5].split()[3]
                            CM_10 = lines[5].split()[4]
                            CM_11 = lines[5].split()[5]
                            CM_12 = lines[5].split()[6]
                            CM_20 = lines[5].split()[7]
                            CM_21 = lines[5].split()[8]
                            CM_22 = lines[5].split()[9]

                        out_line = cur + ', '
                        out_line += status + ', '
                        out_line += calib_level + ', '
                        out_line += accuracy + ', '
                        out_line += DOP + ', '
                        out_line += bx + ', '
                        out_line += by + ', '
                        out_line += bz + ', '
                        out_line += CM_00 + ', ' + CM_01 + ', ' + CM_02 + ', '
                        out_line += CM_10 + ', ' + CM_11 + ', ' + CM_12 + ', '
                        out_line += CM_20 + ', ' + CM_21 + ', ' + CM_22 + '\n'

                        res_file.write(out_line)

    except Exception as ex:

        print("Exception happened. Exception code:", ex.args)
        log_file_name = 'Calib_Tests_Error_log' + '.txt'
        logging.basicConfig(filename=log_file_name, level=logging.INFO,
                            format='%(asctime)s %(levelname)s %(name)s %(message)s')
        logger = logging.getLogger(__name__)
        logger.error(ex, exc_info=True)

        raise SystemExit
