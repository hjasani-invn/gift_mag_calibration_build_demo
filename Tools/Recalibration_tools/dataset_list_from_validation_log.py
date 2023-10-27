import sys
import os
import re


def get_val_res_path(root_path):
    val_res_mask = '(.*validation_results\.csv)'
    val_res_path = ''

    for cur, dirs, files in os.walk(root_path):
        for f in files:
            if re.match(val_res_mask, f) is not None:
                val_res_path = os.path.join(cur, f)
                return val_res_path

    return val_res_path


if __name__ == "__main__":
    root_path = sys.argv[1].replace("\r", "")

    # search for validation_results
    val_res_path = get_val_res_path(root_path)
    # val_res_path = ''
    if len(val_res_path) == 0:
        print('Failed to find validation results file')

    if len(sys.argv) > 2:
        path_starting_segment = str(sys.argv[2])
    else:
        path_starting_segment = '//cayyc-proj01/compute02/ykotik/SurveyTool/datasets/prod/TDK_HQ_Nihonbashi/SurveyDatasets/'

    skip_symbols = len('input/')  # TODO: change it if paths in validation log don't begin with "input/"
    print(val_res_path)

    new_paths = []

    with open(val_res_path, 'r') as val_res:
        lines = val_res.readlines()
        for l in lines[1:]:
            elements = l.split(',')
            dataset_path = elements[0][skip_symbols:]
            final_dataset_path = os.path.dirname(os.path.join(path_starting_segment, dataset_path))
            final_dataset_path = '["' + final_dataset_path + '"],' + '\n'
            new_paths.append(final_dataset_path)

    new_paths[-1] = new_paths[-1][:-2] + '\n' # removing the ',' symbol at the end of the last string

    with open('input_list.json', 'w') as output:
        output.write('{')
        output.write('\n')
        output.write('"datasets_list":')
        output.write('\n')
        output.write('[')
        output.write('\n')
        for path in new_paths:
            output.write(path)
        output.write(']')
        output.write('\n')
        output.write('}')
