===========================================================================

dataset_list_from_validation_log.py

Takes a list of datasets from "validation_results.csv" and creates a JSON file to be used as input for FP builder.
This needs to be done if there are some datasets which exist, but are disabled for FP builder on server.
Allows to run FP builder locally with the same set of datasets.

Takes two input parameters:
    1) root_path - folder with "validation_results.csv"
    2) path_starting_segment - path to folder where survey datasets are located
        Example: '//cayyc-proj01/compute02/ykotik/SurveyTool/datasets/prod/TDK_HQ_Nihonbashi/SurveyDatasets/'
        "validation_results.csv" created by server FP builder starts with "input/", this script replaces this part of path with "path_starting_segment"
        
Result is saved as "input_list.json"

===========================================================================

phone_names_from_validation_log.py

Input_file is "validation_results.csv", path is set in code.
Output gives the list of phones used in survey and how many datasets were collected with each phone.

===========================================================================

get_biases_list.py

"root_path" is set in code. It should be the path to all datasets which were processed by FP builder with recalibration mode ON.
Only the last iteration of biases is taken. Then they are combined in a CSV file with the following structure:

"dataset_path", "mag_bias_X", "mag_bias_Y", "mag_bias_Z"
...

===========================================================================

process_bias_history.py

"input_path" is set in code.  It should be the path to all datasets which were processed by FP builder with recalibration mode ON.
"iterations_number" must be also set in code manually. 
    It must be the same value as the one set in FP_builder.hpp, currently line 52. Here is the example of that line:
    #define NUMBER_OF_RECALIB_ITERATIONS	3

Script creates three files:
    x_outfile = 'x_bias_history.csv'
    y_outfile = 'y_bias_history.csv'
    z_outfile = 'z_bias_history.csv'

Each of them has the following format:

"dataset_path", "mag_bias_1st_iteration",  "mag_bias_2nd_iteration", ... ,  "mag_bias_last_iteration"

===========================================================================