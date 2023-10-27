#!/bin/sh
#

cd Scripts/fpbl_testing

release=release_1.5.4
case=rc-1


fpbuilder_path=/projects/compute02/FPL_DATA/test_data/FPBL_release_testing/Builder_Linux_executable
fpbuilder_app=fpbuilder.out
test_schedule=/projects/compute02/FPL_DATA/test_data/FPBL_release_testing/tests_schedule_linux.json
input_data_path=/projects/compute02/FPL_DATA/test_data/FPBL_release_testing/
output_data_path=/projects/compute02/FPL_DATA/test_results/FPBL/$release/$case/test07/linux/

#ls
echo "=========="
echo $fpbuilder_path
echo $fpbuilder_app
echo $test_schedule
echo $input_data_path
echo $output_data_path
echo "=========="

mkdir -p $output_data_path
python main.py  -app $fpbuilder_path/$fpbuilder_app -in $input_data_path  -t $test_schedule -out $output_data_path

# "-in", "--input", help="input data path"
# "-t", "--tests", help="json file with list of tests"
# "-out", "--output", help="output folder"

