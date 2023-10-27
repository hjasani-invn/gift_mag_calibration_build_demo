echo FPBL automatic testing tool
cd "\\cayyc-proj01\compute02\dchurikov\Polygon\fpbl_testing\"

set "fpbuilder_path=\\cayyc-proj01\compute02\dchurikov\Polygon\fp_builder.console\rc-1.2.3\"
set "fpbuilder_app=FP_builder.exe"
set "test_schedule=\\cayyc-proj01\compute02\dchurikov\Polygon\fpbl_testing\tests_schedule.json"
set "input_data_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\FPBL_release_testing\"
set "output_data_path=\\cayyc-proj01\compute02\FPL_DATA\test_results\FPBL\release_1.2.3\"

python main.py  -app %fpbuilder_path%\%fpbuilder_app% -in %input_data_path%  -t %test_schedule% -out %output_data_path%

rem "-in", "--input", help="input data path"
rem "-t", "--tests", help="json file with list of tests"
rem "-out", "--output", help="output folder"
