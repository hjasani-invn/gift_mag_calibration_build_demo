rem "-app", "--application", help="FP builder name"
rem "-in", "--input", help="input data path"
rem "-t", "--tests", help="json file with list of tests"
rem "-out", "--output", help="output folder"

python main.py  -app FP_builder.exe -in \\cayyc-proj01\compute02\FPL_DATA\test_data\FPBL_release_testing  -t tests_schedule.json -out release_1.2.0 