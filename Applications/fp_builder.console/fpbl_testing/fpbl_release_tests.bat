echo on
echo FPBL automatic testing tool
cd "Scripts\fpbl_testing\"

set "release=release_1.5.4"
set "case=rc-1"

rem set "fpbuilder_path=c:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\Project\Win32\Release"
set "fpbuilder_path=C:\GitHub\Gift\Applications\fp_builder.console\Project\Win32\Release\" 
set "fpbuilder_app=FP_builder.exe"
set "input_data_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\FPBL_release_testing\"  

rem set "test_schedule=c:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\tests_schedule.json"      
set "test_schedule=\\cayyc-proj01\compute02\FPL_DATA\test_data\FPBL_release_testing\tests_schedule.json"      
set "output_data_path=\\cayyc-proj01\compute02\FPL_DATA\test_results\FPBL\%release%\%case%"

python main.py  -app %fpbuilder_path%\%fpbuilder_app% -in %input_data_path%  -t %test_schedule% -out %output_data_path%

rem "-in", "--input", help="input data path"
rem "-t", "--tests", help="json file with list of tests"
rem "-out", "--output", help="output folder"
