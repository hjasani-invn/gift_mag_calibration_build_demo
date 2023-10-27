set "release_folder=\\CAYYC-PROJ01\compute02\FPL_DATA\test_results\FPBL"
set "topic=release_1.5.5"
set "case=rc-1"

rem copy for test02
copy /Y \\cayyc-proj01\compute02\FPL_DATA\test_data\FPBL_release_testing\ISJ\FPBLIn\venue.json %release_folder%\%topic%\%case%\test_02\output
copy /Y \\cayyc-proj01\compute02\FPL_DATA\test_data\FPBL_release_testing\ISJ\geojson_files\*.geojson  %release_folder%\%topic%\%case%\test_02\output

rem copy for test03
copy /Y \\cayyc-proj01\compute02\FPL_DATA\test_data\FPBL_release_testing\AeonShintoshinMall\FPBLIn\venue.json %release_folder%\%topic%\%case%\test_03\output
copy /Y \\cayyc-proj01\compute02\FPL_DATA\test_data\FPBL_release_testing\AeonShintoshinMall\geojson_files\*.geojson  %release_folder%\%topic%\%case%\test_03\output

 
rem use ASCA, must be real path to ASCA\main.ru

C:\Python39\Python.exe ..\..\..\Tools\ASCA\main.py   %release_folder%\%topic%\%case%\test_02\output   %release_folder%\%topic%\%case%\test_04\output\test_02 
C:\Python39\Python.exe ..\..\..\Tools\ASCA\main.py   %release_folder%\%topic%\%case%\test_03\output   %release_folder%\%topic%\%case%\test_04\output\test_03 

