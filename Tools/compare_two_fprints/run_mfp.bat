set "input_folder_ref=\\cayyc-proj01\compute02\vpentyukhov\fpbl_testing\release_1.5.5_July_19\rc-1"
set "input_folder_work=\\cayyc-proj01\compute02\FPL_DATA\test_results\FPBL\release_1.6.0\rc-1"


set "json_file=%input_folder_ref%\test_01\output_Kurume\venue.json"
set "ref_mfp3=%input_folder_ref%\test_01\output_Kurume\Bridgestone-Kurume.mfp3"
set "work_mfp3=%input_folder_work%\test_01\output_Kurume\Bridgestone-Kurume.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test01.csv

set "json_file=%input_folder_ref%\test_03\output\venue.json"
set "ref_mfp3=%input_folder_ref%\test_03\output\ISJ.mfp3"
set "work_mfp3=%input_folder_work%\test_03\output\ISJ.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test03.csv

set "json_file=%input_folder_ref%\test_04\output\venue.json"
set "ref_mfp3=%input_folder_ref%\test_04\output\IS_SJ_4.mfp3"
set "work_mfp3=%input_folder_work%\test_04\output\IS_SJ_4.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test04.csv

set "json_file=%input_folder_ref%\test_05\output\venue.json"
set "ref_mfp3=%input_folder_ref%\test_05\output\ISJ.mfp3"
set "work_mfp3=%input_folder_work%\test_05\output\ISJ.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test05.csv

set "json_file=%input_folder_ref%\test_06\output_blacklist\venue.json"
set "ref_mfp3=%input_folder_ref%\test_06\output_blacklist\JEIS_9th_floor.mfp3"
set "work_mfp3=%input_folder_work%\test_06\output_blacklist\JEIS_9th_floor.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test06.csv

set "json_file=%input_folder_ref%\test_07\output_whitelist\venue.json"
set "ref_mfp3=%input_folder_ref%\test_07\output_whitelist\JEIS_9th_floor.mfp3"
set "work_mfp3=%input_folder_work%\test_07\output_whitelist\JEIS_9th_floor.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test07.csv

set "json_file=%input_folder_ref%\test_08\output\venue.json"
set "ref_mfp3=%input_folder_ref%\test_08\output\ICA_CrowdSource.mfp3"
set "work_mfp3=%input_folder_work%\test_08\output\ICA_CrowdSource.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test08.csv

set "json_file=%input_folder_ref%\test_09\output_part1\venue.json"
set "ref_mfp3=%input_folder_ref%\test_09\output_part1\TDK_HQ_Nihonbashi.mfp3"
set "work_mfp3=%input_folder_work%\test_09\output_part1\TDK_HQ_Nihonbashi.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test09_part1.csv

set "json_file=%input_folder_ref%\test_09\output_part1+2\venue.json"
set "ref_mfp3=%input_folder_ref%\test_09\output_part1+2\TDK_HQ_Nihonbashi.mfp3"
set "work_mfp3=%input_folder_work%\test_09\output_part1+2\TDK_HQ_Nihonbashi.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test09_part1+2.csv

set "json_file=%input_folder_ref%\test_09\output_iterative\venue.json"
set "ref_mfp3=%input_folder_ref%\test_09\output_iterative\TDK_HQ_Nihonbashi.mfp3"
set "work_mfp3=%input_folder_work%\test_09\output_iterative\TDK_HQ_Nihonbashi.mfp3"
python main_mfp.py   %json_file% %ref_mfp3% %work_mfp3% > test09_iterative.csv

rem set "json_file=C:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\venues\ICA_MultiFloor\venue_for_python.json"

rem set "new_mfp3=c:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\venues\ICA_MultiFloor\output_test2\ICA_MultiFloor.mfp3 "
rem set "old_mfp3=c:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\venues\ICA_MultiFloor\output_tests_\output_test2\ICA_MultiFloor.mfp3 "
rem python main_mfp.py   %json_file% %new_mfp3% %old_mfp3% > test2.csv

rem set "new_mfp3=c:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\venues\ICA_MultiFloor\output_test2\ICA_MultiFloor.mfp3 "
rem set "old_mfp3=c:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\venues\ICA_MultiFloor\output_tests_\output_test3\ICA_MultiFloor.mfp3 "
rem python main_mfp.py   %json_file% %new_mfp3% %old_mfp3% > test3.csv

