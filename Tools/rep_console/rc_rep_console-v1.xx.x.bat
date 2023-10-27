echo on
echo Release console reprocessing

set "py_interp=C:\Python\Anaconda3\python" rem python3 is required
set "script=C:\dchurikov\Scripts\rtfppl_runner\rtfppl_runner_cl.py"

rem set correct values here
set console_collection_path="\\cayyc-proj01\compute02\dchurikov\Polygon\rtfppl-console\alfa-versions"
set "release_name=v1.xx.x"

rem do not change script below
set "release_path=\\cayyc-proj01\compute02\FPL_DATA\test_results\RTFPPL\"
set "output_path=%release_path%\%release_name%\rep.console\"
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\ICA_RelSet\general\"

rem MM + Mag + proximity reprocessing test with initial BLE bias error - ICA
set "fp_collection=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\ICA_RelSet\Venue\ica_mf_2020_05_27.mwp4m\"
set "rtfppl_collection=%console_collection_path%\%release_name%_b10"
%py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%

rem pulling test
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\ICA_RelSet\pulling\"
set "fp_collection=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\ICA_RelSet\Venue\ica_copy_2020_04_03.mp4\"
set "rtfppl_collection=%console_collection_path%\%release_name%"
%py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%

rem VDR reprocessing test
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\Cambrian_p2.vdr\"
set "fp_collection=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\Cambrian_p2.vdr\venue\fp-20_03_12.m4\"
set "rtfppl_collection=%console_collection_path%\%release_name%_vdr"
%py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%

rem Sato reprocessing test
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\Sato\2019_11_13\"
set "fp_collection=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\Sato\2019_11_13\venues\sato.mwp4m\"
set "rtfppl_collection=%console_collection_path%\%release_name%"
%py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%

rem TDK HQ reprocessing test MM off
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\TDK_HQ_Nihonbashi\datasets\"
set "fp_collection=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\TDK_HQ_Nihonbashi\venues\tdkhq.mp4\"
set "rtfppl_collection=%console_collection_path%\%release_name%"
%py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%

rem TDK HQ reprocessing test MM on
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\TDK_HQ_Nihonbashi\datasets\"
set "fp_collection=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\TDK_HQ_Nihonbashi\venues\tdkhq.mp4m\"
set "rtfppl_collection=%console_collection_path%\%release_name%"
%py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%