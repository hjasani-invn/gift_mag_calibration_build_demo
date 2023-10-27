echo on
echo FPBL runner script

set "py_interp=C:\Python\Anaconda3\python" rem python3 is required
set "script=C:\dchurikov\Scripts\fpbl_runner\fpbl_runner.py"

rem v RWS.epsilon
set "fpbuilder_collection=\\cayyc-proj01\compute02\dchurikov\Polygon\fp_builder.console\"
set "fpbuilder_version=alfa\RWS1to10+stuc_protection"
set "fpbuilder_path=%fpbuilder_collection%\%fpbuilder_version%"
set "topic=C:\temp\fp#1"

rem Hikone
set "input_data_path=\\cayyc-proj01\compute02\FPL_DATA\SurveyMap\datasets\prod\Bridgestone_hikone\SurveyDatasets\"


set "name=RWS.stuck_prot.test2x2.whl5g.w5x5.m2x2.selproc"
set "output_data_path=%topic%\%name%\"
set "json=%output_data_path%\bsh2x2_venue.json"
set "input_list=%output_data_path%\bs-hikone2x2-d_list.json"
set "ignore_list=%output_data_path%\bsh_il_w5G.json"
%py_interp% %script%  --exe_path %fpbuilder_path% --input_data_path %input_data_path%  --output_path %output_data_path%  --settings_json %json% --input_list %input_list% --ignore_list %ignore_list%

set "name=whl5g"
set "json=\\cayyc-proj01\compute02\dchurikov\Tasks_of_deployment\2020_02-BS-Hikone-aftercamp_WFP_investigation\venues.adjustm\cases#001\%name%.json"
set "output_data_path=%topic%\%fpbuilder_version%.%name%\"
rem %py_interp% %script%  --exe_path %fpbuilder_path% --input_data_path %input_data_path%  --output_path %output_data_path%  --settings_json %json%

set "name=whl2+5g"
set "json=\\cayyc-proj01\compute02\dchurikov\Tasks_of_deployment\2020_02-BS-Hikone-aftercamp_WFP_investigation\venues.adjustm\cases#001\%name%.json"
set "output_data_path=%topic%\%fpbuilder_version%.%name%\"
rem %py_interp% %script%  --exe_path %fpbuilder_path% --input_data_path %input_data_path%  --output_path %output_data_path%  --settings_json %json%

set "name=whl5g+wcell2m"
set "json=\\cayyc-proj01\compute02\dchurikov\Tasks_of_deployment\2020_02-BS-Hikone-aftercamp_WFP_investigation\venues.adjustm\cases#001\%name%.json"
set "output_data_path=%topic%\%fpbuilder_version%.%name%\"
rem %py_interp% %script%  --exe_path %fpbuilder_path% --input_data_path %input_data_path%  --output_path %output_data_path%  --settings_json %json%

set "name=whl2+5g+wcell2m"
set "json=\\cayyc-proj01\compute02\dchurikov\Tasks_of_deployment\2020_02-BS-Hikone-aftercamp_WFP_investigation\venues.adjustm\cases#001\%name%.json"
set "output_data_path=%topic%\%fpbuilder_version%.%name%\"
rem %py_interp% %script%  --exe_path %fpbuilder_path% --input_data_path %input_data_path%  --output_path %output_data_path%  --settings_json %json%
