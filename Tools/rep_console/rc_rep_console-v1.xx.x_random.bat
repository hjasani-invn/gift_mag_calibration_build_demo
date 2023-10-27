echo on
echo Release console reprocessing

set "py_interp=C:\Python\Anaconda3\python" rem python3 is required
set "script=C:\dchurikov\Scripts\rtfppl_runner\rtfppl_runner_cl.py"

rem set correct values here
set console_collection_path="\\cayyc-proj01\compute02\dchurikov\Polygon\rtfppl-console\alfa-versions"
set "release_name=v1.xx.x"

rem do not change script below
set "release_path=\\cayyc-proj01\compute02\FPL_DATA\test_results\RTFPPL\"
set "output_path=%release_path%\%release_name%\rep.console-x10\"
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\ICA_RelSet\general\"

rem TDK HQ reprocessing test MM off
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\TDK_HQ_Nihonbashi\datasets\"
set "fp_collection=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\TDK_HQ_Nihonbashi\venues\tdkhq.mp4\"
set "rtfppl_collection=%console_collection_path%\%release_name%_random"
rem %py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%
for /l %%i in (1,1,10) do %py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%%%i

rem TDK HQ reprocessing test MM on
set "input_path=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\TDK_HQ_Nihonbashi\datasets\"
set "fp_collection=\\cayyc-proj01\compute02\FPL_DATA\test_data\Vectors_sources.rel\TDK_HQ_Nihonbashi\venues\tdkhq.mp4m\"
set "rtfppl_collection=%console_collection_path%\%release_name%_random"
rem %py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%
for /l %%i in (1,1,10) do %py_interp% %script% -i %input_path% -f %fp_collection% -c %rtfppl_collection% -o %output_path%%%i
