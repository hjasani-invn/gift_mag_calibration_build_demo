rem start

rem cd "c:\dchurikov\realtime_wrapper"
cd "C:\KPI_tool_git\realtime_wrapper"

echo KPI processing for RTFPPL release

set "topic=release_1.5.5"
set "case=rc-1"
set "branch=master"
set "release_folder=\\cayyc-proj01\compute02\FPL_DATA\test_results\FPBL"
set "kpi_sources_folder=\\cayyc-proj01\compute02\FPL_DATA\test_data\KPI_sources.rel"


copy /Y %kpi_sources_folder%\Northland\venue.fp4\fpl.json        %release_folder%\%topic%\%case%\test_08\output\
copy /Y %kpi_sources_folder%\Northland\venue.fp4\fpl.dbgjson     %release_folder%\%topic%\%case%\test_08\output\

copy /Y %kpi_sources_folder%\ISJ\venue.fp4\venue.json      %release_folder%\%topic%\%case%\test_09\output\
copy /Y %kpi_sources_folder%\ISJ\venue.fp4\fpl.dbgjson     %release_folder%\%topic%\%case%\test_09\output\


rem Northland fp4: 
rem C:\Python27\python.exe scripts\fullGiftTest.py	%kpi_sources_folder%\Northland\Input_data_4\  %release_folder%\%topic%\%case%\test_11\output\test_08\   %release_folder%\%topic%\%case%\test_08\output\  --b %branch%   --dbg_output none

rem ISJ fp4:
C:\Python27\python.exe scripts\fullGiftTest.py	%kpi_sources_folder%\ISJ\Collection2\	%release_folder%\%topic%\%case%\test_11\output\test_09\  %release_folder%\%topic%\%case%\test_09\output\ --b %branch%  --dbg_output none

