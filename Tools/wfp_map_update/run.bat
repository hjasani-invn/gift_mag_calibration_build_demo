rem    wfp_1 - file to be modified - .wifi4 format
rem    wfp_2 - file using for modification - .wifi4 format
rem    wfp_3 - new file - .wifi4 format

set "settings_json=settings.json"
set "wfp_1=wfp_1.wifi4"
set "wfp_2=wfp_2.wifi4"
set "wfp_3=wfp_3.wifi4"
set "output=output.csv"

echo ; > %output%
echo %wfp_1% >> %output%
echo %wfp_2% >> %output%
echo ; >> %output%

python main.py  %settings_json%  %wfp_1% %wfp_2% %wfp_3% >> %output%

