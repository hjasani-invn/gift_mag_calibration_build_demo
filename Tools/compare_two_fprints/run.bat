
rem change to real file name and floor number
set "settings_json=settings.json"
set "wfp_1=wfp_1.wifi4"
set "wfp_2=wfp_2.wifi4"
set "floor_number=0"
set "output=output.csv"

echo ; > %output%
echo %wfp_1% >> %output%
echo %wfp_2% >> %output%
echo ; >> %output%

python main.py  %settings_json%  %wfp_1% %wfp_2% %floor_number% >> %output%

