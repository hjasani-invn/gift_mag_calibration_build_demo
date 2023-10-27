set "json_file=Northland_map_update\original_FP\venue.json"
set "origin_mfp3=Northland_map_update\original_FP\venue.mfp3"
set "modif_mfp3=Northland_map_update\elevators_FP\venue.mfp3"
set "result_mfp3=Northland_map_update\original+elevators_FP\venue.mfp3"

python main.py   %json_file% %origin_mfp3% %modif_mfp3% %result_mfp3%
