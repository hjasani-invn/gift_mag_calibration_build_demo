# WFP AP list tool

python version: 3.6 or late

main file: main.py

function call:process_WFP_list.process(output_folder, input_file_list)

input_file_list have to contain

WFP_db3_file - WFP database (format wifi3)
WFP_db4_file - WFP database (format wifi4)

out_folder - an output folder. It is created if not exist.


Release notes
WFP_AP_list v1.0.0
+initial version

WFP_AP_list v1.0.1
+ Wfp-db file only is Input parameter for the script
+ Add automatic WFP type detection and loading, wfp3 or wfp4.  
+ Remove venue.json using from script interface (we do not need venue.json for this task)
+ Generation ap list for whole venue, not for each floor. Each WFP cell contains the same APs now.

