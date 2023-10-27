rem input_path - the path to datasets folder
rem fp_collection - path to fingerprint folder
rem rtfppl_collection - path to RTFPPL console
rem output_path - path to output folder
rem optional parameters (can be missing)
rem exe_filename - execute file name (default=fp_positioning.console.exe)
rem venue_json - venue settings file name (default=venue.json)
rem dat_file_template - template for data files (default=(.*)TppOutput(.*)dat)

python rtfppl_runner.py -i input_path -f fp_collection -c rtfppl_collection -o output_path -e exe_filename -v venue_json -d dat_file_template
