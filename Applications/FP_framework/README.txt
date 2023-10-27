=============================================================

	Fingerprint generation and testing tool

	README 

	v. 0.3
	
=============================================================

NOTE: v. 0.3 Only new fingerprint building tools (created with the FPBL library) can be used now.
Previous option of using old tools was removed for complexity reasons. Also it was unsafe to use because of different input file formats.
Old tools can still be used for FP building only using the "FP_gen" python project.

1) How to use.

Edit "settings.json" and "fp_settings.txt" (located in folder with input data).
Then launch Main.py.
Fingerprint should be generated.
If "integration_testing" is set to "ON", tests will run and "test_results.txt" appears in folder with test tracks.

2) "settings.json" example:

{
	"input_data_path": "C:/FP_Generation/Regus/",
	"integration_testing": "ON",
	"FP_builder_name": "FP_builder.exe",
	"tests_path": "C:/FP_tests/",
	"venue_checkpoints": "regus_checkpoints.txt",
	"navigator_name": "navigator.exe"
}

"tests_path" - path to folder with test tracks.
This folder also has to contain the appropriate "navigator_name" application executable file, "venue_checkpoints" text file and the "venue.map" file of the venue for navigator application. 

"input_data_path" - path to the folder with Mapper data for fingerprint building.
Mapper data can be either in zip archives, or in folders.
Archives must contain all Mapper data inside a folder named "MapCreator". 

If data is in folders, then each of those folders should contain data collected with only one device.
Data folders must be named in "data*" format. Script is able to sort the data inside of those folders. The resulted subfolders are named "track*" (for example track1, track2, etc).
Data from archives goes into folders "data_from_archive_*". Also script deletes all previously existing "data_from_archive_*" folders before extracting data from archives. 
This is done to avoid errors, so that user is able to remove selected archive from dataset without bothering to find and delete appropriate folder with extracted data.
So, either user manually prepares all input data or the input data is taken from standard Mapper archives. In any case the data is sorted to tracks by script.
Using already sorted data is also possible, but in this case user should make sure that data is already converted. 

This folder also has to contain the settings file "fp_settings.txt". See its description below (4).

"FP_builder_name" - fingerprint building tool name. This file must be present in the folder with "Main.py" and other python script files.
	
"venue_checkpoints" - file name with venue checkpoints.

"navigator_name" - navigator application executable file, used for testing only.

3) "venue_checkpoints" file example:

0, , 15.31, 0.71, 0, 0.2
1, , 17.63, 5.23, 0, 0.2
2, , 15.18, 5.73, 0, 0.2

Format explanations: 

checkpoint_number,  , X(m), Y(m), floor, checkpoint_uncertainty(m)

4) "fp_settings.txt" example:

{
	"name": "regus2f",
	"venue_ID": 1,
	"CELL_SIZE_MFP": 1.0,
	"CELL_SIZE_WIFI": 2.5,
	"CELL_SIZE_BLE": 2.0,
	"MIN_X": 0,
	"MAX_X": 2400,
	"MIN_Y": 0,
	"MAX_Y": 1500,
	"MIN_Z": 0,
	"MAX_Z": 0,
	"wifi_points_number": 40,
	"ble_points_number": 15,
	"c_min": 5,
	"additional_wifi_params": "",
	"MFPsigA": 2,
	"MFPsigX": 1,
	"MFPsigY": 1,
	"MFPsigZ": 1
}

CELL_SIZE_MFP - magnetic fingerprint grid dimension (in meters)
CELL_SIZE_WIFI - WiFi fingerprint grid dimension (in meters)
CELL_SIZE_BLE - bluetooth fingerprint grid dimension (in meters)
MIN_X, MAX_X, MIN_Y, MAX_Y - venue one floor horizontal dimensions (in centimeters), those are the same for each floor
MIN_Z, MAX_Z - minimum and maximum floor numbers (integer number)

Note: other parameters were only used with old fingerprint building tools, they exist for compatibility.

5) Python script files description.

----------------------------
Main.py

Main script. Uses parameters from "settings.json".
Calls the process_mapper_input.process_input() function.
Then generates fingerprints by calling call_generator function. This function calls console application tool for fingerprint building.
If settings file enables integration testing, then script performs it using the FingerprintsTester type object. 

----------------------------
process_mapper_input.py

Calls archive processing and data conversion tools. All tracks are sorted into subfolders.
NOTE: "Track*.trk" files are used for sorting (because this file presence means a correct Mapper track). 
	Later this should be changed for new Mapper tool.

----------------------------
process_archives.py

Works in current directory <dir>. 
Deletes all data folders /data_from_archive_*/ left from previous execution of the script.
Extracts each zip archive in <dir>. 
For each archive checks that archive contained a "MapCreator" folder.
	If it did:
		1) Creates /data_from_archive_<num>/, where <num> is iterated from 0
		2) Fills folder with all the files from archive. Results in folder with unsorted data from tracks.
 
----------------------------
generate_fingerprints.py

Contains FingerprintsGenerator class. Object of this class must be initialized with settings file.

FingerprintsGenerator has method:

	call_generator(self, main_settings)

This method calls "FP_builder" tool using data from settings files.
	
----------------------------
test_fingerprints.py

Contains FingerprintsTester class. Object of this class must be initialized with settings file.

Method process_tests() performs all testing logic by calling other auxiliary methods.

The output is 'test_results.txt'. It has the following format:

mean_pos_error 		mean_floor_error 	mean_availability
median_pos_error 	median_floor_error 	median_availability


