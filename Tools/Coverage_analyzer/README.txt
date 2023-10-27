Main script to launch: TotalCoverage.py

======================

Input parameters (ordered):

floor number (starts from 1)
settings_json (full path to "*.json" file)
BFP_gridFile (full path to "*.blegrid" file)
WFP_gridFile (full path to "*.wifigrid" file)
MFP_gridFile (full path to "*.maggrid" file)
BFP_db3_file (full path to "*.ble3" file)
WFP_db3_file (full path to "*.wifi3" file)
out_folder (path to output folder)

======================

Main output:

1) Total coverage image: "total_coverage_F.png" (F - floor number)
2) File with coordinates of corners for total output: "total_corners_F.txt" (F - floor number)

"total_corners_F.txt" has the following structure (all numbers separated by whitespaces):

	Latitude_BottomLeft 	Longitude_BottomLeft
	Latitude_BottomRight 	Longitude_BottomRight
	Latitude_TopRight 		Longitude_TopRight
	Latitude_TopLeft 		Longitude_TopLeft

======================

Additionaly Wi-Fi, BLE and magnetic coverage maps are provided as output, as well as corresponding corners coordinates in similar files.