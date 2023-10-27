=============================================

Beacons Placement tool description

This tool uses simulated annealing algorithm to generate random beacon positions N times, then selects the best generated set.

=============================================

Using the Beacons Placement tool

Tool uses Python 3.6. The following additional Python modules are required to be installed:

PIL
numpy
matplotlib
fastkml
shapely
tqdm

Main script - main.py

Input is set in main.py variables:

1) image_file_full_path - path to image file
2) settings_file_full_path - path JSON file

Image file must be in bmp format. 
Traversible areas must have red color (any shade of red).
High priority areas (if they exist) must have green color (any shade of green).
All other colors are considered to be obstacles.


JSON file example:
------------------------
	{
	  "beacon_angle": 60,  // degrees
	  "beacon_RSSI_1m": -65, // dBm
	  "beacon_grid_cellsize": 2.0, // meters
	  "maximum_beacon_count": 60,
	  "priority_area_factor": 3.0,
	  "min_intersection_RSSI": -78, // dBm
	  "rejection_coefficient": 1.0,
	  "number_of_iterations": 5000,
	  "venue" :
			{
				"origin_lattitude" :  38.60185487701,  // degrees
				"origin_longitude" : -90.44140364513,  // degrees
				"origin_azimuth" : 89.305, // degrees
				"size_x" : 143, // meters
				"size_y" : 94   // meters
			}
	}
------------------------

"beacon_angle" - maximum beacon signal angle simulated, it is calculated from the central direction (60 means +/-60 degrees from beacon direction)

"beacon_RSSI_1m" - simulated beacon RSSI at 1m distance

"beacon_grid_cellsize" - beacons are placed on a grid with this cell size, maximum one beacon for each cell (beacon can only be placed on an obstacle like a wall)
						lower grid cell size means more possible beacon positions will be used, but it takes more time

"maximum_beacon_count" - the resulting set of beacons will never contain more than this amount of beacons (note - it may be less, if selected number is too high)

"priority_area_factor" - covering high priority areas give more value to the beacon, this factor sets this additional value (3.0 is 3 times higher than the normal area)

"min_intersection_RSSI" - minimum RSSI for each of two beacon signals in one cell for those beacons to be considered intersecting (this is never allowed to happen)
							
"rejection_coefficient" - after getting all possible beacon positions algorithm estimates quality for each position,
							then all beacon positions with quality < (median * rejection_coefficient) are rejected immediately
							Note: this is usually used for the algorithm to have less iterations and to avoid placing beacons in very small rooms
							Note: setting rejection coefficient to a very high value may lead to beacons only being placed in high priority areas

"number_of_iterations" - number of times for simulated annealing algorithm to run
							
"venue" - parameters are taken from venue.json
			Note: venue parameters must match the input image file, 
				if image needs to be cut (for example if beacons placement is required only in small part of the venue),
				then venue parameters should be adjusted accordingly

				
Output: kml file "beacons.kml", image file "beacons.png"
Image file shows internal obstacle format as well as beacons positions (yellow) and their signals (green).

=============================================
