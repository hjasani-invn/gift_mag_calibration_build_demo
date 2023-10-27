=============================================================

	Script Generator for RAMP Matlab model
	
	README
	
	v. 0.1


=============================================================

1) Overall description

A script for RAMP model consists of two sections. The header section contains commands for errors and biases.
The second section contains commands, that set the movement of simulated user and phone (track section of the script).

In order for generator to create scripts "header_parameters" and "track_parameters" must be set.

Then "main" script generates header sections and track sections of the script in separate folders.
Header sections contain errors chosen at random from "header_parameters" structure.
The recommended way of using the Generator is to create a large number of headers,
since each new track takes a new header and the total number of tracks is hard to predict.
Once generated a set of headers can be saved for later usage and "generate_headers" function call can be commented.

"track_parameters" structure is set in the "main" script. 

"compose_scripts" function adds header to each track section of the script resulting in full script.
	NOTE : for track script number N the header number N is added

2) "choose_parameters" function description

This function contains predefined sets of commands that set errors in header parts of the scripts.
Input arguement is the set number.

3) "track_parameters" structure description

track_parameters.min_X, track_parameters.min_Y, track_parameters.max_X, track_parameters.max_X - venue geometry
	NOTE : generator works for only one floor currently

track_parameters.random - 0 for regular grid, 1 for random tracks generation

track_parameters.space_period - distance between tracks for regular grid in meters

track_parameters.number_of_grid_repetitions - grid is created N times, if this is set to N (each time new header is taken)

track_parameters.mode -  'empty_space' or 'corridor', in case of corridor their geometric parameters are taken from 'corridors.txt' (default name).

track_parameters.any_heading - if '0', then heading of tracks is chosen at random from [0, 90, 180, 270] degrees, 
								if '1' - heading chosen at random from [0..360] degrees

track_parameters.vel  - user velocity for all tracks (no reason to set different for each track, usually set at 1 m/s)

track_parameters.sample_rate - sample rate of magnetic data measurements

track_parameters.euler_angles - starting euler angles of the device orientation in hand 
	NOTE : currently generator does not simulate changing device orientation in hand

track_parameters.random_tracks_number - only used if 'track_parameters.random' is set to 1

track_parameters.starting_index - allows generating scripts starting from that index

4) 'corridors.txt' file format:

	For each line:
	"x1 y1 x2 y2 w"
	
		(x1,y1) - starting point of corridor, 
		(x2,y2) - end point, 
		'w' - corridor width (in meters)
	
