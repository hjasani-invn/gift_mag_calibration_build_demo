clc;
% clear all;
close all;

set_number = 12;
header_parameters = choose_parameters( set_number );

headers_folder = strcat(pwd, '\headers\');

header_parameters.headers_number = 1000; % WARNING - it is better to generate a lot of headers

% ======================================================
generate_headers(headers_folder, header_parameters);
% ======================================================

track_parameters.origin_lattitude = 55.724253553756192;
track_parameters.origin_longitude = 37.653916182652893;
track_parameters.origin_azimuth = 0.0;
track_parameters.alfa = 0.000000162179509;
track_parameters.beta = -0.000000271000819;

track_parameters.min_X = 0.0;
track_parameters.min_Y = 0.0;

track_parameters.max_X = 30;
track_parameters.max_Y = 20;

track_parameters.random = 0; % 0 - regular grid, 1 - random tracks
track_parameters.position_offset = 0.01; % used to move the tracks, so that measurements are never taken exactly at the border of the cell
track_parameters.space_period = 1.0; % distance between tracks for regular grid in meters
track_parameters.number_of_grid_repetitions = 13; % grid is created N times, if this is set to N (each time new header)

% track_parameters.mode = 'corridor';
% NOTE: corridors are set in a separate file
C = importdata('corridors.txt');

for i = 1:size(C, 1)
    track_parameters.corridors(i).x1 = C(i, 1);
    track_parameters.corridors(i).y1 = C(i, 2);
    track_parameters.corridors(i).x2 = C(i, 3);
    track_parameters.corridors(i).y2 = C(i, 4);
    track_parameters.corridors(i).w = C(i, 5);
end

track_parameters.mode = 'empty_space';
track_parameters.any_heading = 0; % if any_heading == 0, then heading can only be 0, 90, 180, or 270 degrees
                                  % if any_heading == 1, then truly random heading [0..360] degrees 
                                  
track_parameters.vel = 1.0; % m/s
track_parameters.sample_rate = 20;
track_parameters.gyr = [0 0 1];

% track_parameters.euler_angles = [0 0 0];
track_parameters.MIN_euler_angles = [-pi -pi/3 -pi];
track_parameters.MAX_euler_angles = [pi pi/3 pi];

track_parameters.random_tracks_number = 50; % only used if tracks are generated randomly

track_parameters.starting_index = 1;

tracks_folder = strcat(pwd, '\tracks\');

% ======================================================
number_of_scripts = generate_tracks(tracks_folder, track_parameters);
% ======================================================

number_of_scripts

output_folder = strcat(pwd, '\scripts\');

% number_of_scripts = 58;

% ======================================================
compose_scripts( output_folder, headers_folder, tracks_folder, number_of_scripts );
% ======================================================

'finished'



