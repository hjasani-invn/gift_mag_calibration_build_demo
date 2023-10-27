function [ output_args ] = compose_scripts( output_folder, headers_folder, tracks_folder, N )
% composes scripts from first N tracks by adding headers to them, puts to output directory

mkdir(output_folder);
cd(output_folder);

for i = 1 : N
    str_i = num2str(i);
    track_i_folder = strcat('track', str_i);
    mkdir(track_i_folder);
    cd(track_i_folder);
    current_script_path = pwd;
    
    % current_header_path - full path to header number i
    current_header_path = strcat(headers_folder, str_i);
    current_header_path = strcat(current_header_path, '\');
    current_header_path = strcat(current_header_path, 'script');
    current_header_path = strcat(current_header_path, str_i);
    current_header_path = strcat(current_header_path, '.txt');
    
    % current_track_path - full path to track number i
    current_track_path = strcat(tracks_folder, str_i);
    current_track_path = strcat(current_track_path, '\');
    current_track_path = strcat(current_track_path, 'script');
    current_track_path = strcat(current_track_path, str_i);
    current_track_path = strcat(current_track_path, '.txt');
    
    copyfile(current_header_path, current_script_path);
    
    % current_script_file - full path to current script number i
    current_script_file = strcat(current_script_path, '\');
    current_script_file = strcat(current_script_file, 'script');
    current_script_file = strcat(current_script_file, str_i);
    current_script_file = strcat(current_script_file, '.txt');
    
    % adding contents of track file to script file
    fid = fopen(current_track_path, 'r');
    allData = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    
    number_of_strings = size(allData{1, 1}, 1);
    
    for j = 1 : number_of_strings
        fid = fopen( current_script_file, 'at' );
        fprintf( fid, '%s \n', allData{1, 1}{j, 1} );
        fclose(fid);
    end
    
    cd('../');
end

cd('../');


end

