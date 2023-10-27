function [ Number_of_tracks ] = generate_tracks( output_folder, parameters )

if ( parameters.min_X >= parameters.max_X ) || ( parameters.min_Y >= parameters.max_Y )
    'Error: check min and max X and Y' %#ok<NOPRT>
    return;
end

mkdir(output_folder);
cd(output_folder);

if (parameters.random == 0)
    k = parameters.starting_index - 1;
    % generate a regular grid of tracks across whole area
    % "vertical" tracks (X is set, Y from 0 to max)
    
    for j = 1 : parameters.number_of_grid_repetitions
        
        N = floor( (parameters.max_X - parameters.min_X) / parameters.space_period ) - 1;
        for i = 0 : N
            k = k + 1;
            str_k = num2str(k);
            mkdir(str_k);
            cd(str_k);
            script = strcat('script', str_k);
            script = strcat(script, '.txt');
            fid = fopen(script, 'wt');
            
            t = (parameters.max_Y - parameters.position_offset - parameters.min_Y) / parameters.vel;
            
            X = i * parameters.space_period + parameters.position_offset;
            Y = parameters.position_offset;
            
            fprintf( fid, 'set_origin_lattitude %.15f \n', parameters.origin_lattitude );
            fprintf( fid, 'set_origin_longitude %.15f \n', parameters.origin_longitude );
            fprintf( fid, 'set_origin_azimuth %.15f \n', parameters.origin_azimuth );
            fprintf( fid, 'set_alfa %.15f \n', parameters.alfa );
            fprintf( fid, 'set_beta %.15f \n', parameters.beta );
            
            fprintf( fid, 'set_t0 %f \n', 0.0 );
            fprintf( fid, 'set_tend %f \n', t );
            fprintf( fid, 'set_sample_rate %f \n', parameters.sample_rate );
            fprintf( fid, 'set_pos %f %f %f %f \n', 0.0, X, Y, 0.0 );
            
            device_roll = parameters.MIN_euler_angles(1) + ( parameters.MAX_euler_angles(1) - parameters.MIN_euler_angles(1) ) * rand;
            device_pitch = parameters.MIN_euler_angles(2) + ( parameters.MAX_euler_angles(2) - parameters.MIN_euler_angles(2) ) * rand;
            device_heading = parameters.MIN_euler_angles(3) + ( parameters.MAX_euler_angles(3) - parameters.MIN_euler_angles(3) ) * rand;
            
            fprintf( fid, 'set_euler_angles %f %f %f %f \n', 0.0, device_roll, device_pitch, device_heading );
            
            fprintf( fid, 'set_gyr %f %f %f %f \n', 0.0, parameters.gyr(1), parameters.gyr(2), parameters.gyr(3) );
            fprintf( fid, 'set_vel %f %f %f %f \n', 0.0, 0, parameters.vel, 0 );
            fprintf( fid, 'set_gyr %f %f %f %f \n', t, 0, 0, 0 );
            fprintf( fid, 'set_vel %f %f %f %f \n', t, 0, 0, 0 );
            fclose( fid );
            cd('../');
        end
        % "horizontal" tracks (Y is set, X from 0 to max)
        M = floor( (parameters.max_Y - parameters.min_Y) / parameters.space_period ) - 1;
        for i = 0 : M
            k = k + 1;
            str_k = num2str(k);
            mkdir(str_k);
            cd(str_k);
            script = strcat('script', str_k);
            script = strcat(script, '.txt');
            fid = fopen(script, 'wt');
            
            t = (parameters.max_X - parameters.position_offset - parameters.min_X) / parameters.vel;
            
            X = parameters.position_offset;
            Y = i * parameters.space_period + parameters.position_offset;
            
            fprintf( fid, 'set_origin_lattitude %.15f \n', parameters.origin_lattitude );
            fprintf( fid, 'set_origin_longitude %.15f \n', parameters.origin_longitude );
            fprintf( fid, 'set_origin_azimuth %.15f \n', parameters.origin_azimuth );
            fprintf( fid, 'set_alfa %.15f \n', parameters.alfa );
            fprintf( fid, 'set_beta %.15f \n', parameters.beta );
            
            fprintf( fid, 'set_t0 %f \n', 0.0 );
            fprintf( fid, 'set_tend %f \n', t );
            fprintf( fid, 'set_sample_rate %f \n', parameters.sample_rate );
            fprintf( fid, 'set_pos %f %f %f %f \n', 0.0, X, Y, 0.0 );
            
            device_roll = parameters.MIN_euler_angles(1) + ( parameters.MAX_euler_angles(1) - parameters.MIN_euler_angles(1) ) * rand;
            device_pitch = parameters.MIN_euler_angles(2) + ( parameters.MAX_euler_angles(2) - parameters.MIN_euler_angles(2) ) * rand;
            device_heading = parameters.MIN_euler_angles(3) + ( parameters.MAX_euler_angles(3) - parameters.MIN_euler_angles(3) ) * rand;
            
            fprintf( fid, 'set_euler_angles %f %f %f %f \n', 0.0, device_roll, device_pitch, device_heading );
            
            fprintf( fid, 'set_gyr %f %f %f %f \n', 0.0, parameters.gyr(1), parameters.gyr(2), parameters.gyr(3) );
            fprintf( fid, 'set_vel %f %f %f %f \n', 0.0, parameters.vel, 0, 0 );
            fprintf( fid, 'set_gyr %f %f %f %f \n', t, 0, 0, 0 );
            fprintf( fid, 'set_vel %f %f %f %f \n', t, 0, 0, 0 );
            fclose( fid );
            cd('../');
        end
        
    end
    
    Number_of_tracks = k;
end

% *************************************************************************

if (parameters.random == 1)
    N = parameters.random_tracks_number;
    
    % we need N tracks
    for i = parameters.starting_index : N
        
        if ( strcmp(parameters.mode, 'empty_space') )
            [startX, startY, track_length_updated, heading] = generate_random_track( parameters );
        elseif ( strcmp(parameters.mode, 'corridor') )
            [startX, startY, track_length_updated, heading] = generate_corridor_track( parameters );
        else
            'Error: check track_parameters.mode' %#ok<NOPRT>
            return;
        end
        
        % plotting
        endX = startX + track_length_updated * cos(heading);
        endY = startY + track_length_updated * sin(heading);
        
        plot([startX endX], [startY endY]);
        hold on
         
        t = track_length_updated / parameters.vel;
        % write successful track to the next script file
        str_i = num2str(i);
        mkdir(str_i);
        cd(str_i);
        % generate script in current folder ("1", "2", etc folder inside the "scripts" folder)
        % script = 'script'+str_i+'.txt';
        script = strcat('script', str_i);
        script = strcat(script, '.txt');
        fid = fopen(script, 'wt');
        
        fprintf( fid, 'set_origin_lattitude %.15f \n', parameters.origin_lattitude );
        fprintf( fid, 'set_origin_longitude %.15f \n', parameters.origin_longitude );
        fprintf( fid, 'set_origin_azimuth %.15f \n', parameters.origin_azimuth );
        fprintf( fid, 'set_alfa %.15f \n', parameters.alfa );
        fprintf( fid, 'set_beta %.15f \n', parameters.beta );
        
        fprintf( fid, 'set_t0 %f \n', 0.0 );
        fprintf( fid, 'set_tend %f \n', t );
        fprintf( fid, 'set_sample_rate %f \n', parameters.sample_rate );
        fprintf( fid, 'set_pos %f %f %f %f \n', 0.0, startX, startY, 0.0 );
        
        device_roll = parameters.MIN_euler_angles(1) + ( parameters.MAX_euler_angles(1) - parameters.MIN_euler_angles(1) ) * rand;
        device_pitch = parameters.MIN_euler_angles(2) + ( parameters.MAX_euler_angles(2) - parameters.MIN_euler_angles(2) ) * rand;
        device_heading = parameters.MIN_euler_angles(3) + ( parameters.MAX_euler_angles(3) - parameters.MIN_euler_angles(3) ) * rand;
        
        fprintf( fid, 'set_euler_angles %f %f %f %f \n', 0.0, device_roll, device_pitch, device_heading );
        
        fprintf( fid, 'set_gyr %f %f %f %f \n', 0.0, parameters.gyr(1), parameters.gyr(2), parameters.gyr(3) );
        fprintf( fid, 'set_gyr %f %f %f %f \n', t, 0, 0, 0 );
        
        v_x = parameters.vel * cos(heading);
        v_y = parameters.vel * sin(heading);
        fprintf( fid, 'set_vel %f %f %f %f \n', 0.0, v_x, v_y, 0 );
        fprintf( fid, 'set_vel %f %f %f %f \n', t, 0, 0, 0 );
               
        fclose( fid );
        cd('../');
    end
    Number_of_tracks = N;
end


cd('../');

end

function [startX, startY, track_length_updated, heading] = generate_random_track( parameters )
% generates one track randomly, provided that its length is Ok and it fits the map

min_track_length = 0.25 * min( ( parameters.max_X - parameters.min_X ), ( parameters.max_Y - parameters.min_Y ) );
max_track_length = 0.5 * max( ( parameters.max_X - parameters.min_X ), ( parameters.max_Y - parameters.min_Y ) );

% only take the track if it fits the map (success == 1)
success = 0;
while ( success == 0 )
    % generate a random starting point
    startX = parameters.min_X + ( parameters.max_X - parameters.min_X) * rand();
    startY = parameters.min_Y + ( parameters.max_Y - parameters.min_Y) * rand();
    
    number_of_tries = 0;
    while ( ( success == 0 ) && ( number_of_tries < 10 ) )
        
        if ( parameters.any_heading == 1 )
            heading = 2 * pi * rand();
        else
            possible_headings = [0; pi/2; pi; (3/2)*pi];
            heading = possible_headings(randi(numel(possible_headings)));
        end
        
        track_length = min_track_length + ( max_track_length - min_track_length ) * rand();
        
        [success, track_length_updated] = border_check( startX, startY, track_length, min_track_length, heading, parameters );
        number_of_tries = number_of_tries + 1;
    end
end

end

function [startX, startY, track_length_updated, heading] = generate_corridor_track( parameters )
% generates one track randomly, provided that it begins at one end
% of the corridor and ends at its other end
% corridor width limits the track scatter

idx = randi(numel( parameters.corridors ));
heading = atan2( parameters.corridors(idx).y2 - parameters.corridors(idx).y1, parameters.corridors(idx).x2 - parameters.corridors(idx).x1 );
dr = parameters.corridors(idx).w * (0.5 - rand());
dr_x = cos( heading + pi/2 ) * dr;
dr_y = -sin( heading + pi/2 ) * dr;

startX = parameters.corridors(idx).x1 + dr_x;
startY = parameters.corridors(idx).y1 + dr_y;
endX = parameters.corridors(idx).x2 + dr_x;
endY = parameters.corridors(idx).y2 + dr_y;

track_length_updated = sqrt( (endY - startY)^2 + (endX - startX)^2 );


end

function [ result, track_length_updated ] = border_check( startX, startY, track_length, min_track_length, heading, parameters )
% checking that track fits the map - if it crosses borders, track length is adjusted
% then if length is >= minimal, then track is Ok, else - failed

result = 0;
v_x = parameters.vel * cos(heading);
v_y = parameters.vel * sin(heading);

endX = startX + track_length * cos(heading);
endY = startY + track_length * sin(heading);
track_length_updated = track_length;

if ( endX > parameters.max_X )
    endX = parameters.max_X;
    track_length_updated = (endX - startX) / v_x;
    endY = startY + track_length_updated * sin(heading);
end

if ( endX < parameters.min_X )
    endX = parameters.min_X;
    track_length_updated = (endX - startX) / v_x;
    endY = startY + track_length_updated * sin(heading);
end

if ( endY > parameters.max_Y )
    endY = parameters.max_Y;
    track_length_updated = (endY - startY) / v_y;
    endX = startX + track_length_updated * cos(heading);
end

if ( endY < parameters.min_Y )
    endY = parameters.min_Y;
    track_length_updated = (endY - startY) / v_y;
    endX = startX + track_length_updated * cos(heading);
end

if ( track_length_updated < min_track_length )
    result = 0;
else
    result = 1;
end


end


