function [ output_args ] = generate_headers( output_folder, parameters )

mkdir(output_folder);
cd(output_folder);

N = parameters.headers_number;

for i = 1 : N
    str_i = num2str(i);
    mkdir(str_i);
    cd(str_i);
    % generate script in current folder ("1", "2", etc folder inside the "scripts" folder)
    % script = 'script'+str_i+'.txt';
    script = strcat('script', str_i);
    script = strcat(script, '.txt');
    fid = fopen(script, 'wt');
    
    mg_bias(1) = randn() * parameters.std_mg_bias;
    mg_bias(2) = randn() * parameters.std_mg_bias;
    mg_bias(3) = randn() * parameters.std_mg_bias;
    
    XY_error(1) = parameters.min_XY_error(1) + (parameters.max_XY_error(1) - parameters.min_XY_error(1)) * rand();
    XY_error(2) = parameters.min_XY_error(2) + (parameters.max_XY_error(2) - parameters.min_XY_error(2)) * rand();
    XY_error_angle = rand() * 2 * pi;
    R_error = randn() * parameters.std_XY_error;
    XY_error(1) = XY_error(1) + R_error * cos(XY_error_angle);
    XY_error(2) = XY_error(2) + R_error * sin(XY_error_angle);
    
    init_heading_error = parameters.min_init_heading_error + (parameters.max_init_heading_error - parameters.min_init_heading_error) * rand();
    
    heading_std = parameters.min_heading_std + (parameters.max_heading_std - parameters.min_heading_std) * rand();
    
    angles_std(1) = parameters.min_angles_std(1) + (parameters.max_angles_std(1) - parameters.min_angles_std(1)) * rand();
    angles_std(2) = parameters.min_angles_std(2) + (parameters.max_angles_std(2) - parameters.min_angles_std(2)) * rand();
    angles_std(3) = parameters.min_angles_std(3) + (parameters.max_angles_std(3) - parameters.min_angles_std(3)) * rand();
    
    heading_drift = parameters.min_heading_drift + (parameters.max_heading_drift - parameters.min_heading_drift) * rand();
    
    init_angles_error(1) = parameters.min_init_angles_error(1) + (parameters.max_init_angles_error(1) - parameters.min_init_angles_error(1)) * rand();
    init_angles_error(2) = parameters.min_init_angles_error(2) + (parameters.max_init_angles_error(2) - parameters.min_init_angles_error(2)) * rand();
    init_angles_error(3) = parameters.min_init_angles_error(3) + (parameters.max_init_angles_error(3) - parameters.min_init_angles_error(3)) * rand();
    
    angles_drift(1) = parameters.min_angles_drift(1) + (parameters.max_angles_drift(1) - parameters.min_angles_drift(1)) * rand();
    angles_drift(2) = parameters.min_angles_drift(2) + (parameters.max_angles_drift(2) - parameters.min_angles_drift(2)) * rand();
    angles_drift(3) = parameters.min_angles_drift(3) + (parameters.max_angles_drift(3) - parameters.min_angles_drift(3)) * rand();
        
    fprintf( fid, 'set_mg_bias %f %f %f \n', mg_bias(1), mg_bias(2), mg_bias(3) );
    fprintf( fid, 'set_init_XY_error %f %f \n', XY_error(1), XY_error(2) );
    fprintf( fid, 'set_init_heading_error %f \n', init_heading_error );
    fprintf( fid, 'set_heading_drift %f \n', heading_drift );
    fprintf( fid, 'set_heading_std %f \n', heading_std );
    fprintf( fid, 'set_init_angles_error %f %f %f \n', init_angles_error(1), init_angles_error(2), init_angles_error(3) );
    fprintf( fid, 'set_angles_drift %f %f %f \n', angles_drift(1), angles_drift(2), angles_drift(3) );
    fprintf( fid, 'set_angles_std %f %f %f \n', angles_std(1), angles_std(2), angles_std(3) );
    
    fclose( fid );
    cd('../');
end
cd('../');
end
