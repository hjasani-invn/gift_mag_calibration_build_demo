function [ sensor_data ] = SaveRAMPoutput_for_Old_FPBL( fname, angle, track_data, sensor_data, user_model )

% NOTE: this function provides absolute values of coordinates, not dX, dY

sz = size(track_data.t, 2);
fout = fopen(fname, 'wt');

period = 1;

X = 0;
Y = 0;
Z = 0;
%q = angle2quat(angle, 0, 0);
q = quatconj(angle2quat(angle, 0, 0));

for i = 1 : period : sz
    X = track_data.r(i, 1);
    Y = track_data.r(i, 2);
    Z = track_data.r(i, 3);
    %     ??
    r_q = quatmultiply(q, track_data.q(i, 1:4)); % Body to local
    %     ??
    %rotated_XYZ = quatrotate(q, [dX dY dZ]);
    rotated_XYZ = quatmultiply(quatmultiply(q, [0 X Y Z]), quatconj(q));
    
    X = rotated_XYZ(2);
    Y = rotated_XYZ(3);
    Z = rotated_XYZ(4);
    
    C_PF_2_MFP = [0 -1 0;
                 1 0 0;
                 0 0 1];
             
    q_PF_2_MFP = quatconj( dcm2quat(C_PF_2_MFP) );
    r_q = quatmultiply( q_PF_2_MFP, r_q );
     
    q_cov = ones(4,4) * 0.0001;
    
    % ************************** 
    % WARNING - change when changes in model
    %misalignment_angle = rad2deg(0);

    floor = 0;
    height = 0;
    height_std = 0.0002;
    
    fprintf( fout, '%.0f, %.0f, %.10f, %.10f, ', track_data.t(i)*1000, 1, X, Y);
    
    fprintf( fout, '%.10f, %0.f, ', height, floor );

    track_data.X_uncertainty(i) = track_data.X_uncertainty(i) + 0.001;
    track_data.Y_uncertainty(i) = track_data.Y_uncertainty(i) + 0.001;
    XY_cov = 0.0001;
    YX_cov = 0.0001;

    fprintf( fout, '%.10f, %.10f, %.10f, %.10f, ',  track_data.X_uncertainty(i), XY_cov, YX_cov, track_data.Y_uncertainty(i));
    fprintf( fout, '%.10f, %.0f, ', height_std, 0 ); % floor_std is 0
    %fprintf( fout, ' %.10f, %.10f, ',  misalignment_angle, misalignment_std);
    fprintf( fout, '%.0f, ', 1 ); % attitude is valid
    fprintf( fout, '%.10f, %.10f, %.10f, %.10f, ', r_q(1), r_q(2), r_q(3), r_q(4) );
    
    fprintf( fout, '%.10f, %.10f, %.10f, %.10f, ', q_cov(1,1), q_cov(1,2), q_cov(1,3), q_cov(1,4) );
    fprintf( fout, '%.10f, %.10f, %.10f, %.10f, ', q_cov(2,1), q_cov(2,2), q_cov(2,3), q_cov(2,4) );
    fprintf( fout, '%.10f, %.10f, %.10f, %.10f, ', q_cov(3,1), q_cov(3,2), q_cov(3,3), q_cov(3,4) );
    fprintf( fout, '%.10f, %.10f, %.10f, %.10f, ', q_cov(4,1), q_cov(4,2), q_cov(4,3), q_cov(4,4) );

    fprintf( fout, '\n');
    
end
fclose (fout);

end

