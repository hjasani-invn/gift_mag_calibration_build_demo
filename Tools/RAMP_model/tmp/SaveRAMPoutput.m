function [ sensor_data ] = SaveRAMPoutput( fname, angle, track_data, sensor_data, user_model )

sz = size(track_data.t, 2);
fout = fopen(fname, 'wt');

period = 1;

dX = 0;
dY = 0;
dZ = 0;
X_prev = track_data.r(1, 1);
Y_prev = track_data.r(1, 2);
Z_prev = track_data.r(1, 3);
% q = angle2quat(angle, 0, 0, 'ZYX');
q = angle2quat(angle, 0, 0);

for i = 1 : period : sz
    if (i > 1)
        dX = track_data.r(i, 1) - X_prev;
        dY = track_data.r(i, 2) - Y_prev;
        dZ = track_data.r(i, 3) - Z_prev;
        
        X_prev = track_data.r(i, 1);
        Y_prev = track_data.r(i, 2);
        Z_prev = track_data.r(i, 3);
    end
    
    %     ??
    r_q = quatmultiply(quatconj(q), track_data.q(i, 1:4));
    %     ??
    rotated_XYZ = quatrotate(q, [dX dY dZ]);
    
    dX = rotated_XYZ(1);
    dY = rotated_XYZ(2);
    dZ = rotated_XYZ(3);
    
%     fprintf( fout, '%.0f %.6f %.6f %.10f %.10f %.10f %.10f %.5f %.5f %.5f \n', track_data.t(i)*1000, dX, dY, r_q(1), r_q(2), r_q(3), r_q(4), sensor_data.mg_data(i, 2), sensor_data.mg_data(i, 3), sensor_data.mg_data(i, 4));
%     fprintf( fout, '%.0f, %.6f, %.6f, %.6f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.5f, %.5f, %.5f \n',  track_data.t(i)*1000, dX, dY, dZ, user_model.state.covXY(1,1), user_model.state.covXY(1,2), user_model.state.covXY(2,1), user_model.state.covXY(2,2), r_q(1), r_q(2), r_q(3), r_q(4), sensor_data.mg_data(i, 2), sensor_data.mg_data(i, 3), sensor_data.mg_data(i, 4));
    %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f,
    
    q_cov = ones(4,4) * 0.000001;
     
    fprintf( fout, '%.0f, %.6f, %.6f, %.6f, %.5f, %.5f, %.5f, %.5f, %.10f, %.10f, %.10f, %.10f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %d, %.5f, %.5f, %.5f \n',  track_data.t(i)*1000, dX, dY, dZ, user_model.state.covXY(1,1), user_model.state.covXY(1,2), user_model.state.covXY(2,1), user_model.state.covXY(2,2), r_q(1), r_q(2), r_q(3), r_q(4), q_cov(1,1), q_cov(1,2), q_cov(1,3), q_cov(1,4), q_cov(2,1), q_cov(2,2), q_cov(2,3), q_cov(2,4), q_cov(3,1), q_cov(3,2), q_cov(3,3), q_cov(3,4), q_cov(4,1), q_cov(4,2), q_cov(4,3), q_cov(4,4), 1, sensor_data.mg_data(i, 2), sensor_data.mg_data(i, 3), sensor_data.mg_data(i, 4));
    
end
fclose (fout);


end
