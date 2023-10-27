function [ sensor_data ] = SaveRAMPoutput_IRL( fname, angle, track_data, sensor_data, user_model, environment_model )

% NOTE: this function provides absolute values of coordinates, not dX, dY

sz = size(track_data.t, 2);
fout = fopen(fname, 'wt');

period = 1;

X = 0;
Y = 0;
Z = 0;
%q = angle2quat(angle, 0, 0);
system_misalignment_q = quatconj(angle2quat(angle, 0, 0));

% DEBUG_DATA.MG_PF(1, 1:3) = [0; 0; 0];

for i = 1 : period : sz
    X = track_data.r(i, 1);
    Y = track_data.r(i, 2);
    Z = track_data.r(i, 3);
    
    microTesla_2_mGauss = 10;
    % converting to mGauss (and UDF)
    MG_x =  1 * sensor_data.mg_data(i, 2) * microTesla_2_mGauss;
    MG_y = -1 * sensor_data.mg_data(i, 3) * microTesla_2_mGauss;
    MG_z = -1 * sensor_data.mg_data(i, 4) * microTesla_2_mGauss;
    
%     MG_UDF = [MG_x; MG_y; MG_z];
%     MG_UDF_q = [0 MG_x MG_y MG_z];
    
    %     ??
    r_q = quatmultiply(system_misalignment_q, track_data.q(i, 1:4)); % Body to local (at this point body - model phone system, local - PF frame (Z is up))
    
%     r_q = track_data.q(i, 1:4);
    
%     saved_q = r_q;
    
    % new r_q must be UDF to NED
    
    C_udf_2_body = [1 0 0;
                    0 -1 0;
                    0 0 -1];
    C_PF_2_quasiNED = [1 0 0;
                   0 -1 0;
                   0 0 -1];  % PF is NWU (Z is up)
    
    az = deg2rad(environment_model.origin_azimuth);
              
    C_az = [cos(az) sin(az) 0;
            -sin(az) cos(az) 0;
            0 0 1];
              
    C_PF_2_NED = C_az * C_PF_2_quasiNED;
    
    % WARNING: C_PF_2_quasiNED matrix is C_PF_2_NED ONLY if PF X axis is aligned with North,
    % otherwise use azimuth angle
    
    % C_PF_2_NED = [cos(az) sin(az) 0;
    %               sin(az) -cos(az) 0;
    %               0 0 -1];  
    
%     C_PF_2_MFP = [0 -1 0;
%                 1 0 0;
%                 0 0 1];
    
%     q_C_body_2_PF = quat2dcm( quatconj(saved_q) ); % conj !!!
%     C_body_2_PF = track_data.C_B2ENU{i};
    
    % ------------------------------------------
    
%     C_UDF_2_NED = C_PF_2_NED * C_body_2_PF * C_udf_2_body; % C0 in document
%     
%     R = C_PF_2_MFP * C_PF_2_NED' * C_UDF_2_NED;
%     
%     sig_mx = 1; % microTesla
%     sig_my = 1; % microTesla
%     sig_mz = 1; % microTesla
%     
%     sig_mag_bias = 1; % microTesla
%     
%     sig_fi = track_data.roll_uncertainty(i);
%     sig_teta = track_data.pitch_uncertainty(i);
%     sig_psy = track_data.heading_uncertainty(i);
%     
%     MG_x = MG_x / microTesla_2_mGauss;
%     MG_y = MG_y / microTesla_2_mGauss;
%     MG_z = MG_z / microTesla_2_mGauss;
%    
%     P1(1, 1) = sig_psy^2 * MG_y^2 + sig_teta^2 * MG_z^2;    P1(1, 2) = -sig_psy^2 * MG_x * MG_y;    P1(1, 3) = -sig_teta^2 * MG_x * MG_z;
%     P1(2, 1) = -sig_psy^2 * MG_x * MG_y;    P1(2, 2) = sig_psy^2 * MG_x^2 + sig_fi^2 * MG_z^2;    P1(2, 3) = -sig_fi^2 * MG_y * MG_z;
%     P1(3, 1) = -sig_teta^2 * MG_x * MG_z;    P1(3, 2) = -sig_fi^2 * MG_y * MG_z;    P1(3, 3) = sig_fi^2 * MG_y^2 + sig_teta^2 * MG_x^2;
%     
%     P2 = [sig_mx^2 0 0;
%          0 sig_my^2 0;
%          0 0 sig_mz^2];
%     
%     P3 = eye(3, 3) * sig_mag_bias^2;
%            
%     cov_MFP = (R * (P1 + P2 + P3) * R');
%     
%     MG_x = MG_x * microTesla_2_mGauss;
%     MG_y = MG_y * microTesla_2_mGauss;
%     MG_z = MG_z * microTesla_2_mGauss;
    
    % ------------------------------------------
                      
    q_udf_2_body = quatconj( dcm2quat(C_udf_2_body) );
%     q_PF_2_NED = quatconj( dcm2quat(C_PF_2_quasiNED) );
    q_PF_2_NED = quatconj( dcm2quat(C_PF_2_NED) );
    
    r_q = quatmultiply( q_PF_2_NED, quatmultiply(r_q, q_udf_2_body) );
    
    C_UDF_2_NED_from_q = TPN_quat2dcm( r_q );
    
    %     ??
    %rotated_XYZ = quatrotate(q, [dX dY dZ]);
%     rotated_XYZ = quatmultiply(quatmultiply(system_misalignment_q, [0 X Y Z]), quatconj(system_misalignment_q));
%     
%     X = rotated_XYZ(2);
%     Y = rotated_XYZ(3);
%     Z = rotated_XYZ(4);
    
%     q_cov = ones(4,4) * 0.000001;
    
    geo = convertXYZ_to_GEO( X, Y, Z, environment_model );
    
%     [roll, pitch, heading] = quat2angle( r_q, 'XYZ' );  % WARNING - test if this is correct
%     [test_roll, test_pitch, test_heading] = TPN_dcm2angle( C_UDF_2_NED );
    
    % adding internal frame switching 

    [roll, pitch, heading] = TPN_dcm2angle( C_UDF_2_NED_from_q );
  
    roll = rad2deg(roll);
    pitch = rad2deg(pitch);
    heading = rad2deg(heading); % heading is already rotated by azimuth here
    internal_device_frame_flag = 0;
    
    % adding logic to check pitch, if it is bad - change frame and angles
    % NOTE - for now only vertical up frame is supported 
    if ( abs( abs(pitch) - 90 ) < 20 )
        C_UDF_2_IF = [0 0 1;
                    0 1 0;
                    -1 0 0];

        C_IF_2_NED = C_UDF_2_NED_from_q * C_UDF_2_IF';
        [roll, pitch, heading] = TPN_dcm2angle( C_IF_2_NED );
        roll = rad2deg(roll);
        pitch = rad2deg(pitch);
        heading = rad2deg(heading); % heading is already rotated by azimuth here
        internal_device_frame_flag = 1;
    end
            
%     [roll1, pitch1, heading1] = TPN_dcm2angle( C_UDF_2_NED );
%   
%     roll1 = rad2deg(roll1);
%     pitch1 = rad2deg(pitch1);
%     heading1 = rad2deg(heading1);
        
    % ************************** 
    % WARNING - change when changes in model
    %misalignment_angle = rad2deg(0);
    user_heading = rad2deg(0);
    %misalignment_std = 0.005;
%     user_heading_std = 0.005;
%     roll_std = 0.03;
%     pitch_std = 0.03;
%     heading_std = 0.03;
    
    floor = 0;
    height = 0;
    height_std = 0.0002;
    fidgeting_flag = 0;
    navigation_phase = 1;
    
    step_length = 0.5;
    
    if ( norm(track_data.v(i, :)) < 0.001 )
        step_length = 0.0;
    end
  
    
%     MG_PF = C_body_2_PF * C_udf_2_body * MG_UDF;
    
%     MG_PF_q = quatmultiply( quatmultiply( r_q , MG_UDF_q ), quatconj(r_q) );
    
%     DEBUG_DATA.MG_PF(i, 1:3) = MG_PF;
%     DEBUG_DATA.MG_PF_q(i, 1:3) = MG_PF_q(2:4);
    
    misalignment = 0;
    % ************************** 
    
%     fprintf( fout, '%.0f, %.10f, %.10f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.0f, %.5f, %.5f, %.5f, %.0f, %.5f, %.5f \n', track_data.t(i), geo.lat, geo.lon, user_model.state.covXY(1,1), user_model.state.covXY(2,2), misalignment_angle, misalignment_std, roll, pitch, heading, roll_std, pitch_std, heading_std, internal_device_frame_flag, sensor_data.mg_data(i, 2), sensor_data.mg_data(i, 3), sensor_data.mg_data(i, 4), floor, height, height_std);
    fprintf( fout, '%.0f,  %.10f, %.10f,', track_data.t(i)*1000, geo.lat, geo.lon);
    
%     fprintf( fout, ' %.5f, %.5f, ',  user_model.state.covXY(1,1), user_model.state.covXY(2,2));
    fprintf( fout, ' %.5f, %.5f,',  track_data.X_uncertainty(i), track_data.Y_uncertainty(i));
    
    %fprintf( fout, ' %.5f, %.5f, ',  misalignment_angle, misalignment_std);
    fprintf( fout, ' %.0f, %.5f, %.5f ,', floor, height, height_std);
    
    % !!!! WARNING - Add this later, removed now for debugging !!!!!
    fprintf( fout, ' %.5f,', misalignment); 
    % !!!!!!!!!!!!!!!!!

	fprintf( fout, ' %.5f, %.5f,',  user_heading, user_model.heading_std);  % user_heading - angle between device heading and user velocity vector?
    fprintf( fout, ' %.10f, %.10f, %.10f, %.10f, %.10f, %.10f,', roll, pitch, heading, rad2deg(track_data.roll_uncertainty(i)), rad2deg(track_data.pitch_uncertainty(i)), rad2deg(track_data.heading_uncertainty(i)));
    fprintf( fout, ' %.0f, %.5f, %.5f, %.5f,', internal_device_frame_flag, MG_x, MG_y, MG_z);
    fprintf( fout, ' %.0f,', navigation_phase);
    fprintf( fout, ' %.0f,', fidgeting_flag);
    fprintf( fout, ' %.5f', step_length);
    fprintf( fout, '\n');
    
    % total amount : 24 (24th is step_length)
end
fclose (fout);


end


function [ geo ] = convertXYZ_to_GEO( x, y, z, environment_model )

lat0 = deg2rad( environment_model.origin_lattitude );
lon0 = deg2rad( environment_model.origin_longitude );

azimuth = deg2rad(environment_model.origin_azimuth);

h0 = 0;

alfa_lat = environment_model.alfa;
alfa_lon = environment_model.beta;


geo.lat = lat0 + alfa_lat * ( x * cos(azimuth) - y * sin(azimuth) );
geo.lon = lon0 + alfa_lon * ( x * sin(azimuth) + y * cos(azimuth) );
geo.h = 0;  % height 

geo.lat = rad2deg(geo.lat);
geo.lon = rad2deg(geo.lon);

end
