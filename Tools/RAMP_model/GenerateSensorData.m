function [sensor_data] = GenerateSensorData(track_data, sensor_model, env_model, ref_flag)

    addpath('quat_lib\');
    addpath('mfp_map\');
    
    if ref_flag == 0
        ref_flag = 0;
    else
        ref_flag = 1;
    end
    
    %MEAN_G = [0, 0, 9.81];
    %MEAN_MG = [10, 0, 35];
    
    sz = max(size(track_data.sys_t));
    
    sensor_data.acc_data(sz,4) = 0;
    sensor_data.gyr_data(sz,4) = 0;
    sensor_data.mg_data(sz,4) = 0;
    
    % get magnetic meassurements
    pos_h = track_data.r(:, 1:2);
    pos_h = [pos_h zeros(size(pos_h,1), 1)];
    switch (env_model.mg.model_type)
        case 'const'
            mg_value(size(pos_h,1),:) = env_model.mg.mean_mg; 
            for i=1: size(pos_h,1)
                mg_value(i,:) = env_model.mg.mean_mg; 
            end
        case 'fp_interpolated'    
            mg_value = get_meas(pos_h, env_model.mg.map, env_model.mg.cell_size); % mg_meas has magnetometer measurements for each [X,Y] in ENU (???)
        case 'fp_fixed'
            mg_value = get_meas_fp(pos_h, env_model.mg.map, env_model.mg.cell_size); % mg_meas has magnetometer measurements for each [X,Y] in ENU (???)
        case 'fp_fixed_with_noise'        
            mg_value = get_meas_fp_with_noise(pos_h, env_model.mg.map, env_model.mg.map_sigma, env_model.mg.cell_size); % mg_meas has magnetometer measurements for each [X,Y] in ENU (???)
        otherwise
            mg_value  = zeros(size(pos_h,1),3);
    end

        
    for i = 1:sz
        %fi = track_data.ang(i,1);
        %teta = track_data.ang(i,2);
        %psy = track_data.ang(i,3);
        %CL2B = angle2dcm(psy, teta, fi, 'ZYX'); % local to body  
        %sensor_data.acc_data(i,:) = GetSensorMeasurement(track_data.t(1),track_data.t(i),track_data.a(i,:)+MEAN_G, CL2B, sensor_model.acc, ref_flag);
        %sensor_data.gyr_data(i,:) = GetSensorMeasurementFromBF(track_data.t(1),track_data.t(i),track_data.w(i,:),  sensor_model.gyro, ref_flag);
        %sensor_data.mg_data(i,:) = GetSensorMeasurement(track_data.t(1),track_data.t(i),MEAN_MG, CL2B, sensor_model.mg, ref_flag);
        %sensor_data.mg_bias(i,:) = GetSensorBias(track_data.t(1),track_data.t(i),MEAN_MG, CL2B, sensor_model.mg, ref_flag);
        
        %sensor_data.acc_data(i,:) = GetSensorMeasurement_Q(track_data.t(1),track_data.t(i),track_data.a(i,:)+env_model.gp.mean_g, quatconj(quatnormalize(track_data.q(i,:))), sensor_model.acc, ref_flag);
        sensor_data.acc_data(i,:) = GetSensorMeasurement_Q(track_data.t(1), track_data.t(i), track_data.a(i, :)+env_model.gp.mean_g, quatconj(quatnormalize(track_data.q(i,:))), sensor_model.acc, ref_flag);
        sensor_data.gyr_data(i,:) = GetSensorMeasurementFromBF(track_data.t(1), track_data.t(i), track_data.w(i, :), sensor_model.gyro, ref_flag);
        
        % Note: only MG data is used for IRL format output
        sensor_data.mg_data(i,:) = GetSensorMeasurement_Q(track_data.t(1), track_data.t(i), mg_value(i, :), quatconj(quatnormalize(track_data.q(i, :))), sensor_model.mg, ref_flag);
        sensor_data.mg_bias(i,:) = GetSensorBias(track_data.t(1), track_data.t(i), sensor_model.mg, ref_flag);
        
        sensor_data.acc_data_DCM(i, :) = GetSensorMeasurement(track_data.t(1), track_data.t(i), track_data.a(i, :)+env_model.gp.mean_g, track_data.C_B2ENU{i}', sensor_model.acc, ref_flag);
        sensor_data.mg_data_DCM(i, :) = GetSensorMeasurement(track_data.t(1), track_data.t(i), mg_value(i, :), track_data.C_B2ENU{i}', sensor_model.mg, ref_flag);
        
    end
    
end


function [meas_data] = GetSensorMeasurement(t0, t, real_vector, CL2B, sensor, ref_flag)
    meas_vector = CL2B*real_vector';
    if (ref_flag)
        C(1,1) = sensor.scale(1);   C(1,2) = 0;                 C(1,3) = 0;
        C(2,1) = 0;                 C(2,2) = sensor.scale(2);   C(2,3) = 0;
        C(3,1) = 0;                 C(3,2) = 0;                 C(3,3) = sensor.scale(3);
        noise = sensor.sigma .* randn(1,3);
        meas_vector = C * meas_vector + sensor.bias' + sensor.drift' * (t-t0) + noise';
    end
    meas_data = [t,meas_vector'];
end

function [meas_data] = GetSensorMeasurement_Q(t0, t, real_vector, q_L2B, sensor, ref_flag)
    %meas_vector = CL2B*real_vector';
    q_vec = [0 real_vector(1) real_vector(2) real_vector(3)];
    q_vec  = quatmultiply(q_L2B, q_vec);
    q_vec = quatmultiply(q_vec, quatconj(q_L2B));
    meas_vector = [q_vec(2), q_vec(3), q_vec(4)]';
    if (ref_flag)
        C(1,1) = sensor.scale(1);   C(1,2) = 0;                 C(1,3) = 0;
        C(2,1) = 0;                 C(2,2) = sensor.scale(2);   C(2,3) = 0;
        C(3,1) = 0;                 C(3,2) = 0;                 C(3,3) = sensor.scale(3);
        noise = sensor.sigma .* randn(1,3);
        meas_vector = C * meas_vector + sensor.bias' + sensor.drift' * (t-t0) + noise';
    end
    meas_data = [t,meas_vector'];
end

function [meas_data] = GetSensorMeasurementFromBF(t0, t, real_vector, sensor, ref_flag)
    meas_vector = real_vector';
    if (ref_flag)
        C(1,1) = sensor.scale(1);   C(1,2) = 0;                 C(1,3) = 0;
        C(2,1) = 0;                 C(2,2) = sensor.scale(2);   C(2,3) = 0;
        C(3,1) = 0;                 C(3,2) = 0;                 C(3,3) = sensor.scale(3);
        noise = sensor.sigma .* randn(1,3);
        meas_vector = C * meas_vector + sensor.bias' + sensor.drift' * (t-t0) + noise';
    end
    meas_data = [t,meas_vector'];
end


function [meas_data] = GetSensorBias(t0, t, sensor, ref_flag)
    %real_vector, CL2B - reserved for soft iron
    meas_vector = [0, 0, 0]';
    if (ref_flag)
        C(1,1) = sensor.scale(1);   C(1,2) = 0;                 C(1,3) = 0;
        C(2,1) = 0;                 C(2,2) = sensor.scale(2);   C(2,3) = 0;
        C(3,1) = 0;                 C(3,2) = 0;                 C(3,3) = sensor.scale(3);
        meas_vector = C * meas_vector + sensor.bias' + sensor.drift' * (t-t0);
    end
    meas_data = [t,meas_vector'];
end
