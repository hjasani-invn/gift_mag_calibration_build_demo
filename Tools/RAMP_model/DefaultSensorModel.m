function sensor_model  = DefaultSensorModel ()

    %sensor acc
    sensor_model.acc.bias = [0, 0, 0];     
    sensor_model.acc.drift = [0, 0, 0];
    sensor_model.acc.scale = [1, 1, 1];
    %sensor_model.acc.sigma = [0, 0, 0];
    sensor_model.acc.sigma = [0.1, 0.1, 0.1]; 
    %sensor_model.acc.sigma = [0.03, 0.03, 0.03]; %real values from Nexus 5 (0.01-0.05)
    
    %sensor gyro
    sensor_model.gyro.bias = [0, 0, 0]; 
    sensor_model.gyro.drift = [0, 0, 0];
    sensor_model.gyro.scale = [1, 1, 1];
    %sensor_model.gyro.sigma = [0, 0, 0]; 
    sensor_model.gyro.sigma = [0.001, 0.001, 0.001]; 
    %sensor_model.gyro.sigma = [0.003, 0.003, 0.003]; % real values of calibrated gyro from Nexus 5 (0.001-0.005)

    %sensor magnetic
    sensor_model.mg.bias = [0, 0, 0]; 
    sensor_model.mg.drift = [0, 0, 0];
    sensor_model.mg.scale = [1, 1, 1];
    sensor_model.mg.sigma = [0, 0, 0]; 
    %sensor_model.mg.sigma = [0.75, 0.75, 0.75]; % real values of uncalibrated gyro from Nexus 5 (0.5-1.0)

end
