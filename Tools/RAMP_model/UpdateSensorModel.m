function sensor_model  = UpdateSensorModel(track_script, sensor_model)

    cmd_count = size(track_script, 2);
    for i = 1:cmd_count
        [sensor_model ] = ParseSensorCmd(track_script{i}, sensor_model);
    end
end

function [sensor_model, is_applied] = ParseSensorCmd(cmd_line, sensor_model)
    
    if (~isempty(cmd_line) && ischar(cmd_line))
        cmd = sscanf( cmd_line, '%s', 1);
    else
        cmd = 'rem';
    end
    is_applied = true;
    switch (cmd)
        case 'set_acc_bias'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.acc.bias(1) = data(1);
                    sensor_model.acc.bias(2) = data(2);
                    sensor_model.acc.bias(3) = data(3);
            end
         case 'set_acc_drift'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.acc.drift(1) = data(1);
                    sensor_model.acc.drift(2) = data(2);
                    sensor_model.acc.drift(3) = data(3);
            end
         case 'set_acc_scale'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.acc.scale(1) = data(1);
                    sensor_model.acc.scale(2) = data(2);
                    sensor_model.acc.scale(3) = data(3);
            end
         case 'set_acc_sigma'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.acc.sigma(1) = data(1);
                    sensor_model.acc.sigma(2) = data(2);
                    sensor_model.acc.sigma(3) = data(3);
            end
        case 'set_gyro_bias'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.gyro.bias(1) = data(1);
                    sensor_model.gyro.bias(2) = data(2);
                    sensor_model.gyro.bias(3) = data(3);
            end
         case 'set_gyro_drift'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.gyro.drift(1) = data(1);
                    sensor_model.gyro.drift(2) = data(2);
                    sensor_model.gyro.drift(3) = data(3);
            end
         case 'set_gyro_scale'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.gyro.scale(1) = data(1);
                    sensor_model.gyro.scale(2) = data(2);
                    sensor_model.gyro.scale(3) = data(3);
            end
         case 'set_gyro_sigma'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.gyro.sigma(1) = data(1);
                    sensor_model.gyro.sigma(2) = data(2);
                    sensor_model.gyro.sigma(3) = data(3);
            end
        case 'set_mg_bias'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.mg.bias(1) = data(1);
                    sensor_model.mg.bias(2) = data(2);
                    sensor_model.mg.bias(3) = data(3);
            end
         case 'set_mg_drift'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.mg.drift(1) = data(1);
                    sensor_model.mg.drift(2) = data(2);
                    sensor_model.mg.drift(3) = data(3);
            end
         case 'set_mg_scale'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.mg.scale(1) = data(1);
                    sensor_model.mg.scale(2) = data(2);
                    sensor_model.mg.scale(3) = data(3);
            end
         case 'set_mg_sigma'
            [data, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
            if (cnt == 3)
                    sensor_model.mg.sigma(1) = data(1);
                    sensor_model.mg.sigma(2) = data(2);
                    sensor_model.mg.sigma(3) = data(3);
            end
        otherwise 
            is_applied = false;
    end
    
end
