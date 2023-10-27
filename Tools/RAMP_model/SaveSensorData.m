function SaveSensorData ( path, group_name, sensor_data)
    
    facc = strcat(path, '\acc_', group_name, '.', 'log');
    SaveData(facc, sensor_data.acc_data);
    
    fgyr = strcat(path, '\gyr_', group_name, '.', 'log');
    SaveData(fgyr, sensor_data.gyr_data);
    
    fmg = strcat(path, '\mgraw_', group_name, '.', 'log');
    SaveDataAndBias(fmg, sensor_data.mg_data, sensor_data.mg_bias);
%     fmg = strcat(path, '\mg_ub_', group_name, '.', 'log');
    fmg = strcat(path, '\fpbl_mag_', group_name, '.', 'log');
    SaveData(fmg, sensor_data.mg_data);
    
end

function SaveData(fname, data)
    sz = size(data,1);
    fout = fopen(fname, 'wt');
    for i = 1:sz
        %fprintf(fout, '%.0f, 0.0, %.6f, %.6f, %.6f\n', data(i,1)*1000,data(i,2), data(i,3), data(i,4));
        fprintf(fout, '%.0f, 0.0, %.15f, %.15f, %.15f\n', data(i,1)*1000, data(i,2), data(i,3), data(i,4));
    end
    fclose (fout);
end


function SaveDataAndBias(fname, data, bias)
    sz = size(data,1);
    fout = fopen(fname, 'wt');
    for i = 1:sz
        %fprintf(fout, '%.0f, 0.0, %.6f, %.6f, %.6f,  %.6f, %.6f, %.6f\n', data(i,1)*1000,data(i,2), data(i,3), data(i,4), bias(i,2), bias(i,3), bias(i,4));
        fprintf(fout, '%.0f, 0.0, %.15f, %.15f, %.15f,  %.15f, %.15f, %.15f\n', data(i,1)*1000,data(i,2), data(i,3), data(i,4), bias(i,2), bias(i,3), bias(i,4));
    end
    fclose (fout);
end
