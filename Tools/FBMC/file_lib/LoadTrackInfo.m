function track_info = LoadTrackInfo(info_file_name)
 
    fin = fopen(info_file_name, 'rt');
    track_info.device_id = 0;
    track_info.utc0 = 0;
    if (fin > 0)
        str = 0;
        while (str ~= -1)
            str = fgetl(fin);
            if (size(str) < 5 )   
                continue;
            end
            [token, remain] = strtok(str,'=');
            switch (token)
                case 'UTC-TIME'
                    track_info.utc0 = sscanf(remain(2:end), '%f');
                case 'DEVICE-ID'
                    track_info.device_id = sscanf(remain(2:9), '%x',1);
            end
        end
        fclose(fin);
    end
        
end