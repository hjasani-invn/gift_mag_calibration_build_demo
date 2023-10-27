function [meas] = LoadMapperSensorData(file_name)
    meas = [];
    cnt = 0;
    fileID = fopen(file_name, "rt");
    if (fileID > 0)
        fline = fgetl(fileID);
        while (ischar (fline))
           %parse data
           %data = sscanf(fline,'%f %*s %f');
           data = sscanf(fline,'%f,%f,%f,%f,%f,');
           if (size(data,1) >= 5)
               cnt = cnt+1;
               meas(cnt,:) = [data(1)' , data(3:5)'];
           end
           fline = fgetl(fileID);
        end 
    end
    fclose(fileID);
end

% function [data] = LoadMapperSensorData(file_name)
%     %log_data = dlmread(file_name, ',', 0, 0);
%     dlmread(file_name, ',', [0  0  end 5]);
%     if (isempty(log_data))
%         data = [];
%     else
%         data = [log_data(:,1), log_data(:,3:5)];
%     end
%end