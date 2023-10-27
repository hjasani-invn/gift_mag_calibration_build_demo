function file_names = GetFileOnMask(dst_path, mask)
    file_names = {};
    file_info = dir(strcat(dst_path,'\' ,mask));
    
    for (i = 1:size(file_info,1))
        file_names{i} =  strcat(dst_path,'\' ,file_info(i).name);
    end
end