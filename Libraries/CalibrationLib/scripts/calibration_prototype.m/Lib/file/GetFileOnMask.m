function file_name = GetFileOnMask(dst_path, mask)
    file_name = [];
    file_info = dir(strcat(dst_path,'\' ,mask));
    if ~isempty(file_info)
        file_name =  strcat(dst_path,'\' ,file_info.name);
    end
end