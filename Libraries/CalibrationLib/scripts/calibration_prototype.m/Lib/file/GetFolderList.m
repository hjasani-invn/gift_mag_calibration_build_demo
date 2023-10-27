function path_list = GetFolderList(path, file_mask)
    path_list = {};
    subfolder_list = GetSubFolders(path, {});
    cnt = 0;
    for i = 1 : size(subfolder_list,1)
        file_name = GetFileOnMask(subfolder_list{i,1}, file_mask);
        if (~isempty(file_name))
            cnt = cnt+1;
            path_list(cnt,1) = subfolder_list(i);
        end 
    end
end

function folder_list = GetSubFolders(path, in_folder_list)
    folder_list = in_folder_list;
    file_info = dir (path);
    for i = 1 : size(file_info,1)
        if ((file_info(i).isdir) && (~strcmp(file_info(i).name,'.')) && (~strcmp(file_info(i).name,'..')))
            subpath = strcat(file_info(i).folder, '\', file_info(i).name);
            cnt = size(folder_list, 1) + 1;
            folder_list{cnt,1} = subpath;
            folder_list = GetSubFolders(subpath, folder_list);
        end 
    end
end
