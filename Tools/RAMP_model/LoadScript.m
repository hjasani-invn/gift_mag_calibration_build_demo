function script  = LoadScript (file)
    script = cell(1);
    fid = fopen(file,'rt');
    while (~feof(fid))
       line = fgetl(fid);
       if (size(line,1) > 0)
         idx = size(script,2);
         script{idx+1} = line;
       end
    end
    fclose(fid);
end