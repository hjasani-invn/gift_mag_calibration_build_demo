function SaveReferenceTrack ( file_name, track_data)
    
    sz = size(track_data.sys_t, 2);

    fout = fopen(file_name, 'wt');
    for i = 1:sz
      fprintf(fout, '%.0f,  %.3f, %.3f, %.3f,  %.3f, %.3f, %.3f\n', ...
              track_data.sys_t(i), ...
              track_data.r(i,1), track_data.r(i,2), track_data.r(i,3),... % x,y,z
              track_data.v(i,1), track_data.v(i,2), track_data.v(i,3));% Vx,Vy,Vz
    end
    
    fclose(fout);
end
