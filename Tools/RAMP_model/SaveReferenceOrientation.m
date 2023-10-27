function SaveReferenceOrientation ( file_name, track_data)
    
    sz = size(track_data.sys_t, 2);

    fout = fopen(file_name, 'wt');
    for i = 1:sz
      fprintf(fout, '%.0f', track_data.sys_t(i));
      fprintf(fout, ',  %.6f, %.6f, %.6f', track_data.ang(i,1), track_data.ang(i,2), track_data.ang(i,3));
      fprintf(fout, ',  %.6f, %.6f, %.6f', track_data.w(i,1), track_data.w(i,2), track_data.w(i,3));
      fprintf(fout, ',  %.6f, %.6f, %.6f, %.6f', track_data.q(i,1), track_data.q(i,2), track_data.q(i,3), track_data.q(i,4));
      fprintf(fout, '\n');
    end
    
    fclose(fout);
end