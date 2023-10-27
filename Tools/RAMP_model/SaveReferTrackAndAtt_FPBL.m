function SaveReferTrackAndAtt_FPBL ( file_name, track_data)
    
    sz = size(track_data.sys_t, 2);

    fout = fopen(file_name, 'wt');
    for i = 1:sz
      fprintf(fout, '%.0f, %d',track_data.sys_t(i), 1); %t, validity
      fprintf(fout, ' ,%.3f,%.3f,%.3f, %d', track_data.r(i,1), track_data.r(i,2), track_data.r(i,3), 0);% x,y,h,floor
      fprintf(fout, ' ,%.3e,%.3e,%.3e,%.3e' , 4e-2, 0., 0., 4e-2);% x,y acc
      fprintf(fout, ' ,%.3e, %.3e' , 4e-1, 0.0025); % ...%h acc, floor acc
      
      fprintf(fout, ' ,%d', 1); %att validity
      fprintf(fout, ' ,%.9f,%.9f,%.9f,%.9f' , track_data.q(i,1), track_data.q(i,2), track_data.q(i,3), track_data.q(i,4));%q
      fprintf(fout, ' ,%.3g,%.3g,%.3g,%.3g,  %.3g,%.3g,%.3g,%.3g,  %.3g,%.3g,%.3g,%.3g,  %.3g,%.3g,%.3g,%.3g', ...
                      0.1, 0.0, 0.0, 0.0,...
                      0.0, 0.2, 0.0, 0.0,...
                      0.0, 0.0, 0.3, 0.0,...      
                      0.0, 0.0, 0.0, 0.4);
      fprintf(fout, '\n');

    end
    
    fclose(fout);
end