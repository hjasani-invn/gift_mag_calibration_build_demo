function GenerateModelScript(fname, t0, duration, x0, y0, z0, vx0, vy0, vz0, wx0, wy0, wz0)
    
fout = fopen(fname, 'wt');

fprintf(fout,'rem This is automaticaly generated script');

fprintf(fout,'\nset_t0 %.3f', t0);
fprintf(fout,'\nset_tend %.3f', t0+duration);
fprintf(fout,'\nset_sample_rate	50');

fprintf(fout,'\n');
fprintf(fout,'\nset_pos  %.3f   %.3f %.3f %.3f' , t0, x0, y0, z0);
fprintf(fout,'\nset_vel  %.3f   %.3f %.3f %.3f' , t0, vx0, vy0, vz0);
fprintf(fout,'\nset_ang_vel  %.3f   %.3f %.3f %.3f' , t0, wx0, wy0, wz0);

fclose(fout);
    
end
