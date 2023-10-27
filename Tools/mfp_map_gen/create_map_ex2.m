function create_map_ex2

close all

% constant field magnetic map generation
ny_cells = 39;
nx_cells = 85;
sig_map = 1;
mean_vec = [10 20 40];
cellsize = 1;
map = mg_map( ny_cells, nx_cells, sig_map, mean_vec) % y-cels, x-cels, sig, mean_veector

% save map
sig_meas1 = 0.001;% парметры Гаусовского распределения
fname = 'mg_85x39_const_field.mfp3';
mg_export(fname, map, sig_meas1, cellsize);
map1 = LoadMfp3 (fname,nx_cells,ny_cells,1);

end
