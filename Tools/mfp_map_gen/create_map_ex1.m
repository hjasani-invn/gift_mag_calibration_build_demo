function create_map_ex1

close all

% constant field magnetic map generation
ny_cells = 10;
nx_cells = 10;
sig_map = 1;
mean_vec = [10 20 40];
cellsize = 1;
map = mg_map( ny_cells, nx_cells, sig_map, mean_vec) % y-cels, x-cels, sig, mean_veector

% save map
sig_meas1 = 2;% парметры Гаусовского распределения
fname = 'mg_10x10_cv_n2.mfp3';
mg_export(fname, map, sig_meas1, cellsize);
map1 = LoadMfp3 (fname,nx_cells,ny_cells,1);

fname2 = 'mg_10x10_cv_n1-20.mfp3';
sig_meas2(1:10,1:10) = 20;    % парметры Гаусовского распределения
sig_meas2(1:8,1:8) = 10;    % парметры Гаусовского распределения
sig_meas2(1:6,1:6) = 5;    % парметры Гаусовского распределения
sig_meas2(1:4,1:4) = 2;    % парметры Гаусовского распределения
sig_meas2(1:2,1:2) = 1;    % парметры Гаусовского распределения

mg_export(fname2, map, sig_meas2, cellsize);
map2 = LoadMfp3 (fname2,nx_cells,ny_cells,1);

 
end
