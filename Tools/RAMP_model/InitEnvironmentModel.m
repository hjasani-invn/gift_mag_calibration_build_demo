function [env_model] = InitEnvironmentModel(MG_parameters)
        
    addpath('mfp_map\');
    % magnetic map
%     mg_field_file = 'fp\test_30x30.mfp3';
    mg_field_file = MG_parameters.path;
    x_cells = MG_parameters.sizeX;
    y_cells = MG_parameters.sizeY;
    cell_size = MG_parameters.cell_size;

%    mg_field_file = 'fp\regus2f.mfp3';    
%     x_cells = 24;
%     y_cells = 15;
%     cell_size = 1;
    
    %env_model.mg.model_type = 'const'; %'fp'
%     env_model.mg.model_type = 'fp_interpolated';
    env_model.mg.model_type = 'fp_fixed';  % no interpolation 
%     env_model.mg.model_type = 'fp_fixed_with_noise';
    
    env_model.mg.mean_mg = [5, 10, 50]; % for 'const' mode
    %env_model.mg.map = LoadMfp3(mg_field_file, x_cells, y_cells, cell_size); 
    [env_model.mg.map, env_model.mg.map_sigma] = LoadMfp3Ex(mg_field_file, x_cells, y_cells, cell_size); 
    env_model.mg.x_cells = x_cells;
    env_model.mg.y_cells = y_cells;
    env_model.mg.cell_size = cell_size;
    
    % default values for venue parameters
    env_model.origin_lattitude = 55.724253553756192;
    env_model.origin_longitude = 37.653916182652893;
    env_model.origin_azimuth = 0.0;
    env_model.alfa = 0.000000162179509;
    env_model.beta = -0.000000271000819;
	    
    % gravity vector
    MEAN_G = [0, 0, 9.81];
    env_model.gp.mean_g = MEAN_G;
end