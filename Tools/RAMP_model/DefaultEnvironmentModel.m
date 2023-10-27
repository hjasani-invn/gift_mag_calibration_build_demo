function [env_model] = InitEnvironmentModel()
        
    addpath('mfp_map\');
    % magnetic map
    mg_field_file = 'fp\test_mg1.mfp3';
    x_cells = 128;
    y_cells = 64;
    cell_size = 1;

%    mg_field_file = 'fp\regus2f.mfp3';    
%     x_cells = 24;
%     y_cells = 15;
%     cell_size = 1;
    
    %env_model.mg.model_type = 'const'; %'fp'
%     env_model.mg.model_type = 'fp_interpolated';
    %env_model.mg.model_type = 'fp_fixed';  % no interpolation 
    env_model.mg.model_type = 'fp_fixed_with_noise';
    
    env_model.mg.mean_mg = [5, 10, 50]; % for 'const' mode
    %env_model.mg.map = LoadMfp3(mg_field_file, x_cells, y_cells, cell_size); 
    [env_model.mg.map, env_model.mg.map_sigma] = LoadMfp3Ex(mg_field_file, x_cells, y_cells, cell_size); 
    env_model.mg.x_cells = x_cells;
    env_model.mg.y_cells = y_cells;
    env_model.mg.cell_size = cell_size;
    
    
    % gravity vector
    MEAN_G = [0, 0, 9.81];
    env_model.gp.mean_g = MEAN_G;
end