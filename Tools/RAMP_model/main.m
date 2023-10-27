clc;
close all;

% 'scripts_folder' has N scripts subfolders, this global script calls 'run_script' for each one

% scripts_folder = 'C:\Matlab_scripts\Script_Generator\scripts\';
% scripts_folder = 'C:\FP_Generation\Model_Test\30x30\data1\';
% scripts_folder = 'C:\FP_Generation\Model_Test\pos_error_2\data1\';
% scripts_folder = 'C:\FP_Generation\Model_Test\bias_and_heading_2\data1\';
scripts_folder = 'C:\FP_Generation\60x30_test\data4\';

% scripts_folder = 'C:\FP_Generation\Model_Test\test1\data\';
% scripts_folder = 'C:\FP_Generation\Model_Test\test2\data\';

% MG_parameters.path = 'fp\test_30x30.mfp3';
% MG_parameters.path = 'fp\mg_30x20.mfp3';
MG_parameters.path = 'fp\mg_60x30_sig1.mfp3';
% MG_parameters.path = 'fp\test_0.mfp3';
MG_parameters.sizeX = 60;
MG_parameters.sizeY = 30;
MG_parameters.cell_size = 1;

d = dir(scripts_folder);
isub = [d(:).isdir]; %# returns logical vectord
nameFolds = {d(isub).name}';
nameFolds(ismember(nameFolds,{'.', '..'})) = [];

for i = 1 : size(nameFolds, 1)
    path = strcat(scripts_folder, nameFolds{i});
    path = strcat(path, '\');
    script_file = dir(fullfile(path, '*script*.txt'));
    run_script(script_file.name, path, MG_parameters);
end

fclose all
'finished' %#ok<NOPTS>