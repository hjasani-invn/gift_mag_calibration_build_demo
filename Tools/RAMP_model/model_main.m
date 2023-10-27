function model_main (script_file, out_path)
clc;
%clear all;
addpath('mfp_map');      % include magnetic field model

[token, remain] = strtok(script_file, '.');
[token, remain] = strtok(token, '\'); % to do: parse path, name and extansion
script_name = strtok(remain, '\');

%load script fo dest folder
track_script = LoadScript(script_file);

out_dir = strcat(out_path, '\', script_name);

if (exist (out_dir,'dir') ~= 7)
  mkdir(out_dir);
end

%copy script fo dest folder
copyfile(script_file,out_dir);
%movefile(script_file,script_name);

MG_parameters.path = 'fp\test_0.mfp3';
MG_parameters.sizeX = 20;
MG_parameters.sizeY = 10;
MG_parameters.cell_size = 1;

user_model  = DefaultUserModel();
sensor_model = DefaultSensorModel();
environment_model = InitEnvironmentModel(MG_parameters);

sensor_model  = UpdateSensorModel(track_script, sensor_model);

TB = TrackBuilder;
[user_model, environment_model, ideal_track_data] = TB.ExecuteTrack(track_script, user_model, environment_model);
[user_model, environment_model, track_data] = TB.ExecuteTrackWithErrors(track_script, user_model, environment_model);
track_data = TB.Add_Uncertainties(track_data, ideal_track_data); % adds uncertainties of position, now they are equal to real difference: (idealX - X)


SaveReferenceTrack(strcat(out_dir, '\track_trj.csv'), track_data);
SaveReferenceOrientation(strcat(out_dir, '\track_att.csv'), track_data);
%SaveReferTrackAndAtt_FPBL(strcat(out_dir, '\track_pos_att.csv'), track_data);
SaveReferTrackAndAtt_FPBL(strcat(out_dir, '\pos_att_2016_xx_xx.csv'), track_data);

sensor_data = GenerateSensorData(ideal_track_data, sensor_model, environment_model, 0);
SaveSensorData(out_dir,'ref_2016',sensor_data);
% WARNING - generating ideal data should always be done BEFORE generating data with noise
sensor_data = GenerateSensorData(ideal_track_data, sensor_model, environment_model, 1);
SaveSensorData(out_dir,'2016',sensor_data);

sensor_data = SaveRAMPoutput_IRL( strcat( out_dir, '\RAMP_out.log' ), 0, track_data, sensor_data, user_model, environment_model ); % note 0 is misalingment angle

 end