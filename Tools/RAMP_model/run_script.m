function run_script(script_name, path_to_script, MG_parameters)

% addpath('..\mfp_map');      % include magnetic field model

full_path_to_script = strcat(path_to_script, script_name);

track_script = LoadScript(full_path_to_script);

user_model  = DefaultUserModel();
sensor_model = DefaultSensorModel();
environment_model = InitEnvironmentModel(MG_parameters);

sensor_model  = UpdateSensorModel(track_script, sensor_model);

TB = TrackBuilder;
[user_model, environment_model, ideal_track_data] = TB.ExecuteTrack(track_script, user_model, environment_model);
[user_model, environment_model, track_data] = TB.ExecuteTrackWithErrors(track_script, user_model, environment_model);
track_data = TB.Add_Uncertainties(track_data, ideal_track_data); % adds uncertainties of position, now they are equal to real difference: (idealX - X)

% SaveReferenceTrack(strcat(path_to_script, '\ideal_track_trj.log'), ideal_track_data);
% SaveReferenceTrack(strcat(path_to_script, '\track_trj.log'), track_data);
% SaveReferenceOrientation(strcat(path_to_script, '\track_att.log'), ideal_track_data);
% SaveReferTrackAndAtt_FPBL(strcat(path_to_script, '\pos_att_2016_xx_xx.log'), ideal_track_data);

% sensor_data = GenerateSensorData(ideal_track_data, sensor_model, environment_model, 0);
% SaveSensorData(path_to_script, 'ref_2016', sensor_data);
% WARNING - generating ideal data should always be done BEFORE generating data with noise

sensor_data = GenerateSensorData(ideal_track_data, sensor_model, environment_model, 1);
% SaveSensorData(path_to_script, '2016', sensor_data);

sensor_data = SaveRAMPoutput_IRL( strcat( path_to_script, '\RAMP_out.log' ), user_model.heading_error, track_data, sensor_data, user_model, environment_model ); % note 0 is misalingment angle
% sensor_data = SaveRAMPoutput_for_Old_FPBL( strcat( path_to_script, '\fpbl_pos.log' ), 0, track_data, sensor_data, user_model ); % saves data in old FPBL format (with quaternions)

end