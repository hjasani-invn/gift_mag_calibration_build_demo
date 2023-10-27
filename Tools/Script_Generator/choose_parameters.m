function header_parameters = choose_parameters( set )

if ( set == 1 )
    header_parameters.std_mg_bias = 0.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 0.0];
    header_parameters.max_angles_std = [0.0 0.0 0.0];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 2 )
    header_parameters.std_mg_bias = 0.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [deg2rad(0.5) deg2rad(0.5) 0.0];
    header_parameters.max_angles_std = [deg2rad(0.5) deg2rad(0.5) 0.0];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 3 )
    header_parameters.std_mg_bias = 0.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [deg2rad(1.0) deg2rad(1.0) 0.0];
    header_parameters.max_angles_std = [deg2rad(1.0) deg2rad(1.0) 0.0];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 4 )
    header_parameters.std_mg_bias = 0.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 deg2rad(2.0)];
    header_parameters.max_angles_std = [0.0 0.0 deg2rad(2.0)];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 5 )
    header_parameters.std_mg_bias = 0.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 deg2rad(4.0)];
    header_parameters.max_angles_std = [0.0 0.0 deg2rad(4.0)];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 6 )
    header_parameters.std_mg_bias = 2.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 0.0];
    header_parameters.max_angles_std = [0.0 0.0 0.0];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 7 )
    header_parameters.std_mg_bias = 4.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 0.0];
    header_parameters.max_angles_std = [0.0 0.0 0.0];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 8 )
    header_parameters.std_mg_bias = 10.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 0.0];
    header_parameters.max_angles_std = [0.0 0.0 0.0];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 9 )
    header_parameters.std_mg_bias = 0.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 1.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 0.0];
    header_parameters.max_angles_std = [0.0 0.0 0.0];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 10 )
    header_parameters.std_mg_bias = 0.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 2.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 0.0];
    header_parameters.max_angles_std = [0.0 0.0 0.0];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 11 )
    header_parameters.std_mg_bias = 4.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 deg2rad(4.0)];
    header_parameters.max_angles_std = [0.0 0.0 deg2rad(4.0)];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

if ( set == 12 )
    header_parameters.std_mg_bias = 10.0;
    header_parameters.min_XY_error = [0.0 0.0];
    header_parameters.max_XY_error = [0.0 0.0];
    header_parameters.std_XY_error = 0.0;
    header_parameters.min_init_heading_error = 0;
    header_parameters.max_init_heading_error = 0;
    header_parameters.min_heading_std = 0.0;
    header_parameters.max_heading_std = 0.0;
    header_parameters.min_angles_std = [0.0 0.0 deg2rad(6.0)];
    header_parameters.max_angles_std = [0.0 0.0 deg2rad(6.0)];
    header_parameters.min_heading_drift = 0.0;
    header_parameters.max_heading_drift = 0.0;
    header_parameters.min_init_angles_error = [0.0 0.0 0.0];
    header_parameters.max_init_angles_error = [0.0 0.0 0.0];
    header_parameters.min_angles_drift = [0.0 0.0 0.0];
    header_parameters.max_angles_drift = [0.0 0.0 0.0];
end

end