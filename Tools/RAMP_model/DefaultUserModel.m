function user_model  = DefaultUserModel ()

    addpath('quat_lib');

    user_model.sample_rate = 20.0; %Hz
    
    user_model.t0 = 0;
    user_model.tend = 10;
    
    user_model.state.t = 0;
    
    user_model.state.r(1) = 0;   % x position
    user_model.state.r(2) = 0;   % y position
    user_model.state.r(3) = 0;   % z position

    user_model.state.v(1) = 0;   % x velocity
    user_model.state.v(2) = 0;   % y velocity
    user_model.state.v(3) = 0;   % z velocity
    
    user_model.state.a(1) = 0;   % x velocity
    user_model.state.a(2) = 0;   % y velocity
    user_model.state.a(3) = 0;   % z velocity
    
    user_model.state.covXY = [ 0.0001 0.0000001;
                               0.0000001 0.0001 ];
                                 % XY position error covariance matrix

    %euler angles: ZYX notation
    fi = 0;
    teta = 0;    
    psy = pi/2;
    user_model.state.ang(1) = fi;   % angle psy
    user_model.state.ang(2) = teta;   % angle teta
    user_model.state.ang(3) = psy;   % angle fi

    %Orientation quaternion
%     user_model.state.q = angle2quat(psy, teta, fi, 'ZYX'); % ENU to body
    user_model.state.q = angle2quat(psy, teta, fi);  % body to ENU
    
    %C1 = quat2dcm(user_model.state.q);
    
%     user_model.state.C_L2B = angle2dcm(psy, teta, fi, 'ZYX'); % ENU to body
    
    C_roll = [1 0 0;
            0 cos(fi) -sin(fi);
            0 sin(fi) cos(fi)];
    
    C_pitch = [cos(teta) 0 sin(teta);
                0 1 0;
               -sin(teta) 0 cos(teta)];
    
    C_heading = [cos(psy) -sin(psy) 0;
                sin(psy) cos(psy) 0;
                0 0 1];
    
    user_model.state.C_B2ENU = C_heading * C_pitch * C_roll;
    
    
    user_model.state.w(1) = 0;   % x angular velocity
    user_model.state.w(2) = 0;   % y angular velocity
    user_model.state.w(3) = 0;   % z angular velocity
    
    %Body frame angular acceleration
    user_model.state.e(1) = 0;   % x angular acceleration 
    user_model.state.e(2) = 0;   % y angular acceleration
    user_model.state.e(3) = 0;   % z angular  acceleration
    
    user_model.heading_error = 0;
    
end
