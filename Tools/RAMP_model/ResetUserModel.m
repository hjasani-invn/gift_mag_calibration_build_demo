function user_model  = ResetUserModel (user_model0)
    
    user_model = user_model0;
    %user_model.sample_rate = 50.0; %Hz
    
    user_model.t0 = user_model.t0;
    user_model.tend = max(user_model.t0, user_model.tend);
    
    user_model.state.t = user_model.t0;

    user_model.state.r(1) = 0;   % x position
    user_model.state.r(2) = 0;   % y position
    user_model.state.r(3) = 0;   % z position

    user_model.state.v(1) = 0;   % x velocity
    user_model.state.v(2) = 0;   % y velocity
    user_model.state.v(3) = 0;   % z velocity

    user_model.state.a(1) = 0;   % x acceleration
    user_model.state.a(2) = 0;   % y acceleration
    user_model.state.a(3) = 0;   % z acceleration

    %euler angles: ZYX notation
    fi = 0;
    teta = 0;    
    psy = 0;
    user_model.state.ang(1) = fi;   % angle psy
    user_model.state.ang(2) = teta;   % angle teta
    user_model.state.ang(3) = psy;   % angle fi

    %Orientation quaternion
%     user_model.state.q = angle2quat(psy, teta, fi, 'ZYX'); % ENU to body
    user_model.state.q = angle2quat(psy, teta, fi); % body to ENU 
    
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
    
    %C = angle2dcm(psy, teta, fi, 'ZYX'); % ENU to body
    
    user_model.state.w(1) = 0;   % x angular velocity
    user_model.state.w(2) = 0;   % y angular velocity
    user_model.state.w(3) = 0;   % z angular velocity
    
    %Body frame angular acceleration
    user_model.state.e(1) = 0;   % x angular acceleration 
    user_model.state.e(2) = 0;   % y angular acceleration
    user_model.state.e(3) = 0;   % z angular  acceleration
    
    
end
