classdef TrackBuilder < handle
    
    methods(Static = true)
        function obj = TrackBuilder()
        end
        
        function [user_model_new, environment_model_new, track_data] = ExecuteTrack(track_script, user_model, environment_model)
            
            [user_model, environment_model_new] = TrackBuilder.InitTrack(track_script, user_model, environment_model);
            % user_model.state;
            % reading all script commands to an array (doing it once greatly speeds up the execution, before it was done at every step)
            [cmd_structure] = TrackBuilder.ParseTrackCmd(track_script);
            
            %   user_model.init_XY_error is set here
            
            track_data = [];
            
            tau = 1 / user_model.sample_rate;
            print_period = 50;
            print_line_size = 10;
            for t = user_model.t0 : tau : user_model.tend
                %disp(t)
                
                if (mod(t, print_period*tau) < tau*0.9)
                    if (mod(t, print_period*print_line_size*tau) < tau*0.9)
                        fprintf('%.3f\n',t)
                    else
                        fprintf('.')
                    end
                end
                
                %new command from script
                user_model = TrackBuilder.UpdateState( cmd_structure, user_model, 'ideal');
                
                %save current state - update track data
                track_data = TrackBuilder.UpdateTrackData(track_data, user_model);
                
                %new command from script
                %user_model = UpdateState( track_script, user_model);
                
                %calculate next state
                user_model.state = TrackBuilder.PropagateState(tau, user_model);
                
            end
            
            user_model_new = user_model;
        end
        
        function [user_model_new, environment_model_new, track_data] = ExecuteTrackWithErrors(track_script, user_model, environment_model)
            
            [user_model, environment_model_new] = TrackBuilder.InitTrack(track_script, user_model, environment_model);
            % reading all script commands to an array (doing it once greatly speeds up the execution, before it was done at every step)
            [cmd_structure] = TrackBuilder.ParseTrackCmd(track_script);
            
            track_data = [];
            
            tau = 1 / user_model.sample_rate;
            print_period = 50;
            print_line_size = 10;
                       
            for t = user_model.t0 : tau : user_model.tend
                %disp(t)
                
                if (mod(t, print_period*tau) < tau*0.9)
                    if (mod(t, print_period*print_line_size*tau) < tau*0.9)
                        fprintf('%.3f\n',t)
                    else
                        fprintf('.')
                    end
                end
                
                %new command from script
                user_model = TrackBuilder.UpdateState( cmd_structure, user_model, 'errors');
                
                %save current state - update track data
                track_data = TrackBuilder.Errors_UpdateTrackData(track_data, user_model);
                
                %new command from script
                %user_model = UpdateState( track_script, user_model);
                
                dt = t - user_model.t0;
                
                alfa = user_model.heading_drift * dt + randn * user_model.heading_std; % additional error due to noise and drift (applied at every step)
                
                error_v = [cos(alfa) sin(alfa); -sin(alfa) cos(alfa)] * [user_model.state.v(1); user_model.state.v(2)];
                
                user_model.state.v(1) = error_v(1);
                user_model.state.v(2) = error_v(2);

                %calculate next state
                user_model.state = TrackBuilder.PropagateState(tau, user_model);
                
                % WARNING : adding errors here will affect further data
                % add errors to position
                
                %       user_model.state.r(1) = user_model.state.r(1) + normrnd(0, user_model.stdXY(1));
                %       user_model.state.r(2) = user_model.state.r(2) + normrnd(0, user_model.stdXY(2));
                %
                %       % add errors to angles and quaternion
                %       user_model.state.ang(1) = user_model.state.ang(1) + normrnd(0, user_model.stdangles(1));
                %       user_model.state.ang(2) = user_model.state.ang(2) + normrnd(0, user_model.stdangles(2));
                %       user_model.state.ang(3) = user_model.state.ang(3) + normrnd(0, user_model.stdangles(3));
                %
                %       user_model.state.q = angle2quat( user_model.state.ang(3), user_model.state.ang(2), user_model.state.ang(1) );
            end
            
            user_model_new = user_model;
            
        end
        
        
        function [ user_model, environment_model ]  = InitTrack(track_script, user_model0, environment_model0)
            user_model = user_model0;
            environment_model = environment_model0;
            
            cmd_count = size(track_script, 2);
            for i = 1:cmd_count
                [user_model, environment_model, is_applied] = TrackBuilder.ParseInitCmd(track_script{i}, user_model, environment_model);
            end
            user_model = ResetUserModel(user_model);
            for i = 1:cmd_count
                [user_model is_applied] = TrackBuilder.ParseCmd(track_script{i}, user_model);
            end
                        
        end
        
        function [user_model, environment_model, is_applied] = ParseInitCmd(cmd_line, user_model0, environment_model0)
            
            if (~isempty(cmd_line) && ischar(cmd_line))
                cmd = sscanf( cmd_line, '%s', 1);
            else
                cmd = 'rem';
            end
            
            user_model = user_model0;
            environment_model = environment_model0;
            
            is_applied = true;
            
            switch (cmd)
                case 'set_t0'
                    [t, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        user_model.t0 = t;
                        user_model.state.t = t;
                    end
                case 'set_tend'
                    [t, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        user_model.tend = t;
                    end
                case 'set_sample_rate'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        user_model.sample_rate = dt;
                    end
                case 'set_init_XY_error'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf %lf');
                    if (cnt == 2)
                        user_model.init_XY_error = [dt(1) dt(2)];
                    end
                case 'set_XY_std' % WARNING - this command is unused now and is not generated in script generator
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf %lf');
                    if (cnt == 2)
                        user_model.stdXY = [dt(1) dt(2)];
                    end
                case 'set_angles_std'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
                    if (cnt == 3)
                        user_model.stdangles = [dt(1) dt(2) dt(3)];
                    end
                case 'set_init_heading_error'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        user_model.heading_error = dt;
                    end
                case 'set_heading_std'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        user_model.heading_std = dt;
                    end
                case 'set_init_angles_error'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
                    if (cnt == 3)
                        user_model.init_angles_error = [dt(1) dt(2) dt(3)];
                    end
                case 'set_angles_drift'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf');
                    if (cnt == 3)
                        user_model.angles_drift = [dt(1) dt(2) dt(3)];
                    end
                case 'set_heading_drift'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        user_model.heading_drift = dt;
                    end
                case 'set_origin_lattitude'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        environment_model.origin_lattitude = dt;
                    end
                case 'set_origin_longitude'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        environment_model.origin_longitude = dt;
                    end
                case 'set_origin_azimuth'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        environment_model.origin_azimuth = dt;
                    end
                case 'set_alfa'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        environment_model.alfa = dt;
                    end
                case 'set_beta'
                    [dt, cnt] = sscanf( cmd_line, '%*s %lf');
                    if (cnt == 1)
                        environment_model.beta = dt;
                    end
                otherwise
                    is_applied = false;
            end
        end
        
        function [cmd_structure] = ParseTrackCmd(track_script)
            cmd_count = size(track_script, 2);
            cmd_structure(cmd_count, 5) = 0; % pre-allocating (will have empty elements, but no problem)
            
            k = 0;
            for i = 1:cmd_count
                cmd_line = track_script{i};
                if (~isempty(cmd_line) && ischar(cmd_line))
                    cmd = sscanf( cmd_line, '%s', 1);
                else
                    cmd = 'rem';
                end
                
                switch(cmd)
                    case 'set_pos'
                        [pos, ~] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                        k = k + 1;
                        cmd_structure(k, 1) = pos(1); % time
                        cmd_structure(k, 2) = 1; % position command code
                        cmd_structure(k, 3) = pos(2); % x
                        cmd_structure(k, 4) = pos(3); % y
                        cmd_structure(k, 5) = pos(4); % z
                    case 'set_vel'
                        [pos, ~] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                        k = k + 1;
                        cmd_structure(k, 1) = pos(1); % time
                        cmd_structure(k, 2) = 2; % velocity command code
                        cmd_structure(k, 3) = pos(2); % Vx
                        cmd_structure(k, 4) = pos(3); % Vy
                        cmd_structure(k, 5) = pos(4); % Vz
                    case 'set_acc'
                        [pos, ~] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                        k = k + 1;
                        cmd_structure(k, 1) = pos(1); % time
                        cmd_structure(k, 2) = 3; % acceleration command code
                        cmd_structure(k, 3) = pos(2); % acc_x
                        cmd_structure(k, 4) = pos(3); % acc_y
                        cmd_structure(k, 5) = pos(4); % acc_z
                    case 'set_euler_angles'
                        [pos, ~] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                        k = k + 1;
                        cmd_structure(k, 1) = pos(1); % time
                        cmd_structure(k, 2) = 4; % euler angles command code
                        cmd_structure(k, 3) = pos(2); % roll
                        cmd_structure(k, 4) = pos(3); % pitch
                        cmd_structure(k, 5) = pos(4); % heading
                    case {'set_ang_vel', 'set_gyr'}
                        [pos, ~] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                        k = k + 1;
                        cmd_structure(k, 1) = pos(1); % time
                        cmd_structure(k, 2) = 5; % angular velocity command code
                        cmd_structure(k, 3) = pos(2); % w_x
                        cmd_structure(k, 4) = pos(3); % w_y
                        cmd_structure(k, 5) = pos(4); % w_z
                end
            end
            cmd_structure = cmd_structure(1:k, :);
        end
        
        function [user_model, is_applied] = ParseCmd(cmd_line, user_model0)
            
            if (~isempty(cmd_line) && ischar(cmd_line))
                cmd = sscanf( cmd_line, '%s', 1);
            else
                cmd = 'rem';
            end
            
            user_model = user_model0;
            
            dt = 1/user_model.sample_rate;
            
            is_applied = true;
            
            t = user_model.state.t;
            switch (cmd)
                case 'set_pos'
                    [pos, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                    if (cnt == 4)
                        %if ( ((pos(1) >= user_model.state.t)) && (pos(1) < user_model.state.t+dt))
                        if ((pos(1) >= (t-0.5*dt)) && (pos(1) < (t+0.5*dt)))
                            user_model.state.r(1) = pos(2);
                            user_model.state.r(2) = pos(3);
                            user_model.state.r(3) = pos(4);
                        end
                    end
                case 'set_vel'
                    [pos, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                    if (cnt == 4)
                        %if ( ((pos(1) >= user_model.state.t)) && (pos(1) < user_model.state.t+dt))
                        if ((pos(1) >= (t-0.5*dt)) && (pos(1) < (t+0.5*dt)))
                            user_model.state.v(1) = pos(2);
                            user_model.state.v(2) = pos(3);
                            user_model.state.v(3) = pos(4);
                        end
                    end
                case 'set_acc'
                    [pos, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                    if (cnt == 4)
                        %if ( ((pos(1) >= user_model.state.t)) && (pos(1) < user_model.state.t+dt))
                        if ((pos(1) >= (t-0.5*dt)) && (pos(1) < (t+0.5*dt)))
                            user_model.state.a(1) = pos(2);
                            user_model.state.a(2) = pos(3);
                            user_model.state.a(3) = pos(4);
                        end
                    end
                case 'set_euler_angles'
                    [pos, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                    if (cnt == 4)
                        %if ( ((pos(1) >= user_model.state.t)) && (pos(1) < user_model.state.t+dt))
                        if ((pos(1) >= (t-0.5*dt)) && (pos(1) < (t+0.5*dt)))
                            fi = pos(2);    teta = pos(3);      psy = pos(4);
                            user_model.state.ang(1) = fi;
                            user_model.state.ang(2) = teta;
                            user_model.state.ang(3) = psy;
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
                            
                        end
                    end
                case {'set_ang_vel', 'set_gyr'}
                    [pos, cnt] = sscanf( cmd_line, '%*s %lf %lf %lf %lf');
                    if (cnt == 4)
                        %if ( ((pos(1) >= user_model.state.t)) && (pos(1) < user_model.state.t+dt))
                        if ((pos(1) >= (t-0.5*dt)) && (pos(1) < (t+0.5*dt)))
                            user_model.state.w(1) = pos(2);
                            user_model.state.w(2) = pos(3);
                            user_model.state.w(3) = pos(4);
                        end
                    end
                otherwise
                    is_applied = false;
            end
        end
                
        function user_model = UpdateState( cmd_structure, user_model, mode )
            dt = 1/user_model.sample_rate;
            t = user_model.state.t;
            
            for i = 1:size(cmd_structure, 1)
                if ( (cmd_structure(i, 1) >= (t-0.5*dt)) && (cmd_structure(i, 1) < (t+0.5*dt)) )
                    % found a command, applying it to user_model
                    switch cmd_structure(i, 2)
                        case 1
                            user_model.state.r(1) = cmd_structure(i, 3);
                            user_model.state.r(2) = cmd_structure(i, 4);
                            user_model.state.r(3) = cmd_structure(i, 5);
                            if ( strcmp(mode, 'errors') )
                                user_model.state.r(1) = user_model.state.r(1) + user_model.init_XY_error(1);
                                user_model.state.r(2) = user_model.state.r(2) + user_model.init_XY_error(2);
                            end
                        case 2
                            user_model.state.v(1) = cmd_structure(i, 3);
                            user_model.state.v(2) = cmd_structure(i, 4);
                            user_model.state.v(3) = cmd_structure(i, 5);
                            if ( strcmp(mode, 'errors') )
                                % change velocity according to heading angle error
                                %rotating velocity vector using the heading errors
                                alfa = user_model.heading_error;
                                error_v = [cos(alfa) sin(alfa); -sin(alfa) cos(alfa)] * [user_model.state.v(1); user_model.state.v(2)];
                                user_model.state.v(1) = error_v(1);
                                user_model.state.v(2) = error_v(2);
                            end
                        case 3
                            user_model.state.a(1) = cmd_structure(i, 3);
                            user_model.state.a(2) = cmd_structure(i, 4);
                            user_model.state.a(3) = cmd_structure(i, 5);
                        case 4
                            fi = cmd_structure(i, 3);
                            teta = cmd_structure(i, 4);
                            psy = cmd_structure(i, 5);
                            
                            user_model.state.ang(1) = fi;
                            user_model.state.ang(2) = teta;
                            user_model.state.ang(3) = psy;
                            
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
                            
                        case 5
                            user_model.state.w(1) = cmd_structure(i, 3);
                            user_model.state.w(2) = cmd_structure(i, 4);
                            user_model.state.w(3) = cmd_structure(i, 5);
                    end
                end
            end
        end
        
        
        function st = PropagateState(tau, user_model)
            st = user_model.state;
            
            st.t = st.t + tau;
            
            st.r = st.r + st.v * tau + 0.5 * st.a * tau^2;
            st.v = st.v + st.a * tau;
            st.a = st.a;
            
            %to do: RungeKutta integration
            w0 = st.w;
            st.w = st.w + st.e * tau;
            st.e = st.e;
            %st.ang = CalculateeulerAngles(st.ang, w0, st.w, tau);
            st.q = TrackBuilder.PropagateAttitudeQuaternion(st.q, w0, st.e, tau);
%             st.C_B2ENU = TrackBuilder.PropagateC_B2ENU(st.C_B2ENU, w0, st.e, tau);
            [psy, teta, fi] = quat2angle(st.q , 'ZYX');
            st.ang(1) = fi;
            st.ang(2) = teta;
            st.ang(3) = psy;
        end
        
        function q = PropagateAttitudeQuaternion(q0, w0, e, tau)
            qw = [0 w0(1) w0(2) w0(3)];
            dq1 = quatmultiply(q0, qw);
            %     dq(1) = -q0(2)*w0(1) - q0(3)*w0(2) - q0(4)*w0(3);
            %     dq(2) =  q0(1)*w0(1) + q0(3)*w0(3) - q0(4)*w0(2);
            %     dq(3) =  q0(1)*w0(2) - q0(2)*w0(3) + q0(4)*w0(1);
            %     dq(4) =  q0(1)*w0(3) - q0(2)*w0(2) - q0(3)*w0(1);
            q = q0 + 0.5*dq1*tau;
            q = q / norm(q);
        end
        
        function C_B2ENU = PropagateC_B2ENU(C_B2ENU, w0, e, tau)
            C_B2ENU = C_B2ENU * expm( Kos(w0 * tau) );
            [S,U,D] = svd(C_B2ENU);
            C_B2ENU = S*D';
        end
        
        function ang = CalculateeulerAngles(ang0, w0, w1, tau)
            we0 = GeteulerAngularRate(ang0, w0);
            ang = ang0 + we0*tau;
            we1 = GeteulerAngularRate(ang, w1);
            ang = ang0 + we0*tau/2 + we1*tau/2;
        end
        
        function we = GeteulerAngularRate(ang, w)
            sin_teta = sin(ang(2)); cos_teta = cos(ang(2));
            sin_fi = sin(ang(3));   cos_fi = cos(ang(3));
            p = w(1); q = w(2); r = w(3);
            we(1) = (p*sin_fi + q*cos_fi)/sin_teta;
            we(2) = p*cos_fi - q*sin_fi;
            we(3) = r - ((p*sin_fi + q*cos_fi)/sin_teta) * cos_teta;
        end
        
        function track_data = UpdateTrackData(track_data, user_model)
            
            if (~isempty(track_data))
                i = size(track_data.sys_t,2)+1;
            else
                i = 1;
            end
            
            track_data.sys_t(i) = user_model.state.t*1000;
            track_data.t(i) = user_model.state.t;
            
            track_data.r(i,:) = user_model.state.r;
            track_data.v(i,:) = user_model.state.v;
            track_data.a(i,:) = user_model.state.a;
            
            track_data.ang(i,:) = user_model.state.ang ;
            track_data.w(i,:) = user_model.state.w ;
            track_data.e(i,:) = user_model.state.e;
            track_data.q(i,:) = user_model.state.q;
            track_data.C_B2ENU{i} = user_model.state.C_B2ENU;
        end
        
        function track_data = Errors_UpdateTrackData(track_data, user_model)
            
            if (~isempty(track_data))
                i = size(track_data.sys_t,2)+1;
            else
                i = 1;
            end
            
            track_data.sys_t(i) = user_model.state.t*1000;
            track_data.t(i) = user_model.state.t;
            
            track_data.r(i,:) = user_model.state.r;
            track_data.v(i,:) = user_model.state.v;
            track_data.a(i,:) = user_model.state.a;
            
            track_data.ang(i,:) = user_model.state.ang ;
            track_data.w(i,:) = user_model.state.w ;
            track_data.e(i,:) = user_model.state.e;
            track_data.q(i,:) = user_model.state.q;
            track_data.C_B2ENU{i} = user_model.state.C_B2ENU;
            
            % add errors to position
            %track_data.r(i, 1) = track_data.r(i, 1) + normrnd(0, user_model.stdXY(1));
            %track_data.r(i, 2) = track_data.r(i, 2) + normrnd(0, user_model.stdXY(2));

            dt = (track_data.sys_t(i) - track_data.sys_t(1)) / 1000;

            % add errors to angles and quaternion
            track_data.ang(i, 1) = track_data.ang(i, 1) + user_model.init_angles_error(1) + user_model.angles_drift(1) * dt + randn * user_model.stdangles(1);
            track_data.ang(i, 2) = track_data.ang(i, 2) + user_model.init_angles_error(2) + user_model.angles_drift(2) * dt + randn * user_model.stdangles(2);
            track_data.ang(i, 3) = track_data.ang(i, 3) + user_model.init_angles_error(3) + user_model.angles_drift(3) * dt + randn * user_model.stdangles(3);
            
            % adding uncertainties of orientation angles to track data
            track_data.roll_uncertainty(i) = min(pi, abs(user_model.init_angles_error(1)) + abs(user_model.angles_drift(1)) * dt + user_model.stdangles(1));
            track_data.pitch_uncertainty(i) = min(pi, abs(user_model.init_angles_error(2)) + abs(user_model.angles_drift(2)) * dt + user_model.stdangles(2));
            track_data.heading_uncertainty(i) = min(pi, abs(user_model.init_angles_error(3)) + abs(user_model.angles_drift(3)) * dt + user_model.stdangles(3));
            
            track_data.q(i, :) = angle2quat( track_data.ang(i, 3), track_data.ang(i, 2), track_data.ang(i, 1) );
        end
        
        function track_data = Add_Uncertainties( track_data, ideal_track_data )
            % adding XY uncertainties to track data
            N = size(track_data.sys_t,2);
            for i = 1:N
                track_data.X_uncertainty(i) = abs( track_data.r(i, 1) - ideal_track_data.r(i, 1) );
                track_data.Y_uncertainty(i) = abs( track_data.r(i, 2) - ideal_track_data.r(i, 2) );
%                 track_data.roll_uncertainty(i) = abs( TrackBuilder.normalize_angle(track_data.ang(i, 1) - ideal_track_data.ang(i, 1)) );
%                 track_data.pitch_uncertainty(i) = abs( TrackBuilder.normalize_angle(track_data.ang(i, 2) - ideal_track_data.ang(i, 2)) );
%                 track_data.heading_uncertainty(i) = abs( TrackBuilder.normalize_angle(track_data.ang(i, 3) - ideal_track_data.ang(i, 3)) );
            end
        end
        
        function output_angle = normalize_angle( input_angle )
            output_angle = acos( cos (input_angle) );
        end
        
    end
end