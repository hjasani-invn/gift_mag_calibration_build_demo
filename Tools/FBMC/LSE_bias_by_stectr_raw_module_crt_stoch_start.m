function [m_bias, m_bias_cov, level] = LSE_bias_by_stectr_raw_module_crt_stoch_start ( m_raw, m_bias_0)
    
    h = sqrt(calc_h(m_raw, m_bias_0)/2);
    disp(sprintf('start  0-iteration:  b=(%.15f,%.15f,%.15f) h=%.15f',m_bias_0(1),m_bias_0(2),m_bias_0(3), h));
    [m_bias, m_bias_cov, level] = LSE_bias_by_stectr_raw_module_crt ( m_raw, m_bias_0);
    h = sqrt(calc_h(m_raw, m_bias)/2);
    disp(sprintf('finish 0-iteration: lv=%d, b=(%.15f,%.15f,%.15f) h=%.15f',level,m_bias(1),m_bias(2),m_bias(3), h));

    
    if ((level < 3) || (h > 0.5))
        DBX = 50;
        DBY = 50;
        DBZ = 50;
        
    [r1, next_state_rnd] = external_rnd_norm(abs(m_bias_0(1)));
    [r2,  next_state_rnd] = external_rnd_norm(next_state_rnd);
    [r3,  next_state_rnd] = external_rnd_norm(next_state_rnd);

    for i = 1:12
            %if level
                m_bias_1 = m_bias_0 + [DBX*r1 DBY*r2 DBZ*r3];
            %else
                %m_bias_1 = m_bias + [DBX*randn() DBY*randn() DBZ*randn()];
            %end
            disp(sprintf('start  %d-iteration: b=(%.15f,%.15f,%.15f) h=%.15f', i, m_bias_1(1),m_bias_1(2),m_bias_1(3), h));
            [m_bias_1, m_bias_cov_1, level_1] = LSE_bias_by_stectr_raw_module_crt ( m_raw, m_bias_1);
            h1 = sqrt(calc_h(m_raw, m_bias_1)/2);
            disp(sprintf('finish %d-iteration: lv=%d, b=(%.15f,%.15f,%.15f) h=%.15f',i,level_1,m_bias_1(1),m_bias_1(2),m_bias_1(3),h1));
            if ((level_1 >= level) && (h1 < h))
                m_bias = m_bias_1;
                m_bias_cov = m_bias_cov_1;
                level = level_1;
                h = h1;
                if ((level >= 3) && (h <= 0.5))
                    break;
                end
            end
                [r1,  next_state_rnd] = external_rnd_norm(next_state_rnd);
                [r2,  next_state_rnd] = external_rnd_norm(next_state_rnd);
                [r3,  next_state_rnd] = external_rnd_norm(next_state_rnd);
            
        end
    end
    pause(1);
    h = sqrt(calc_h(m_raw, m_bias)/2);
    disp(sprintf('final result: lv=%d, b=(%.15f,%.15f,%.15f) h=%.15f',level,m_bias(1),m_bias(2),m_bias(3),h));
   
end

function q_mfp = calc_m_mfp(q_bf2mfp, m_bf)
    qm = [0, m_bf(1), m_bf(2), m_bf(3)];
    qm = quat_mlt(q_bf2mfp, qm);
    qm = quat_mlt(qm, quat_conj(q_bf2mfp));
    q_mfp = qm(2:4);
end

function [y] = calc_h(m_raw, m_bias)

    Fs = 20; %Hz - sample rate
    for i = size(m_raw, 1):-1:1
        m_bf = m_raw(i,:) - m_bias;
        m_mfp_module(i,1) = norm(m_bf);
    end
    
    y = spectral_criterion_with_grapth(m_mfp_module, Fs);
end

function [H] = calc_h_matrix(m_raw, m_bias)

    dx = 0.1;   dy = 0.1;   dz = 0.1;
    d_bias_x = [0.1 0.0 0.0];
    d_bias_y = [0.0 0.1 0.0];
    d_bias_z = [0.0 0.0 0.1];
    
    h0 = calc_h(m_raw, m_bias);
    dhx = calc_h(m_raw, m_bias+d_bias_x) - h0;
    dhy = calc_h(m_raw, m_bias+d_bias_y) - h0;
    dhz = calc_h(m_raw, m_bias+d_bias_z) - h0;
    
    H(3,1) = 0;
    H(1,1) = dhx/dx;
    H(2,1) = dhy/dy;
    H(3,1) = dhz/dz;
end

function result = quat_mlt( first, second)
    w1 = first(1);
    x1 = first(2);
    y1 = first(3);
    z1 = first(4);

    w2 = second(1);
    x2 = second(2);
    y2 = second(3);
    z2 = second(4);

    result(1) = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    result(2) = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    result(3) = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    result(4) = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
end

function q_conj = quat_conj( q)
    q_conj(1) = q(1);
    q_conj(2:4) = -q(2:4);
end

% Uniformity random number generator [0 32767]
function [rand, next_state] = external_rnd(current_state)
    state = bitand(uint64( uint64(fix(current_state)) * 1103515245 + 12345), uint64(4294967295) );
    next_state = uint64(state);
    tmp = bitshift(next_state, -16);
    rand64 = bitand(  tmp  , uint64(32767) );
    rand = uint32(rand64);
end

% normal random number generator mean == 0 and dispersion
function [norm_rand, next_state] = external_rnd_norm(current_state)
   sum = 0;
   N = 12;
   next_state = current_state;
   for i = 1:N
       [rand, next_state] = external_rnd(next_state); 
       sum = sum + double(rand);
   end
   norm_rand = ( sum * ( 1. / 32767  ) - N / 2 );
end

