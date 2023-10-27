function [m_bias, m_bias_cov, level] = LSE_bias_by_stectr_raw_module_crt ( m_raw, m_bias_0)

    m_bias = m_bias_0;
    delta_m_bias = [100,-200,300]';
    lamda = 50;
    cnt = 0;
    max_count = 500;
    
    while((norm(delta_m_bias) > 0.0001)&&(cnt <= max_count))
        
    
        cnt;
        m_bias;
                    %disp(sprintf(' bias  %.15f  %.15f  %.15f', m_bias(1),m_bias(2),m_bias(3)));
        H = calc_h_matrix(m_raw, m_bias);
        
        H;
                    %disp(sprintf(' H  %.15f  %.15f  %.15f', H(1),H(2),H(3)));

        
        
        lamda = lamda * 1.1;
                    %disp(sprintf(' lamda  %.15f', lamda));
                
        delta_m_bias = -H*lamda;
        
        y0 = calc_h( m_raw, m_bias);
        y1 = calc_h( m_raw, m_bias + delta_m_bias');
        
        if(y1 < y0)
            m_bias = m_bias + delta_m_bias';
        else 
            lamda = lamda/2;
        end
        cnt = cnt + 1;
        %y = calc_h( m_raw, m_bias);
        %disp(sprintf('%d, (%.2f,%.2f,%.2f), (%.4f,%.4f,%.4f), %.6f, %.1f',cnt,m_bias(1),m_bias(2),m_bias(3), delta_m_bias(1), delta_m_bias(2), delta_m_bias(3), y, lamda));
    end
        y = calc_h( m_raw, m_bias);
    H;
    m_bias_cov = sqrt(y/2)* norm(H)^-2 *H*H';
    %m_bias_cov = sqrt(y/2)*(H*H')^-1;
    
    % bias class estimation
    if (cnt < 10)
        m_bias_cov(3,3) = 0;
        level = 0;
    elseif (cnt < 100)
        level = 3;
    elseif ( cnt < 250)
        level = 2;
    elseif ( cnt < max_count)
        level = 1;
    else
        level = 0;
    end
    
    cut_off_threshold = 2;
    if (sqrt(y/2) > cut_off_threshold)
        level = 0;
    end
    
end

function [y] = calc_h(m_raw, m_bias)

    Fs = 20; %Hz - sample rate
    for i = size(m_raw, 1):-1:1
        m_bf = m_raw(i,:) - m_bias;
        m_mfp_module(i,1) = sqrt(m_bf(1)*m_bf(1) + m_bf(2)*m_bf(2) + m_bf(3)*m_bf(3));
    end
    
    %y = spectral_criterion_with_grapth(m_mfp_module, Fs);
    y = spectral_criterion_local(m_mfp_module, Fs);
end

function [H] = calc_h_matrix(m_raw, m_bias)

    dx = 1;   dy = 1;   dz = 1;
    d_bias_x = [1 0.0 0.0];
    d_bias_y = [0.0 1 0.0];
    d_bias_z = [0.0 0.0 1];
    
    h0 = calc_h(m_raw, m_bias);
    dhx = calc_h(m_raw, m_bias+d_bias_x) - h0;
    dhy = calc_h(m_raw, m_bias+d_bias_y) - h0;
    dhz = calc_h(m_raw, m_bias+d_bias_z) - h0;
    
    H(3,1) = 0;
    H(1,1) = dhx/dx;
    H(2,1) = dhy/dy;
    H(3,1) = dhz/dz;
end

function crt = spectral_criterion_local(X, Fs)
f_start = 0.6;
f_end = 3.0;
%f_start = 0.7;
%f_end = 1.8;
[f, P] = calck_spectr(X, Fs);
idx = find(f > f_start);
f = f(idx);
P = P(idx);
idx = find(f<f_end);
f = f(idx);
P = P(idx);
% figure;
%  plot(f,P);
%  pause(0.001);
%crt = sum(P);
crt = sum(P.*P);
%crt = max(P.*P);
%crt = max(P);
end
