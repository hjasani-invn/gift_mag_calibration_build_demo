function im = perlin_noise(n, m, sig)
    im = zeros(n,m);
    i = 0;
    w = sqrt(n*m);

    figure, hold on, axis equal;
    while w > 1
        i = i + 1;
        i_n = floor(max(n/(2^(max(i-2,0))), 1) + 0.5);
        i_m = floor(max(m/(2^(max(i-2,0))), 1) + 0.5);
        intert_n_count = (i_n-1)*2^(i-1)+1;
        if (intert_n_count < n) 
            i_n = i_n + 1;
        end
        intert_m_count = (i_m-1)*2^(i-1)+1;
        if (intert_m_count < m) 
            i_m = i_m + 1;
        end
        
        d = interp2(sig*randn(i_n, i_m), i-1, 'spline');
        im = im + i*d(1:n, 1:m);
%         w = w - max(ceil(w/2 - 1),1);
        w = ceil(w/2);
        imagesc(im); colormap gray;
        %pause(0.5);
    end
end