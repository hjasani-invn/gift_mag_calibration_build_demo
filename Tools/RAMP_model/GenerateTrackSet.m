function GenerateTrackSet

    duration = 25;
    x0 = 0.02;       xend = 127.98;   dx = 0.1;
    y0 = 0.02;       yend = 63.98;
    z0 = 0;
    
    
    vx0 = 0;
    vy0 = (yend - y0)/duration;
    vz0 = 0;
    
    for x = x0:dx:xend
        t0 =fix( x*1000);
        str = sprintf('%.0f', t0);
        script_name = strcat('auto_scripts\2016_02_29_', str);
        wx0 = randn(1)/5;
        wy0 = randn(1)/5;
        wz0 = randn(1)/5;
        GenerateModelScript(strcat(script_name,'.txt'), t0, duration, x, y0, z0, vx0, vy0, vz0, wx0, wy0, wz0);
        model_main (script_name);
    end
end