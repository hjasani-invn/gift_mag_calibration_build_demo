function bias = CalcBias_MagnitudeStdMin(raw_mg)
    
    %initial bias
    bias = zeros(1,size(raw_mg,2))';
    
%      bias(1) = 50 * randn();
%      bias(2) = 50 * randn();
%      bias(3) = 50 * randn();
    
    %raw filtering
    mg(:,1) = MeanSlideWindow_Filter (raw_mg(:,1), 20);
    mg(:,2) = MeanSlideWindow_Filter (raw_mg(:,2), 20);
    mg(:,3) = MeanSlideWindow_Filter (raw_mg(:,3), 20);
%      mg(:,1) = raw_mg(:,1);
%     mg(:,2) = raw_mg(:,2);
%     mg(:,3) = raw_mg(:,3);
    
    %initial iteration step
    h = 0.1;
    delta_bias = [h h h]';
    lamda = 1;
    cnt = 1000;
    %iteration cicle: gradient descend
    while ((lamda > 1e-6) && (cnt > 0))
        cnt = cnt - 1;
        mag_mg_std = CalcMagMgStd(mg, bias); % calc mag std
        %calc gradient
        for i = 1:3
            dB = zeros(1,size(bias,1))';
            dB(i) = h;
            mag_mg_std2 = CalcMagMgStd(mg, bias+dB);
            dy = mag_mg_std2 - mag_mg_std;
            DF(i) = dy / h;
        end
        
        %callc nex bias estimate
        delta_bias = -lamda*DF';
        mag_mg_std2 = CalcMagMgStd(mg, bias+delta_bias);
        dy = mag_mg_std2 - mag_mg_std;
        fprintf('%f; %f, %f, %f; %f %f %f\n', lamda, mag_mg_std, mag_mg_std2, dy, bias(1), bias(2), bias(3));
        if ( dy < 0)
            lamda = min(lamda*1.1,100);
        else
            delta_bias = [0 0 0]';
            lamda = lamda/2;
        end
        
        bias = bias + delta_bias;
    end
end

function mag_mag_clb_mg_std = CalcMagMgStd(mg, bias)
    for i = size(bias,1):-1:1
        clb_mg(:,i) = mg(:,i) - bias(i);
    end
    mag_clb_mg = vecnorm(clb_mg,2,2);
    mag__mg = vecnorm(mg,2,2);
    
    mag_mag_clb_mg_std = std(mag_clb_mg);
end

