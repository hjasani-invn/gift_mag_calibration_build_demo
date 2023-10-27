function mag_stat = CalcSensorMagnitudeVariation(meas, bias)
    mag_stat.valid = 1;
    mag_stat.n = 0;
        
%     start_delay_ms = 2000;
%     end_delay_ms = 1000;
%     [t, meas] = LoadSensorMeas(filename, start_delay_ms, end_delay_ms);
        
    hard_bias = bias(1:3);
    if (length(bias) == 12)
        C(1,1) = bias(4);
        C(1,2) = bias(5);
        C(1,3) = bias(6);
        C(2,1) = bias(7);
        C(2,2) = bias(8);
        C(2,3) = bias(9);
        C(3,1) = bias(10);
        C(3,2) = bias(11);
        C(3,3) = bias(12);
    else 
        C = eye(3);
    end
        
    if (~isempty(meas))
        mag_stat.valid = 1;
        
        for i = size(meas,1):-1:1
            clb_mg(i,:) = C*(meas(i,:)' - hard_bias);
        end
    
        mag_stat.n = length(clb_mg);
        magnitude = sqrt(clb_mg(:,1).*clb_mg(:,1) + clb_mg(:,2).*clb_mg(:,2) + clb_mg(:,3).*clb_mg(:,3));
        mag_stat.avg = mean(magnitude);
        mag_stat.std = std(magnitude);
        mag_stat.min = min(magnitude);
        mag_stat.max = max(magnitude);
    else 
        mag_stat.valid = 0;
        
    end
    
end