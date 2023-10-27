function main_CalcMagBias_xrun

    addpath('lib\file');

    main_path = '\\cayyc-proj01\compute02\FPL_DATA\test_data\Robotic_survey\Calibration_with_gimbal\faults.mag\'; 

    datatype = 11; % 1:mapper-mag, 11:tag-mag, 12:tag-acc, 21: dbg_output-mag, 22: dbg_output-acc, 
    
    switch datatype
        case 1
            mask = 'mgraw_*.log';
            out_file = 'mg_raw_bias.csv';
        case 11
            mask = '*_mag.csv';
            out_file = 'mg_raw_bias.csv';
        case 12
            mask = '*_imu.csv';
        out_file = 'acc_raw_bias.csv';
        case 21
            mask = 'mag*.txt';
            out_file = 'mg_raw_bias.csv';
        case 22
            mask = 'acc*.txt';
            out_file = 'acc_raw_bias.csv';
        otherwise
            mask = 'unknown.csv';
            out_file = 'unknown.csv';
   end

    fout = fopen(strcat(main_path,'\', out_file), 'wt');
    
    disp(strcat('Processing in folder: ',main_path));
    path_list = GetFolderList(main_path, mask);
    fprintf('%d subfolder with data found',size(path_list,1));

    %fout = fopen(strcat(main_path,'\', 'mag_from_rt_bias.csv'), 'wt');
    
    for i = 1:length(path_list)
        disp(strcat('Processing for the folder: ',path_list{i}));
        file_names = GetFilesOnMask(path_list{i}, mask);
        disp(strcat('Processing for the folder: ',path_list{i}, ' ,', num2str(length(file_names)), ' files'));
        for j = 1:length(file_names)
            file_name = file_names{j};
            switch datatype
                case 1
                    [t, meas] = LoadSensorMeas(file_name, 0, 0);
                    temp = [];
                case 11
                    [t, meas, temp] = LoadTagMeas(file_name, 0, 0);
                case 12
                    [t, meas, temp] = LoadTagAccMeas(file_name, 0, 0);
                case 21
                    [t, meas, temp] = LoadDbgMagMeas(file_name, 0, 0);
                case 22
                    [t, meas, temp] = LoadDbgAccMeas(file_name, 0, 0);
                otherwise
                    meas = [];  temp = [];
            end

            [avgT, varT] = CalcTemperatureAvgAndVar(temp);

%             figure;
%             hold on;
%             plot(t, meas(:,1), t, meas(:,2), t, meas(:,3));
%             grid on;
            
            [t, meas] = SelectData_avg_fit_v3(t, meas, 1, 15);
            %[t, meas] = SelectData_avg_fit_v2(t, meas, 20);
            %[t, meas] = SelectData_avg(t, meas, 20);
            mag_stat = CalcSensorMagnitudeVariation(meas, [0 0 0]');
            fprintf(fout, '%s', file_name);
            fprintf(fout, ',%d,%d', mag_stat.valid, mag_stat.n);
            fprintf(fout, ',,%f,%f', avgT, varT);
            if (mag_stat.valid)
                fprintf(fout, ',,%f,%f,%f,%f', mag_stat.avg, mag_stat.std, mag_stat.min, mag_stat.max);

                 %bias = CalcBias_MagnitudeStdMin(meas);
                 bias = CalcHardAndSoftBias_LSE_v2(meas);

                 fprintf(fout, ',,%f,%f,%f', bias(1), bias(2), bias(3));
                 if (length(bias) > 3)
                    fprintf(fout, ',');
                     for i = 4 : length(bias)
                         fprintf(fout, ',%f', bias(i));
                     end
                 end

                 mag_stat = CalcSensorMagnitudeVariation(meas, bias);
                 fprintf(fout, ',,%f,%f,%f,%f', mag_stat.avg, mag_stat.std, mag_stat.min, mag_stat.max);

                 clb_meas = CalcMagClb(meas, bias);
                 [DOP , axis_dop] = CalcDop(clb_meas);
                 fprintf(fout, ',,%f,', DOP);
                 fprintf(fout, ',,%f,%f,%f', axis_dop(1), axis_dop(2), axis_dop(3));
                 fprintf(fout, '\n');

                 
                 %plot
                 %[t, meas] = LoadSensorMeas(file_name, 0, 0);
                 %magnitude = sqrt(meas(:,1).*meas(:,1) + meas(:,2).*meas(:,2) + meas(:,3).*meas(:,3));
                 %magnitude_flt = MeanSlideWindow_Filter(magnitude, 20);

                 clb_magnitude = sqrt(clb_meas(:,1).*clb_meas(:,1) + clb_meas(:,2).*clb_meas(:,2) + clb_meas(:,3).*clb_meas(:,3));
                 clb_magnitude_flt = MeanSlideWindow_Filter(clb_magnitude, 50);
                 figure;
                 hold on;
%                      plot(t, magnitude);
%                      plot(t, magnitude_flt);
                 plot(t, clb_magnitude);
                 plot(t, clb_magnitude_flt);
                 grid on;
                 idx = strfind(file_name,'test');
                 title(file_name(idx(end):end));
                 hold off;

                 figure;scatter3(clb_meas(:,1),clb_meas(:,2),clb_meas(:,3),10,'filled');axis equal;grid on;title('Before calibration');
                 title(file_name(idx(end):end));
                 pause(0.1);
            else
            end
        end
    end
    
    fclose(fout);
    
end

function clb_mg = CalcMagClb(mg, bias)
    
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
    for i = size(mg,1):-1:1
        clb_mg(i,:) = C*(mg(i,:)' - hard_bias);
    end
end

function [DOP , axis_dop] = CalcDop(clb_mg)
    R = vecnorm(clb_mg')';
    A = clb_mg ./ R;
    Q = (A'*A)^-1;
    DOP = sqrt(trace(Q));
    axis_dop = sqrt([Q(1,1), Q(2,2), Q(3,3)]);
end

function clb_mg = CalcMagClb_quat(mg, bias)
    
    hard_bias = bias(1:3);
    if (length(bias) == 7)
        q_soft = bias(4:7)';
    else 
        q_soft = [1 0 0 0];
    end
    for i = size(mg,1):-1:1
        clb_mg(i,:) = quatrotate(q_soft, mg(i,:) - hard_bias');
    end
end

function [t, mg] = SelectData_avg(t_raw, mg_raw, avg_num)
    len2=length(mg_raw);
    maxi=floor(len2/avg_num)-1;
    t = [];
    mg = [];
    for i=1:maxi
        kb = 1 + (i - 1) * avg_num;
        ke = kb + avg_num-1;
        tav = mean(t_raw(kb:ke));
        mav = mean(mg_raw(kb:ke,:),1);
        vstd = std(mg_raw(kb:ke,:),0,1);
        if max(vstd)<0.7 %0.5
            t = [t tav];
            mg=[mg; mav];
        end
    end
end

function [avgT, varT] = CalcTemperatureAvgAndVar(temp)
    fltT = MeanSlideWindow_Filter(temp, 20);
    avgT = mean(fltT);
    varT = max(fltT) - min(fltT);
end

function [t, mg] = SelectData_avg_fit_v3(t_raw, mg_raw, avg_period, min_sample_number)
    
    %calculate magnitude profile
    sz=length(mg_raw);
    t = [];
    mg_std = [];
    data_avg = [];
    rejected = 0;
    for i=2:sz-1
        kb = FindRightNeightbour(t_raw(i) - 0.5*avg_period,t_raw);
        kb = max(kb, 1);
        ke = FindLeftNeightbour(t_raw(i) + 0.5*avg_period,t_raw);
        ke = min(ke, sz);
        
        if ((ke-kb) >= min_sample_number)
            tav = t_raw(i);
            vavg = mean(mg_raw(kb:ke,:));
            vstd = std(mg_raw(kb:ke,:),0,1);
            t = [t tav];
            mg_std = [mg_std; vstd];
            data_avg = [data_avg; vavg];
        else 
            rejected = rejected  + 1;
        end
    end
    m = sqrt( mg_std(:,1).*mg_std(:,1) + mg_std(:,2).*mg_std(:,2) +mg_std(:,3).*mg_std(:,3));
   
%         t = t - t(1);
%         figure;
%         plot (t, data_avg(:,1),t, data_avg(:,2),t, data_avg(:,3));
        
    rejected
    
       figure; 
       hold on;
       plot(t, m);
    
    % find extremums
    mmin = [];    tmin = [];
    mmax = [];    tmax = [];
    sz=length(m);
    for i=3:sz-2
        dm = m(i:i+1) - m(i-1:i);
        %minimums which less than 1
        if ( (dm(1)*dm(2) < 0) && ( dm(1) < 0) && ( m(i) < 1)) 
            mmin = [mmin; m(i)];
            tmin = [tmin; t(i)];
        end
        %maximums which more than 1
        if ( (dm(1)*dm(2) < 0) && ( dm(1) > 0) && ( m(i) > 1))
            mmax = [mmax; m(i)];
            tmax = [tmax; t(i)];
        end
        %one slope is zero
        if ((dm(1)*dm(2) == 0))
            dm = m(i-1:i+1) - m(i-2:i);
            %minimums which less than 1
            if ( (dm(1)*dm(3) < 0) && ( dm(1) < 0) && ( m(i) < 1)) 
                mmin = [mmin; m(i)];
                tmin = [tmin; t(i)];
            end
            %maximums which more than 1
            if ( (dm(1)*dm(3) < 0) && ( dm(1) > 0) && ( m(i) > 1))
                mmax = [mmax; m(i)];
                tmax = [tmax; t(i)];
            end
        end
    end
    
%        plot(tmin, mmin, '*');
%        plot(tmax, mmax, '*');

    %select optimal points
    m_opt = [];
    t_opt = [];
    sz=length(tmax);
    for i = 1:sz+1
        if ( (i-1) < 1)
            idx1 = 1;
        else
            idx1 = FindRightNeightbour(tmax(i-1),tmin);
        end
        if (i > sz)
            idx2 = length(tmin);
        else
            idx2 = FindLeftNeightbour(tmax(i),tmin); 
        end
            
       if (idx1 <= idx2)
            m_opt = [m_opt; min(mmin(idx1:idx2))];
            idx_opt = find(mmin(idx1:idx2)== m_opt(end));
            %tmin(idx1 + idx_opt - 1)            
            t_opt = [t_opt; tmin(idx1 + idx_opt(1) - 1)];
       end
    end
    
%       plot(t_opt, m_opt, '*'); grid on;
%       hold off;

    %calc average values
    sz=length(t_opt);
    t = [];
    mg = [];
    for i=1:sz
        idx = find(t_raw == t_opt(i));
        kb = FindRightNeightbour(t_raw(idx) - 0.5*avg_period,t_raw);
        kb = max(kb, 1);
        ke = FindLeftNeightbour(t_raw(idx) + 0.5*avg_period,t_raw);
        ke = min(ke, length(t_raw));
       
        tav = mean(t_raw(kb:ke));
        mav = mean(mg_raw(kb:ke,:),1);
        t = [t tav];
        mg=[mg; mav];
    end
end

function [t, mg] = SelectData_avg_fit_v2(t_raw, mg_raw, avg_num)
    
    %calculate magnitude profile
    sz=length(mg_raw);
    t = [];
    mg_std = [];
    for i=1:sz
        kb = max(i - 0.5 * avg_num, 1);
        ke = min(i + 0.5 * avg_num, sz);
        tav = mean(t_raw(i));
        vstd = std(mg_raw(kb:ke,:),0,1);
         t = [t tav];
         mg_std = [mg_std; vstd];
        %end
    end
    m = sqrt( mg_std(:,1).*mg_std(:,1) + mg_std(:,2).*mg_std(:,2) +mg_std(:,3).*mg_std(:,3));
       
%     figure; 
%     hold on;
%     plot(t, m);
    
    % find extremums
    mmin = [];    tmin = [];
    mmax = [];    tmax = [];
    for i=2:sz-1
        dm = m(i:i+1) - m(i-1:i);
        %minimums which less than 1
        if ( (dm(1)*dm(2) < 0) && ( dm(1) < 0) && ( m(i) < 1)) 
            mmin = [mmin; m(i)];
            tmin = [tmin; t(i)];
        end
        %maximums which more than 1
        if ( (dm(1)*dm(2) < 0) && ( dm(1) > 0) && ( m(i) > 1))
            mmax = [mmax; m(i)];
            tmax = [tmax; t(i)];
        end
    end
    
%     plot(tmin, mmin, '*');
    
    %select optimal points
    m_opt = [];
    t_opt = [];
    sz=length(tmax);
    for (i = 1:sz+1)
        if ( (i-1) < 1)
            idx1 = 1;
        else
            idx1 = FindRightNeightbour(tmax(i-1),tmin);
        end
        if (i > sz)
            idx2 = length(tmin);
        else
            idx2 = FindLeftNeightbour(tmax(i),tmin); 
        end
            
       if (idx1 <= idx2)
            m_opt = [m_opt; min(mmin(idx1:idx2))];
            idx_opt = find(mmin(idx1:idx2)== m_opt(end));
            t_opt = [t_opt; tmin(idx1 + idx_opt - 1)];
       end
    end
    
%     plot(t_opt, m_opt, '*'); grid on;
%     hold off;

    sz=length(t_opt);
    t = [];
    mg = [];
    mg_std = [];
    for i=1:sz
        idx = find(t_raw == t_opt(i));
        kb = max(idx - 0.5 * avg_num, 1);
        ke = min(idx + 0.5 * avg_num, length(t_raw));
        tav = mean(t_raw(kb:ke,:));
        mav = mean(mg_raw(kb:ke,:),1);
        t = [t tav];
        mg=[mg; mav];
    end
end

function [t, mg] = SelectData_avg_fit(t_raw, mg_raw, avg_num)
    sz=length(mg_raw);
    t = [];
    mg = [];
    mg_std = [];
    for i=1:sz
        kb = max(i - 0.5 * avg_num, 1);
        ke = min(i + 0.5 * avg_num, sz);
        tav = mean(t_raw(i));
        mav = mean(mg_raw(kb:ke,:),1);
        vstd = std(mg_raw(kb:ke,:),0,1);
        t = [t tav];
        mg=[mg; mav];
        mg_std = [mg_std; vstd];
    end
    m = sqrt( mg_std(:,1).*mg_std(:,1) + mg_std(:,2).*mg_std(:,2) +mg_std(:,3).*mg_std(:,3));
       
    figure; 
    hold on;
    plot(t, m);
    
    % find extremums
    mmin = [];    tmin = [];
    mmax = [];    tmax = [];
    for i=2:sz-1
        dm = m(i:i+1) - m(i-1:i);
        if ( (dm(1)*dm(2) < 0) && ( dm(1) < 0) && ( m(i) < 1))
            mmin = [mmin; m(i)];
            tmin = [tmin; t(i)];
        end
        if ( (dm(1)*dm(2) < 0) && ( dm(1) > 0) && ( m(i) > 1))
            mmax = [mmax; m(i)];
            tmax = [tmax; t(i)];
        end
    end
    
    plot(tmin, mmin, '*');
    
    m_opt = [];
    t_opt = [];
    sz=length(mmin);
    i = 1;
    while (i<FindRightNeightbour(tmax(end),tmin))
       t_left =  tmax(FindLeftNeightbour(tmin(i),tmax));
       t_right = tmax(FindRightNeightbour(tmin(i),tmax));
       idx1 =  FindRightNeightbour(t_left,tmin);
       idx2 =  FindLeftNeightbour(t_right,tmin);
       
       if (idx1 <= idx2)
            m_opt = [m_opt; min(mmin(idx1:idx2))];
            idx_opt = find(mmin(idx1:idx2)== m_opt(end));
            t_opt = [t_opt; tmin(idx1 + idx_opt - 1)];
       end
       
       i =  FindRightNeightbour(t_right,tmin);
    end
    
   
    plot(t_opt, m_opt, '*');
    
    hold off;

    sz=length(t_opt);
    t = [];
    mg = [];
    mg_std = [];
    for i=1:sz
        idx = find(t_raw == t_opt(i));
        kb = max(idx - 0.5 * avg_num, 1);
        ke = min(idx + 0.5 * avg_num, length(t_raw));
        tav = mean(t_raw(kb:ke,:));
        mav = mean(mg_raw(kb:ke,:),1);
        vstd = std(mg_raw(kb:ke,:),0,1);
        %if max(vstd)<0.7 %0.5
            t = [t tav];
            mg=[mg; mav];
            mg_std = [mg_std; vstd];
        %end
    end
    
end

function in = FindLeftNeightbour(ti, t)
    idx =  find(t < ti);
    if (isempty(idx))
        in = 1;
    else 
        in = idx(end);
    end
end

function in = FindRightNeightbour(ti, t)
    idx =  find(t > ti);
    if (isempty(idx))
        in = length(t);
    else 
        in = idx(1);
    end
end