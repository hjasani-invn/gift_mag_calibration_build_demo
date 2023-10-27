function [t, meas, temp] = LoadDbgMagMeas(filename, start_delay_ms, end_delay_ms)

    t = [];
    meas = [];
    temp = [];
    
    data  = importdata(filename,' ');
   
    if (~isempty(data))
        t = data(:,1) - data(1,1);
        idx1 = find(t >= start_delay_ms);
        idx2 = find(t <= (t(end) - end_delay_ms));
        if (~isempty(idx1) && ~isempty(idx2) && (idx1(1) < idx2(end)))
            t = t(idx1(1):idx2(end), 1);
            meas = data(idx1(1):idx2(end), 2:4)/10;
            temp = data(idx1(1):idx2(end), 5);
        end
    end
end