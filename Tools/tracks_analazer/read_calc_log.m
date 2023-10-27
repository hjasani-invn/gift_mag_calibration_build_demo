function  A = read_calc_log(logfile, delim) % A is 3d array with x,y,timestamp
     
    B = importdata(logfile, delim);
    
    lengthB = length(B(:,1));
    
    A = zeros(lengthB, 3);
    A(:,1) = B(:,1);
    A(:,2) = B(:,2);
    A(:,3) = B(:,6);
end
