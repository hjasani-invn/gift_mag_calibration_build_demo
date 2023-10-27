function  A = read_pos_log(logfile, delim) % A is 3d array with x,y,timestamp
     
    B = importdata(logfile, delim);
    
    lengthB = length(B(:,1));
    
    A = zeros(lengthB, 3);
    A(:,1) = B(:,3);
    A(:,2) = B(:,4);
    A(:,3) = B(:,1);
end
