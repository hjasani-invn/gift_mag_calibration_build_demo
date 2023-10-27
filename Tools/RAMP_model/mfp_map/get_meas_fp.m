function [meas] = get_meas_fp(pos, mg, cellsize)

 for i = size(pos,1):-1:1;
    x_idx = ceil(pos(i,2)/cellsize);
    y_idx = ceil(pos(i,1)/cellsize);
    meas_x(i) = mg(x_idx,y_idx,1);
    meas_y(i) = mg(x_idx,y_idx,2);
    meas_z(i) = mg(x_idx,y_idx,3);
 end
 
 % Conversion from MFP frame to model local frame
 meas(:,3) =  meas_z; % z-meas 
 meas(:,1) =  meas_y; % x-meas 
 meas(:,2) = -meas_x; % y-meas 

end