function [meas] = get_meas(pos, mg, cellsize)
 Yi=cellsize/2:cellsize:cellsize*size(mg,1)-cellsize/2;
 Xi=cellsize/2:cellsize:cellsize*size(mg,2)-cellsize/2;
 
%  meas_x = interp2(Xi, Yi, mg(:,:,1), pos(:,1), pos(:,2), 'spline');
%  meas_y = interp2(Xi, Yi, mg(:,:,2), pos(:,1), pos(:,2), 'spline');
%  meas_z = interp2(Xi, Yi, mg(:,:,3), pos(:,1), pos(:,2), 'spline');
% 
 meas_x = interp2(Xi, Yi, mg(:,:,1), pos(:,1), pos(:,2));
 meas_y = interp2(Xi, Yi, mg(:,:,2), pos(:,1), pos(:,2));
 meas_z = interp2(Xi, Yi, mg(:,:,3), pos(:,1), pos(:,2));

 meas(:,3) =   meas_z;
 meas(:,1) =   meas_y;
 meas(:,2) = - meas_x;
 
end