% visualization reference and calculated track
% and calculate difference between these

figure;

% plan sizes
xv_size = 24;
yv_size = 15;

% plan name
img = imread('regus_plan_gray.bmp');
image(img),axis equal, hold on;
title( 'InvenSense Moscow Office' ); 
%axis off;
grid on
xlabel( '0 - 24m' ); ylabel( '0 - 15m' );
%axis( [ 0, xv_size, 0, yv_size ] ) 

% bitmap size
xim_size = length( img(1,:,1) );
yim_size = length( img(:,1, 1) );
   
% read logs
% calculated track
A = read_calc_log('pdr_mag.log', ' ' );
% true track
B = read_pos_log('pos_att_2016_xx_xx.log', ',');

% visualization
hPlotB = plot(B(:,1)/xv_size*xim_size, yim_size - B(:,2)/yv_size*yim_size, 'r');
set( hPlotB, 'LineWidth', 2 );

%hPlotA = plot(A(:,1)/xv_size*xim_size, yim_size - A(:,2)/yv_size*yim_size, 'b-', A(:,1)/xv_size*xim_size, yim_size - A(:,2)/yv_size*yim_size, 'bo');
%set( hPlotA, 'LineWidth', 2 );

plot(A(:,1)/xv_size*xim_size, yim_size - A(:,2)/yv_size*yim_size, 'b-', ...
A(:,1)/xv_size*xim_size, yim_size - A(:,2)/yv_size*yim_size, 'bo', ...
'LineWidth',2, ...
    'MarkerSize',5,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5] ...
);

avererr = calc_diff(A, B, 'errors.txt');
