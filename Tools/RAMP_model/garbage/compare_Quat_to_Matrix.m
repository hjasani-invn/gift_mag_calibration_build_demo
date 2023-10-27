clc;
clear all;
close all;

M = load('script3_ont_Matrix.dat');
Q = load('script3_ont_Quat.dat');


for i = 1:size(Q, 1)
    Diff(i, 1) = AngleCMP( Q(i, 2) , M(i, 2) );
    Diff(i, 2) = AngleCMP( Q(i, 3) , M(i, 3) );
    Diff(i, 3) = AngleCMP( Q(i, 4) , M(i, 4) );
end

figure;
subplot(3,1,1); plot(Diff(:,1));
subplot(3,1,2); plot(Diff(:,2));
subplot(3,1,3); plot(Diff(:,3));