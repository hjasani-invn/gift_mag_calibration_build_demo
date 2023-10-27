fid = fopen('availability.dat', 'rb');
%K = 3420;
M = 47;
N = 71;
K = M * N
A = fread(fid, K, 'int16');
B = reshape(A,M,N);

pcolor(B);

axis equal;
