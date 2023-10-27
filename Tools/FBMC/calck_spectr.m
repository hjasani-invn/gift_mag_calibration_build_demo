function [f, P1] = calck_spectr(X, Fs)
L = size(X,1);
Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:fix(L/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
%P3 = 2*P1(2:end-1);
f = Fs*(0:fix(L/2))/L;
end 