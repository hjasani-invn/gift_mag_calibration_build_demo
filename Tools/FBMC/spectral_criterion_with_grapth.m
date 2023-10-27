function crt = spectral_criterion_with_grapth(X, Fs)
f_start = 0.6;
f_end = 3.0;
[f, P] = calck_spectr(X, Fs);
idx = find(f > f_start);
f = f(idx);
P = P(idx);
idx = find(f<f_end);
f = f(idx);
P = P(idx);
plot(f,P, f,P.*P);
grid on;
%crt = sum(P);
crt = sum(P.*P);
pause(0.01);
end