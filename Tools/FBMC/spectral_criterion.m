function crt = spectral_criterion(X, Fs)
f_start = 0.6;
f_end = 3.0;
[f, P] = calck_spectr(X, Fs);
idx = find(f > f_start);
f = f(idx);
P = P(idx);
idx = find(f<f_end);
f = f(idx);
P = P(idx);
% figure;
% plot(f,P);
crt = sum(P);
end