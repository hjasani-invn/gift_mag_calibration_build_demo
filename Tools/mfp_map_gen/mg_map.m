% n - y cells
% m - x cels
% sig - вариативность в соседних клетках 0.5-1 
% mean - средний вектор поля
function mg = mg_map(n, m, sig, mean)
    N = length(mean)
    mg = zeros(n,m,N);
    
    for i=1:N
        mg(:,:,i) = ones(n,m)*mean(i);
        mg(:,:,i) = mg(:,:,i) + perlin_noise(n,m,sig);
    end
    
    figure;
    for i=1:N
        subplot(3,1,i);contourf(mg(:,:,i)); axis equal;
    end
end