%% n-ノルムの計算
function nnorm = n_norm(z)
    global epsilon;

    nnorm = (1/epsilon) * (sqrt(1 + epsilon * norm(z)^2) - 1);
    
end
