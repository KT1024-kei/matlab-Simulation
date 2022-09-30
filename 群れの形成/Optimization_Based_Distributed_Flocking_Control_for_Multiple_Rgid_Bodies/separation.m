%% 散開する関数

function tmp3 = separation(g_ij, i, j)

global kc Dc
% 近傍との距離を取得
% g_ij = pinv(g_wi(:, :, i)) * g_wi(:, :, j);
p_ij = g_ij;
A = p_ij';
b = kc * (norm(p_ij)^2 - Dc^2);

tmp3 = [A  b];
%v = quadprog(H, f, A, b);
end