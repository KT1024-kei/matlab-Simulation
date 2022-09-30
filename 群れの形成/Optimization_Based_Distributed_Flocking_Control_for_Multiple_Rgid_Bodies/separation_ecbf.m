%% 散開する関数

function tmp3 = separation_ecbf(g_ij, i, j)

global kc Dc
% 近傍との距離を取得
p_ij = g_ij;
A = p_ij';

b = kc * (norm(p_ij)^2 - Dc^2) - norm(p_ij) * norm([0.5 ,0.5, 0.5]);

tmp3 = [A, b];
%v = quadprog(H, f, A, b);
end