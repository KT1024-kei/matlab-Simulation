%% 集合を達成する関数
function tmp2 = cohesion(i, g_wi, neighbor)

global  kp 

% 近傍剛体の相対位置の和とノルムを計算
p_sum = [0 0 0]';
inv_gi = pinv(g_wi(:, :, i));
g_ij = zeros(4, 4, length(neighbor));
for j = 1:length(neighbor)
    g_ij(:, :, j) = inv_gi * g_wi(:, :, neighbor(j));
    p_sum = p_sum + g_ij(1:3, 4, j);
end
p_nom = norm(p_sum)^2;


% 制約を満たす速度を計算
% H = eye(3);
% f = zeros(3, 1);
A = -p_sum';
b = -kp * p_nom;

tmp2 = [A b];

% v = quadprog(H, f, A, b);
end

