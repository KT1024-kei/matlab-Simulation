%% �?合を達�?�する関数
function tmp2 = coheision2(i, g_wi, neighbor)

global  kp 

p_sum = [0 0 0]';
g_ij = zeros(3, 1, length(neighbor));
for j = 1:length(neighbor)
    g_ij = -(g_wi(:, :, i) - (g_wi(:, :, neighbor(j)) - [0.01;0.01; 0.01]));
    p_sum = p_sum + g_ij;
end

p_nom = norm(p_sum)^2;


% 制�?を�?たす速度を計�?
% H = eye(3);
% f = zeros(3, 1);
A = -p_sum';
b = -kp * p_nom;

tmp2 = [A b];

% v = quadprog(H, f, A, b);
end

