%% 姿勢をそろえる関数
function tmp1 = alignment(i, zeta_theta, neighbor)

global ke

% 近傍剛体の姿勢の和とそのノルムを計算
w_sum = 0;
theta_ans = 0;
for j = 1:length(neighbor)
    % 通信可能なエージェントとの角度の差を取得
    theta_ij = -([0 0 zeta_theta(2, 1, i)]' - [0 0 zeta_theta(2, 1, neighbor(j))]');
    % 和を計算
    w_sum = theta_ij + w_sum;
    
    % QPの解が既知であることを利用した場合
    theta_ans = theta_ans + sin(theta_ij)';
end
w_norm = norm(w_sum)^2;

% QPのための行列を計算
A = -w_sum';
b = -ke * w_norm;

tmp1 = [A b theta_ans];

end
