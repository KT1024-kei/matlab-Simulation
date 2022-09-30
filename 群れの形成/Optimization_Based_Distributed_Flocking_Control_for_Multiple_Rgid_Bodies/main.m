%% Optimaization-Based Distributed Flocking Control for Multiple Rigid Bodies
clear; close all; clc;
% 関数
% cohesion: 集合する関数
% alignment: 姿勢をそろえる関数
% separation: 散開する関数
% phy: 行列対数関数を計算

% 座標関連
% g_wi: フレームwiの同次座標
% g_wj: フレームwjの同次座標
% g_ij: フレームijの同次座標
% V_wi: フレームwiの同次座標
% v_wi: フレームwiの速度
% p_wi: フレームwiの座標 
% p_ij: フレームijの座標
% e_wi: フレームwiの姿勢
% R_z: 回転軸をzにもつ回転行列

% ハイパーパラメーター
% sigma: 制約を緩和する定数
% kp: 集合(cohesion)の制約に影響する
% ke: 姿勢同期(alignment)の制約に影響する
% kc: 散開(separation)の制約に影響する
% Da: 剛体の衝突判定範囲
% Dc: 剛体の絶対衝突範囲

% グラフ理論関連
% V: ノード集合
% E: エッジ集合
% N: 隣接集合

% n: 剛体の数
%% 各パラメータを初期化

% ハイパーパラメータの設定
global sigma kp ke kc Da Dc theta_sum
sigma = 0;
kp = 3;
ke = 3;
kc = 1;
Dc = 3;
Da = 5;

% 各剛体の初期位置と姿勢を決定
n = 20;
p_wi = zeros(3, 1, n);
e_wi = zeros(3, 3, n);
g_wi = zeros(4, 4, n);
zeta = [0 -1 0; 1 0 0 ; 0 0 0];
zeta_theta = zeros(3, 3, n);
for i = 1:n
    p_wi(:, :, i) = [double(idivide(int16(i),int16(4))), rem(i*0.5,2), 4.0]';
    p_wi(:, :, i) = 50*[rand() rand() rand()];
    theta = (pi - 0) * rand() - pi/2;
    zeta_theta(:, :, i) = zeta .* theta;
    e_wi(:, :, i) = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    g_wi(1:4, 4, i) = [p_wi(:, :, i);1];
    g_wi(1:3, 1:3, i) = e_wi(:, :, i);
end

% 群れの接続構造を設定
G = zeros(n, n);
G(1, 2) = 1; G(1, n) = 1;
G(n, 1) = 1; G(n, n-1) = 1;
for i = 2:n-1
   G(i, i+1) = 1;
   G(i, i-1) = 1;
end


%% シミュレーション

simulation_time = 10;
% plot3(px, py, pz, '.');
atti_wi = zeros(3, 3, n);

figure(1) 
v_all = [];
w_all = [];
px = zeros(n, 1);
py = zeros(n, 1);
pz = zeros(n, 1);
tmp = [];
attitude = zeros(n, 1);
fy = 0.0001;
H = eye(7);
f = zeros(7, 1)';
%%
for t = 1:fy:simulation_time
    
    
    tmp_g_wi = g_wi;
    for i = 1:n
        A = [];
        B = [];
        % 自身と通信可能なエージェントをリストアップ
        neighbor = find(G(i, :) == 1);
        
        % 位置合意
        tmp1 = coheision2(i, tmp_g_wi(1:3, 4, :), neighbor);
        A = [tmp1(1:3) 0 0 0 -1];
        b = tmp1(4);
        
        % 姿勢合意
        tmp2 = alignment(i, zeta_theta, neighbor);
        A = [A;0 0 0 tmp2(1:3) 0];
        b = [b;tmp2(4)];
        
        % 衝突回避
        for j = 1:n
           g_ij = pinv(tmp_g_wi(:, :, i)) * tmp_g_wi(:, :, j);
           if i ~= j && norm(g_ij(1:3, 4)) - Da < 0
              % cbf
%               tmp3 = separation(g_ij(1:3, 4), i, j);

              % extended cbf
              tmp3 = separation_ecbf(g_ij(1:3, 4), i, j);
              
              A = [A;tmp3(1:3) 0 0 0 0 ];
              b = [b;tmp3(4)];
              
           end
        end
        V = quadprog(H, f, A, b);
        v = [V(1) V(2) V(3)]';
        
        
        % 最適化によって求めた場合
        %w = V(4:6);
        % 姿勢付きの解を直接解いた場合
        w = tmp2(5:7)';
       
        v_all = [v_all v];
        w_all = [w_all w];
    end
    % 1ステップ移動
    for i = 1:n
        v = v_all(:, i);
        w = w_all(:, i);
        % 位置姿勢を更新
        % g_wi(1:3, 4, i) = g_wi(1:3, 4, i) + v * fy + [10 * sin(2*pi*t/1000) 10 * cos(2*pi*t/1000) 0]' * 10^(-2);

        % 速度入力に外乱を加える
        g_wi(1:3, 4, i) = g_wi(1:3, 4, i) + g_wi(1:3, 1:3, i)*(v(1:3)+ [0.5-1*rand() ; 0.5-1*rand(); 0.5-1*rand()] * 10^(0) ) * fy ;%+ [10 * sin(2*pi*t/1000) 10 * cos(2*pi*t/1000) 0]'*fy;
        
        g_wi(1:3, 1:3, i) = g_wi(1:3, 1:3, i) + g_wi(1:3, 1:3, i) * [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0] * fy;
        zeta_theta(:, :, i) = g_wi(1:3, 1:3, i);
        
        
        %plot(t, zeta_theta(2, 1, i), '.', 'Color', 'r');
        %plot(t, zeta_theta(1, 3, i), '.', 'Color', 'g');
        %plot(t, zeta_theta(3, 2, i), '.', 'Color', 'b');
        
        atti_wi(:, :, i) = g_wi(1:3, 1:3, i);
        p_wi(:, :, i) = g_wi(1:3, 4, i);
        tmp = [tmp zeta_theta(2, 1, i)];
    end
    v_all= [];
    w_all = [];
    % 位置姿勢を記録
    px = [px reshape(p_wi(1, :, :), [n, 1])];
    py = [py reshape(p_wi(2, :, :), [n, 1])];
    pz = [pz reshape(p_wi(3, :, :), [n, 1])];
    attitude = [attitude, tmp'];
    tmp = [];
    
end


%% 位置
% plot3(px, py, pz, '.')
l = ['r' 'b' 'g' 'y' 'k'];
figure(6)
view(3)
hold on
for i = 1:n
     m = rem(i, length(l));
     plot3(px(i, 1:end), py(i, 1:end), pz(i, 1:end), '.', 'Color', l(1 + m), 'LineWidth', 10)
     
end
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]')
set(gca, 'FontSize',25, 'FontName', 'Times')
xlim([-20 20]);ylim([-20 20]);zlim([-20 20]);
grid on
hold off
%% 姿勢
figure(2)
l = ['r' 'b' 'g' 'y' 'k'];
for i = 2:n
     m = rem(i, length(l));
     plot(attitude(i, 2:end), '-', 'Color', l(1 + m), 'LineWidth', 3)
     hold on
end
xlabel('T[s]'); ylabel('Yaw[rad]');
set(gca, 'FontSize',25, 'FontName', 'Times')
xlim([1, 720])
grid on
hold off
%% 衝突距離
tmpmin = 10000000;
tmpmax = 0;
for i = 1:n
   for j = i+1:n
       if i ~= j
           if tmpmin > norm([px(i, end), py(i, end), pz(i, end)] - [px(j, end), py(j, end), pz(j, end)])
              tmpmin =  norm([px(i, end), py(i, end), pz(i, end)] - [px(j, end), py(j, end), pz(j, end)]);
              minj = j;
              mini = i;
           end
           if tmpmax < norm([px(i, end), py(i, end), pz(i, end)] - [px(j, end), py(j, end), pz(j, end)])
              tmpmax = norm([px(i, end), py(i, end), pz(i, end)] - [px(j, end), py(j, end), pz(j, end)]);
              maxj = j;
              maxi = i;
           end
       end
   end
end
mindist = [];
maxdist = [];
for i = 2:length(px(1, 1:end))-1
    mindist = [mindist  norm([px(mini, i), py(mini, i), pz(mini, i)] - [px(minj, i), py(minj, i), pz(minj, i)])];
    maxdist = [maxdist  norm([px(maxi, i), py(maxi, i), pz(maxi, i)] - [px(maxj, i), py(maxj, i), pz(maxj, i)])];
end

figure(3)
plot(mindist, '.', 'Color', 'g', 'LineWidth', 3)
hold on
plot(maxdist, '-', 'Color', 'b', 'LineWidth', 3)
line([1,length(px(1, 1:end))], [Dc, Dc], 'Color', 'r', 'LineWidth', 3);
line([1,length(px(1, 1:end))], [Da, Da], 'Color', 'm', 'LineWidth', 3);
xlim([1,720])
grid on
xlabel('T[s]'); ylabel('Relative Distance[m]');
set(gca, 'FontSize',25, 'FontName', 'Times')
legend({'Min$\|p_{ij} \|$', 'Max$\|p_{ij} \|$', '$D_c$' '$D_a$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
hold off