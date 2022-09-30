%% リヤプノフ関数を用いたフォーメーションの形成
clear; clc; close all;
%% 理想的なフォーメーションを設定
% 実験する空間は二次元
m = 2;
% 注意：全ての位置ベクトル一行の縦ベクトルで表している,奇数番目はx, 偶数番目はy軸を取る.
% 達成したいフォーメーション: 正六角形
z_des = [[-0.5, sqrt(3)/2]';
    [-1, 0]'
    [-0.5, -sqrt(3)/2]'
    [0.5, -sqrt(3)/2]'
    [1, 0]'
    [0.5, sqrt(3)/2]'];       % (12, 1)
% 初期位置はランダム
z_ini = [ 0.5377
    1.8339
   -2.2588
    0.8622
    0.3188
   -1.3077
   -0.4336
    0.3426
    3.5784
    2.7694
   -1.3499
    3.0349];
for i = 1:length(z_des)
    z_ini(i, 1) = randn(); % (12, 1)
end

% フォーメーショングラフのエージェント集合とエッジ集合
Gv = [1, 2, 3, 4, 5, 6];
Ge = [1, 2; 2, 1;2, 3;3, 2;3, 4;4, 3;4, 5;5, 4;5, 6;6, 5;
      6, 1;1, 6;1, 4;4, 1;2, 5;5, 2;3, 6;6, 3;2, 6;6, 2;
      3, 5;5, 3;1, 5;5, 1;1, 3;3, 1;4, 2;2, 4;4, 6;6, 4];
 
%% ポテンシャル関数 V = 1/4 * (norm(z)^2 - d^2)^2 と設定

% 隣接行列の構築
B = zeros(length(Gv), length(Ge));      % (6, 18)
for i = 1:length(Gv)
    for j = 1:length(Ge)
       if i == Ge(j, 1)
           B(i, j) = 1;
       elseif i == Ge(j, 2)
           B(i, j) = -1;
       end
    end
end
% 次元に合わせる
B_hat = kron(B, eye(m));        % (12, 36)

% 達成したいフォーメーションの距離制約は
d = B_hat' * z_des;             % (36, 1)

% 初期位置の隣接している各エージェントの相対距離は
z = B_hat' * z_ini;             % (36, 1)

% f(p) = z^2 としたときのこの行列のヤコビアンR
D_z = [];
for i = 1:length(Ge)
   D_z(1 + 2*(i-1):2*i, i) = z(1 + 2*(i-1):2*i); 
end
R = D_z' * B_hat'*2;                % (18, 12)

D_zdes = eye(length(Ge));


%% シミュレーションの準備

% 実行時間
simulation_time = 20;
% グラフの初期化
xlabel('x');
ylabel('y');
grid on;
hold on;
figure(1)
% 各エージェントの初期位置
agent1 = plot(gca, z_ini(1), z_ini(2), 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
agent2 = plot(gca, z_ini(3), z_ini(4), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 5);
agent3 = plot(gca, z_ini(5), z_ini(6), 'o', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
agent4 = plot(gca, z_ini(7), z_ini(8), 'o', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
agent5 = plot(gca, z_ini(9), z_ini(10), 'o', 'MarkerFaceColor', 'y', 'MarkerSize', 5);
agent6 = plot(gca, z_ini(11), z_ini(12), 'o', 'MarkerFaceColor', 'c', 'MarkerSize', 5);

% 各エージェントの軌道
agent1_commet_fig = plot(gca, z_ini(1), z_ini(2), '-', 'MarkerFaceColor', 'r', 'MarkerSize', 2);
agent2_commet_fig = plot(gca, z_ini(3), z_ini(4), '-', 'MarkerFaceColor', 'g', 'MarkerSize', 2);
agent3_commet_fig = plot(gca, z_ini(5), z_ini(6), '-', 'MarkerFaceColor', 'b', 'MarkerSize', 2);
agent4_commet_fig = plot(gca, z_ini(7), z_ini(8), '-', 'MarkerFaceColor', 'k', 'MarkerSize', 2);
agent5_commet_fig = plot(gca, z_ini(9), z_ini(10), '-', 'MarkerFaceColor', 'y', 'MarkerSize', 2);
agent6_commet_fig = plot(gca, z_ini(11), z_ini(12), '-', 'MarkerFaceColor', 'c', 'MarkerSize', 2);

agent1_commet = [];
agent2_commet = [];
agent3_commet = [];
agent4_commet = [];
agent5_commet = [];
agent6_commet = [];


% 閉ループ系の各パラメータ
e = [];
l = 2;
for k = 1:length(Ge)
    e(k, 1) = ( sqrt(z(1 + 2*(k-1))^2 + z(2*k)^2)^l - sqrt(d(1 + 2*(k-1))^2 + d(2*k)^2)^l );
end
p_dot = 0;
z_dot = 0;
e_dot = 0;
p = z_ini;

%% シミュレーション 
tmp = d;
for i = 1:simulation_time/0.01
    figure(1)
    
    % 閉ループ系

    p_dot = - R' * D_zdes * e * 10^(-2.9) + 0.01 * [1 1 zeros(10, 1)']';
    z_dot = B_hat' * p_dot;
    e_dot = 2 * D_zdes * D_z' * z_dot;
    
    p = p + p_dot;
    z = z + z_dot;
    e = e + e_dot;
%     d = tmp .* kron(ones(30, 1), [sin(i * 2* pi/1000); cos(i * 2* pi/1000)]);
%     for k = 1:length(Ge)
%     e(k, 1) = ( sqrt(z(1 + 2*(k-1))^2 + z(2*k)^2)^l - sqrt(d(1 + 2*(k-1))^2 + d(2*k)^2)^l );
%     end
    for j = 1:length(Ge)
        D_z(1 + 2*(j-1):2*j, j) = z(1 + 2*(j-1):2*j); 
    end
    R = D_z' * B_hat' * 2;
    
%     agent1_commet = [agent1_commet; [p(1) p(2)]];
%     agent2_commet = [agent2_commet; [p(3) p(4)]];
%     agent3_commet = [agent3_commet; [p(5) p(6)]];
%     agent4_commet = [agent4_commet; [p(7) p(8)]];
%     agent5_commet = [agent5_commet; [p(9) p(10)]];
%     agent6_commet = [agent6_commet; [p(11) p(12)]];
    
    
    
    
    
    % グラフ状の各エージェントの位置と軌跡を描画
    set(agent1, ...
        'XData', p(1), ...
        'YData', p(2));
%     set(agent1_commet_fig, ...
%         'XData', agent1_commet(:, 1), ...
%         'YData', agent1_commet(:, 2));
    
    
    set(agent2, ...
        'XData', p(3), ...
        'YData', p(4));
%     set(agent2_commet_fig, ...
%         'XData', agent2_commet(:, 1), ...
%         'YData', agent2_commet(:, 2));
    
    set(agent3, ...
        'XData', p(5), ...
        'YData', p(6));
%     set(agent3_commet_fig, ...
%         'XData', agent3_commet(:, 1), ...
%         'YData', agent3_commet(:, 2));
%     
    set(agent4, ...
        'XData', p(7), ...
        'YData', p(8));
%     set(agent4_commet_fig, ...
%         'XData', agent4_commet(:, 1), ...
%         'YData', agent4_commet(:, 2));
    
    set(agent5, ...
        'XData', p(9), ...
        'YData', p(10));
%     set(agent5_commet_fig, ...
%         'XData', agent5_commet(:, 1), ...
%         'YData', agent5_commet(:, 2));
    
    set(agent6, ...
        'XData', p(11), ...
        'YData', p(12));
%     set(agent6_commet_fig, ...
%         'XData', agent6_commet(:, 1), ...
%         'YData', agent6_commet(:, 2));
    
end






