%% Distributed Formation Control of Multi-Agent Systems Using Complex Laplacian
clc; clear; close all;
%% 各エージェントの初期位置とつながりを表すグラフ
% 論文の中でシミュレーションしていたフォーメーション
% zeta = [-2 + 2j; 2 + 2j; -1 + 3j; 4j; 1 + 3j; -2; -1 + 1j; 2j; 1 + 1j; 2; -1 - 1j; -2j; 1- 1j; -2 - 2j; -1 - 3j; -4j; 1 - 3j; 2 - 2j];
% formation_conf = [ -1  0  3  0  0  6  0  0  0  0  0  0  0  0  0  0  0  0;
%                     0 -1  0  0  5  0  0  0  0 10  0  0  0  0  0  0  0  0;
%                     1  0 -1  4  0  0  0  0  0  0  0  0  0  0  0  0  0  0;
%                     0  0  3 -1  5  0  0  0  0  0  0  0  0  0  0  0  0  0;
%                     0  2  0  4 -1  0  0  0  0  0  0  0  0  0  0  0  0  0;
%                     1  0  0  0  0 -1  7  0  0  0 11  0  0 14  0  0  0  0;
%                     0  0  0  0  0  6 -1  8  0  0  0  0  0  0  0  0  0  0;
%                     0  0  0  0  0  0  7 -1  9  0  0  0  0  0  0  0  0  0;
%                     0  0  0  0  0  0  0  8 -1 10  0  0  0  0  0  0  0  0;
%                     0  2  0  0  0  0  0  0  9 -1  0  0 13  0  0  0  0 18;
%                     0  0  0  0  0  6  0  0  0  0 -1 12  0  0  0  0  0  0;
%                     0  0  0  0  0  0  0  0  0  0 11 -1 13  0  0  0  0  0;
%                     0  0  0  0  0  0  0  0  0 10  0 12 -1  0  0  0  0  0;
%                     0  0  0  0  0  6  0  0  0  0  0  0  0 -1 15  0  0  0;
%                     0  0  0  0  0  0  0  0  0  0  0  0  0 14 -1 16  0  0;
%                     0  0  0  0  0  0  0  0  0  0  0  0  0  0 15 -1 17  0;
%                     0  0  0  0  0  0  0  0  0  0  0  0  0  0  0 16 -1 18;
%                     0  0  0  0  0  0  0  0  0 10  0  0  0  0  0  0 17 -1;
%                   ];

% 論文で実機で実験していたフォーメーション
zeta = [-1 + 1j; -2 - 1j; -3j; 2 - 1j; 1 + 1j; 0 + 0j];
len_zeta = length(zeta);
formation_conf = [  0  2  0  0  0  6;
                    1  0  3  0  0  0;
                    0  2  0  4  0  0;
                    0  0  3  0  5  0;
                    0  0  0  4  0  6;
                    1  0  0  0  5  0;
                 ];
             
%% グラフラプラシアンの構築

% 各パラメータの初期化
p = 10^(-0) *(1 + 0);
omega = zeros(length(zeta));
L = zeros(length(zeta));

% 達成したいフォーメーションのグラフラプラシアンの構築
for n = 1:len_zeta
    
    % 近傍エージェントが2つより多いい場合
    if length(find(formation_conf(n, :) == 0)) < len_zeta - 2
        
        neighboors = find(formation_conf(n, :) > 0); % 自分の近傍のエージェントの番号の配列を取得
        two_neighboors = nchoosek(neighboors, 2); % 近傍の二項係数を取得

        for m = 1:length(two_neighboors(:, 1))
            agents = two_neighboors(m, :);

            omega(n, min(agents)) = p * (zeta(max(agents)) - zeta(n)) + omega(n, min(agents));
            omega(n, max(agents)) = p * (zeta(n) - zeta(min(agents))) + omega(n, max(agents));
        end
        
    % 近傍のエージェントの数が2つの場合
    else

        agents = find(formation_conf(n, :) > 0); % 自分の近傍のエージェントの番号の配列を取得
        omega(n, min(agents)) = p * (zeta(max(agents)) - zeta(n)); % wij = p * (zeta_k - zeta_i)
        omega(n, max(agents)) = p * (zeta(n) - zeta(min(agents))); % wik = p * (zeta_i - zeta_j)
    end
end

for n = 1:len_zeta
    for m = 1:len_zeta
       if n == m
           L(n, m) = - sum(omega(n, :));
       else
           L(n, m) = omega(n, m);
       end
    end
end

%% 安定化行列の初期設定

% 各パラメータの初期化
I = eye(len_zeta-2);
Fk = zeros(4, 1);
G = [];

% 以下のsigmaの固有値0は　n-1 n 番目なので,列基本変形を行い固有値0に対応する固有ベクトルが n-1 n 番目になるようにする.
L = [L(1:2, :); L(4, :); L(6, :); L(3, :); L(5, :)]; 

% 左複素平面に固有値を持つ対角行列　この固有値になるような安定化行列 K を求める
sigma = - 0.01 * diag([1 + 2j; 1.5 + 1j; 2 - 1j; 2.5 - 2j; 0; 0]);
% 安定化行列の初期値
K = diag([2 + 1j; 3 - 1j; 4; 4.5 + 3j; 5 - 1j; 6 - 3j]);

% 複素ラプラシアンを特異値分解し，二つの行列に分解する． L = U * V
[u, s, v] = svd(L);
v2 = v(:, 1:len_zeta-2)';
u2 = u(:, 1:len_zeta-2);
s2 = s(1:len_zeta-2, 1:len_zeta-2); 
U = u2;
V = s2 * v2;


%% 安定化行列をニュートン法で求める

for n = 1:100
    
    % 論文参照
    Ak = V * K * U;
    
    % 設計した各固有値との差分の行列式を計算
    for q = 1:len_zeta-2
        Fk(q) = det(Ak + sigma(q, q) * I);  
    end

    % 固有ベクトルの取得
    [X, x] = eig(Ak);
    A_ = X * Ak * X';
    
    % Fkベクトルの勾配？行列を計算
    for q = 1: len_zeta-2
        if Fk(q) ~= 0 + 0j 
            for p = 1: len_zeta-2
                G(q, p) = det(Ak + sigma(q, q) * I) * trace(V(:, p) * U(p, :) / ( Ak + sigma(q, q) * I));
%                 G(q, p) = det(Ak + sigma(q, q) * I) * trace( pinv(Ak + sigma(q, q) * I) * V(:, p) * U(p, :));
            end
        else
            for p = 1: len_zeta-2
                G(q, p) = trace( adjoint(Ak + sigma(q, q) * I) * V(:, p) * U(p, :));
            end
        end
    end

    
    % 安定化行列 K の更新
%     K = [diag(K(1:4, 1:4)) - eye(4, 4)/G * Fk;1;1];
    K = [diag(K(1:4, 1:4)) - pinv(G) * Fk;1;1];
    
    % K を対角行列に変換
    K = diag(K);
    
end
% K = diag(K);
% K = diag([K(4) K(2) K(3) K(1) K(2) K(6)]);
%% シミュレーション初期設定
close all;
% シミュレーション時間
simulationTime = 40;

% 各エージェントの初期位置
Init_pos_all_agent = 0.1*[[0; 1] [-3; 0] [1; 4] [-3; 2] [0.5; 0.5] [1.5; 1]];
Init_pos_all_agent_Im = 0.1 * [0+ 1j -3+ 0j 1+ 4j -3+ 2j 0.5+ 0.5j 1.5+ 1j];

% グラフの初期化

xlabel('x');
ylabel('y');

grid on;
hold on
% xlim([-200 50]); ylim([-200 50]);

% 各エージェントの位置をプロット
agent1 = plot(gca, Init_pos_all_agent(1, 1), Init_pos_all_agent(2, 1), 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
agent2 = plot(gca, Init_pos_all_agent(1, 2), Init_pos_all_agent(2, 2), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 5);
agent3 = plot(gca, Init_pos_all_agent(1, 3), Init_pos_all_agent(2, 3), 'o', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
agent4 = plot(gca, Init_pos_all_agent(1, 4), Init_pos_all_agent(2, 4), 'o', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
agent5 = plot(gca, Init_pos_all_agent(1, 5), Init_pos_all_agent(2, 5), 'o', 'MarkerFaceColor', 'y', 'MarkerSize', 5);
agent6 = plot(gca, Init_pos_all_agent(1, 6), Init_pos_all_agent(2, 6), 'o', 'MarkerFaceColor', 'c', 'MarkerSize', 5);

% 各エージェントの軌跡をプロット
agent1_commet_fig = plot(gca, Init_pos_all_agent(1, 1), Init_pos_all_agent(2, 1), '-', 'MarkerFaceColor', 'r', 'MarkerSize', 2);
agent2_commet_fig = plot(gca, Init_pos_all_agent(1, 2), Init_pos_all_agent(2, 2), '-', 'MarkerFaceColor', 'g', 'MarkerSize', 2);
agent3_commet_fig = plot(gca, Init_pos_all_agent(1, 3), Init_pos_all_agent(2, 3), '-', 'MarkerFaceColor', 'b', 'MarkerSize', 2);
agent4_commet_fig = plot(gca, Init_pos_all_agent(1, 4), Init_pos_all_agent(2, 4), '-', 'MarkerFaceColor', 'k', 'MarkerSize', 2);
agent5_commet_fig = plot(gca, Init_pos_all_agent(1, 5), Init_pos_all_agent(2, 5), '-', 'MarkerFaceColor', 'y', 'MarkerSize', 2);
agent6_commet_fig = plot(gca, Init_pos_all_agent(1, 6), Init_pos_all_agent(2, 6), '-', 'MarkerFaceColor', 'c', 'MarkerSize', 2);

agent1_commet = [];
agent2_commet = [];
agent3_commet = [];
agent4_commet = [];
agent5_commet = [];
agent6_commet = [];

% hold(gca, 'off')

% 各エージェントの初期位置
pos_Re= Init_pos_all_agent;
pos_Im = Init_pos_all_agent_Im;

% 各エージェントの軌跡を記録
pos_Re_x_commet = [];
pos_Re_y_commet = [];
%%
% シミュレーション
for i = 1:simulationTime/0.01
    
    figure(1)
    % 複素数平面での各エージェントの入力
    u_Im = -K * L * pos_Im';
    
    
%     
% %     ルートエージェントの入力を別の制御則で決める場合 できない
%     alpha = 10^(-3);
%     d = 60;
%     
%     Zf = pos_Im(1:4)';
%     Zl = pos_Im(5:6)';
%     
%     Kf = K(1:4, 1:4);
%     Lf = L(1:4, 1:4);
%     Ll = L(1:4, 5:6);
%     
%     Zl_dot = [alpha * (Zl(2) - Zl(1)) * (norm(Zl(2) - Zl(1), 2) - d^2); ...
%               alpha * (Zl(1) - Zl(2)) * (norm(Zl(1) - Zl(2), 2) - d^2)];
% 
%     Zf_dot = -Kf * Lf * Zf - Kf * Ll * Zl;
% %     Zf_dot = -K * L * pos_Im';
%     x = (-Kf * Lf * (Zf + (pinv(Lf) * Ll * Zl)) + (pinv(Lf) * Ll * Zl_dot));
    
%     % 複素数平面での各エージェントの入力
%     u_Im = [Zf_dot(1:4, 1);Zl_dot];
    
    
    % 入力を実ベクトルに変換
    u_Re_x = real(u_Im');
    u_Re_y = imag(u_Im');
    u_Re = [u_Re_x; u_Re_y];
    
    % 各エージェントの位置を更新
    pos_Im = pos_Im + 10^(-0) * u_Im';
    pos_Re = pos_Re + 10^(-7) * u_Re;
    
    % 複素数平面での各エージェントの位置はグラフラプラシアンのカーネルに収束するか？
%     L * pos_Im'
    
    %　各エージェントの軌跡を記録
    agent1_commet = [agent1_commet ; pos_Re(:, 1)'];
    agent2_commet = [agent2_commet ; pos_Re(:, 2)'];
    agent3_commet = [agent3_commet ; pos_Re(:, 3)'];
    agent4_commet = [agent4_commet ; pos_Re(:, 4)'];
    agent5_commet = [agent5_commet ; pos_Re(:, 5)'];
    agent6_commet = [agent6_commet ; pos_Re(:, 6)'];
    
    % グラフ状の各エージェントの位置と軌跡を描画
    set(agent1, ...
        'XData', pos_Re(1, 1), ...
        'YData', pos_Re(2, 1));
    set(agent1_commet_fig, ...
        'XData', agent1_commet(:, 1), ...
        'YData', agent1_commet(:, 2));
    
    
    set(agent2, ...
        'XData', pos_Re(1, 2), ...
        'YData', pos_Re(2, 2));
    set(agent2_commet_fig, ...
        'XData', agent2_commet(:, 1), ...
        'YData', agent2_commet(:, 2));
    
    set(agent3, ...
        'XData', pos_Re(1, 3), ...
        'YData', pos_Re(2, 3));
    set(agent3_commet_fig, ...
        'XData', agent3_commet(:, 1), ...
        'YData', agent3_commet(:, 2));
    
    set(agent4, ...
        'XData', pos_Re(1, 4), ...
        'YData', pos_Re(2, 4));
    set(agent4_commet_fig, ...
        'XData', agent4_commet(:, 1), ...
        'YData', agent4_commet(:, 2));
    
    set(agent5, ...
        'XData', pos_Re(1, 5), ...
        'YData', pos_Re(2, 5));
    set(agent5_commet_fig, ...
        'XData', agent5_commet(:, 1), ...
        'YData', agent5_commet(:, 2));
    
    set(agent6, ...
        'XData', pos_Re(1, 6), ...
        'YData', pos_Re(2, 6));
    set(agent6_commet_fig, ...
        'XData', agent6_commet(:, 1), ...
        'YData', agent6_commet(:, 2));
end
















%%