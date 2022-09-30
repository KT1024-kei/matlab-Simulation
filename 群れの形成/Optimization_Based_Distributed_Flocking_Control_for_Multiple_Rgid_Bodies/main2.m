%% ������
global kp ke kc Da Dc

kp = 2;
ke = 3;
kc = 1;
Dc = 6;
Da = 7;

n = 5;
p_wi = zeros(3, 1, n);

for i = 1:n
   p_wi(:, :, i) = [5*i 5*i 5*i]';
end

G = ones(n) - eye(n);
%% �V�~�����[�V��������

sim_T = 5;

v_all = [];
w_all = [];

px = zeros(n, 1);
py = zeros(n, 1);
pz = zeros(n, 1);
tmp = [];
F = 0.001;

%% �V�~�����[�V�����f�[�^�擾

for t = 1:F:sim_T
    px = [px reshape(p_wi(1, :, :), [n, 1])];
    py = [py reshape(p_wi(2, :, :), [n, 1])];
    pz = [pz reshape(p_wi(3, :, :), [n, 1])];
    
    for i=1:n
       A = [];
       b = [];
       neighbor = find(G(i, :) == 1);
       
       tmp1 = coheision2(i, p_wi, neighbor);
       
       tmp3 = [];
       
       A = [tmp1(1:3) -1];
       b = tmp1(4);
       for j = 1:n
           p_ij = p_wi(:, :, j) - p_wi(:, :, i);
          if i ~= j &&  norm(p_ij) - Da < 0
%               p_ij
              tmp3 = separation(p_ij, i, j);
              A = [A;tmp3(1:3) 0];
              b = [b tmp3(4)];
          end
       end
%        A
        H = eye(4);
        f = zeros(4, 1)';
        V = quadprog(H, f, A, b');
        v = [V(1) V(2) V(3)]';      
        v_all = [v_all v];   
%         A
    end
    for i=1:n
        v = v_all(:, i);
        p_wi(:, :, i) = p_wi(:, :, i) + v * F;
    end
    v_all = [];
end

%%
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
for i = 2:length(px(1, 1:end))
    mindist = [mindist  norm([px(mini, i), py(mini, i), pz(mini, i)] - [px(minj, i), py(minj, i), pz(minj, i)])];
    maxdist = [maxdist  norm([px(maxi, i), py(maxi, i), pz(maxi, i)] - [px(maxj, i), py(maxj, i), pz(maxj, i)])];
end

figure(3)
plot(mindist, '.', 'Color', 'g', 'LineWidth', 5)
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

%% Optimaization-Based Distributed Flocking Control for Multiple Rigid Bodies
clear; close all; clc;
% 関数
% cohesion: �?合する関数
% alignment: 姿勢をそろえる関数
% separation: 散開する関数
% phy: 行�?�対数関数を計�?

% 座標関連
% g_wi: フレー�?wiの同次座�?
% g_wj: フレー�?wjの同次座�?
% g_ij: フレー�?ijの同次座�?
% V_wi: フレー�?wiの同次座�?
% v_wi: フレー�?wiの速度
% p_wi: フレー�?wiの座�? 
% p_ij: フレー�?ijの座�?
% e_wi: フレー�?wiの姿勢
% R_z: 回転軸をzにもつ回転行�??

% ハイパ�?�パラメーター
% sigma: 制�?を緩和する定数
% kp: �?�?(cohesion)の制�?に影響する
% ke: 姿勢同期(alignment)の制�?に影響する
% kc: 散�?(separation)の制�?に影響する
% Da: 剛体�?�衝突判定�?囲
% Dc: 剛体�?�絶対衝突�?囲

% グラフ理論関連
% V: ノ�?�ド集�?
% E: エ�?ジ�?�?
% N: 隣接�?�?

% n: 剛体�?�数
%% �?パラメータを�?�期�?

% ハイパ�?�パラメータの設�?
global sigma kp ke kc Da Dc theta_sum
sigma = 0;
kp = 2;
ke = 3;
kc = 1;
Dc = 4;
Da = 6;

% �?剛体�?�初期位置と姿勢を決�?
n = 5;
p_wi = zeros(3, 1, n);
e_wi = zeros(3, 3, n);
g_wi = zeros(4, 4, n);
zeta = [0 -1 0; 1 0 0 ; 0 0 0];
zeta_theta = zeros(3, 3, n);
for i = 1:n
    p_wi(:, :, i) = [5*i 5*i 5*i]';
    theta = (pi - 0) * rand() - pi/2;
    zeta_theta(:, :, i) = zeta .* theta;
    e_wi(:, :, i) = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    g_wi(1:4, 4, i) = [p_wi(:, :, i);1];
    g_wi(1:3, 1:3, i) = e_wi(:, :, i);
end

% 群れ�?�接続構�??を設�?
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
fy = 0.001;
%%
for t = 1:fy:simulation_time
    
    
    tmp_g_wi = g_wi;
    px = [px reshape(p_wi(1, :, :), [n, 1])];
    py = [py reshape(p_wi(2, :, :), [n, 1])];
    pz = [pz reshape(p_wi(3, :, :), [n, 1])];
%     figure(1)
%     plot3(px, py, pz, '.');
%     hold on
%     figure(1)
%     plot(px, py, 'o');
%     xlim([-20, 20]*3); ylim([-20, 20]*3);zlim([-20, 20]*3);
%     xlim([-5, 5]); ylim([-5, 5]);zlim([-5, 5]);
    for i = 1:n
        
        % 自身と通信可能なエージェントをリストア�?�?
        neighbor = find(G(i, :) == 1);
        
        % 位置合意
        tmp1 = cohesion(i, tmp_g_wi, neighbor);
        
        % 姿勢合意
        tmp2 = alignment(i, zeta_theta, neighbor);
        
        % 衝突回避
        tmp3 = [];
        
        flag = "False";
        for j = 1:n
            g_ij = tmp_g_wi(1:3, 4, i) - tmp_g_wi(1:3, 4, j);
           if i ~= j && norm(g_ij) - Da < 0
              % cbf
              tmp3 = [tmp3; separation(g_ij, i, j)];
              % extended cbf
              %tmp3 = [tmp3; separation_ecbf(tmp_g_wi, i, j)];
              flag = "True";
           else
               norm(tmp_g_wi(1:3, 4, i) - tmp_g_wi(1:3, 4, j));
           end
        end
        
        % QPを解く行�?��?�構�?
        l = size(tmp3);
        if flag == "False"
            A = [tmp1(1:3) 0 0 0 -1; 0 0 0 tmp2(1:3) 0];
            b = [tmp1(4) tmp2(4)]';
        else

            A = [tmp1(1:3) 0 0 0 1;tmp3(:, 1:3) zeros(l(1), 4) ;0 0 0 tmp2(1:3) 0];
            b = [tmp1(4) tmp3(:, 4)' tmp2(4)]';
        end

        H = eye(7);
        f = zeros(7, 1);

        % QPを解�?
        V = quadprog(H, f, A, b);
        v = [V(1) V(2) V(3)]';
        w = V(4:6);
        
        % 姿勢付きの解を直接解�?た�?��?
        w = tmp2(5:7)';
       
        v_all = [v_all v];
        w_all = [w_all w];
    end
    for i = 1:n
        v = v_all(:, i);
        w = w_all(:, i);
        % 位置姿勢を更新
        g_wi(1:3, 4, i) = g_wi(1:3, 4, i) + v * fy;% + [10 * sin(2*pi*t/1000) 10 * cos(2*pi*t/1000) 0]' * 10^(-2);

        % 速度入力に外乱を加える
        %g_wi(1:3, 4, i) = g_wi(1:3, 4, i) + v(1:3) * 10^(-1.5) + [randn() ; randn(); randn()] * 10^(-2);
        
        
%         g_wi(1:3, 1:3, i) = g_wi(1:3, 1:3, i) + [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0] * fy;
        g_wi(1:3, 1:3, i) = g_wi(1:3, 1:3, i);

        zeta_theta(:, :, i) = g_wi(1:3, 1:3, i);
        
        
        plot(t, zeta_theta(2, 1, i), '.', 'Color', 'r');
        plot(t, zeta_theta(1, 3, i), '.', 'Color', 'g');
        plot(t, zeta_theta(3, 2, i), '.', 'Color', 'b');
        
        atti_wi(:, :, i) = g_wi(1:3, 1:3, i);
        p_wi(:, :, i) = g_wi(1:3, 4, i);
        tmp = [tmp zeta_theta(2, 1, i)];
    end
    v_all= [];
    w_all = [];
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
     plot3(px(i, end), py(i, end), pz(i, end), '.', 'Color', l(1 + m), 'LineWidth', 3)
     
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
%%
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
for i = 2:length(px(1, 1:end))
    mindist = [mindist  norm([px(mini, i), py(mini, i), pz(mini, i)] - [px(minj, i), py(minj, i), pz(minj, i)])];
    maxdist = [maxdist  norm([px(maxi, i), py(maxi, i), pz(maxi, i)] - [px(maxj, i), py(maxj, i), pz(maxj, i)])];
end

figure(3)
plot(mindist, '.', 'Color', 'g', 'LineWidth', 5)
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



