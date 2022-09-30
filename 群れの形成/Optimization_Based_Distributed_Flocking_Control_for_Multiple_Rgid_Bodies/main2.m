%% 初期化
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
%% シミュレーション準備

sim_T = 5;

v_all = [];
w_all = [];

px = zeros(n, 1);
py = zeros(n, 1);
pz = zeros(n, 1);
tmp = [];
F = 0.001;

%% シミュレーションデータ取得

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
% 髢｢謨ｰ
% cohesion: 髮?蜷医☆繧矩未謨ｰ
% alignment: 蟋ｿ蜍｢繧偵◎繧阪∴繧矩未謨ｰ
% separation: 謨｣髢九☆繧矩未謨ｰ
% phy: 陦悟?怜ｯｾ謨ｰ髢｢謨ｰ繧定ｨ育ｮ?

% 蠎ｧ讓咎未騾｣
% g_wi: 繝輔Ξ繝ｼ繝?wi縺ｮ蜷梧ｬ｡蠎ｧ讓?
% g_wj: 繝輔Ξ繝ｼ繝?wj縺ｮ蜷梧ｬ｡蠎ｧ讓?
% g_ij: 繝輔Ξ繝ｼ繝?ij縺ｮ蜷梧ｬ｡蠎ｧ讓?
% V_wi: 繝輔Ξ繝ｼ繝?wi縺ｮ蜷梧ｬ｡蠎ｧ讓?
% v_wi: 繝輔Ξ繝ｼ繝?wi縺ｮ騾溷ｺｦ
% p_wi: 繝輔Ξ繝ｼ繝?wi縺ｮ蠎ｧ讓? 
% p_ij: 繝輔Ξ繝ｼ繝?ij縺ｮ蠎ｧ讓?
% e_wi: 繝輔Ξ繝ｼ繝?wi縺ｮ蟋ｿ蜍｢
% R_z: 蝗櫁ｻ｢霆ｸ繧築縺ｫ繧ゅ▽蝗櫁ｻ｢陦悟??

% 繝上う繝代?ｼ繝代Λ繝｡繝ｼ繧ｿ繝ｼ
% sigma: 蛻ｶ邏?繧堤ｷｩ蜥後☆繧句ｮ壽焚
% kp: 髮?蜷?(cohesion)縺ｮ蛻ｶ邏?縺ｫ蠖ｱ髻ｿ縺吶ｋ
% ke: 蟋ｿ蜍｢蜷梧悄(alignment)縺ｮ蛻ｶ邏?縺ｫ蠖ｱ髻ｿ縺吶ｋ
% kc: 謨｣髢?(separation)縺ｮ蛻ｶ邏?縺ｫ蠖ｱ髻ｿ縺吶ｋ
% Da: 蜑帑ｽ薙?ｮ陦晉ｪ∝愛螳夂ｯ?蝗ｲ
% Dc: 蜑帑ｽ薙?ｮ邨ｶ蟇ｾ陦晉ｪ∫ｯ?蝗ｲ

% 繧ｰ繝ｩ繝慕炊隲夜未騾｣
% V: 繝弱?ｼ繝蛾寔蜷?
% E: 繧ｨ繝?繧ｸ髮?蜷?
% N: 髫｣謗･髮?蜷?

% n: 蜑帑ｽ薙?ｮ謨ｰ
%% 蜷?繝代Λ繝｡繝ｼ繧ｿ繧貞?晄悄蛹?

% 繝上う繝代?ｼ繝代Λ繝｡繝ｼ繧ｿ縺ｮ險ｭ螳?
global sigma kp ke kc Da Dc theta_sum
sigma = 0;
kp = 2;
ke = 3;
kc = 1;
Dc = 4;
Da = 6;

% 蜷?蜑帑ｽ薙?ｮ蛻晄悄菴咲ｽｮ縺ｨ蟋ｿ蜍｢繧呈ｱｺ螳?
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

% 鄒､繧後?ｮ謗･邯壽ｧ矩??繧定ｨｭ螳?
G = zeros(n, n);
G(1, 2) = 1; G(1, n) = 1;
G(n, 1) = 1; G(n, n-1) = 1;
for i = 2:n-1
   G(i, i+1) = 1;
   G(i, i-1) = 1;
end


%% 繧ｷ繝溘Η繝ｬ繝ｼ繧ｷ繝ｧ繝ｳ

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
        
        % 閾ｪ霄ｫ縺ｨ騾壻ｿ｡蜿ｯ閭ｽ縺ｪ繧ｨ繝ｼ繧ｸ繧ｧ繝ｳ繝医ｒ繝ｪ繧ｹ繝医い繝?繝?
        neighbor = find(G(i, :) == 1);
        
        % 菴咲ｽｮ蜷域э
        tmp1 = cohesion(i, tmp_g_wi, neighbor);
        
        % 蟋ｿ蜍｢蜷域э
        tmp2 = alignment(i, zeta_theta, neighbor);
        
        % 陦晉ｪ∝屓驕ｿ
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
        
        % QP繧定ｧ｣縺剰｡悟?励?ｮ讒狗ｯ?
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

        % QP繧定ｧ｣縺?
        V = quadprog(H, f, A, b);
        v = [V(1) V(2) V(3)]';
        w = V(4:6);
        
        % 蟋ｿ蜍｢莉倥″縺ｮ隗｣繧堤峩謗･隗｣縺?縺溷?ｴ蜷?
        w = tmp2(5:7)';
       
        v_all = [v_all v];
        w_all = [w_all w];
    end
    for i = 1:n
        v = v_all(:, i);
        w = w_all(:, i);
        % 菴咲ｽｮ蟋ｿ蜍｢繧呈峩譁ｰ
        g_wi(1:3, 4, i) = g_wi(1:3, 4, i) + v * fy;% + [10 * sin(2*pi*t/1000) 10 * cos(2*pi*t/1000) 0]' * 10^(-2);

        % 騾溷ｺｦ蜈･蜉帙↓螟紋ｹｱ繧貞刈縺医ｋ
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


%% 菴咲ｽｮ
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
%% 蟋ｿ蜍｢
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



