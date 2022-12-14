%% ú»
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
%% V~[Võ

sim_T = 5;

v_all = [];
w_all = [];

px = zeros(n, 1);
py = zeros(n, 1);
pz = zeros(n, 1);
tmp = [];
F = 0.001;

%% V~[Vf[^æ¾

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
% é¢æ°
% cohesion: é?åããé¢æ°
% alignment: å§¿å¢ãããããé¢æ°
% separation: æ£éããé¢æ°
% phy: è¡å?å¯¾æ°é¢æ°ãè¨ç®?

% åº§æ¨é¢é£
% g_wi: ãã¬ã¼ã?wiã®åæ¬¡åº§æ¨?
% g_wj: ãã¬ã¼ã?wjã®åæ¬¡åº§æ¨?
% g_ij: ãã¬ã¼ã?ijã®åæ¬¡åº§æ¨?
% V_wi: ãã¬ã¼ã?wiã®åæ¬¡åº§æ¨?
% v_wi: ãã¬ã¼ã?wiã®éåº¦
% p_wi: ãã¬ã¼ã?wiã®åº§æ¨? 
% p_ij: ãã¬ã¼ã?ijã®åº§æ¨?
% e_wi: ãã¬ã¼ã?wiã®å§¿å¢
% R_z: åè»¢è»¸ãzã«ãã¤åè»¢è¡å??

% ãã¤ãã?¼ãã©ã¡ã¼ã¿ã¼
% sigma: å¶ç´?ãç·©åããå®æ°
% kp: é?å?(cohesion)ã®å¶ç´?ã«å½±é¿ãã
% ke: å§¿å¢åæ(alignment)ã®å¶ç´?ã«å½±é¿ãã
% kc: æ£é?(separation)ã®å¶ç´?ã«å½±é¿ãã
% Da: åä½ã?®è¡çªå¤å®ç¯?å²
% Dc: åä½ã?®çµ¶å¯¾è¡çªç¯?å²

% ã°ã©ãçè«é¢é£
% V: ãã?¼ãéå?
% E: ã¨ã?ã¸é?å?
% N: é£æ¥é?å?

% n: åä½ã?®æ°
%% å?ãã©ã¡ã¼ã¿ãå?æå?

% ãã¤ãã?¼ãã©ã¡ã¼ã¿ã®è¨­å®?
global sigma kp ke kc Da Dc theta_sum
sigma = 0;
kp = 2;
ke = 3;
kc = 1;
Dc = 4;
Da = 6;

% å?åä½ã?®åæä½ç½®ã¨å§¿å¢ãæ±ºå®?
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

% ç¾¤ãã?®æ¥ç¶æ§é??ãè¨­å®?
G = zeros(n, n);
G(1, 2) = 1; G(1, n) = 1;
G(n, 1) = 1; G(n, n-1) = 1;
for i = 2:n-1
   G(i, i+1) = 1;
   G(i, i-1) = 1;
end


%% ã·ãã¥ã¬ã¼ã·ã§ã³

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
        
        % èªèº«ã¨éä¿¡å¯è½ãªã¨ã¼ã¸ã§ã³ãããªã¹ãã¢ã?ã?
        neighbor = find(G(i, :) == 1);
        
        % ä½ç½®åæ
        tmp1 = cohesion(i, tmp_g_wi, neighbor);
        
        % å§¿å¢åæ
        tmp2 = alignment(i, zeta_theta, neighbor);
        
        % è¡çªåé¿
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
        
        % QPãè§£ãè¡å?ã?®æ§ç¯?
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

        % QPãè§£ã?
        V = quadprog(H, f, A, b);
        v = [V(1) V(2) V(3)]';
        w = V(4:6);
        
        % å§¿å¢ä»ãã®è§£ãç´æ¥è§£ã?ãå?´å?
        w = tmp2(5:7)';
       
        v_all = [v_all v];
        w_all = [w_all w];
    end
    for i = 1:n
        v = v_all(:, i);
        w = w_all(:, i);
        % ä½ç½®å§¿å¢ãæ´æ°
        g_wi(1:3, 4, i) = g_wi(1:3, 4, i) + v * fy;% + [10 * sin(2*pi*t/1000) 10 * cos(2*pi*t/1000) 0]' * 10^(-2);

        % éåº¦å¥åã«å¤ä¹±ãå ãã
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


%% ä½ç½®
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
%% å§¿å¢
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



