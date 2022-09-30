%% Flocking for Multi-agent System 
%論文で紹介されいてる三つのアルゴリズムのうち二つを実行する
%{
    エージェントのキネマティック
    q_dot = p :速度
    p_dot = u :加速度
    q, p, u は二次元ベクトル
    
    Ni :エージェントiの近傍集合
    E : グラフのエッジの集合
    
    r :エージェント間の相互作用を有無範囲
    d :エージェント間の距離　ユークリッドノルム(scale)
    kap = r/d ;(ratio)
    sigma :quasi alpha-Lattice線形制約を緩和する
    
    E : deviation energy
    a :関数phyのパラメータ
    b :関数phyのパラメータ
    c :abs(a-b)/sqrt(4*a*b)

    % 関数
    
    n_norm :本論文で定義されているn-normを計算する
    lo_h :本論文で定義されている隆起関数を計算する
    phy :パラメータa, b, cを取る本論文で定義されたシグモイド関数
    phy_alpha :関数phyとlo_hの積
    phy_alpha2 :phy_alphaの積分
    omega(z) :z/sqrt(1 + z**2)
    
%}



%% Algorithm1
clear; close all; clc;
global d_alpha r_alpha epsilon a b c h

% エージェントの数
n = 30;
% エージェントの初期位置
all_pos = zeros(n, 2);
all_vel = zeros(n, 2);
for i = 1:n
   all_pos(i, :) = [randn() randn()] * 20; 
end

figure(1);
xlabel('x');
ylabel('y');
xlim([-20, 20] * 100); ylim([-20, 20] * 100);
grid on;

simulation_time = 20;

d = 7;
r = 1.6 * d;
epsilon = 0.1;
a = 5; b = 5; c = abs(a - b)/sqrt(4*a*b);
h = 0.2;
d_alpha = n_norm(d);
r_alpha = n_norm(r);
E = [];
u = zeros(n, 2);
c1 = 0.01;
c2 = 0.01;
%%
for i = 1:simulation_time/0.001
   figure(1) 
   plot(all_pos(:, 1), all_pos(:, 2), 'o', 'MarkerSize', 10);
   %quiver(all_pos(:, 1), all_pos(:, 2), all_vel(:, 1), all_vel(:, 2), 'LineStyle', '-', 'Marker', '+');
   xlim([-20, 20]*3); ylim([-20, 20]*3);
   for j = 1:n
      for k = j+1:n
         if norm(all_pos(k, :) - all_pos(j, :)) <= r
             E = [E; [j k] ;[k j]];
         end
      end
   end
   
   for j = 1:n
       tmp = find(E(:, 1) == j);
       
       for k = 1:length(tmp)
           pos_diff = -(all_pos(j, :) - all_pos(E(tmp(k), 2), :));
           vel_diff = -(all_vel(j, :) - all_vel(E(tmp(k), 2), :));
%            u(j, :) = u(j, :) + phy_alpha_z(n_norm(pos_diff)) * (pos_diff/sqrt(1 + epsilon*norm(pos_diff)^2)) %+ lo_h(n_norm(pos_diff)/r_alpha)*vel_diff;
           u(j, :) = u(j, :) + phy_alpha_z(n_norm(pos_diff))/(1 + epsilon * n_norm(pos_diff)) * pos_diff + lo_h(n_norm(pos_diff)/r_alpha)*vel_diff...
                             + -c1 * (all_pos(j, :) - [10*cos(2 * pi * i/1000) 10*sin(2 * pi * i/1000)]) -c2 * (all_vel(j, :) - 0.01*[-sin(2 * pi * i/1000) cos(2 * pi * i/1000)]);
       end

   end
   i
   all_vel = all_vel + u*10^(-1);
   all_pos = all_pos + all_vel*10^(-1);
   E = [];
   u = zeros(n, 2);
end





