%% エージェントが一つの場合の，有界領域が[0,0][1,0][0,1][1,1]の最適配置問題
close all; clear; clc;
x = -0.5:0.05:0.5;
y = -0.5:0.05:0.5;

[X, Y] = meshgrid(x, y);
Z = zeros(21);
M = [];
N = [];
for i=1:21
    for j=1:21
        for k=1:21
            Z(i, j) = Z(i, j) + (x(k) - x(i))^2 + (y(k) - y(j))^2;
            
        end
        N = [N -Z(i, j)];
        M = [M (x(k) - x(i))^2 + (y(k) - y(j))^2];
    end
    
end
figure(1)
surf(X, Y, Z)
xlabel('X'); ylabel('Y'); zlabel('V')
set(gca, 'FontSize',20, 'FontName', 'Times')

%% エージェントが二つの場合

% 有界領域を設定する
x = 0:0.01:1;
y = 0:0.01:1;
[X, Y] = meshgrid(x, y);
lenx = length(x) ;
leny = length(y);

agent_1 = [0.1; 0.9];
agent_2 = [0.3; 0.1];

% ドロネーグラフの直線の式の傾き
a_del = (agent_1(2) - agent_2(2))/(agent_1(1) - agent_2(1));
% ドロネーグラフの直線の切片
b_del = agent_1(2) - a_del*agent_1(1) ;

% ボロノイ領域に分ける直線の傾きを求める
a_vor = -1/a_del;
% ボロノイ領域に分ける直線の切片を求める
b_vor = (agent_1(2) + agent_2(2))/2 - a_vor*(agent_1(1) + agent_2(1))/2;

% エージェントの位置をボロノイ図の境界線の描画
figure(2)
hold on
scatter([agent_1(1), agent_2(1)], [agent_1(2), agent_2(2)])
plot([0; 1], [b_vor; a_vor + b_vor])
% hold off
% この時の二つのエージェントの関数Jを求める

agent_1_J = zeros(lenx);
agent_2_J = zeros(leny);

for i=1:lenx
    for j=1:leny
       if y(j) <= a_vor*x(i) + b_vor
           for n=1:lenx
               for m=1:leny
                   if y(m) <= a_vor*x(n) + b_vor
                       agent_1_J(i, j) = agent_1_J(i, j) + (x(n) - x(i))^2 + (y(j) - y(m))^2;
                   end
               end
           end
       end
       if y(j) > a_vor*x(i) + b_vor
           for n=1:lenx
               for m=1:leny
                   if y(m) > a_vor*x(n) + b_vor
                       agent_2_J(i, j) = agent_2_J(i, j) + (x(n) - x(i))^2 + (y(j) - y(m))^2;
                   end
               end
           end
       end
    end
end
figure(3)
surf(Y, X, agent_1_J + agent_2_J);
