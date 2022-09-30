%% 初期設定（第４回の問題２のようにある範囲内でランダムにデータ点を取ってからラベル付けすることを推奨）
clc; clear; close all;  %初期化
N = 20;                 %D_+とD_-それぞれのデータ数（今回は同じ数ずつ用意）
load data1;             %D_-のデータ（今回は配布データを使用するが，自分で作成しても良い）          
load data2;             %D_+のデータ


%% 訓練データと境界線の描画
figure;
plot(data2(:, 1), data2(:, 2), 'kx', 'MarkerSize', 14, 'Linewidth', 2);
hold on;
plot(data1(:, 1), data1(:, 2), 'ko', 'MarkerSize', 8, 'Linewidth', 2);
theta = 0:0.01:2*pi;                                                                             %プロット用の角度の等間隔データ
plot(1.5*cos(theta) - 2, sin(theta) - 1, 'Linestyle', '--', 'Color', 'blue', 'Linewidth', 3);    %真の楕円状の境界線のプロット
set(gca, 'FontSize',20, 'FontName', 'Times')
legend({'$\lambda=+1$', '$\lambda=-1$', 'Boundary'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 20); ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 20);
xlim([-5 1]); ylim([-3 1]);


%% 学習アルゴリズム
data = [data1; data2];                      %D_-とD_+をまとめる
Lambda = diag([-ones(1, N) ones(1, N)]);    %あわせてラベル付け
K = [];                                     %カーネル関数行列の箱

%カーネル関数行列の再現
for i = 1:2*N
    for j = 1:2*N
       K(i,j) = [1; data(i, 1); data(i, 2); data(i, 1)^2; data(i, 2)^2]'*[1; data(j, 1); data(j, 2); data(j, 1)^2; data(j, 2)^2];   
    end
end
       
H = [K zeros(2*N, 1); zeros(1, 2*N + 1)];
f = zeros(2*N + 1, 1);
A = -Lambda * [K ones(2*N, 1)];
b = -ones(2*N, 1);

c = quadprog(H, f, A, b);

%% 学習結果の描画（3つ目のf(x,y)=0のプロットは好きな方法で良い）
figure;
plot(data2(:, 1), data2(:, 2), 'kx', 'MarkerSize', 14, 'Linewidth', 2);
hold on;
plot(data1(:, 1), data1(:, 2), 'ko', 'MarkerSize', 8, 'Linewidth', 2);
fimplicit(@(x,y) c(1:2*N)'*[ones(2*N,1) data data.^2]*[1; x; y; x.^2; y.^2] + c(2*N + 1), 'Linestyle', '-', 'Color', 'red', 'LineWidth', 3);
set(gca, 'FontSize',20, 'FontName', 'Times')
legend({'$\lambda=+1$', '$\lambda=-1$', '$f(x,y)=0$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 20); ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 20);
xlim([-5 1]); ylim([-3 1]);