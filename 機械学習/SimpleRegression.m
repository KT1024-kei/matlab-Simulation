clc; clear; close all;  %初期化
n = 20;                 %データ数
a1 = -1;                %傾き
a0 = 3;                 %切片
e = 5.0;                %ノイズの大きさ

x = [-n/2:n/2]';  %[-3,3]の範囲に適当に訓練データを生成
lam = a1*x + a0 + e*(0.5 - rand(n+1,1));  %訓練データにはノイズが含まれる


%% 訓練データと真の関数（ノイズなし）の描画
x1 = min(x);
x2 = max(x);
lam_true_min = a1*x1 + a0;
lam_true_max = a1*x2 + a0;

figure(1)
plot(x,lam,'k+', 'MarkerSize', 20, 'LineWidth', 3);
hold on; grid on;
line([x1 x2],[lam_true_min lam_true_max],'Color', 'blue', 'LineStyle', '-', 'LineWidth', 3);



%% 学習（単回帰）アルゴリズム

X = [ones(n+1,1) x];      %行列Xの生成
c = inv(X'*X)*X'*lam;   %係数の決定（これが最適解）


%% 学習結果の描画

lam_learned_min = c(2)*x1 + c(1);
lam_learned_max = c(2)*x2 + c(1);

line([x1 x2],[lam_learned_min lam_learned_max],'Color', 'red', 'LineStyle', '-', 'LineWidth', 3);