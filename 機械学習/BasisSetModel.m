%% 初期設定（好きに設定して良い）
clc; clear; close all;  %初期化
n = 30; %データ数
a2 = 3;
a1 = -1;                %傾き
a0 = 1;                 %切片
e = 1.5;                %ノイズの大きさ

x_rand = -3 + 6*rand(n,1); %[-3,3]の範囲に適当に訓練データを生成
x = sort(x_rand);
lam = a2 + a1*cos(x) + a0*x + e*(0.5 - rand(n,1));  %訓練データにはノイズが含まれる


%% 訓練データと真の関数（ノイズなし）の描画

lam_true = a2 + a1*cos(x) + a0 * x;     %line関数でプロットするための設定:y座標

figure(1)
plot(x, lam, 'k+', 'MarkerSize', 20, 'LineWidth', 3);
hold on; grid on;
plot(x, lam_true, 'Color', 'blue', 'LineStyle', '-', 'LineWidth', 3);
xlabel('x', 'Fontsize', 20); ylabel('lambda', 'Fontsize', 20);
legend({'Training Data', 'True Function (without Noise)'}, 'Location', 'best');
set(gca, 'FontSize', 20);
axis equal;


%% 学習（単回帰）アルゴリズム
X = [ones(n,1) cos(x) x];      %行列Xの生成
c = inv(X'*X)*X'*lam;   %係数の決定（これが最適解）


%% 学習結果の描画

lam_led = c(3)*x+ c(2)*cos(x) + c(1); %line関数でプロットするための設定:y座標

 
plot(x, lam_led, 'Color', 'red', 'LineStyle', '-', 'LineWidth', 3);
legend({'Training Data', 'True Function (without Noise)', 'Solution'}, 'Location', 'best'); 