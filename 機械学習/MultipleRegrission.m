clc; clear; close all;  %初期化
n = 20;                 %データ数
a2 = 2;
a1 = -1;                %傾き
a0 = 3;                 %切片
e = 5.0;                %ノイズの大きさ

x = -3 + 6*rand(n,1);  %[-3,3]の範囲に適当に訓練データを生成
y = 3 + 4*rand(n,1);
lam = a2*y + a1*x + a0 + e*(0.5 - rand(n,1));  %訓練データにはノイズが含まれる


%% 訓練データと真の関数（ノイズなし）の描画
x1 = linspace(min(x),max(x),50);
x2 = linspace(min(y),max(y),50);
[X1,X2] = meshgrid(x1,x2);

Lam_true = a2*X2 + a1*X1+ a0;

figure(1)
plot3(x,y,lam, 'k+', 'MarkerSize', 20, 'LineWidth', 3);
hold on; grid on;
surf(X1,X2,Lam_true,...
    'FaceLighting','none',...
    'EdgeLighting','flat',...
    'LineStyle','none',...
    'FaceColor','r',...
    'EdgeColor','r',...
    'FaceAlpha',0.3);
legend({'Training Data', 'True Function (without Noise)'})
xlabel('x')
ylabel('y')
zlabel('lam')


%% 学習アルゴリズム

X = [ones(n,1) x y];      %行列Xの生成
c = inv(X'*X)*X'*lam;   %係数の決定（これが最適解）


%% 学習結果の描画
lam_learned =c(3,:)*X2 +c(2,:)*X1 +c(1,:);
surf(X1,X2,lam_learned,...
    'FaceLighting','none',...
    'EdgeLighting','flat',...
    'LineStyle','none',...
    'FaceAlpha',0.3,...
    'FaceColor','b',...
    'EdgeColor','b');
legend({'Training Data', 'True Function (without Noise)', 'Solution'}, 'Location', 'best');